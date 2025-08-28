using System;
using System.Collections.Generic;
using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using FFXIVClientStructs.FFXIV.Client.Game;
using Navmesh.Utilities;

namespace Navmesh.Movement;

public class FollowPath : IDisposable
{
    // 卡住恢复相关常量
    private const float GROUND_STUCK_THRESHOLD         = 1f; // 地面移动卡住检测阈值
    private const float FLYING_STUCK_THRESHOLD         = 3f; // 飞行移动卡住检测阈值
    private const float SIGNIFICANT_MOVEMENT_THRESHOLD = 3f; // 显著移动阈值
    private const int   STUCK_COUNTER_THRESHOLD        = 3;  // 连续几次检测卡住才判定为真卡住

    public bool          MovementAllowed      { get; set; } = true;
    public bool          IgnoreDeltaY         { get; set; }
    public float         Tolerance            { get; set; } = 1f;
    public float         DestinationTolerance { get; set; }
    public List<Vector3> Waypoints            { get; set; } = [];
    public bool          IsStuck              { get; set; }

    private readonly IDalamudPluginInterface PI;
    private readonly NavmeshManager          NavmeshManager;
    private readonly OverrideCamera          Camera   = new();
    private readonly OverrideMovement        Movement = new();

    private DateTime NextJump;

    private Vector3? posPreviousFrame;

    // 卡住检测相关
    private          Vector3  LastSignificantPosition;
    private          DateTime LastMovementTime;
    private          DateTime LastStuckCheckTime;
    private          int      StuckCounter;
    private readonly TimeSpan StuckCheckInterval = TimeSpan.FromSeconds(1f); // 卡住检测间隔
    private readonly TimeSpan StuckTimeout       = TimeSpan.FromSeconds(2f); // 判定为卡住的时间阈值
    private          int      RecoveryAttempts;
    private          DateTime NextRecoveryTime;
    private          Vector3? RecoveryDirection;
    private          DateTime StuckStartTime;                                       // 记录开始卡住的时间
    private          bool     inRecoveryProcess;                                    // 标记是否正在恢复过程中
    private          DateTime LastSignificantMovementTime;                          // 上次有显著移动的时间
    private          bool     PathRecalculationRequested;                           // 是否已请求重新计算路径
    private          Vector3? OriginalTargetPosition;                               // 原始目标位置
    private          Vector3? AlternativeTargetPosition;                            // 备选目标位置
    private          DateTime LastPathRecalculationTime;                            // 上次重新计算路径的时间
    private readonly TimeSpan PathRecalculationCooldown = TimeSpan.FromSeconds(5f); // 路径重新计算冷却时间
    private          bool     IsFlying;                                             // 当前是否处于飞行状态

    public event Action<Vector3, Vector3, bool>? RequestPathRecalculation;
    
    // entries in dalamud shared data cache must be reference types, so we use an array
    private readonly bool[] sharedPathIsRunning;

    private const string _sharedPathTag = "vnav.PathIsRunning";

    public FollowPath(IDalamudPluginInterface dalamud, NavmeshManager manager)
    {
        PI                  = dalamud;
        sharedPathIsRunning = PI.GetOrCreateData<bool[]>(_sharedPathTag, () => [false]);
        NavmeshManager      = manager;
        
        NavmeshManager.OnNavmeshChanged += OnNavmeshChanged;
        OnNavmeshChanged(NavmeshManager.Navmesh, NavmeshManager.Query);
        LastMovementTime            = DateTime.Now;
        LastStuckCheckTime          = DateTime.Now;
        NextRecoveryTime            = DateTime.Now;
        LastSignificantMovementTime = DateTime.Now;
        LastPathRecalculationTime   = DateTime.Now;
    }

    public void Dispose()
    {
        UpdateSharedState(false);
        PI.RelinquishData(_sharedPathTag);
        NavmeshManager.OnNavmeshChanged -= OnNavmeshChanged;
        Camera.Dispose();
        Movement.Dispose();
    }

    private void UpdateSharedState(bool isRunning) => sharedPathIsRunning[0] = isRunning;

    public void Update(IFramework fwk)
    {
        var player = Service.ClientState.LocalPlayer;
        if (player == null) return;

        // 更新飞行状态
        IsFlying = Service.Condition.Any(ConditionFlag.InFlight, ConditionFlag.Diving) || !IgnoreDeltaY;

        // 如果正在计算路径，暂时不更新路点和移动
        if (PathRecalculationRequested && Plugin.Instance().AsyncMove.TaskInBusy)
        {
            // 在路径计算过程中保持当前位置，不继续向目标移动
            if (MovementAllowed) Movement.Enabled = false;

            posPreviousFrame = player.Position;
            return;
        }

        while (Waypoints.Count > 0)
        {
            var a = Waypoints[0];
            var b = player.Position;
            var c = posPreviousFrame ?? b;

            if (DestinationTolerance > 0 && (b - Waypoints[^1]).Length() <= DestinationTolerance)
            {
                Waypoints.Clear();
                break;
            }

            if (IgnoreDeltaY)
            {
                a.Y = 0;
                b.Y = 0;
                c.Y = 0;
            }

            if (DistanceToLineSegment(a, b, c) > Tolerance)
                break;

            Waypoints.RemoveAt(0);
        }

        // 检测卡住情况
        var now        = DateTime.Now;
        var currentPos = player.Position;

        if (posPreviousFrame.HasValue && Waypoints.Count > 0 && 
            (IgnoreDeltaY || Service.Condition[ConditionFlag.Mounted]))
        {
            CheckForStuck(currentPos, now);

            // 如果检测到卡住，尝试恢复
            if (IsStuck)
                TryRecoverFromStuck(currentPos, now);
            else if (!inRecoveryProcess)
            {
                // 只有在非恢复过程中才重置恢复相关变量
                RecoveryAttempts           = 0;
                RecoveryDirection          = null;
                PathRecalculationRequested = false;
                OriginalTargetPosition     = null;
                AlternativeTargetPosition  = null;
            }
        }

        posPreviousFrame = currentPos;

        if (Waypoints.Count == 0)
        {
            Movement.Enabled         = Camera.Enabled = false;
            Camera.SpeedH            = Camera.SpeedV  = default;
            Movement.DesiredPosition = player.Position;
            UpdateSharedState(false);

            // 重置卡住状态
            ResetStuckState();
        }
        else
        {
            posPreviousFrame = player.Position;

            if (Service.Config.CancelMoveOnUserInput && Movement.UserInput)
            {
                Stop();
                return;
            }

            OverrideAFK.ResetTimers();

            // 如果路径正在重新计算，保持移动暂停
            if (PathRecalculationRequested && Plugin.Instance().AsyncMove.TaskInBusy)
                Movement.Enabled = false;
            else
                Movement.Enabled = MovementAllowed;

            // 如果正在执行恢复动作，使用恢复方向
            if (IsStuck && RecoveryDirection.HasValue && now < NextRecoveryTime)
                Movement.DesiredPosition = player.Position + RecoveryDirection.Value;
            else
                Movement.DesiredPosition = Waypoints[0];

            if (Movement.DesiredPosition.Y > player.Position.Y && 
                !Service.Condition[ConditionFlag.InFlight] && !Service.Condition[ConditionFlag.Diving] &&
                !IgnoreDeltaY) //Only do this bit if on a flying path
            {
                // walk->fly transition (TODO: reconsider?)
                if (Service.Condition[ConditionFlag.Mounted])
                    ExecuteJump(); // Spam jump to take off
                else
                {
                    Movement.Enabled = false; // Don't move, since it'll just run on the spot
                    return;
                }
            }

            Camera.Enabled         = Service.Config.AlignCameraToMovement;
            Camera.SpeedH          = Camera.SpeedV = 360.Degrees();
            Camera.DesiredAzimuth  = Angle.FromDirectionXZ(Movement.DesiredPosition - player.Position) + 180.Degrees();
            Camera.DesiredAltitude = -30.Degrees();
        }
    }

    private void CheckForStuck(Vector3 currentPos, DateTime now)
    {
        // 如果正在计算新路径，暂时不进行卡住检测，防止重复判断
        if (Plugin.Instance().AsyncMove.TaskInBusy ||
            PathRecalculationRequested)
        {
            // 如果正在计算路径，重置卡住检测计时器，防止误判
            LastStuckCheckTime = now;
            LastMovementTime   = now;
            return;
        }

        // 每隔一段时间检查一次是否卡住
        if (now - LastStuckCheckTime < StuckCheckInterval)
            return;

        LastStuckCheckTime = now;

        // 根据当前移动模式选择适当的阈值
        var stuckThreshold = IsFlying ? FLYING_STUCK_THRESHOLD : GROUND_STUCK_THRESHOLD;

        // 计算距离上次显著位置的移动距离
        var movementDistance = IgnoreDeltaY
                                   ? Vector2.Distance(new Vector2(currentPos.X, currentPos.Z), new Vector2(LastSignificantPosition.X, LastSignificantPosition.Z))
                                   : Vector3.Distance(currentPos, LastSignificantPosition);

        // 检测是否有显著移动（超过显著移动阈值）
        var hasSignificantMovement = movementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD;
        if (hasSignificantMovement)
        {
            LastSignificantMovementTime = now;

            // 只有非恢复过程中或者有显著移动时才重置卡住状态
            if (!inRecoveryProcess)
            {
                LastSignificantPosition = currentPos;
                LastMovementTime        = now;
                StuckCounter            = 0;
                IsStuck                 = false;
            }
        }

        // 如果移动距离小于阈值，可能卡住了
        if (movementDistance < stuckThreshold)
        {
            StuckCounter++;

            // 如果连续多次检测都没有移动，且超过了超时时间，则判定为卡住
            if (StuckCounter >= STUCK_COUNTER_THRESHOLD && now - LastMovementTime > StuckTimeout)
            {
                if (!IsStuck)
                {
                    // 再次检查是否在计算路径，防止误判
                    if (Plugin.Instance().AsyncMove.TaskInBusy || PathRecalculationRequested)
                    {
                        Service.Log.Debug("检测到可能卡住，但正在计算路径，暂不判定为卡住");
                        StuckCounter--;
                        return;
                    }

                    Service.Log.Debug($"检测到导航卡住: 位置 {currentPos}, 未移动时间: {(now - LastMovementTime).TotalSeconds:F1}秒, 移动模式: {(IsFlying ? "飞行" : "地面")}");
                    IsStuck           = true;
                    StuckStartTime    = now;
                    inRecoveryProcess = true;

                    // 记录原始目标位置
                    if (Waypoints.Count > 0 && OriginalTargetPosition == null) OriginalTargetPosition = Waypoints[^1]; // 保存最终目标点
                }
            }
        }
        else if (!inRecoveryProcess)
        {
            // 仅在非恢复过程中更新位置和时间
            LastSignificantPosition = currentPos;
            LastMovementTime        = now;

            // 只有移动大于阈值且不在恢复过程中，才减少计数器
            if (StuckCounter > 0)
                StuckCounter--;
        }

        // 检查是否长时间卡住（超过8秒），无论是否有微小移动
        if (IsStuck && now - StuckStartTime > TimeSpan.FromSeconds(8) && now - LastSignificantMovementTime > TimeSpan.FromSeconds(4))
        {
            // 确保不在路径计算中再触发重新计算
            if (!Plugin.Instance().AsyncMove.TaskInBusy && !PathRecalculationRequested)
            {
                // 长时间卡住，直接重新计算路径
                TryRecalculatePath(currentPos, now, true);
            }
        }
    }

    private void TryRecalculatePath(Vector3 currentPos, DateTime now, bool forceFindAlternative = false)
    {
        if (PathRecalculationRequested && !forceFindAlternative) return;

        if (now - LastPathRecalculationTime < PathRecalculationCooldown && !forceFindAlternative)
            return;

        // 如果已经在计算路径，不要重复请求
        if (Plugin.Instance().AsyncMove.TaskInBusy)
        {
            Service.Log.Debug("正在计算路径中，忽略重新计算请求");
            return;
        }

        LastPathRecalculationTime = now;

        if (OriginalTargetPosition.HasValue)
        {
            // 临时暂停当前移动，防止继续向卡住位置移动
            Movement.Enabled = false;
            Service.Log.Debug("暂停当前移动，等待路径重新计算");

            var targetPos = OriginalTargetPosition.Value;

            if (forceFindAlternative || AlternativeTargetPosition == null)
            {
                var random  = new Random();
                var offsetX = (float)((random.NextDouble() * 5.0) - 2.5);
                var offsetZ = (float)((random.NextDouble() * 5.0) - 2.5);
                var offsetY = IsFlying ? (float)((random.NextDouble() * 4.0) - 2.0) : 0; // 飞行时才考虑Y轴偏移

                AlternativeTargetPosition = new Vector3(
                    targetPos.X + offsetX,
                    targetPos.Y + offsetY, // 飞行模式下Y轴也添加随机偏移
                    targetPos.Z + offsetZ
                );

                Service.Log.Debug($"生成备选目标点: 原始{targetPos} -> 备选{AlternativeTargetPosition}, 移动模式: {(IsFlying ? "飞行" : "地面")}");
            }

            var destination = forceFindAlternative && AlternativeTargetPosition.HasValue ? AlternativeTargetPosition.Value : targetPos;

            // 注册路径计算完成的回调
            Plugin.Instance().AsyncMove.OnPathCalculationComplete += OnPathCalculationComplete;

            // 触发路径重新计算事件
            Service.Log.Debug($"请求重新计算路径: 从 {currentPos} 到 {destination}, 移动模式: {(IsFlying ? "飞行" : "地面")}");
            RequestPathRecalculation?.Invoke(currentPos, destination, IgnoreDeltaY);
            PathRecalculationRequested = true;

            // 设置较长的恢复时间，等待路径计算完成
            NextRecoveryTime  = now.AddSeconds(2.5); // 增加等待时间，确保有足够时间计算路径
            RecoveryDirection = null;

            // 重置卡住检测的计时器，避免在计算路径期间触发卡住判定
            LastMovementTime = now;
        }
    }

    // 路径计算完成的回调方法
    private void OnPathCalculationComplete(List<Vector3> newPath, bool isSuccess)
    {
        // 取消订阅事件，防止多次触发
        Plugin.Instance().AsyncMove.OnPathCalculationComplete -= OnPathCalculationComplete;

        // 获取当前玩家位置
        var player = Service.ClientState.LocalPlayer;
        if (player == null) return;

        if (isSuccess && newPath.Count > 0)
        {
            // 更新最近的显著位置为当前位置，避免继续判定为卡住
            LastSignificantPosition     = player.Position;
            LastMovementTime            = DateTime.Now;
            LastSignificantMovementTime = DateTime.Now;

            // 路径已在AsyncMoveRequest.Update中设置到Waypoints，这里不需要再次设置
            // 但需要确保启用移动
            Movement.Enabled = MovementAllowed;

            // 重置卡住状态，但保持在恢复过程中的标记
            IsStuck          = false;
            StuckCounter     = 0;
            RecoveryAttempts = 0;

            Service.Log.Debug($"路径计算完成，获取到{newPath.Count}个路径点，恢复导航");
        }
        else
        {
            Service.Log.Error("路径计算失败或未返回有效路径");

            // 如果路径计算失败，恢复移动能力，但不改变恢复过程状态
            Movement.Enabled = MovementAllowed;

            // 标记路径计算请求已结束，但保持恢复过程状态
            PathRecalculationRequested = false;
        }
    }

    private void TryRecoverFromStuck(Vector3 currentPos, DateTime now)
    {
        if (now < NextRecoveryTime) return;

        // 如果正在计算路径，等待计算完成
        if (PathRecalculationRequested && Plugin.Instance().AsyncMove.TaskInBusy)
        {
            Service.Log.Debug("正在等待路径计算完成，暂停恢复尝试");
            NextRecoveryTime = now.AddSeconds(1.0);
            return;
        }

        RecoveryAttempts++;
        inRecoveryProcess = true;

        // 根据当前是否飞行选择不同的恢复策略
        if (IsFlying)
            RecoverFromFlyingStuck(currentPos, now);
        else
            RecoverFromGroundStuck(currentPos, now);
    }

    // 飞行状态下的恢复策略
    private void RecoverFromFlyingStuck(Vector3 currentPos, DateTime now)
    {
        // 检测是否可能因为高度限制而卡住
        var potentialHeightLimit = false;

        // 如果当前位置和上次位置的Y轴差值很小，可能是达到了高度限制
        if (posPreviousFrame.HasValue)
        {
            // 如果向上移动但Y轴增量接近于0，可能是遇到高度限制
            if (RecoveryDirection is { Y: > 0 } &&
                Math.Abs(currentPos.Y - posPreviousFrame.Value.Y)              < 0.1f)
            {
                potentialHeightLimit = true;
                Service.Log.Debug("检测到可能存在高度限制，Y轴变化很小");
            }
        }

        // 获取当前目标方向
        var targetDirection                      = Vector3.UnitZ; // 默认向前
        if (Waypoints.Count > 0) targetDirection = Vector3.Normalize(Waypoints[0] - currentPos);

        switch (RecoveryAttempts)
        {
            // 第一次尝试：先水平移动（向目标方向）
            case 1:
                // 首次尝试以水平移动为主
                var horizontalDirection = targetDirection with { Y = 0 };
                horizontalDirection = horizontalDirection.Length() > 0.001f ? Vector3.Normalize(horizontalDirection) :
                                          // 没有明确的水平方向，选择向前
                                          Vector3.UnitZ;

                RecoveryDirection = horizontalDirection * 3.0f;
                Service.Log.Debug("飞行模式：尝试水平移动朝向目标方向");
                NextRecoveryTime = now.AddSeconds(1.0f);
                break;

            // 第二次尝试：根据第一次尝试的效果决定是侧向移动还是上升
            case 2:
                if (potentialHeightLimit)
                {
                    // 检测到高度限制，尝试左右侧向移动
                    const float sideAngle = 90.0f * (float)Math.PI / 180.0f;
                    var         direction = Vector3.Normalize(targetDirection with { Y = 0 });

                    RecoveryDirection = new Vector3(
                                             (direction.X * (float)Math.Cos(sideAngle)) - (direction.Z * (float)Math.Sin(sideAngle)),
                                             0, // 不改变高度
                                             (direction.X * (float)Math.Sin(sideAngle)) + (direction.Z * (float)Math.Cos(sideAngle))) * 3.0f;

                    Service.Log.Debug("检测到高度限制，尝试侧向移动");
                }
                else
                {
                    // 没有高度限制，尝试缓慢上升加水平移动的组合动作
                    var direction = new Vector3(targetDirection.X, 0.5f, targetDirection.Z);
                    if (direction.Length() > 0.001f)
                        direction = Vector3.Normalize(direction);

                    RecoveryDirection = direction * 3.0f;
                    Service.Log.Debug("飞行模式：尝试缓慢上升加水平移动");
                }

                NextRecoveryTime = now.AddSeconds(1.2f);
                break;

            // 第三次尝试：尝试另一个方向的侧向移动或下降
            case 3:
                if (potentialHeightLimit)
                {
                    // 如果检测到高度限制，尝试向下移动
                    var direction = new Vector3(targetDirection.X, -1.5f, targetDirection.Z);
                    if (direction.Length() > 0.001f)
                        direction = Vector3.Normalize(direction);

                    RecoveryDirection = direction * 3.0f;
                    Service.Log.Debug("检测到高度限制，尝试向下移动并前进");
                }
                else
                {
                    // 尝试向另一侧移动
                    const float rightAngle = -90.0f * (float)Math.PI / 180.0f;
                    var         direction  = Vector3.Normalize(new Vector3(targetDirection.X, 0, targetDirection.Z));

                    RecoveryDirection = new Vector3(
                                             (direction.X * (float)Math.Cos(rightAngle)) - (direction.Z * (float)Math.Sin(rightAngle)),
                                             0.2f, // 轻微上升
                                             (direction.X * (float)Math.Sin(rightAngle)) + (direction.Z * (float)Math.Cos(rightAngle))) * 3.0f;

                    Service.Log.Debug("飞行模式：尝试另一侧向移动");
                }

                NextRecoveryTime = now.AddSeconds(1.2f);

                break;

            // 第四次尝试：尝试更大幅度的上升或向后移动
            case 4:
                if (potentialHeightLimit)
                {
                    // 在高度限制的情况下，尝试水平后退
                    var backDirection = -Vector3.Normalize(new Vector3(targetDirection.X, 0, targetDirection.Z));
                    RecoveryDirection = backDirection * 4.0f;
                    Service.Log.Debug("检测到高度限制，尝试水平后退");
                }
                else
                {
                    // 检测是否需要更大幅度上升
                    var needVerticalMovement = Waypoints.Count > 0 && Waypoints[0].Y - currentPos.Y > 3.0f;

                    if (needVerticalMovement)
                    {
                        // 需要上升才能接近目标
                        RecoveryDirection = new Vector3(0, 4.0f, 0);
                        Service.Log.Debug("飞行模式：尝试垂直上升");
                    }
                    else
                    {
                        // 尝试后退再前进的策略
                        var backDirection = -Vector3.Normalize(new Vector3(targetDirection.X, 0, targetDirection.Z));
                        RecoveryDirection = new Vector3(backDirection.X, 1.0f, backDirection.Z) * 3.0f;
                        Service.Log.Debug("飞行模式：尝试后退上升");
                    }
                }

                NextRecoveryTime = now.AddSeconds(1.5f);
                break;

            // 第五次尝试：重新计算路径，并考虑高度限制
            case 5:
                Service.Log.Debug("飞行模式：尝试重新计算路径");
                TryRecalculatePath(currentPos, now);
                break;

            // 第六次尝试：使用备选目标点重新计算
            case 6:
                Service.Log.Debug("飞行模式：使用备选目标点重新计算");
                // 如果检测到高度限制，强制将备选目标点的高度调低
                if (potentialHeightLimit && OriginalTargetPosition.HasValue)
                {
                    var targetPos = OriginalTargetPosition.Value;

                    // 将目标点的高度调整为低于当前高度
                    AlternativeTargetPosition = new Vector3(
                        targetPos.X  + (((float)new Random().NextDouble() * 5.0f) - 2.5f),
                        currentPos.Y - 5.0f, // 将高度设定为当前高度以下
                        targetPos.Z  + (((float)new Random().NextDouble() * 5.0f) - 2.5f)
                    );
                    Service.Log.Debug($"检测到高度限制，生成低高度备选目标点: {AlternativeTargetPosition}");
                }

                TryRecalculatePath(currentPos, now, true);
                break;

            // 多次尝试失败，停止导航
            default:
                Service.Log.Error("多次飞行恢复尝试失败，停止导航");
                Stop();
                break;
        }
    }

    // 地面行走状态的恢复策略
    private void RecoverFromGroundStuck(Vector3 currentPos, DateTime now)
    {
        // 获取当前目标方向
        var targetDirection = Vector3.UnitZ; // 默认向前
        if (Waypoints.Count > 0)
        {
            targetDirection = Vector3.Normalize(Waypoints[0] - currentPos);
            // 移除Y轴分量，纯水平移动
            targetDirection.Y = 0;
            if (targetDirection.Length() > 0.001f)
                targetDirection = Vector3.Normalize(targetDirection);
        }

        switch (RecoveryAttempts)
        {
            // 第一次尝试：跳跃加向前移动
            case 1:
                Service.Log.Debug("地面模式：尝试跳跃加前进");
                ExecuteJump();

                // 跳跃的同时向前移动
                RecoveryDirection = targetDirection * 2.0f;

                NextRecoveryTime        = now.AddSeconds(1.0);
                LastSignificantPosition = currentPos;
                break;

            // 第二次尝试：检查跳跃效果，否则尝试侧向移动
            case 2:
                var jumpMovementDistance = Vector3.Distance(currentPos, LastSignificantPosition);
                if (jumpMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"跳跃前进有效，移动了 {jumpMovementDistance:F2} 距离，重置卡住状态");

                    // 重置卡住状态，但保留当前位置作为新的参考点
                    IsStuck                 = false;
                    StuckCounter            = 0;
                    RecoveryAttempts        = 0;
                    inRecoveryProcess       = false;
                    LastMovementTime        = now;
                    LastSignificantPosition = currentPos;
                    return;
                }

                // 跳跃效果不明显，尝试左转移动
                const float leftAngle = 60.0f * (float)Math.PI / 180.0f;

                // 计算左转60度的方向
                RecoveryDirection = new Vector3(
                                         (targetDirection.X * (float)Math.Cos(leftAngle)) - (targetDirection.Z * (float)Math.Sin(leftAngle)),
                                         0,
                                         (targetDirection.X * (float)Math.Sin(leftAngle)) + (targetDirection.Z * (float)Math.Cos(leftAngle))
                                     ) * 2.0f;

                Service.Log.Debug("地面模式：尝试左侧移动");
                NextRecoveryTime        = now.AddSeconds(1.0);
                LastSignificantPosition = currentPos; // 更新参考位置
                break;

            // 第三次尝试：检查左转效果，否则尝试右转移动
            case 3:
                // 检查左转移动是否有效
                var leftMovementDistance = Vector3.Distance(currentPos, LastSignificantPosition);

                if (leftMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"左侧移动有效，移动了{leftMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    LastMovementTime        = now;
                    LastSignificantPosition = currentPos;
                    return;
                }

                // 左转移动效果不明显，尝试右转移动
                const float rightAngle = -60.0f * (float)Math.PI / 180.0f;

                // 计算右转60度的方向
                RecoveryDirection = new Vector3(
                                         (targetDirection.X * (float)Math.Cos(rightAngle)) - (targetDirection.Z * (float)Math.Sin(rightAngle)),
                                         0,
                                         (targetDirection.X * (float)Math.Sin(rightAngle)) + (targetDirection.Z * (float)Math.Cos(rightAngle))
                                     ) * 2.0f;

                Service.Log.Debug("地面模式：尝试右侧移动");
                NextRecoveryTime        = now.AddSeconds(1.0);
                LastSignificantPosition = currentPos;
                break;

            // 第四次尝试：检查右转效果，否则尝试后退
            case 4:
                // 检查右转移动是否有效
                var rightMovementDistance = Vector3.Distance(currentPos, LastSignificantPosition);

                if (rightMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"右侧移动有效，移动了{rightMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    LastMovementTime        = now;
                    LastSignificantPosition = currentPos;
                    return;
                }

                // 微调方向都无效，尝试直接后退
                var backDirection = -targetDirection;
                RecoveryDirection = backDirection * 3.0f;

                Service.Log.Debug("地面模式：尝试后退");
                NextRecoveryTime        = now.AddSeconds(1.5);
                LastSignificantPosition = currentPos;
                break;

            // 第五次尝试：检查后退效果，否则重新计算路径
            case 5:
                // 检查后退是否有效
                var backMovementDistance = Vector3.Distance(currentPos, LastSignificantPosition);

                if (backMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"后退移动有效，移动了{backMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    LastMovementTime        = now;
                    LastSignificantPosition = currentPos;
                    return;
                }

                // 后退无效，尝试重新计算路径
                Service.Log.Debug("地面模式：方向调整未奏效，尝试重新计算路径");
                TryRecalculatePath(currentPos, now);
                break;

            // 第六次尝试：检查新路径效果，或使用备选目标点重新计算
            case 6:
                // 如果新路径已计算完成但仍卡住
                if (!Plugin.Instance().AsyncMove.TaskInBusy && now - LastPathRecalculationTime > TimeSpan.FromSeconds(3.0))
                {
                    // 尝试跳跃+向前冲刺组合
                    Service.Log.Debug("地面模式：尝试跳跃冲刺组合");
                    ExecuteJump();

                    // 跳跃加速冲刺
                    RecoveryDirection = targetDirection * 4.0f;

                    NextRecoveryTime        = now.AddSeconds(1.5);
                    LastSignificantPosition = currentPos;
                }
                else
                {
                    // 路径计算中或刚计算完，多等一会
                    NextRecoveryTime = now.AddSeconds(1.0);
                }

                break;

            // 第七次尝试：使用备选目标点重新计算路径
            case 7:
                // 检查跳跃冲刺是否有效
                var sprintMovementDistance = Vector3.Distance(currentPos, LastSignificantPosition);

                if (sprintMovementDistance > SIGNIFICANT_MOVEMENT_THRESHOLD * 0.5f)
                {
                    Service.Log.Debug($"跳跃冲刺有效，移动了{sprintMovementDistance:F2}距离，重置卡住状态");
                    ResetStuckState();
                    LastMovementTime        = now;
                    LastSignificantPosition = currentPos;
                    return;
                }

                // 使用备选目标点重新计算路径
                Service.Log.Debug("地面模式：使用备选目标点重新计算");
                TryRecalculatePath(currentPos, now, true);
                break;

            // 最后尝试：原地旋转跳跃组合
            case 8:
                Service.Log.Debug("地面模式：尝试原地旋转跳跃组合");
                ExecuteJump();

                // 随机一个方向
                var random      = new Random();
                var randomAngle = random.Next(0, 360) * (float)Math.PI / 180.0f;

                RecoveryDirection = new Vector3(
                    (float)Math.Cos(randomAngle) * 2.0f,
                    0,
                    (float)Math.Sin(randomAngle) * 2.0f
                );

                NextRecoveryTime        = now.AddSeconds(2.0);
                LastSignificantPosition = currentPos;
                break;

            default:
                // 所有策略都失败，停止导航
                Service.Log.Error("多次地面恢复尝试失败，停止导航");
                Stop();
                break;
        }
    }

    private void ResetStuckState()
    {
        IsStuck                    = false;
        StuckCounter               = 0;
        RecoveryAttempts           = 0;
        RecoveryDirection          = null;
        inRecoveryProcess          = false;
        PathRecalculationRequested = false;
        OriginalTargetPosition     = null;
        AlternativeTargetPosition  = null;
    }

    private static float DistanceToLineSegment(Vector3 v, Vector3 a, Vector3 b)
    {
        var ab = b - a;
        var av = v - a;

        if (ab.Length() == 0 || Vector3.Dot(av, ab) <= 0)
            return av.Length();

        var bv = v - b;
        if (Vector3.Dot(bv, ab) >= 0)
            return bv.Length();

        return Vector3.Cross(ab, av).Length() / ab.Length();
    }

    public void Stop()
    {
        UpdateSharedState(false);
        ResetStuckState();
        Waypoints.Clear();
    }

    private unsafe void ExecuteJump()
    {
        if (Service.Condition[ConditionFlag.Diving]) return;
        if (DateTime.Now >= NextJump)
        {
            ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
            NextJump = DateTime.Now.AddMilliseconds(100);
        }
    }

    public void Move(List<Vector3> waypoints, bool ignoreDeltaY, float destTolerance = 0)
    {
        UpdateSharedState(true);
        Waypoints            = waypoints;
        IgnoreDeltaY         = ignoreDeltaY;
        DestinationTolerance = destTolerance;

        // 重置卡住检测状态
        ResetStuckState();

        var player = Service.ClientState.LocalPlayer;
        if (player != null)
        {
            LastSignificantPosition     = player.Position;
            LastMovementTime            = DateTime.Now;
            LastSignificantMovementTime = DateTime.Now;
        }
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        UpdateSharedState(false);
        Waypoints.Clear();
    }
}
