using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using FFXIVClientStructs.FFXIV.Client.Game;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Volume;
using vnavmesh.Navigation.Volume.Map;
using vnavmesh.Navigation.Volume.Search;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Execution;

internal sealed class MovementUnstuckController
(
    Config         config,
    NavmeshManager manager
)
{
    private const double CHECK_EXPIRATION             = 1.0;
    private const double UNSTUCK_DURATION_SECONDS     = 1.0;
    private const float  MIN_MOVEMENT_DISTANCE        = 2.0f;
    private const float  MIN_RECOVERY_TARGET_DISTANCE = 25f;
    private const float  MAX_RECOVERY_TARGET_DISTANCE = 35f;
    private const int    RANDOM_TARGET_RESOLVE_ATTEMPTS = 12;

    private DateTime _lastMovement    = DateTime.MinValue;
    private DateTime _unstuckStart    = DateTime.MinValue;
    private DateTime _lastCheck       = DateTime.MinValue;
    private DateTime _lastJumpAttempt = DateTime.MinValue;
    private Vector3  _lastPosition;
    private Vector3  _recoveryTarget;

    public bool IsRunning { get; private set; }

    public bool SuspendsPathExecution => IsRunning;

    public void Reset()
    {
        _lastMovement    = DateTime.MinValue;
        _unstuckStart    = DateTime.MinValue;
        _lastCheck       = DateTime.MinValue;
        _lastJumpAttempt = DateTime.MinValue;
        _lastPosition    = default;
        _recoveryTarget  = default;
        IsRunning        = false;
    }

    public UnstuckUpdate Update(MovementExecutionContext context, bool suspended)
    {
        if (IsRunning)
            return UpdateRunning(context);

        if (!config.StopOnStuck || suspended || !context.MovementAllowed || !context.HasRemainingWaypoints)
        {
            ResetTracking();
            return default;
        }

        var destination = ResolvePathGoal(context);
        var now         = DateTime.Now;

        if (now.Subtract(_unstuckStart).TotalSeconds < config.UnstuckCooldownSeconds || destination == default)
        {
            _lastCheck = DateTime.MinValue;
            return default;
        }

        var lastCheck = _lastCheck;
        _lastCheck = now;

        if (now.Subtract(lastCheck).TotalSeconds > CHECK_EXPIRATION)
        {
            _lastPosition = context.Player.Position;
            _lastMovement = now;
            return default;
        }

        if (MeasureDistance(_lastPosition, context.Player.Position, IsGroundSegment(context)) >= MIN_MOVEMENT_DISTANCE)
        {
            _lastPosition = context.Player.Position;
            _lastMovement = now;
            return default;
        }

        if (now.Subtract(_lastMovement).TotalSeconds <= config.UnstuckDetectionSeconds)
            return default;

        Service.Log.Warning
        (
            $"[自动防卡] 角色疑似卡住：阶段 = {context.Segment.Kind}，位移 = {MeasureDistance(_lastPosition, context.Player.Position, IsGroundSegment(context)):f2} 米，持续时长 = {now.Subtract(_lastMovement).TotalSeconds:f2} 秒"
        );

        if (now.Subtract(_lastJumpAttempt).TotalSeconds > config.UnstuckCooldownSeconds * 2.0 && CanAttemptJump(context))
        {
            _lastMovement    = now;
            _lastJumpAttempt = now;
            _lastPosition    = context.Player.Position;
            Service.Log.Information("[自动防卡] 优先尝试跳跃脱困");
            return new(BuildRecoveryCommand(context, ResolvePathGoal(context), false, true));
        }

        if (manager.Query is not { } query || !TryResolveRecoveryTarget(query, context.Player.Position, IsFlightSegment(context), out _recoveryTarget))
        {
            Service.Log.Warning("[自动防卡] 未找到可用的随机脱困目标点，本次跳过位移脱困");
            _lastMovement = now;
            _lastPosition = context.Player.Position;
            return default;
        }

        IsRunning     = true;
        _unstuckStart = now;
        Service.Log.Information($"[自动防卡] 开始随机位移脱困：目标点 = {_recoveryTarget:f3}");
        return new(BuildRecoveryCommand(context, _recoveryTarget, IsFlightSegment(context), false), true);
    }

    private UnstuckUpdate UpdateRunning(MovementExecutionContext context)
    {
        if (DateTime.Now.Subtract(_unstuckStart).TotalSeconds > UNSTUCK_DURATION_SECONDS)
        {
            StopRunning();
            Service.Log.Information($"[自动防卡] 随机位移脱困结束，准备从当前位置重新算路：目标 = {context.Plan.RequestedDestination:f3}");
            return new(RequestRepath: true);
        }

        return new(BuildRecoveryCommand(context, _recoveryTarget, IsFlightSegment(context), false), true);
    }

    private static MovementFrameCommand BuildRecoveryCommand(MovementExecutionContext context, Vector3 desired, bool allowVerticalControl, bool requestJump)
    {
        var delta = desired - context.Player.Position;
        return new
        (
            desired,
            true,
            allowVerticalControl,
            context.Config.AlignCameraToMovement,
            Angle.FromDirectionXZ(delta) + 180.Degrees(),
            context.Config.AlignCameraHeight.Degrees(),
            requestJump,
            false,
            default
        );
    }

    private void StopRunning()
    {
        IsRunning     = false;
        _lastCheck    = DateTime.MinValue;
        _lastPosition = default;
    }

    private void ResetTracking()
    {
        _lastCheck    = DateTime.MinValue;
        _lastMovement = DateTime.MinValue;
        _lastPosition = default;
    }

    internal static bool TryResolveRecoveryTarget(NavmeshQuery query, Vector3 origin, bool fly, out Vector3 target)
    {
        target = default;

        for (var attempt = 0; attempt < RANDOM_TARGET_RESOLVE_ATTEMPTS; attempt++)
        {
            var candidate = fly ? ResolveRandomFlightTarget(query, origin) : ResolveRandomGroundTarget(query, origin);
            if (candidate is not { } resolved)
                continue;

            target = resolved;
            return true;
        }

        return false;
    }

    private static Vector3? ResolveRandomGroundTarget(NavmeshQuery query, Vector3 origin)
    {
        var radius = RandomDistance();
        var target = query.FindRandomPointOnMeshAroundCircle(origin, radius, false);
        return target is { } resolved && IsValidRecoveryTarget(origin, resolved, flatten: true) ? resolved : null;
    }

    private static Vector3? ResolveRandomFlightTarget(NavmeshQuery query, Vector3 origin)
    {
        var volume = query.VolumeQuery?.Volume;
        if (volume == null)
            return null;

        var radius         = RandomDistance();
        var verticalOffset = Random.Shared.NextSingle() * 16f - 8f;
        var angle          = Random.Shared.NextSingle() * MathF.Tau;
        var sample         = origin + new Vector3(MathF.Cos(angle) * radius, verticalOffset, MathF.Sin(angle) * radius);
        var voxel          = query.FindNearestVolumeVoxel(sample, 4f, 4f);
        if (voxel == VoxelMap.INVALID_VOXEL)
            return null;

        var target = VoxelSearch.FindClosestVoxelPoint(volume, voxel, sample);
        return IsValidRecoveryTarget(origin, target, flatten: false) ? target : null;
    }

    private static float RandomDistance() =>
        Random.Shared.NextSingle() * (MAX_RECOVERY_TARGET_DISTANCE - MIN_RECOVERY_TARGET_DISTANCE) + MIN_RECOVERY_TARGET_DISTANCE;

    private static bool IsValidRecoveryTarget(Vector3 origin, Vector3 target, bool flatten)
    {
        var delta = target - origin;
        return flatten
            ? new Vector2(delta.X, delta.Z).Length() >= MIN_RECOVERY_TARGET_DISTANCE
            : delta.Length() >= MIN_RECOVERY_TARGET_DISTANCE;
    }

    private static Vector3 ResolvePathGoal(MovementExecutionContext context) =>
        context.ActiveWaypoint ?? context.Plan.FinalDestination;

    private static bool IsGroundSegment(MovementExecutionContext context) => context.Segment.Kind == MovementSegmentKind.GroundTraverse;

    private static bool IsFlightSegment(MovementExecutionContext context) => context.Segment.Kind == MovementSegmentKind.FlightTraverse;

    private static float MeasureDistance(Vector3 from, Vector3 to, bool flatten)
    {
        var delta = to - from;
        return flatten ? new Vector2(delta.X, delta.Z).Length() : delta.Length();
    }

    private static bool CanAttemptJump(MovementExecutionContext context)
    {
        if (!IsGroundSegment(context)                 ||
            Service.Condition[ConditionFlag.InFlight] ||
            Service.Condition[ConditionFlag.Diving]   ||
            Service.Condition[ConditionFlag.Jumping])
            return false;

        unsafe
        {
            var actionManager = ActionManager.Instance();
            return actionManager != null && actionManager->GetActionStatus(ActionType.GeneralAction, 2) == 0;
        }
    }
}

internal readonly record struct UnstuckUpdate
(
    MovementFrameCommand? OverrideCommand      = null,
    bool                  SuspendPathExecution = false,
    bool                  RequestRepath        = false
);
