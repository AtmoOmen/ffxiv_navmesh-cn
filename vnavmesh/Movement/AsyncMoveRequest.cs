using System.Numerics;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement;

public class AsyncMoveRequest : IDisposable
{
    private const float DUPLICATE_REQUEST_DISTANCE_SQ = 0.000001f;

    private readonly NavmeshManager           manager;
    private readonly MovementPlanExecutor     executor;
    private readonly MovementPlanBuilder      planBuilder = new();
    private          Task<PostprocessedPath>? pendingTask;
    private          CancellationTokenSource? pendingTaskCancelSource;
    private          PendingMoveRequest?      pendingMoveRequest;
    private          PendingMoveRequest?      activeMoveRequest;
    private          ExternalMoveRequest?     activeExternalMoveRequest;
    private          RecoveryRetry?           recoveryRetry;

    public bool TaskInProgress => pendingTask != null;

    public AsyncMoveRequest(NavmeshManager manager, MovementPlanExecutor executor)
    {
        this.manager  = manager;
        this.executor = executor;
        this.executor.OnMovementFailure += failure =>
        {
            if (failure.Reason != MovementFailureReason.RepathRequiredAfterUnstuck)
                return;

            Service.Log.Information($"[自动防卡] 随机位移结束，开始重新算路：目标 = {failure.RequestedDestination:f3}");
            MoveToInternal
            (
                failure.RequestedDestination,
                failure.RequestedMode == MovementMode.Flight,
                failure.DestinationTolerance,
                PathRequestOrigin.RepathAfterUnstuck
            );
        };
    }

    public void Dispose()
    {
        pendingTaskCancelSource?.Cancel();
        if (pendingTask != null)
        {
            if (!pendingTask.IsCompleted)
                pendingTask.Wait();
            pendingTask.Dispose();
            pendingTask = null;
        }

        pendingTaskCancelSource?.Dispose();
        pendingTaskCancelSource = null;
    }

    public void Update()
    {
        if (pendingTask == null && recoveryRetry == null && !executor.IsRunning)
        {
            activeMoveRequest         = null;
            activeExternalMoveRequest = null;
        }

        if (recoveryRetry is { } scheduled && pendingTask == null && DateTime.Now >= scheduled.ExecuteAt)
        {
            recoveryRetry = null;
            executor.Stop();
            Service.Log.Information($"[自动防卡] 随机位移完成，重新算路：目标 = {scheduled.Destination:f3}");
            MoveToInternal(scheduled.Destination, scheduled.Fly, scheduled.Range, PathRequestOrigin.RepathAfterUnstuck);
        }

        if (pendingTask is { IsCompleted: true })
        {
            Service.Log.Information("算路任务已完成");

            try
            {
                var result  = pendingTask.Result;
                var request = pendingMoveRequest;

                if (result.Succeeded)
                {
                    executor.Execute(BuildPlan(result));
                    activeMoveRequest = request;
                    activeExternalMoveRequest = request is { Origin: PathRequestOrigin.Normal }
                                                    ? new(request.Value.Destination, request.Value.Fly, request.Value.Range)
                                                    : null;
                    recoveryRetry = null;
                }
                else if (request is { Origin: PathRequestOrigin.RepathAfterUnstuck })
                {
                    activeMoveRequest = request;
                    HandleFailedRepathAfterUnstuck(request.Value);
                }
                else
                {
                    activeMoveRequest         = null;
                    activeExternalMoveRequest = null;
                }
            }
            catch (OperationCanceledException)
            {
                activeMoveRequest         = null;
                activeExternalMoveRequest = null;
            }
            catch (Exception ex)
            {
                activeMoveRequest         = null;
                activeExternalMoveRequest = null;
                Plugin.DuoLog(ex, "算路失败");
            }

            pendingTask.Dispose();
            pendingTask = null;
            pendingTaskCancelSource?.Dispose();
            pendingTaskCancelSource = null;
            pendingMoveRequest      = null;
        }
    }

    public bool MoveTo(Vector3 dest, bool fly, float range = 0)
        => MoveToInternal(dest, fly, range, PathRequestOrigin.Normal);

    public void Stop()
    {
        recoveryRetry             = null;
        pendingMoveRequest        = null;
        activeMoveRequest         = null;
        activeExternalMoveRequest = null;
        executor.Stop();
        CancelPendingPathfind();
    }

    private void CancelPendingPathfind()
    {
        pendingTaskCancelSource?.Cancel();

        if (pendingTask is { IsCompleted: true })
        {
            pendingTask.Dispose();
            pendingTask = null;
        }

        pendingTaskCancelSource?.Dispose();
        pendingTaskCancelSource = null;
    }

    private bool MoveToInternal(Vector3 dest, bool fly, float range, PathRequestOrigin origin)
    {
        var externalRequest = new ExternalMoveRequest(dest, fly, range);

        if (origin == PathRequestOrigin.Normal && IsDuplicateExternalRequest(externalRequest))
        {
            Service.Log.Debug($"忽略重复 {(fly ? "飞行" : "地面")} 移动请求：目标 = {dest:f3}");
            return true;
        }

        if (pendingTask != null)
        {
            Service.Log.Warning("已有算路任务正在进行中");
            return false;
        }

        recoveryRetry = null;
        var resolvedDestinationTolerance = range                        > 0 ? range : executor.ConsumeNextTolerance();
        var toleranceStr                 = resolvedDestinationTolerance > 0 ? $"，终点容差 = {resolvedDestinationTolerance:f3}" : "";
        Service.Log.Info($"已排队 {(fly ? "飞行" : "地面")} 移动：目标 = {dest:f3}{toleranceStr}");
        pendingTaskCancelSource = new();
        pendingTask             = manager.QueryPathDetailed(Service.ObjectTable.LocalPlayer?.Position ?? default, dest, fly, resolvedDestinationTolerance, pendingTaskCancelSource.Token);
        pendingMoveRequest      = new(dest, fly, resolvedDestinationTolerance, origin);
        return true;
    }

    private MovementPlan BuildPlan(PostprocessedPath result)
        => planBuilder.Build(result);

    private bool IsDuplicateExternalRequest(ExternalMoveRequest request)
        => IsEquivalentExternalRequest(pendingMoveRequest, request) || IsEquivalentExternalRequest(activeExternalMoveRequest, request);

    private static bool IsEquivalentExternalRequest(PendingMoveRequest? existing, ExternalMoveRequest request)
    {
        if (existing is not { } current)
            return false;

        return current.Origin == PathRequestOrigin.Normal &&
               IsEquivalentExternalRequest(new ExternalMoveRequest(current.Destination, current.Fly, current.Range), request);
    }

    private static bool IsEquivalentExternalRequest(ExternalMoveRequest? existing, ExternalMoveRequest request)
    {
        if (existing is not { } current)
            return false;

        return current.Fly                                                       == request.Fly   &&
               MathF.Abs(current.Range - request.Range)                          <= float.Epsilon &&
               Vector3.DistanceSquared(current.Destination, request.Destination) <= DUPLICATE_REQUEST_DISTANCE_SQ;
    }

    private void HandleFailedRepathAfterUnstuck(PendingMoveRequest request)
    {
        if (Service.ObjectTable.LocalPlayer is not { } player)
        {
            Service.Log.Warning("[自动防卡] 重新算路失败，且当前不存在本地玩家，无法继续执行随机位移");
            return;
        }

        if (manager.Query is not { } query)
        {
            Service.Log.Warning("[自动防卡] 重新算路失败，且导航查询不可用，无法继续执行随机位移");
            return;
        }

        if (!MovementUnstuckController.TryResolveRecoveryTarget(query, player.Position, request.Fly, out var recoveryTarget))
        {
            Service.Log.Warning("[自动防卡] 重新算路失败，且未找到新的随机位移目标点");
            return;
        }

        Service.Log.Warning($"[自动防卡] 新位置算路失败，继续随机位移 1 秒后再次重算：临时目标 = {recoveryTarget:f3}");
        executor.Move([recoveryTarget], !request.Fly, goalPosition: request.Destination, tolerance: request.Range);
        recoveryRetry = new(request.Destination, request.Fly, request.Range, DateTime.Now.AddSeconds(1));
    }

    private enum PathRequestOrigin
    {
        Normal,
        RepathAfterUnstuck
    }

    private readonly record struct ExternalMoveRequest
    (
        Vector3 Destination,
        bool    Fly,
        float   Range
    );

    private readonly record struct PendingMoveRequest
    (
        Vector3           Destination,
        bool              Fly,
        float             Range,
        PathRequestOrigin Origin
    );

    private readonly record struct RecoveryRetry
    (
        Vector3  Destination,
        bool     Fly,
        float    Range,
        DateTime ExecuteAt
    );
}
