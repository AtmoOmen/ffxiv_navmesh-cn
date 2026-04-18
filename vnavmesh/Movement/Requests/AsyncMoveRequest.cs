using System.Numerics;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement.Requests;

public class AsyncMoveRequest : IDisposable
{
    private readonly NavmeshManager            _manager;
    private readonly MovementPlanExecutor      _executor;
    private readonly GroundMovementPlanBuilder _groundPlanBuilder = new();
    private readonly FlightMovementPlanBuilder _flightPlanBuilder = new();
    private          Task<PostprocessedPath>?  _pendingTask;
    private          PendingMoveRequest?       _pendingMoveRequest;
    private          RecoveryRetry?            _recoveryRetry;

    public bool TaskInProgress => _pendingTask != null;

    public AsyncMoveRequest(Config config, NavmeshManager manager, MovementPlanExecutor executor)
    {
        _manager  = manager;
        _executor = executor;
        _executor.OnMovementFailure += failure =>
        {
            if (failure.Reason != MovementFailureReason.RepathRequiredAfterUnstuck)
                return;

            Service.Log.Information($"[自动防卡] 随机位移结束，开始重新算路：目标 = {failure.RequestedDestination:f3}");
            MoveToInternal(failure.RequestedDestination, failure.RequestedMode == MovementMode.Flight, failure.DestinationTolerance, PathRequestOrigin.RepathAfterUnstuck);
        };
    }

    public void Dispose()
    {
        if (_pendingTask != null)
        {
            if (!_pendingTask.IsCompleted)
                _pendingTask.Wait();
            _pendingTask.Dispose();
            _pendingTask = null;
        }
    }

    public void Update()
    {
        if (_recoveryRetry is { } scheduled && _pendingTask == null && DateTime.Now >= scheduled.ExecuteAt)
        {
            _recoveryRetry = null;
            _executor.Stop();
            Service.Log.Information($"[自动防卡] 随机位移完成，重新算路：目标 = {scheduled.Destination:f3}");
            MoveToInternal(scheduled.Destination, scheduled.Fly, scheduled.Range, PathRequestOrigin.RepathAfterUnstuck);
        }

        if (_pendingTask != null && _pendingTask.IsCompleted)
        {
            Service.Log.Information("算路任务已完成");

            try
            {
                var result = _pendingTask.Result;
                var request = _pendingMoveRequest;
                if (result.Succeeded)
                {
                    _executor.Execute(BuildPlan(result));
                    _recoveryRetry = null;
                }
                else if (request is { Origin: PathRequestOrigin.RepathAfterUnstuck })
                {
                    HandleFailedRepathAfterUnstuck(request.Value);
                }
            }
            catch (Exception ex)
            {
                Plugin.DuoLog(ex, "算路失败");
            }

            _pendingTask.Dispose();
            _pendingTask = null;
            _pendingMoveRequest = null;
        }
    }

    public bool MoveTo(Vector3 dest, bool fly, float range = 0)
        => MoveToInternal(dest, fly, range, PathRequestOrigin.Normal);

    public void Stop()
    {
        _recoveryRetry = null;
        _pendingMoveRequest = null;
        _executor.Stop();

        if (_pendingTask is { IsCompleted: true })
        {
            _pendingTask.Dispose();
            _pendingTask = null;
        }
    }

    private bool MoveToInternal(Vector3 dest, bool fly, float range, PathRequestOrigin origin)
    {
        if (_pendingTask != null)
        {
            Service.Log.Warning("已有算路任务正在进行中");
            return false;
        }

        _recoveryRetry = null;
        var resolvedDestinationTolerance = range > 0 ? range : _executor.ConsumeNextTolerance();
        var toleranceStr = resolvedDestinationTolerance > 0 ? $"，终点容差 = {resolvedDestinationTolerance:f3}" : "";
        Service.Log.Info($"已排队 {(fly ? "飞行" : "地面")} 移动：目标 = {dest:f3}{toleranceStr}");
        _pendingTask = _manager.QueryPathDetailed(Service.ObjectTable.LocalPlayer?.Position ?? default, dest, fly, resolvedDestinationTolerance);
        _pendingMoveRequest = new(dest, fly, resolvedDestinationTolerance, origin);
        return true;
    }

    private MovementPlan BuildPlan(PostprocessedPath result)
    {
        return result.RequestedMode == MovementMode.Flight
                   ? _flightPlanBuilder.Build(result)
                   : _groundPlanBuilder.Build(result);
    }

    private void HandleFailedRepathAfterUnstuck(PendingMoveRequest request)
    {
        if (Service.ObjectTable.LocalPlayer is not { } player)
        {
            Service.Log.Warning("[自动防卡] 重新算路失败，且当前不存在本地玩家，无法继续执行随机位移");
            return;
        }

        if (_manager.Query is not { } query)
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
        _executor.Move([recoveryTarget], !request.Fly, goalPosition: request.Destination, tolerance: request.Range);
        _recoveryRetry = new(request.Destination, request.Fly, request.Range, DateTime.Now.AddSeconds(1));
    }

    private enum PathRequestOrigin
    {
        Normal,
        RepathAfterUnstuck
    }

    private readonly record struct PendingMoveRequest(Vector3 Destination, bool Fly, float Range, PathRequestOrigin Origin);

    private readonly record struct RecoveryRetry(Vector3 Destination, bool Fly, float Range, DateTime ExecuteAt);
}
