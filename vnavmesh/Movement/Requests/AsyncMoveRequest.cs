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
    private readonly Config                    _config;
    private readonly NavmeshManager            _manager;
    private readonly MovementPlanExecutor      _executor;
    private readonly GroundMovementPlanBuilder _groundPlanBuilder = new();
    private readonly FlightMovementPlanBuilder _flightPlanBuilder = new();
    private          Task<PostprocessedPath>?  _pendingTask;

    public bool TaskInProgress => _pendingTask != null;

    public AsyncMoveRequest(Config config, NavmeshManager manager, MovementPlanExecutor executor)
    {
        _config   = config;
        _manager  = manager;
        _executor = executor;
        _executor.OnMovementFailure += failure =>
        {
            if (!_config.RetryOnStuck || failure.Reason != MovementFailureReason.Stuck)
                return;

            MoveTo(failure.RequestedDestination, failure.RequestedMode == MovementMode.Flight, failure.DestinationTolerance);
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
        if (_pendingTask != null && _pendingTask.IsCompleted)
        {
            Service.Log.Information("算路任务已完成");

            try
            {
                var result = _pendingTask.Result;
                if (result.Succeeded)
                    _executor.Execute(BuildPlan(result));
            }
            catch (Exception ex)
            {
                Plugin.DuoLog(ex, "算路失败");
            }

            _pendingTask.Dispose();
            _pendingTask = null;
        }
    }

    public bool MoveTo(Vector3 dest, bool fly, float range = 0)
    {
        if (_pendingTask != null)
        {
            Service.Log.Warning("已有算路任务正在进行中");
            return false;
        }

        var resolvedDestinationTolerance = range > 0 ? range : _executor.ConsumeNextTolerance();
        var toleranceStr = resolvedDestinationTolerance > 0 ? $"，终点容差 = {resolvedDestinationTolerance:f3}" : "";
        Service.Log.Info($"已排队 {(fly ? "飞行" : "地面")} 移动：目标 = {dest:f3}{toleranceStr}");
        _pendingTask = _manager.QueryPathDetailed(Service.ObjectTable.LocalPlayer?.Position ?? default, dest, fly, resolvedDestinationTolerance);
        return true;
    }

    private MovementPlan BuildPlan(PostprocessedPath result)
    {
        return result.RequestedMode == MovementMode.Flight
                   ? _flightPlanBuilder.Build(result)
                   : _groundPlanBuilder.Build(result);
    }
}
