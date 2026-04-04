using System;
using System.Numerics;
using System.Threading.Tasks;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;
using vnavmesh.Navmesh;
using vnavmesh.NavPathfind;

namespace vnavmesh.Movement;

public class AsyncMoveRequest : IDisposable
{
    private readonly NavmeshManager _manager;
    private readonly MovementPlanExecutor _executor;
    private readonly GroundMovementPlanBuilder _groundPlanBuilder = new();
    private readonly FlightMovementPlanBuilder _flightPlanBuilder = new();
    private Task<PathfindResult>? _pendingTask;
    private bool _pendingFly;
    private float _pendingDestRange;
    private float _pendingPathTolerance;

    public bool TaskInProgress => _pendingTask != null;

    public AsyncMoveRequest(NavmeshManager manager, MovementPlanExecutor executor)
    {
        _manager = manager;
        _executor = executor;
        _executor.OnMovementFailure += failure =>
        {
            if (!Service.Config.RetryOnStuck || failure.Reason != MovementFailureReason.Stuck)
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

        var toleranceStr = range > 0 ? $"，容差 = {range:f3}" : "";
        Service.Log.Info($"已排队 {(fly ? "飞行" : "地面")} 移动：目标 = {dest:f3}{toleranceStr}");
        _pendingPathTolerance = _executor.ConsumeNextTolerance();
        _pendingTask = _manager.QueryPathDetailed(Service.ObjectTable.LocalPlayer?.Position ?? default, dest, fly, range: range);
        _pendingFly = fly;
        _pendingDestRange = range;
        return true;
    }

    private MovementPlan BuildPlan(PathfindResult result)
    {
        return _pendingFly
            ? _flightPlanBuilder.Build(result, _pendingDestRange, _pendingPathTolerance)
            : _groundPlanBuilder.Build(result, _pendingDestRange, _pendingPathTolerance);
    }
}
