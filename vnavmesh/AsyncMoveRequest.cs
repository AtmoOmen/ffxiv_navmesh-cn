using Navmesh.Movement;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

namespace Navmesh;

public class AsyncMoveRequest : IDisposable
{
    private NavmeshManager _manager;
    private FollowPath _follow;
    private Task<PathfindResult>? _pendingTask;
    private bool _pendingFly;
    private float _pendingDestRange;

    public bool TaskInProgress => _pendingTask != null;

    public AsyncMoveRequest(NavmeshManager manager, FollowPath follow)
    {
        _manager = manager;
        _follow = follow;

        _follow.OnStuck += (dest, fly, range) =>
        {
            if (!Service.Config.RetryOnStuck)
                return;

            MoveTo(dest, fly, range);
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
                    _follow.Move(result.Waypoints, !_pendingFly, _pendingDestRange, result.RequestedDestination);
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
        _pendingTask = _manager.QueryPathDetailed(Service.ObjectTable.LocalPlayer?.Position ?? default, dest, fly, range: range);
        _pendingFly = fly;
        _pendingDestRange = range;
        return true;
    }
}
