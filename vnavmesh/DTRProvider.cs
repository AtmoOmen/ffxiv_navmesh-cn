using System;
using Dalamud.Game.Gui.Dtr;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;

namespace vnavmesh;

public class DTRProvider : IDisposable
{
    private NavmeshManager _manager;
    private AsyncMoveRequest _asyncMove;
    private MovementPlanExecutor _movementExecutor;
    private IDtrBarEntry _dtrBarEntry;

    public DTRProvider(NavmeshManager manager, AsyncMoveRequest asyncMove, MovementPlanExecutor movementExecutor)
    {
        _manager = manager;
        _asyncMove = asyncMove;
        _movementExecutor = movementExecutor;
        _dtrBarEntry = Service.DtrBar.Get("vnavmesh");
    }

    public void Dispose()
    {
        _dtrBarEntry.Remove();
    }

    public void Update()
    {
        _dtrBarEntry.Shown = Service.Config.EnableDTR;
        if (_dtrBarEntry.Shown)
        {
            var loadProgress = _manager.LoadTaskProgress;
            var meshStatus = loadProgress >= 0 ? $"{loadProgress * 100:f0}%" : _manager.Navmesh != null ? "就绪" : "未就绪";
            
            var statusText = "导航: " + meshStatus;
            
            if (Service.Config.ShowQueryStatusInDTR)
            {
                var pathfindInProgress = _manager.PathfindInProgress;
                var numQueued = _manager.NumQueuedPathfindRequests;
                var asyncMoveActive = _asyncMove.TaskInProgress;
                var isMoving = _movementExecutor.Waypoints.Count > 0;
                
                // Show query status when there's activity
                if (pathfindInProgress || numQueued > 0)
                {
                    var activeCount = pathfindInProgress ? 1 : 0;
                    statusText += $" | 算路: {activeCount}";
                    if (numQueued > 0)
                        statusText += $" (等待中: {numQueued})";
                }
                
                // Show current operations
                if (asyncMoveActive)
                    statusText += " | 算路中";
                if (isMoving)
                    statusText += " | 移动中";
            }
            else
            {
                // Fallback to original simple status for backward compatibility
                if (_asyncMove.TaskInProgress || _movementExecutor.Waypoints.Count > 0)
                    statusText = "导航: 算路中";
            }
            
            _dtrBarEntry.Text = statusText;
        }
    }
}
