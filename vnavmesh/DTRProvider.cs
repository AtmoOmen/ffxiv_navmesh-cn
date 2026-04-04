using Dalamud.Game.Gui.Dtr;
using Dalamud.Plugin.Services;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;

namespace vnavmesh;

public class DTRProvider
(
    Config               config,
    NavmeshManager       manager,
    AsyncMoveRequest     asyncMove,
    MovementPlanExecutor movementExecutor
)
    : IDisposable
{
    private readonly IDtrBarEntry dtrBarEntry = Service.DtrBar.Get("vnavmesh");

    public void Dispose() =>
        dtrBarEntry.Remove();

    public void Update()
    {
        dtrBarEntry.Shown = config.EnableDTR;

        if (dtrBarEntry.Shown)
        {
            var loadProgress = manager.LoadTaskProgress;
            var meshStatus   = loadProgress >= 0 ? $"{loadProgress * 100:f0}%" : manager.Navmesh != null ? "就绪" : "未就绪";

            var statusText = "导航: " + meshStatus;

            if (config.ShowQueryStatusInDTR)
            {
                var pathfindInProgress = manager.PathfindInProgress;
                var numQueued          = manager.NumQueuedPathfindRequests;
                var asyncMoveActive    = asyncMove.TaskInProgress;
                var isMoving           = movementExecutor.Waypoints.Count > 0;

                if (pathfindInProgress || numQueued > 0)
                {
                    var activeCount = pathfindInProgress ? 1 : 0;
                    statusText += $" | 算路: {activeCount}";
                    if (numQueued > 0)
                        statusText += $" (等待中: {numQueued})";
                }

                if (asyncMoveActive)
                    statusText += " | 算路中";
                if (isMoving)
                    statusText += " | 移动中";
            }
            else
            {
                if (asyncMove.TaskInProgress || movementExecutor.Waypoints.Count > 0)
                    statusText = "导航: 算路中";
            }

            dtrBarEntry.Text = statusText;
        }
    }
}
