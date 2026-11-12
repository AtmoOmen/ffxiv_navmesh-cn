using Dalamud.Game.Gui.Dtr;
using vnavmesh.Build;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;

namespace vnavmesh.Internal;

public class PluginDTR
(
    PluginConfig         config,
    NavmeshManager       manager,
    AsyncMoveRequest     asyncMove,
    MovementPlanExecutor movementExecutor
)
    : IDisposable
{
    private readonly IDtrBarEntry dtrBarEntry = Service.DtrBar.Get("vnavmesh");

    public void Dispose()
    {
        dtrBarEntry.Remove();
        GC.SuppressFinalize(this);
    }

    public void Update()
    {
        dtrBarEntry.Shown = config.EnableDTR;

        if (dtrBarEntry.Shown)
        {
            var text         = new Localization(config.UILanguage);
            var loadProgress = manager.LoadTaskProgress;
            var buildProgress = loadProgress >= 0 ?
                                    loadProgress :
                                    manager.ExternalBuildProgress;

            string meshStatus;
            if (buildProgress >= 0)
                meshStatus = $"{buildProgress * 100:f0}%";
            else
                meshStatus = manager.Navmesh != null ?
                                 text.Ready :
                                 text.NotReady;

            var statusText = $"{text.Navigation}: {meshStatus}";

            var pathfindInProgress = manager.PathfindInProgress;
            var numQueued          = manager.NumQueuedPathfindRequests;
            var asyncMoveActive    = asyncMove.TaskInProgress;
            var isMoving           = movementExecutor.Waypoints.Count > 0;

            if (pathfindInProgress || numQueued > 0)
            {
                var activeCount = pathfindInProgress ?
                                      1 :
                                      0;
                statusText += $" | {text.Queries}: {activeCount}";
                if (numQueued > 0)
                    statusText += $" ({text.Queued}: {numQueued})";
            }

            if (asyncMoveActive)
                statusText += $" | {text.Pathfinding}";
            if (isMoving)
                statusText += $" | {text.Moving}";

            dtrBarEntry.Text = statusText;
        }
    }
}
