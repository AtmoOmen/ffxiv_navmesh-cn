using Dalamud.Game.Gui.Dtr;
using Navmesh.Movement;
using System;

namespace Navmesh;

public class DTRProvider(NavmeshManager Manager, AsyncMoveRequest AsyncMove, FollowPath FollowPath) : IDisposable
{
    private IDtrBarEntry DtrBarEntry { get; } = Service.DtrBar.Get("vnavmesh");

    public void Dispose() => 
        DtrBarEntry.Remove();

    public void Update()
    {
        DtrBarEntry.Shown = Service.Config.EnableDTR;
        if (!DtrBarEntry.Shown) return;
        
        var loadProgress = Manager.LoadTaskProgress;
        var status       = loadProgress >= 0 ? $"构建进度 {loadProgress * 100:f0}%" : Manager.Navmesh != null ? "就绪" : "未就绪";
        
        if (FollowPath.Waypoints.Count > 0)
        {
            status = $"寻路中 (剩余点位: {FollowPath.Waypoints.Count})";
        }
        else if (AsyncMove.TaskInProgress)
        {
            var pathfindProgress = Manager.PathfindProgress;
            if (pathfindProgress >= 0)
            {
                // 显示路径计算进度百分比
                status = $"路径计算 {pathfindProgress * 100:f0}%";
            }
            else
            {
                // 对于无法获取进度的路径计算（如网格路径），显示队列信息
                var queuedRequests = Manager.NumQueuedPathfindRequests;
                status = queuedRequests > 0 ? $"路径计算中 (队列: {queuedRequests})" : "路径计算中";
            }
        }
        
        DtrBarEntry.Text = "导航: " + status;
    }
}
