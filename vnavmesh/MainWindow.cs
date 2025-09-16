using Dalamud.Interface.Utility.Raii;
using Dalamud.Interface.Windowing;
using Navmesh.Debug;
using Navmesh.Movement;
using System;

namespace Navmesh;

public class MainWindow : Window, IDisposable
{
    private FollowPath _path;
    private DebugDrawer _dd = new();
    private DebugGameCollision _debugGameColl;
    private DebugNavmeshManager _debugNavmeshManager;
    private DebugNavmeshCustom _debugNavmeshCustom;
    private DebugLayout _debugLayout;
    private string _configDirectory;

    public MainWindow(NavmeshManager manager, FollowPath path, AsyncMoveRequest move, DTRProvider dtr, string configDir) : base("vnavmesh 寻路导航")
    {
        _path = path;
        _configDirectory = configDir;
        _debugGameColl = new(_dd);
        _debugNavmeshManager = new(_dd, _debugGameColl, manager, path, move, dtr);
        _debugNavmeshCustom = new(_dd, _debugGameColl, manager, _configDirectory);
        _debugLayout = new(_debugGameColl);
    }

    public void Dispose()
    {
        _debugLayout.Dispose();
        _debugNavmeshCustom.Dispose();
        _debugNavmeshManager.Dispose();
        _debugGameColl.Dispose();
        _dd.Dispose();
    }

    public void StartFrame()
    {
        _dd.StartFrame();
    }

    public void EndFrame()
    {
        _debugGameColl.DrawVisualizers();
        if (Service.Config.ShowWaypoints)
        {
            var player = Service.ClientState.LocalPlayer;
            if (player != null)
            {
                var from = player.Position;
                var color = 0xff00ff00;
                foreach (var to in _path.Waypoints)
                {
                    _dd.DrawWorldLine(from, to, color);
                    _dd.DrawWorldPointFilled(to, 3, 0xff0000ff);
                    from = to;
                    color = 0xff00ffff;
                }
            }
        }
        _dd.EndFrame();
    }

    public override void Draw()
    {
        using (var tabs = ImRaii.TabBar("Tabs"))
        {
            if (tabs)
            {
                using (var tab = ImRaii.TabItem("配置"))
                    if (tab)
                        Service.Config.Draw();
                using (var tab = ImRaii.TabItem("布局"))
                    if (tab)
                        _debugLayout.Draw();
                using (var tab = ImRaii.TabItem("碰撞"))
                    if (tab)
                        _debugGameColl.Draw();
                using (var tab = ImRaii.TabItem("管理"))
                    if (tab)
                        _debugNavmeshManager.Draw();
                using (var tab = ImRaii.TabItem("自定义"))
                    if (tab)
                        _debugNavmeshCustom.Draw();
            }
        }
    }
}
