using Dalamud.Interface.Utility.Raii;
using Dalamud.Interface.Windowing;
using vnavmesh.Infrastructure;
using vnavmesh.Interface.Debug;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;

namespace vnavmesh.Interface;

public class MainWindow : Window, IDisposable
{
    private readonly Config               _config;
    private readonly MovementPlanExecutor _movementExecutor;
    private readonly DebugDrawer          _dd = new();
    private readonly DebugGameCollision   _debugGameColl;
    private readonly DebugNavmeshManager  _debugNavmeshManager;
    private readonly DebugNavmeshCustom   _debugNavmeshCustom;
    private readonly DebugLayout          _debugLayout;

    public MainWindow
        (Config config, PluginPaths paths, NavmeshManager manager, MovementPlanExecutor movementExecutor, AsyncMoveRequest move, DTRProvider dtr) : base
        ("vnavmesh 寻路导航")
    {
        _config              = config;
        _movementExecutor    = movementExecutor;
        _debugGameColl       = new(config, _dd);
        _debugNavmeshManager = new(_dd, _debugGameColl, manager, movementExecutor, move, dtr);
        _debugNavmeshCustom  = new(config, _dd, _debugGameColl, manager, paths.ConfigDirectory.FullName);
        _debugLayout         = new(_dd, _debugGameColl);
    }

    public void Dispose()
    {
        _debugLayout.Dispose();
        _debugNavmeshCustom.Dispose();
        _debugNavmeshManager.Dispose();
        _debugGameColl.Dispose();
        _dd.Dispose();
    }

    public void StartFrame() =>
        _dd.StartFrame();

    public void EndFrame()
    {
        _debugGameColl.DrawVisualizers();

        if (_config.ShowWaypoints)
        {
            var player = Service.ObjectTable.LocalPlayer;

            if (player != null)
            {
                var from  = player.Position;
                var color = 0xff00ff00;

                foreach (var to in _movementExecutor.Waypoints)
                {
                    _dd.DrawWorldLine(from, to, color);
                    _dd.DrawWorldPointFilled(to, 3, 0xff0000ff);
                    from  = to;
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
                {
                    if (tab)
                        _config.Draw();
                }

                using (var tab = ImRaii.TabItem("布局"))
                {
                    if (tab)
                        _debugLayout.Draw();
                }

                // 临时屏蔽
                // using (var tab = ImRaii.TabItem("碰撞"))
                //     if (tab)
                //         _debugGameColl.Draw();
                using (var tab = ImRaii.TabItem("管理"))
                {
                    if (tab)
                        _debugNavmeshManager.Draw();
                }

                using (var tab = ImRaii.TabItem("自定义"))
                {
                    if (tab)
                        _debugNavmeshCustom.Draw();
                }
            }
        }
    }
}
