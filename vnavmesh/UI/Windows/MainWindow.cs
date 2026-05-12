using Dalamud.Interface.Utility.Raii;
using Dalamud.Interface.Windowing;
using vnavmesh.Bootstrap;
using vnavmesh.Bootstrap.Composition;
using vnavmesh.Configuration;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Requests;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Layout;
using vnavmesh.UI.Debug.Mesh;
using vnavmesh.UI.Editor;

namespace vnavmesh.UI.Windows;

public class MainWindow : Window, IDisposable
{
    private readonly Config                  config;
    private readonly MovementPlanExecutor    movementExecutor;
    private readonly DebugDrawer             debugDrawer = new();
    private readonly DebugGameCollision      debugGameColl;
    private readonly DebugNavmeshManager     debugNavmeshManager;
    private readonly CustomizationEditorView customizationEditor;
    private readonly DebugLayout             debugLayout;

    public MainWindow
    (
        Config               config,
        PluginPaths          paths,
        NavmeshManager       manager,
        MovementPlanExecutor movementExecutor,
        AsyncMoveRequest     move
    ) : base
        ("vnavmesh 寻路导航")
    {
        this.config           = config;
        this.movementExecutor = movementExecutor;
        debugGameColl         = new(config, debugDrawer);
        debugNavmeshManager   = new(debugDrawer, manager, movementExecutor, move);
        customizationEditor   = new(config, debugDrawer, debugGameColl, manager, paths.ConfigDirectory);
        debugLayout           = new(debugDrawer, debugGameColl);
    }

    public void Dispose()
    {
        debugLayout.Dispose();
        customizationEditor.Dispose();
        debugNavmeshManager.Dispose();
        debugGameColl.Dispose();
        debugDrawer.Dispose();
    }

    public void StartFrame() =>
        debugDrawer.StartFrame();

    public void EndFrame()
    {
        debugGameColl.DrawVisualizers();

        if (config.ShowWaypoints)
        {
            var player = Service.ObjectTable.LocalPlayer;
            if (player != null)
            {
                var from  = player.Position;
                var color = 0xff00ff00;

                foreach (var to in movementExecutor.Waypoints)
                {
                    debugDrawer.DrawWorldLine(from, to, color);
                    debugDrawer.DrawWorldPointFilled(to, 3, 0xff0000ff);
                    from  = to;
                    color = 0xff00ffff;
                }
            }
        }

        debugDrawer.EndFrame();
    }

    public override void Draw()
    {
        using var tabs = ImRaii.TabBar("Tabs");
        if (!tabs) return;

        using (var tab = ImRaii.TabItem("配置"))
        {
            if (tab)
                config.Draw();
        }

        using (var tab = ImRaii.TabItem("布局"))
        {
            if (tab)
                debugLayout.Draw();
        }

        using (var tab = ImRaii.TabItem("碰撞"))
        {
            if (tab)
                debugGameColl.Draw();
        }

        using (var tab = ImRaii.TabItem("管理"))
        {
            if (tab)
                debugNavmeshManager.Draw();
        }

        using (var tab = ImRaii.TabItem("自定义编辑器"))
        {
            if (tab)
                customizationEditor.Draw();
        }
    }
}
