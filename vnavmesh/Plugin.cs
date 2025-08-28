using Dalamud.Game.Command;
using Dalamud.Interface.Windowing;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using Navmesh.Movement;
using System;
using System.Globalization;
using System.Numerics;
using Navmesh.Utilities;

namespace Navmesh;

public sealed class Plugin : IDalamudPlugin
{
    private readonly WindowSystem WindowSystem = new("vnavmesh");
    private readonly DTRProvider  DtrProvider;
    private readonly IPCProvider  IPCProvider;
    
    internal readonly NavmeshManager   NavmeshManager;
    internal readonly FollowPath       FollowPath;
    internal readonly AsyncMoveRequest AsyncMove;
    internal readonly MainWindow       MainWindow;

    public static Plugin Instance() => instance;
    
    private static Plugin instance = null!;

    public Plugin(IDalamudPluginInterface dalamud)
    {
        instance = this;
        
        dalamud.Create<Service>();
        
        Service.Config.Load(dalamud.ConfigFile);
        Service.Config.Modified += () => Service.Config.Save(dalamud.ConfigFile);

        NavmeshManager = new(new($"{dalamud.ConfigDirectory.FullName}/meshcache"));
        FollowPath     = new(dalamud, NavmeshManager);
        AsyncMove      = new(NavmeshManager, FollowPath);
        DtrProvider    = new(NavmeshManager, AsyncMove, FollowPath);
        MainWindow     = new(NavmeshManager, FollowPath, AsyncMove, dalamud.ConfigDirectory.FullName) { IsOpen = dalamud.IsDev };
        IPCProvider    = new(NavmeshManager, FollowPath, AsyncMove, MainWindow, DtrProvider);

        WindowSystem.AddWindow(MainWindow);

        dalamud.UiBuilder.Draw += Draw;
        dalamud.UiBuilder.OpenConfigUi += () => MainWindow.IsOpen = true;

        var cmd = new CommandInfo(OnCommand)
        {
            HelpMessage = """
            Opens the debug menu.
            /vnav moveto <X> <Y> <Z> → 移动至该坐标
            /vnav movedir <X> <Y> <Z> → 根据当前面向移动指定单位距离
            /vnav movetarget → 移动至当前目标位置
            /vnav moveflag → 移动至当前标点位置
            /vnav flyto <X> <Y> <Z> → 飞行至该坐标
            /vnav flydir <X> <Y> <Z> → 根据当前面向移动指定单位距离
            /vnav flytarget → 飞行至当前目标位置
            /vnav flyflag → 飞行至当前标点位置
            /vnav stop → 停止所有导航移动任务
            /vnav reload → 从缓存中重新加载本区域导航数据
            /vnav rebuild → 从游戏中重新构建本区域导航数据
            /vnav recalculate → 重新计算当前路径（卡住时使用）
            /vnav aligncamera → 将当前面向对齐移动方向
            /vnav aligncamera true|yes|enable → 启用移动时将当前面向对齐移动方向
            /vnav aligncamera false|no|disable → 禁用移动时将当前面向对齐移动方向
            /vnav dtr → 开关服务器状态栏插件状态显示
            /vnav collider → 开关调试性碰撞显示
            """,
            
            ShowInHelp = true,
        };
        
        Service.CommandManager.AddHandler("/vnav", cmd);

        Service.Framework.Update += OnUpdate;
    }

    public void Dispose()
    {
        Service.Framework.Update -= OnUpdate;

        Service.CommandManager.RemoveHandler("/vnav");
        
        Service.PluginInterface.UiBuilder.Draw -= Draw;
        WindowSystem.RemoveAllWindows();

        IPCProvider.Dispose();
        MainWindow.Dispose();
        DtrProvider.Dispose();
        AsyncMove.Dispose();
        FollowPath.Dispose();
        NavmeshManager.Dispose();
    }

    public static void DuoLog(Exception ex)
    {
        DuoLog(ex, ex.Message);
        throw ex;
    }

    public static void DuoLog(Exception ex, string message)
    {
        Service.ChatGui.Print($"[{Service.PluginInterface.Manifest.Name}] {message}");
        Service.Log.Error(ex, message);
    }

    private void OnUpdate(IFramework fwk)
    {
        NavmeshManager.Update();
        FollowPath.Update(fwk);
        AsyncMove.Update();
        DtrProvider.Update();
    }

    private void Draw()
    {
        MainWindow.StartFrame();
        WindowSystem.Draw();
        MainWindow.EndFrame();
    }

    private void OnCommand(string command, string arguments)
    {
        if (arguments.Length == 0)
        {
            MainWindow.IsOpen ^= true;
            return;
        }

        var args = arguments.Split(' ');
        switch (args[0])
        {
            case "reload":
                NavmeshManager.Reload(true);
                break;
            case "rebuild":
                NavmeshManager.Reload(false);
                break;
            case "moveto":
                MoveToCommand(args, false, false);
                break;
            case "movedir":
                if (args.Length > 3)
                    MoveToCommand(args, true, false);
                break;
            case "movetarget":
                var moveTarget = Service.TargetManager.Target;
                if (moveTarget != null)
                    AsyncMove.MoveTo(moveTarget.Position, false);
                break;
            case "moveflag":
                MoveFlagCommand(false);
                break;
            case "flyto":
                MoveToCommand(args, false, true);
                break;
            case "flydir":
                if (args.Length > 3)
                    MoveToCommand(args, true, true);
                break;
            case "flytarget":
                var flyTarget = Service.TargetManager.Target;
                if (flyTarget != null)
                    AsyncMove.MoveTo(flyTarget.Position, true);
                break;
            case "flyflag":
                MoveFlagCommand(true);
                break;
            case "stop":
                AsyncMove.CancelPathfinding();
                FollowPath.Stop();
                break;
            case "aligncamera":
                if (args.Length == 1)
                    Service.Config.AlignCameraToMovement ^= true;
                else
                    AlignCameraCommand(args[1]);
                Service.Config.NotifyModified();
                break;
            case "dtr":
                Service.Config.EnableDTR ^= true;
                Service.Config.NotifyModified();
                break;
            case "collider":
                Service.Config.ForceShowGameCollision ^= true;
                Service.Config.NotifyModified();
                break;
        }
    }

    private void MoveToCommand(string[] args, bool relativeToPlayer, bool fly)
    {
        var originActor = relativeToPlayer ? Service.ClientState.LocalPlayer : null;
        var origin = originActor?.Position ?? new();
        var offset = new Vector3(
            float.Parse(args[1], CultureInfo.InvariantCulture),
            float.Parse(args[2], CultureInfo.InvariantCulture),
            float.Parse(args[3], CultureInfo.InvariantCulture));
        AsyncMove.MoveTo(origin + offset, fly);
    }

    private void MoveFlagCommand(bool fly)
    {
        if (NavmeshManager.Query == null) return;
        
        var pt = MapUtils.FlagToPoint(NavmeshManager.Query);
        if (pt == null) return;
        
        AsyncMove.MoveTo(pt.Value, fly);
    }

    private static void AlignCameraCommand(string arg)
    {
        arg = arg.ToLower();
        
        Service.Config.AlignCameraToMovement = arg switch
        {
            "true" or "yes" or "enable"  => true,
            "false" or "no" or "disable" => false,
            _                            => Service.Config.AlignCameraToMovement
        };
    }
}
