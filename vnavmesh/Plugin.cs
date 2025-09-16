using Dalamud.Common;
using Dalamud.Game.Command;
using Dalamud.Interface.Windowing;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using Navmesh.Movement;
using System;
using System.IO;
using System.Numerics;
using System.Reflection;

namespace Navmesh;

public sealed class Plugin : IDalamudPlugin
{
    private WindowSystem WindowSystem = new("vnavmesh");
    private NavmeshManager _navmeshManager;
    private FollowPath _followPath;
    private AsyncMoveRequest _asyncMove;
    private DTRProvider _dtrProvider;
    private MainWindow _wndMain;
    private IPCProvider _ipcProvider;

    public Plugin(IDalamudPluginInterface dalamud)
    {
        if (!dalamud.ConfigDirectory.Exists)
            dalamud.ConfigDirectory.Create();
        var dalamudRoot = dalamud.GetType().Assembly.
                GetType("Dalamud.Service`1", true)!.MakeGenericType(dalamud.GetType().Assembly.GetType("Dalamud.Dalamud", true)!).
                GetMethod("Get")!.Invoke(null, BindingFlags.Default, null, Array.Empty<object>(), null);
        var dalamudStartInfo = (DalamudStartInfo)dalamudRoot?.GetType().GetProperty("StartInfo", BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance)?.GetValue(dalamudRoot)!;
        InteropGenerator.Runtime.Resolver.GetInstance.Setup(0, dalamudStartInfo.GameVersion?.ToString() ?? "", new(Path.Combine(dalamud.ConfigDirectory.FullName, "cs.json")));
        FFXIVClientStructs.Interop.Generated.Addresses.Register();
        InteropGenerator.Runtime.Resolver.GetInstance.Resolve();

        dalamud.Create<Service>();
        Service.Config.Load(dalamud.ConfigFile);
        Service.Config.Modified += () => Service.Config.Save(dalamud.ConfigFile);

        _navmeshManager = new(new($"{dalamud.ConfigDirectory.FullName}/meshcache"));
        _followPath = new(dalamud, _navmeshManager);
        _asyncMove = new(_navmeshManager, _followPath);
        _dtrProvider = new(_navmeshManager, _asyncMove, _followPath);
        _wndMain = new(_navmeshManager, _followPath, _asyncMove, _dtrProvider, dalamud.ConfigDirectory.FullName) { IsOpen = dalamud.IsDev };
        _ipcProvider = new(_navmeshManager, _followPath, _asyncMove, _wndMain, _dtrProvider);

        WindowSystem.AddWindow(_wndMain);
        //_wndMain.IsOpen = true;

        dalamud.UiBuilder.Draw += Draw;
        dalamud.UiBuilder.OpenConfigUi += () => _wndMain.IsOpen = true;

        var cmd = new CommandInfo(OnCommand)
        {
            HelpMessage = """
            打开调试菜单
            /vnav moveto <X> <Y> <Z> → 移动到原始坐标
            /vnav movedir <X> <Y> <Z> → 按玩家面向移动指定单位
            /vnav movetarget → 移动到目标位置
            /vnav moveflag → 移动到标记位置
            /vnav flyto <X> <Y> <Z> → 飞行到原始坐标
            /vnav flydir <X> <Y> <Z> → 按玩家面向飞行指定单位
            /vnav flytarget → 飞行到目标位置
            /vnav flyflag → 飞行到标记位置
            /vnav stop → 停止所有移动
            /vnav reload → 从缓存重新加载当前区域导航网格
            /vnav rebuild → 从头重建当前区域导航网格
            /vnav aligncamera → 切换相机跟随移动方向
            /vnav aligncamera true|yes|enable → 启用相机跟随移动方向
            /vnav aligncamera false|no|disable → 禁用相机跟随移动方向
            /vnav dtr → 切换服务器信息栏状态
            /vnav collider → 切换碰撞调试可视化
            """,

            ShowInHelp = true,
        };
        Service.CommandManager.AddHandler("/vnav", cmd);
        Service.CommandManager.AddHandler("/vnavmesh", new CommandInfo(OnCommand) { HelpMessage = cmd.HelpMessage, ShowInHelp = false }); // legacy

        Service.Framework.Update += OnUpdate;
    }

    public void Dispose()
    {
        Service.Framework.Update -= OnUpdate;

        Service.CommandManager.RemoveHandler("/vnav");
        Service.CommandManager.RemoveHandler("/vnavmesh");
        Service.PluginInterface.UiBuilder.Draw -= Draw;
        WindowSystem.RemoveAllWindows();

        _ipcProvider.Dispose();
        _wndMain.Dispose();
        _dtrProvider.Dispose();
        _asyncMove.Dispose();
        _followPath.Dispose();
        _navmeshManager.Dispose();
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
        _navmeshManager.Update();
        _followPath.Update(fwk);
        _asyncMove.Update();
        _dtrProvider.Update();
    }

    private void Draw()
    {
        _wndMain.StartFrame();
        WindowSystem.Draw();
        _wndMain.EndFrame();
    }

    private void OnCommand(string command, string arguments)
    {
        Service.Log.Debug($"cmd: '{command}', args: '{arguments}'");
        if (arguments.Length == 0)
        {
            _wndMain.IsOpen ^= true;
            return;
        }

        var args = arguments.Split(' ');
        switch (args[0])
        {
            case "reload":
                _navmeshManager.Reload(true);
                break;
            case "rebuild":
                _navmeshManager.Reload(false);
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
                    _asyncMove.MoveTo(moveTarget.Position, false);
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
                    _asyncMove.MoveTo(flyTarget.Position, true);
                break;
            case "flyflag":
                MoveFlagCommand(true);
                break;
            case "stop":
                _followPath.Stop();
                //_navmeshManager.CancelAllQueries();
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
            float.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture),
            float.Parse(args[2], System.Globalization.CultureInfo.InvariantCulture),
            float.Parse(args[3], System.Globalization.CultureInfo.InvariantCulture));
        _asyncMove.MoveTo(origin + offset, fly);
    }

    private void MoveFlagCommand(bool fly)
    {
        if (_navmeshManager.Query == null)
            return;
        var pt = MapUtils.FlagToPoint(_navmeshManager.Query);
        if (pt == null)
            return;
        _asyncMove.MoveTo(pt.Value, fly);
    }

    private static void AlignCameraCommand(string arg)
    {
        arg = arg.ToLower();
        if (arg is "true" or "yes" or "enable")
            Service.Config.AlignCameraToMovement = true;
        else if (arg is "false" or "no" or "disable")
            Service.Config.AlignCameraToMovement = false;
        return;
    }
}
