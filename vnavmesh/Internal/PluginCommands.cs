using System.Globalization;
using System.Numerics;
using Dalamud.Game.Command;
using vnavmesh.Movement;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Internal;

internal sealed class PluginCommands : IDisposable
{
    private readonly PluginConfig         config;
    private readonly NavmeshManager       navmeshManager;
    private readonly MovementPlanExecutor movementExecutor;
    private readonly AsyncMoveRequest     asyncMove;
    private readonly PluginWindows        windowProvider;
    private readonly CommandInfo          commandInfo;

    public PluginCommands
    (
        PluginConfig         config,
        NavmeshManager       navmeshManager,
        MovementPlanExecutor movementExecutor,
        AsyncMoveRequest     asyncMove,
        PluginWindows        windowProvider
    )
    {
        this.config           = config;
        this.navmeshManager   = navmeshManager;
        this.movementExecutor = movementExecutor;
        this.asyncMove        = asyncMove;
        this.windowProvider   = windowProvider;

        commandInfo = new(OnCommand)
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
            ShowInHelp = true
        };

        Service.CommandManager.AddHandler("/vnav",     commandInfo);
        Service.CommandManager.AddHandler("/vnavmesh", new(OnCommand) { HelpMessage = commandInfo.HelpMessage, ShowInHelp = false });
    }

    public void Dispose()
    {
        Service.CommandManager.RemoveHandler("/vnav");
        Service.CommandManager.RemoveHandler("/vnavmesh");
    }

    private void OnCommand(string command, string arguments)
    {
        Service.Log.Debug($"命令: '{command}'，参数: '{arguments}'");

        if (string.IsNullOrWhiteSpace(arguments))
        {
            windowProvider.IsOpen ^= true;
            return;
        }

        var args = arguments.Split(' ', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        if (args.Length == 0)
            return;

        switch (args[0])
        {
            case "reload":
                navmeshManager.Reload(true);
                break;
            case "rebuild":
                navmeshManager.Reload(false);
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
                    asyncMove.MoveTo(moveTarget.Position, false);
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
                    asyncMove.MoveTo(flyTarget.Position, true);
                break;
            case "flyflag":
                MoveFlagCommand(true);
                break;
            case "stop":
                movementExecutor.Stop();
                break;
            case "aligncamera":
                if (args.Length == 1)
                    config.AlignCameraToMovement ^= true;
                else
                    AlignCameraCommand(args[1]);
                config.Save();
                break;
            case "dtr":
                config.EnableDTR ^= true;
                config.Save();
                break;
            case "collider":
                config.ForceShowGameCollision ^= true;
                config.Save();
                break;
        }
    }

    private void MoveToCommand(string[] args, bool relativeToPlayer, bool fly)
    {
        if (args.Length < 4)
            return;

        var originActor = relativeToPlayer ? Service.ObjectTable.LocalPlayer : null;
        var origin      = originActor?.Position ?? new();
        var offset = new Vector3
        (
            float.Parse(args[1], CultureInfo.InvariantCulture),
            float.Parse(args[2], CultureInfo.InvariantCulture),
            float.Parse(args[3], CultureInfo.InvariantCulture)
        );
        asyncMove.MoveTo(origin + offset, fly);
    }

    private void MoveFlagCommand(bool fly)
    {
        if (navmeshManager.Query == null)
            return;

        var point = MapUtil.FlagToPoint(navmeshManager.Query);
        if (point == null)
            return;

        asyncMove.MoveTo(point.Value, fly);
    }

    private void AlignCameraCommand(string argument)
    {
        var normalized = argument.ToLowerInvariant();
        config.AlignCameraToMovement = normalized switch
        {
            "true" or "yes" or "enable"  => true,
            "false" or "no" or "disable" => false,
            _                            => config.AlignCameraToMovement
        };
    }
}
