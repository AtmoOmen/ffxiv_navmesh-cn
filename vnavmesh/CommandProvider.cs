using System;
using System.Globalization;
using System.Numerics;
using Dalamud.Game.Command;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;
using vnavmesh.Utils;

namespace vnavmesh;

internal sealed class CommandProvider : IDisposable
{
    private readonly NavmeshManager _navmeshManager;
    private readonly MovementPlanExecutor _movementExecutor;
    private readonly AsyncMoveRequest _asyncMove;
    private readonly WindowProvider _windowProvider;
    private readonly CommandInfo _commandInfo;

    public CommandProvider(NavmeshManager navmeshManager, MovementPlanExecutor movementExecutor, AsyncMoveRequest asyncMove, WindowProvider windowProvider)
    {
        _navmeshManager = navmeshManager;
        _movementExecutor = movementExecutor;
        _asyncMove = asyncMove;
        _windowProvider = windowProvider;

        _commandInfo = new(OnCommand)
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

        Service.CommandManager.AddHandler("/vnav",     _commandInfo);
        Service.CommandManager.AddHandler("/vnavmesh", new(OnCommand) { HelpMessage = _commandInfo.HelpMessage, ShowInHelp = false });
    }

    public void Dispose()
    {
        Service.CommandManager.RemoveHandler("/vnav");
        Service.CommandManager.RemoveHandler("/vnavmesh");
    }

    private void OnCommand(string command, string arguments)
    {
        Service.Log.Debug($"cmd: '{command}', args: '{arguments}'");
        if (string.IsNullOrWhiteSpace(arguments))
        {
            _windowProvider.IsOpen ^= true;
            return;
        }

        var args = arguments.Split(' ', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        if (args.Length == 0)
            return;

        switch (args[0])
        {
            case "reload":
                _navmeshManager.Reload(true);
                break;
            case "rebuild":
                _navmeshManager.Reload(false);
                break;
            case "moveto":
                MoveToCommand(args, relativeToPlayer: false, fly: false);
                break;
            case "movedir":
                if (args.Length > 3)
                    MoveToCommand(args, relativeToPlayer: true, fly: false);
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
                MoveToCommand(args, relativeToPlayer: false, fly: true);
                break;
            case "flydir":
                if (args.Length > 3)
                    MoveToCommand(args, relativeToPlayer: true, fly: true);
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
                _movementExecutor.Stop();
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
        if (args.Length < 4)
            return;

        var originActor = relativeToPlayer ? Service.ObjectTable.LocalPlayer : null;
        var origin = originActor?.Position ?? new();
        var offset = new Vector3(
            float.Parse(args[1], CultureInfo.InvariantCulture),
            float.Parse(args[2], CultureInfo.InvariantCulture),
            float.Parse(args[3], CultureInfo.InvariantCulture));
        _asyncMove.MoveTo(origin + offset, fly);
    }

    private void MoveFlagCommand(bool fly)
    {
        if (_navmeshManager.Query == null)
            return;

        var point = MapUtil.FlagToPoint(_navmeshManager.Query);
        if (point == null)
            return;

        _asyncMove.MoveTo(point.Value, fly);
    }

    private static void AlignCameraCommand(string argument)
    {
        var normalized = argument.ToLowerInvariant();
        if (normalized is "true" or "yes" or "enable")
            Service.Config.AlignCameraToMovement = true;
        else if (normalized is "false" or "no" or "disable")
            Service.Config.AlignCameraToMovement = false;
    }
}
