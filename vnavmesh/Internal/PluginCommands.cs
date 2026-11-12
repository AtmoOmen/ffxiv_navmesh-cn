using System.Globalization;
using System.Numerics;
using Dalamud.Game.Command;
using Dalamud.Plugin.Services;
using Lumina.Data;
using vnavmesh.Build;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Query.Utils;

namespace vnavmesh.Internal;

internal sealed class PluginCommands : IDisposable
{
    private readonly PluginConfig         config;
    private readonly NavmeshManager       navmeshManager;
    private readonly MovementPlanExecutor movementExecutor;
    private readonly AsyncMoveRequest     asyncMove;
    private readonly PluginWindows        windowProvider;
    private readonly CommandInfo          commandInfo;
    private readonly CommandInfo          aliasCommandInfo;

    private Language helpLanguage;

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

        helpLanguage = config.UILanguage;
        commandInfo = new(OnCommand)
        {
            HelpMessage = new Localization(helpLanguage).CommandHelp,
            ShowInHelp  = true
        };
        aliasCommandInfo = new(OnCommand)
        {
            HelpMessage = commandInfo.HelpMessage,
            ShowInHelp  = false
        };

        Service.CommandManager.AddHandler("/vnav",     commandInfo);
        Service.CommandManager.AddHandler("/vnavmesh", aliasCommandInfo);
        Service.Framework.Update += OnFrameworkUpdate;
    }

    public void Dispose()
    {
        Service.Framework.Update -= OnFrameworkUpdate;
        Service.CommandManager.RemoveHandler("/vnav");
        Service.CommandManager.RemoveHandler("/vnavmesh");
    }

    private void OnFrameworkUpdate(IFramework framework)
    {
        if (helpLanguage == config.UILanguage)
            return;

        helpLanguage                 = config.UILanguage;
        commandInfo.HelpMessage      = new Localization(helpLanguage).CommandHelp;
        aliasCommandInfo.HelpMessage = commandInfo.HelpMessage;
    }

    private void OnCommand
    (
        string command,
        string arguments
    )
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
        }
    }

    private void MoveToCommand
    (
        string[] args,
        bool     relativeToPlayer,
        bool     fly
    )
    {
        if (args.Length < 4)
            return;

        var originActor = relativeToPlayer ?
                              Service.ObjectTable.LocalPlayer :
                              null;
        var origin = originActor?.Position ?? new();
        var offset = new Vector3
        (
            float.Parse(args[1], CultureInfo.InvariantCulture),
            float.Parse(args[2], CultureInfo.InvariantCulture),
            float.Parse(args[3], CultureInfo.InvariantCulture)
        );
        asyncMove.MoveTo(origin + offset, fly);
    }

    private void MoveFlagCommand
    (
        bool fly
    )
    {
        if (navmeshManager.Query == null)
            return;

        var point = MapUtil.FlagToPoint(navmeshManager.Query);
        if (point == null)
            return;

        asyncMove.MoveTo(point.Value, fly);
    }

    private void AlignCameraCommand
    (
        string argument
    )
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
