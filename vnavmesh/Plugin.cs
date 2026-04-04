using System;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;
using vnavmesh.Interface;

namespace vnavmesh;

public sealed class Plugin : IDalamudPlugin
{
    private readonly NavmeshManager _navmeshManager;
    private readonly MovementPlanExecutor _movementExecutor;
    private readonly AsyncMoveRequest _asyncMove;
    private readonly DTRProvider _dtrProvider;
    private readonly MainWindow _mainWindow;
    private readonly WindowProvider _windowProvider;
    private readonly CommandProvider _commandProvider;
    private readonly IPCProvider _ipcProvider;

    public Plugin(IDalamudPluginInterface dalamud)
    {
        if (!dalamud.ConfigDirectory.Exists)
            dalamud.ConfigDirectory.Create();

        dalamud.Create<Service>();

        Service.Config          =  dalamud.GetPluginConfig() as Config ?? new();
        Service.Config.Modified += () => dalamud.SavePluginConfig(Service.Config);

        _navmeshManager = new(new($"{dalamud.ConfigDirectory.FullName}/meshcache"));
        _movementExecutor = new(dalamud, _navmeshManager);
        _asyncMove = new(_navmeshManager, _movementExecutor);
        _dtrProvider = new(_navmeshManager, _asyncMove, _movementExecutor);
        _mainWindow = new(_navmeshManager, _movementExecutor, _asyncMove, _dtrProvider, dalamud.ConfigDirectory.FullName);
        _windowProvider = new(dalamud, _mainWindow);
        _commandProvider = new(_navmeshManager, _movementExecutor, _asyncMove, _windowProvider);
        _ipcProvider = new(_navmeshManager, _movementExecutor, _asyncMove, _windowProvider, _dtrProvider);

        Service.Framework.Update += OnUpdate;
    }

    public void Dispose()
    {
        Service.Framework.Update -= OnUpdate;

        _ipcProvider.Dispose();
        _commandProvider.Dispose();
        _windowProvider.Dispose();
        _mainWindow.Dispose();
        _dtrProvider.Dispose();
        _asyncMove.Dispose();
        _movementExecutor.Dispose();
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
        _movementExecutor.Update(fwk);
        _asyncMove.Update();
        _dtrProvider.Update();
    }
}
