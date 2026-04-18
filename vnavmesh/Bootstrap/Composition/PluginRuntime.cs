using Dalamud.Plugin.Services;
using vnavmesh.Integration.Commands;
using vnavmesh.Integration.Ipc;
using vnavmesh.Integration.Status;
using vnavmesh.Integration.Windowing;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Requests;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Bootstrap.Composition;

internal sealed class PluginRuntime : IDisposable
{
    private readonly NavmeshManager       _navmeshManager;
    private readonly MovementPlanExecutor _movementExecutor;
    private readonly AsyncMoveRequest     _asyncMoveRequest;
    private readonly DTRProvider          _dtrProvider;

    public PluginRuntime
    (
        NavmeshManager       navmeshManager,
        MovementPlanExecutor movementExecutor,
        AsyncMoveRequest     asyncMoveRequest,
        DTRProvider          dtrProvider,
        SceneTransitionPathCleaner ____,
        WindowProvider       _,
        CommandProvider      __,
        IPCProvider          ___
    )
    {
        _navmeshManager   = navmeshManager;
        _movementExecutor = movementExecutor;
        _asyncMoveRequest = asyncMoveRequest;
        _dtrProvider      = dtrProvider;

        Service.Framework.Update += OnUpdate;
    }

    public void Dispose() =>
        Service.Framework.Update -= OnUpdate;

    private void OnUpdate(IFramework framework)
    {
        _navmeshManager.Update();
        _movementExecutor.Update(framework);
        _asyncMoveRequest.Update();
        _dtrProvider.Update();
    }
}
