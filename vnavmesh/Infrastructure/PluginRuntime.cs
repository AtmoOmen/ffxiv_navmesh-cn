using Dalamud.Plugin.Services;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;

namespace vnavmesh.Infrastructure;

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
