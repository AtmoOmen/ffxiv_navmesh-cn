using Dalamud.Plugin.Services;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navigation;

namespace vnavmesh.Internal;

internal sealed class PluginRuntime : IDisposable
{
    private readonly NavmeshManager       navmeshManager;
    private readonly MovementPlanExecutor movementExecutor;
    private readonly AsyncMoveRequest     asyncMoveRequest;
    private readonly PluginDTR            dtrProvider;

    public PluginRuntime
    (
        NavmeshManager       navmeshManager,
        MovementPlanExecutor movementExecutor,
        AsyncMoveRequest     asyncMoveRequest,
        PluginDTR            dtrProvider
    )
    {
        this.navmeshManager   = navmeshManager;
        this.movementExecutor = movementExecutor;
        this.asyncMoveRequest = asyncMoveRequest;
        this.dtrProvider      = dtrProvider;

        Service.Framework.Update += OnUpdate;
    }

    public void Dispose() =>
        Service.Framework.Update -= OnUpdate;

    private void OnUpdate(IFramework framework)
    {
        navmeshManager.Update();
        movementExecutor.Update(framework);
        asyncMoveRequest.Update();
        dtrProvider.Update();
    }
}
