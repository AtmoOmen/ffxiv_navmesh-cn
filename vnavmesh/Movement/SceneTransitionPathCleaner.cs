using vnavmesh.Build;

namespace vnavmesh.Movement;

public sealed class SceneTransitionPathCleaner : IDisposable
{
    private readonly NavmeshManager   _navmeshManager;
    private readonly AsyncMoveRequest _asyncMoveRequest;

    public SceneTransitionPathCleaner
    (
        NavmeshManager   navmeshManager,
        AsyncMoveRequest asyncMoveRequest
    )
    {
        _navmeshManager   = navmeshManager;
        _asyncMoveRequest = asyncMoveRequest;

        Service.ClientState.TerritoryChanged += OnTerritoryChanged;
    }

    public void Dispose() =>
        Service.ClientState.TerritoryChanged -= OnTerritoryChanged;

    private void OnTerritoryChanged
    (
        uint territoryType
    )
    {
        _asyncMoveRequest.Stop();
        _navmeshManager.ClearForSceneChange();
    }
}
