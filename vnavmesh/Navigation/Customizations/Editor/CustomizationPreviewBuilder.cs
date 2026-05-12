using System.Numerics;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.Navigation.Volume.Pathfinding;

namespace vnavmesh.Navigation.Customizations.Editor;

internal class CustomizationPreviewBuilder
(
    NavmeshManager manager,
    Config         config
) : IDisposable
{
    public enum State
    {
        NotBuilt,
        InProgress,
        Failed,
        Ready
    }

    public class IntermediateData
    {
        public int                 NumTilesX;
        public int                 NumTilesZ;
        public RcBuilderResult?[,] Tiles;

        public IntermediateData(int numTilesX, int numTilesZ)
        {
            NumTilesX = numTilesX;
            NumTilesZ = numTilesZ;
            Tiles     = new RcBuilderResult?[numTilesX, numTilesZ];
        }
    }

    private SceneDefinition?  _scene;
    private NavmeshBuilder?   _builder;
    private NavmeshQuery?     _query;
    private IntermediateData? _intermediates;
    private Task?             _task;
    private readonly NavmeshManager _manager = manager;
    private readonly Config         _config  = config;

    public State CurrentState => _task == null ? State.NotBuilt : !_task.IsCompleted ? State.InProgress : _task.IsFaulted ? State.Failed : State.Ready;
    public SceneDefinition? Scene => _task != null && _task.IsCompletedSuccessfully ? _scene : null;
    public SceneExtractor? Extractor => _task != null && _task.IsCompletedSuccessfully ? _builder?.Scene : null;
    public IntermediateData? Intermediates => _task != null && _task.IsCompletedSuccessfully ? _intermediates : null;
    public NavmeshQuery? Query => _task != null && _task.IsCompletedSuccessfully ? _query : null;
    public DotRecast.Detour.DtNavMesh? Navmesh => _task != null && _task.IsCompletedSuccessfully ? _builder?.Navmesh.Mesh : null;
    public DotRecast.Detour.DtNavMeshQuery? MeshQuery => _task != null && _task.IsCompletedSuccessfully ? _query?.MeshQuery : null;
    public VoxelMap? Volume => _task != null && _task.IsCompletedSuccessfully ? _builder?.Navmesh.Volume : null;
    public VoxelPathfind? VolumeQuery => _task != null && _task.IsCompletedSuccessfully ? _query?.VolumeQuery : null;
    public NavmeshBuilder.BuildTelemetrySummary? BuildTelemetry => _task != null && _task.IsCompletedSuccessfully ? _builder?.LastBuildTelemetry : null;
    public Exception? LastError => _task?.Exception?.GetBaseException();

    public void Dispose() =>
        Clear();

    public void RebuildFromActiveLayout(NavmeshCustomization customization, bool includeTiles)
    {
        var scene = new SceneDefinition();
        scene.FillFromActiveLayout();
        Rebuild(scene, customization, includeTiles);
    }

    public void Rebuild(SceneDefinition scene, NavmeshCustomization customization, bool includeTiles)
    {
        Clear();
        _scene = scene;
        Service.Log.Debug("[navmesh] schedule async build");
        _task = Task.Run(() => BuildNavmesh(scene, customization, includeTiles));
    }

    public void Clear()
    {
        if (_task != null)
        {
            if (!_task.IsCompleted)
                _task.Wait();
            _task.Dispose();
            _task = null;
        }

        _scene         = null;
        _builder       = null;
        _query         = null;
        _intermediates = null;
    }

    private void BuildNavmesh(SceneDefinition scene, NavmeshCustomization customization, bool includeTiles)
    {
        try
        {
            var timer = StopWatchTimer.Create();
            _builder = new(scene, customization, _config);

            _intermediates = new(_builder.NumTilesX, _builder.NumTilesZ);

            if (includeTiles)
            {
                foreach (var result in _builder.BuildTiles())
                    _intermediates.Tiles[result.TileX, result.TileZ] = result;

                Service.Log.Debug("running customization code");
                customization.CustomizeMesh(_builder.Navmesh, [.. scene.FestivalLayers]);
            }

            _query = new(_builder.Navmesh, _config);
            Service.Log.Debug($"navmesh build time: {timer.Value().TotalMilliseconds}ms");
            _manager.ReplaceMesh(_builder.Navmesh);
        }
        catch (Exception ex)
        {
            Service.Log.Error($"Error building navmesh: {ex}");
            throw;
        }
    }
}
