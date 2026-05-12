using DotRecast.Detour;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
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
    (
        int numTilesX,
        int numTilesZ
    )
    {
        public int                 NumTilesX = numTilesX;
        public int                 NumTilesZ = numTilesZ;
        public RcBuilderResult?[,] Tiles     = new RcBuilderResult?[numTilesX, numTilesZ];
    }

    private SceneDefinition?  sceneDefinition;
    private NavmeshBuilder?   builder;
    private NavmeshQuery?     query;
    private IntermediateData? intermediates;
    private Task?             task;

    public State CurrentState => task == null ? State.NotBuilt : !task.IsCompleted ? State.InProgress : task.IsFaulted ? State.Failed : State.Ready;
    public SceneDefinition? Scene => task is { IsCompletedSuccessfully: true } ? sceneDefinition : null;
    public SceneExtractor? Extractor => task is { IsCompletedSuccessfully: true } ? builder?.Scene : null;
    public IntermediateData? Intermediates => task is { IsCompletedSuccessfully: true } ? intermediates : null;
    public NavmeshQuery? Query => task is { IsCompletedSuccessfully: true } ? query : null;
    public DtNavMesh? Navmesh => task is { IsCompletedSuccessfully: true } ? builder?.Navmesh.Mesh : null;
    public DtNavMeshQuery? MeshQuery => task is { IsCompletedSuccessfully: true } ? query?.MeshQuery : null;
    public VoxelMap? Volume => task is { IsCompletedSuccessfully: true } ? builder?.Navmesh.Volume : null;
    public VoxelPathfind? VolumeQuery => task is { IsCompletedSuccessfully: true } ? query?.VolumeQuery : null;
    public NavmeshBuilder.BuildTelemetrySummary? BuildTelemetry => task is { IsCompletedSuccessfully: true } ? builder?.LastBuildTelemetry : null;
    public Exception? LastError => task?.Exception?.GetBaseException();

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
        sceneDefinition = scene;
        Service.Log.Debug("[navmesh] schedule async build");
        task = Task.Run(() => BuildNavmesh(scene, customization, includeTiles));
    }

    public void Clear()
    {
        if (task != null)
        {
            if (!task.IsCompleted)
                task.Wait();
            task.Dispose();
            task = null;
        }

        sceneDefinition = null;
        builder         = null;
        query           = null;
        intermediates   = null;
    }

    private void BuildNavmesh(SceneDefinition scene, NavmeshCustomization customization, bool includeTiles)
    {
        try
        {
            var timer = StopWatchTimer.Create();
            builder = new(scene, customization, config);

            intermediates = new(builder.NumTilesX, builder.NumTilesZ);

            if (includeTiles)
            {
                foreach (var result in builder.BuildTiles())
                    intermediates.Tiles[result.TileX, result.TileZ] = result;

                Service.Log.Debug("running customization code");
                customization.CustomizeMesh(builder.Navmesh, [.. scene.FestivalLayers]);
            }

            query = new(builder.Navmesh, config);
            Service.Log.Debug($"navmesh build time: {timer.Value().TotalMilliseconds}ms");
            manager.ReplaceMesh(builder.Navmesh);
        }
        catch (Exception ex)
        {
            Service.Log.Error($"Error building navmesh: {ex}");
            throw;
        }
    }
}
