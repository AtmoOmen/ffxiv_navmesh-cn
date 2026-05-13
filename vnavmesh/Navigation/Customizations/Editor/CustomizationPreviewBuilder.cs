using System;
using System.Linq;
using System.Threading;
using DotRecast.Detour;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations.Abstractions;
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

    private sealed class PreviewResult
    {
        public required SceneDefinition                  Scene { get; init; }
        public required SceneExtractor                   Extractor { get; init; }
        public Navmesh?                                 NavmeshData { get; init; }
        public NavmeshQuery?                             Query { get; init; }
        public IntermediateData?                         Intermediates { get; init; }
        public NavmeshBuilder.BuildTelemetrySummary?     BuildTelemetry { get; init; }
        public bool                                      NavmeshOwnedByManager { get; set; }
    }

    private readonly object stateLock = new();

    private PreviewResult?            publishedResult;
    private CancellationTokenSource?  cancelSource;
    private Task?                     activeTask;
    private Exception?                lastError;
    private int                       generation;
    private State                     currentState;
    private float                     buildProgress = -1f;

    public float BuildProgress => Volatile.Read(ref buildProgress);

    public State CurrentState
    {
        get
        {
            lock (stateLock)
                return currentState;
        }
    }

    public SceneDefinition? Scene
    {
        get
        {
            lock (stateLock)
                return publishedResult?.Scene;
        }
    }

    public SceneExtractor? Extractor
    {
        get
        {
            lock (stateLock)
                return publishedResult?.Extractor;
        }
    }

    public IntermediateData? Intermediates
    {
        get
        {
            lock (stateLock)
                return publishedResult?.Intermediates;
        }
    }

    public NavmeshQuery? Query
    {
        get
        {
            lock (stateLock)
                return publishedResult?.Query;
        }
    }

    public DtNavMesh? Navmesh
    {
        get
        {
            lock (stateLock)
                return publishedResult?.NavmeshData?.Mesh;
        }
    }

    public DtNavMeshQuery? MeshQuery
    {
        get
        {
            lock (stateLock)
                return publishedResult?.Query?.MeshQuery;
        }
    }

    public VoxelMap? Volume
    {
        get
        {
            lock (stateLock)
                return publishedResult?.NavmeshData?.Volume;
        }
    }

    public VoxelPathfind? VolumeQuery
    {
        get
        {
            lock (stateLock)
                return publishedResult?.Query?.VolumeQuery;
        }
    }

    public NavmeshBuilder.BuildTelemetrySummary? BuildTelemetry
    {
        get
        {
            lock (stateLock)
                return publishedResult?.BuildTelemetry;
        }
    }

    public Exception? LastError
    {
        get
        {
            lock (stateLock)
                return lastError;
        }
    }

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

        var requestGeneration = Interlocked.Increment(ref generation);
        var requestScene      = CloneScene(scene);
        var requestCancel     = new CancellationTokenSource();

        lock (stateLock)
        {
            cancelSource  = requestCancel;
            currentState  = State.InProgress;
            lastError     = null;
            Volatile.Write(ref buildProgress, 0f);
            activeTask    = Task.Run(() => BuildPreviewAsync(requestGeneration, requestScene, customization, includeTiles, requestCancel.Token));
        }

        Service.Log.Debug("[navmesh] schedule async preview build");
    }

    public void Clear()
    {
        CancellationTokenSource? oldCancel;
        PreviewResult?           oldResult;

        lock (stateLock)
        {
            Interlocked.Increment(ref generation);

            oldCancel       = cancelSource;
            cancelSource    = null;
            activeTask      = null;
            currentState    = State.NotBuilt;
            lastError       = null;
            Volatile.Write(ref buildProgress, -1f);
            oldResult       = publishedResult;
            publishedResult = null;
        }

        oldCancel?.Cancel();
        oldCancel?.Dispose();
        ReleasePreviewResult(oldResult);
    }

    private async Task BuildPreviewAsync(int requestGeneration, SceneDefinition scene, NavmeshCustomization customization, bool includeTiles, CancellationToken cancel)
    {
        try
        {
            var timer  = StopWatchTimer.Create();
            var result = await CreatePreviewResult(requestGeneration, scene, customization, includeTiles, cancel);

            lock (stateLock)
            {
                if (requestGeneration != generation || cancel.IsCancellationRequested)
                {
                    ReleasePreviewResult(result);
                    return;
                }

                if (result.NavmeshData != null)
                {
                    result.NavmeshOwnedByManager = true;
                    manager.ReplaceMesh(result.NavmeshData);
                }

                var oldResult    = publishedResult;
                publishedResult  = result;
                lastError        = null;
                currentState     = State.Ready;
                Volatile.Write(ref buildProgress, -1f);
                manager.ExternalBuildProgress = -1f;
                activeTask       = null;
                cancelSource?.Dispose();
                cancelSource     = null;
                ReleasePreviewResult(oldResult);
            }

            Service.Log.Debug($"[navmesh] preview build time: {timer.Value().TotalMilliseconds:f1}ms");
        }
        catch (OperationCanceledException)
        {
            Service.Log.Debug("[navmesh] preview build canceled");
            FinishCanceledBuild(requestGeneration);
        }
        catch (Exception ex)
        {
            Service.Log.Error($"Error building navmesh preview: {ex}");
            FinishFailedBuild(requestGeneration, ex);
        }
    }

    private async Task<PreviewResult> CreatePreviewResult
    (
        int                  requestGeneration,
        SceneDefinition      scene,
        NavmeshCustomization customization,
        bool                 includeTiles,
        CancellationToken    cancel
    )
    {
        cancel.ThrowIfCancellationRequested();

        var extractor = await Task.Run
        (
            () =>
            {
                cancel.ThrowIfCancellationRequested();
                var created = new SceneExtractor(scene);
                customization.CustomizeScene(created);
                return created;
            },
            cancel
        );

        if (!includeTiles)
        {
            return new()
            {
                Scene         = scene,
                Extractor     = extractor,
                NavmeshData   = null,
                Query         = null,
                Intermediates = null,
                BuildTelemetry = null
            };
        }

        var settings       = customization.GetBuildSettings(scene);
        var flyable        = customization.IsFlyingSupported(scene);
        var buildScene     = extractor.ToBuildScene();
        var buildSettings  = settings.ToBuildSettings(flyable, customization.Version);
        var buildSignature = vnavmesh.Common.Navigation.Mesh.Build.NavmeshBuilder.ComputeBuildSignature(buildScene, buildSettings);
        var cacheKey       = $"editor-preview-{scene.TerritoryID}-{requestGeneration:X8}-{Guid.NewGuid():N}";

        cancel.ThrowIfCancellationRequested();
        var navmesh = await manager.BuildExternalNavmesh
        (
            cacheKey,
            buildScene,
            buildSettings,
            customization.Version,
            buildSignature,
            cancel,
            progress =>
            {
                var clamped = Math.Clamp((float)progress, 0f, 0.99f);
                Volatile.Write(ref buildProgress, clamped);
                manager.ExternalBuildProgress = clamped;
            }
        );
        cancel.ThrowIfCancellationRequested();

        Volatile.Write(ref buildProgress, -1f);
        manager.ExternalBuildProgress = -1f;

        customization.CustomizeMesh(navmesh, [.. scene.FestivalLayers]);
        var query = new NavmeshQuery(navmesh, config);

        return new()
        {
            Scene          = scene,
            Extractor      = extractor,
            NavmeshData    = navmesh,
            Query          = query,
            Intermediates  = null,
            BuildTelemetry = null
        };
    }

    private void FinishCanceledBuild(int requestGeneration)
    {
        lock (stateLock)
        {
            if (requestGeneration != generation)
                return;

            activeTask    = null;
            cancelSource?.Dispose();
            cancelSource  = null;
            Volatile.Write(ref buildProgress, -1f);
            manager.ExternalBuildProgress = -1f;

            if (publishedResult == null)
                currentState = State.NotBuilt;
        }
    }

    private void FinishFailedBuild(int requestGeneration, Exception error)
    {
        lock (stateLock)
        {
            if (requestGeneration != generation)
                return;

            activeTask    = null;
            cancelSource?.Dispose();
            cancelSource  = null;
            lastError     = error;
            currentState  = State.Failed;
            Volatile.Write(ref buildProgress, -1f);
            manager.ExternalBuildProgress = -1f;
        }
    }

    private void ReleasePreviewResult(PreviewResult? result)
    {
        if (result == null)
            return;

        result.Query?.ReleaseRetainedState();
        if (!result.NavmeshOwnedByManager)
            result.NavmeshData?.ReleaseRetainedState();
    }

    private static SceneDefinition CloneScene(SceneDefinition source)
    {
        var clone = new SceneDefinition
        {
            TerritoryID               = source.TerritoryID,
            ContentsFinderConditionID = source.ContentsFinderConditionID
        };

        clone.FestivalLayers = [.. source.FestivalLayers];
        clone.ZoneSGs        = [.. source.ZoneSGs];
        clone.Terrains       = [.. source.Terrains];
        clone.AnalyticShapes = source.AnalyticShapes.ToDictionary(static kvp => kvp.Key, static kvp => kvp.Value);
        clone.MeshPaths      = source.MeshPaths.ToDictionary(static kvp => kvp.Key, static kvp => kvp.Value);
        clone.BgParts        = [.. source.BgParts];
        clone.Colliders      = [.. source.Colliders];
        clone.ExitRanges     = [.. source.ExitRanges];
        return clone;
    }
}
