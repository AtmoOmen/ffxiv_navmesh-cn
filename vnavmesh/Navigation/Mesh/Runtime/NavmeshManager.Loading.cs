using Dalamud.Game.ClientState.Conditions;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Scene;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

public sealed partial class NavmeshManager
{
    public bool Reload(bool allowLoadFromCache)
    {
        ClearState();

        if (CurrentKey.Length > 0)
        {
            var cts = _currentCTS = new();
            ExecuteWhenIdle
            (
                async cancel =>
                {
                    _loadTaskProgress = 0;

                    using var resetLoadProgress = new OnDispose(() => _loadTaskProgress = -1);

                    var waitStart = DateTime.Now;

                    while (InCutscene)
                    {
                        if ((DateTime.Now - waitStart).TotalSeconds >= 5)
                        {
                            waitStart = DateTime.Now;
                            Log("waiting for cutscene");
                        }

                        await Service.Framework.DelayTicks(1, cancel);
                    }

                    var snapshotTimer = StopWatchTimer.Create();
                    var (cacheKey, scene) = await Service.Framework.Run
                                            (
                                                () =>
                                                {
                                                    var scene = new SceneDefinition();
                                                    scene.FillFromActiveLayout();
                                                    var cacheKey = GetCacheKey(scene);
                                                    return (cacheKey, scene);
                                                },
                                                cancel
                                            );
                    Log($"场景快照耗时 {snapshotTimer.Value().TotalMilliseconds:f1} ms");

                    Log($"Kicking off build for '{cacheKey}' (reload={allowLoadFromCache})");
                    var buildResult = await Task.Run(() => BuildNavmesh(scene, cacheKey, allowLoadFromCache, cancel), cancel);
                    var navmesh     = buildResult.Navmesh;
                    Log($"Mesh loaded: '{cacheKey}'");
                    Navmesh = navmesh;
                    Query   = new(Navmesh, _config);

                    var ff = await FloodFill.GetAsync();
                    if (ff.TryLookup(scene.TerritoryID, out var points))
                        Prune(points);

                    OnNavmeshChanged?.Invoke(Navmesh, Query);
                    if (buildResult.CacheFile != null)
                        QueueCacheWrite(cacheKey, buildResult.CacheFile, navmesh);
                },
                cts.Token
            );
        }

        return true;
    }

    internal void ReplaceMesh(Navmesh mesh)
    {
        var retiredNavmesh = Navmesh;
        var retiredQuery   = Query;
        Navmesh = mesh;
        Query   = new(Navmesh, _config);
        Log("Mesh replaced");
        OnNavmeshChanged?.Invoke(Navmesh, Query);
        ReleaseRetiredState(retiredNavmesh, retiredQuery, "网格替换");
    }

    private static bool InCutscene => Service.Condition[ConditionFlag.WatchingCutscene] || Service.Condition[ConditionFlag.OccupiedInCutSceneEvent];

    private void ClearState()
    {
        if (_currentCTS == null)
            return;

        var cts = _currentCTS;
        _currentCTS = null;
        cts.Cancel();
        var retiredNavmesh = Navmesh;
        var retiredQuery   = Query;
        Log("Queueing state clear");
        ExecuteWhenIdle
        (
            () =>
            {
                Log("Clearing state");
                _numActivePathfinds = 0;
                cts.Dispose();
                OnNavmeshChanged?.Invoke(null, null);
                Query   = null;
                Navmesh = null;
                ReleaseRetiredState(retiredNavmesh, retiredQuery, "场景切换卸载");
            },
            default
        );
    }

    private BuildNavmeshResult BuildNavmesh(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel)
    {
        var totalTimer = StopWatchTimer.Create();
        Log($"Build task started: '{cacheKey}'");
        var customization = NavmeshCustomizationRegistry.ForTerritory(scene.TerritoryID);
        Log($"Customization for '{scene.TerritoryID}': {customization.GetType()}");

        var layers         = scene.FestivalLayers.ToList();
        var buildSignature = NavmeshBuilder.ComputeBuildSignature(scene, customization);
        var cache          = new FileInfo($"{_cacheDir.FullName}/{cacheKey}.navmesh");

        if (allowLoadFromCache && TryLoadFromCache(cache, customization, buildSignature, layers, totalTimer, out var cachedResult))
            return cachedResult;

        cancel.ThrowIfCancellationRequested();

        var buildTimer         = StopWatchTimer.Create();
        var builder            = new NavmeshBuilder(scene, customization, _config);
        var totalProgressWeight = Math.Max(builder.TotalEstimatedTileWeight, 1);
        builder.Build
        (weight =>
            {
                _loadTaskProgress += 0.99f * weight / totalProgressWeight;
                cancel.ThrowIfCancellationRequested();
            }
        );
        Log($"冷构建耗时 {buildTimer.Value().TotalMilliseconds:f1} ms");

        customization.CustomizeMesh(builder.Navmesh, layers);
        var runtimeMesh = builder.Navmesh with { CustomizationApplied = true };
        if (runtimeMesh.Volume != null)
        {
            var compactTimer = StopWatchTimer.Create();
            runtimeMesh.Volume.CompactRetainedState();
            Log($"飞行体素常驻压缩耗时 {compactTimer.Value().TotalMilliseconds:f1} ms");
        }

        Log($"总构建耗时 {totalTimer.Value().TotalMilliseconds:f1} ms");
        _loadTaskProgress += 0.01f;
        return new(runtimeMesh, cache);
    }

    private bool TryLoadFromCache
    (
        FileInfo               cache,
        NavmeshCustomization   customization,
        string                 buildSignature,
        List<uint>             layers,
        StopWatchTimer         totalTimer,
        out BuildNavmeshResult result
    )
    {
        if (!cache.Exists)
        {
            result = new(default!, null);
            return false;
        }

        try
        {
            var cacheReadTimer = StopWatchTimer.Create();
            Log($"Loading cache: {cache.FullName}");
            using var stream      = new FileStream(cache.FullName, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 22, FileOptions.SequentialScan);
            using var reader      = new BinaryReader(stream);
            var       cacheResult = Navmesh.Deserialize(reader, customization.Version, buildSignature);
            var       mesh        = cacheResult.Navmesh;
            Log($"缓存读取耗时 {cacheReadTimer.Value().TotalMilliseconds:f1} ms");
            LogCacheSegment("读取", cacheResult.Telemetry.Mesh);
            LogCacheSegment("读取", cacheResult.Telemetry.Volume);
            if (!mesh.CustomizationApplied)
                customization.CustomizeMesh(mesh, layers);
            Log($"缓存命中，总耗时 {totalTimer.Value().TotalMilliseconds:f1} ms");
            result = new(mesh, cacheResult.RequiresRewrite ? cache : null);
            return true;
        }
        catch (Exception ex)
        {
            Log($"Failed to load cache: {ex}");
            result = new(default!, null);
            return false;
        }
    }

}
