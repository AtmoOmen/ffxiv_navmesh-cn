using System.Collections.Concurrent;
using System.Globalization;
using System.Numerics;
using System.Runtime;
using Dalamud.Game.ClientState.Conditions;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Scene;
using vnavmesh.Shared.Utilities;
using Action = System.Action;

namespace vnavmesh.Navigation.Mesh.Runtime;

// manager that loads navmesh matching current zone and performs async pathfinding queries
public sealed class NavmeshManager : IDisposable
{
    private readonly        Config               _config;
    private static readonly DtQueryDefaultFilter s_pruneFilter                  = new();
    private const           float                PruneSeedHalfExtentXZ          = 8.0f;
    private const           float                PruneSeedHalfExtentY           = 16.0f;
    private const           float                PruneSeedMaxHorizontalDistance = 8.0f;
    private const           float                PruneSeedMaxVerticalDistance   = 12.0f;

    private sealed record BuildNavmeshResult
    (
        Navmesh   Navmesh,
        FileInfo? CacheFile
    );

    public bool UseRaycasts      = true;
    public bool UseStringPulling = true;

    public string                                 CurrentKey { get; private set; } = ""; // unique string representing currently loaded navmesh
    public Navmesh?                               Navmesh    { get; private set; }
    public NavmeshQuery?                          Query      { get; private set; }
    public event Action<Navmesh?, NavmeshQuery?>? OnNavmeshChanged;

    private volatile float _loadTaskProgress = -1;
    public           float LoadTaskProgress => _loadTaskProgress; // negative if load task is not running, otherwise in [0, 1] range

    private CancellationTokenSource? _currentCTS; // this is signalled when mesh is unloaded, all pathfinding tasks that use it are then cancelled

    private Task
        _lastLoadQueryTask; // we limit the concurrency to max 1 running task (otherwise we'd need multiple Query objects, which aren't lightweight); note that each task completes on main thread!

    private int  _numActivePathfinds;
    public  bool PathfindInProgress        => _numActivePathfinds > 0;
    public  int  NumQueuedPathfindRequests => _numActivePathfinds > 0 ? _numActivePathfinds - 1 : 0;

    private readonly DirectoryInfo                       _cacheDir;
    private readonly ConcurrentDictionary<string, Task>  _cacheWriteTasks          = new();
    private readonly ConcurrentDictionary<Navmesh, Task> _cacheWriteTasksByNavmesh = new(ReferenceEqualityComparer.Instance);

    public NavmeshManager(DirectoryInfo cacheDir, Config config)
    {
        _config   = config;
        _cacheDir = cacheDir;
        cacheDir.Create(); // ensure directory exists

        // prepare a task with correct task scheduler that other tasks can be chained off
        _lastLoadQueryTask = Service.Framework.Run(() => Log("Tasks kicked off"));
    }

    public void Dispose()
    {
        Log("Disposing");
        ClearState();
    }

    public void Update()
    {
        var curKey = GetCurrentKey();

        if (curKey != CurrentKey)
        {
            // navmesh needs to be reloaded
            if (!_config.AutoLoadNavmesh)
            {
                if (CurrentKey.Length == 0)
                    return;  // nothing is loaded, and auto-load is forbidden
                curKey = ""; // just unload existing mesh
            }

            Log($"Starting transition from '{CurrentKey}' to '{curKey}'");
            CurrentKey = curKey;
            Reload(true);
            // mesh load is now in progress
        }
    }

    public async Task<List<Vector3>> QueryPath(Vector3 from, Vector3 to, bool flying, float range = 0, CancellationToken externalCancel = default)
    {
        var result = await QueryPathDetailed(from, to, flying, range, externalCancel);
        return result.Waypoints;
    }

    internal Task<PostprocessedPath> QueryPathDetailed
    (
        Vector3           from,
        Vector3           to,
        bool              flying,
        float             range          = 0,
        CancellationToken externalCancel = default
    )
    {
        if (_currentCTS == null)
            throw new Exception("Can't initiate query - navmesh is not loaded");

        // task can be cancelled either by internal request (i.e. when navmesh is reloaded) or external
        var combined = CancellationTokenSource.CreateLinkedTokenSource(_currentCTS.Token, externalCancel);
        ++_numActivePathfinds;
        return ExecuteWhenIdle
        (
            async cancel =>
            {
                using var autoDisposeCombined  = combined;
                using var autoDecrementCounter = new OnDispose(() => --_numActivePathfinds);
                LogInfo($"Kicking off pathfind from {from} to {to}");
                var result = await Task.Run
                             (
                                 () =>
                                 {
                                     combined.Token.ThrowIfCancellationRequested();
                                     if (Query == null)
                                         throw new Exception("Can't pathfind, navmesh did not build successfully");
                                     Log($"执行算路：起点 = {from:f3}，终点 = {to:f3}");
                                     var plannerResult = flying
                                                             ? Query.PlanVolumePathDetailed(from, to, UseRaycasts, combined.Token)
                                                             : Query.PlanMeshPathDetailed(from, to, UseRaycasts, range, combined.Token);
                                     return Query.Postprocess(plannerResult, UseStringPulling, combined.Token);
                                 },
                                 combined.Token
                             );
                Log($"算路结束：状态 = {result.Status}，路径点 = {result.Waypoints.Count}");
                return result;
            },
            combined.Token
        );
    }

    // note: pixelSize should be power-of-2
    public (Vector3 min, Vector3 max) BuildBitmap(Vector3 startingPos, string filename, float pixelSize, AABB? mapBounds = null)
    {
        if (Navmesh == null || Query == null)
            throw new InvalidOperationException("Can't build bitmap - navmesh creation is in progress");

        bool inBounds(Vector3 vert)
        {
            return mapBounds is not AABB aabb ||
                   vert.X >= aabb.Min.X && vert.Y >= aabb.Min.Y && vert.Z >= aabb.Min.Z && vert.X <= aabb.Max.X && vert.Y <= aabb.Max.Y && vert.Z <= aabb.Max.Z;
        }

        var startPoly      = Query.FindNearestMeshPoly(startingPos);
        var reachablePolys = Query.FindReachableMeshPolys(startPoly);

        HashSet<long> polysInbounds = [];

        Vector3 min = new(1024), max = new(-1024);

        foreach (var p in reachablePolys)
        {
            Navmesh.Mesh.GetTileAndPolyByRefUnsafe(p, out var tile, out var poly);

            for (var i = 0; i < poly.vertCount; ++i)
            {
                var v = NavmeshBitmap.GetVertex(tile, poly.verts[i]);
                if (!inBounds(v))
                    goto cont;

                min = Vector3.Min(min, v);
                max = Vector3.Max(max, v);
                //Service.Log.Debug($"{p:X}.{i}= {v}");
            }

            polysInbounds.Add(p);

            cont: ;
        }
        //Service.Log.Debug($"bounds: {min}-{max}");

        var bitmap = new NavmeshBitmap(min, max, pixelSize);
        foreach (var p in polysInbounds) bitmap.RasterizePolygon(Navmesh.Mesh, p);
        bitmap.Save(filename);
        Service.Log.Debug($"Generated nav bitmap '{filename}' @ {startingPos}: {bitmap.MinBounds}-{bitmap.MaxBounds}");
        return (bitmap.MinBounds, bitmap.MaxBounds);
    }

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
                    {
                        var pruneSeeds = points.ToArray();
                        Navmesh.DeferMeshMutation(mesh => PruneMesh(mesh, pruneSeeds));
                    }

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

        var buildTimer          = StopWatchTimer.Create();
        var builder             = new NavmeshBuilder(scene, customization, _config);
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

    private unsafe string GetCurrentKey()
    {
        var layout = LayoutWorld.Instance()->ActiveLayout;
        if (layout == null || layout->InitState != 7 || layout->FestivalStatus is > 0 and < 5)
            return "";

        var filter    = LayoutUtil.FindFilter(layout);
        var filterKey = filter != null ? filter->Key : 0;
        var terrRow   = Service.LuminaRow<TerritoryType>(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

        if (terrRow?.TerritoryIntendedUse.RowId == 60)
        {
            var fest = layout->ActiveFestivals[0];
            if (fest.Id == 0 && fest.Phase == 0)
                return "";
        }

        var sgs = LayoutUtil.GetZoneSharedGroupsEnabled(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);
        return $"{terrRow?.Bg}//{filterKey:X}//{LayoutUtil.FestivalsString(layout->ActiveFestivals)}//{string.Join('.', sgs)}";
    }

    internal static unsafe string GetCacheKey(SceneDefinition scene)
    {
        var layout    = LayoutWorld.Instance()->ActiveLayout;
        var filter    = LayoutUtil.FindFilter(layout);
        var filterKey = filter != null ? filter->Key : 0;
        var terrId    = filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId;
        var terrRow   = Service.LuminaRow<TerritoryType>(terrId);
        return $"{terrRow?.Bg.ToString().Replace('/', '_')}__{filterKey:X}__{FormatHexNumbers(scene.FestivalLayers)}__{FormatHexNumbers(scene.ZoneSGs)}";
    }

    private void QueueCacheWrite(string cacheKey, FileInfo cache, Navmesh navmesh)
    {
        if (_cacheWriteTasks.TryGetValue(cacheKey, out var existing) && !existing.IsCompleted)
        {
            Log($"后台缓存写入已在进行，跳过重复调度: {cacheKey}");
            return;
        }

        var writeTask = Task.Run(() => WriteCache(cacheKey, cache, navmesh));
        _cacheWriteTasks[cacheKey]         = writeTask;
        _cacheWriteTasksByNavmesh[navmesh] = writeTask;
        _ = writeTask.ContinueWith
        (
            t =>
            {
                _cacheWriteTasks.TryRemove(cacheKey, out _);
                _cacheWriteTasksByNavmesh.TryRemove(navmesh, out _);
                LogTaskError(t);
            },
            CancellationToken.None,
            TaskContinuationOptions.ExecuteSynchronously,
            TaskScheduler.Default
        );
    }

    private void WriteCache(string cacheKey, FileInfo cache, Navmesh navmesh)
    {
        var timer    = StopWatchTimer.Create();
        var tempPath = $"{cache.FullName}.{Environment.ProcessId}.{Environment.CurrentManagedThreadId}.tmp";

        try
        {
            cache.Directory?.Create();
            Service.Log.Debug($"[vnavmesh] 后台写入缓存: {cache.FullName}");
            var                    serializeTimer = StopWatchTimer.Create();
            Navmesh.CacheTelemetry telemetry;

            using (var stream = new FileStream(tempPath, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 22, FileOptions.SequentialScan))
            using (var writer = new BinaryWriter(stream))
            {
                telemetry = navmesh.Serialize(writer);
                writer.Flush();
                stream.Flush();
            }

            var serializeDuration = serializeTimer.Value();
            var sizeBytes         = new FileInfo(tempPath).Length;

            var replaceTimer = StopWatchTimer.Create();
            File.Move(tempPath, cache.FullName, true);
            var replaceDuration = replaceTimer.Value();
            LogCacheSegment("写入", telemetry.Mesh);
            LogCacheSegment("写入", telemetry.Volume);
            Log
            (
                $"后台缓存写入完成 '{cacheKey}'，总耗时 {timer.Value().TotalMilliseconds:f1} ms，序列化 {serializeDuration.TotalMilliseconds:f1} ms，替换 {replaceDuration.TotalMilliseconds:f1} ms，大小 {sizeBytes / 1024.0 / 1024.0:f2} MiB"
            );
        }
        catch (Exception ex)
        {
            Log($"后台缓存写入失败 '{cacheKey}': {ex}");
        }
        finally
        {
            try
            {
                if (File.Exists(tempPath))
                    File.Delete(tempPath);
            }
            catch
            {
            }
        }
    }

    private void ExecuteWhenIdle(Action task, CancellationToken token)
    {
        var prev = _lastLoadQueryTask;
        _lastLoadQueryTask = Service.Framework.Run
        (
            async () =>
            {
                await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
                _ = prev.Exception;
                task();
            },
            token
        );
    }

    private void ExecuteWhenIdle(Func<CancellationToken, Task> task, CancellationToken token)
    {
        var prev = _lastLoadQueryTask;
        _lastLoadQueryTask = Service.Framework.Run
        (
            async () =>
            {
                await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
                _ = prev.Exception;
                var t = task(token);
                await t.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
                LogTaskError(t);
            },
            token
        );
    }

    private Task<T> ExecuteWhenIdle<T>(Func<CancellationToken, Task<T>> task, CancellationToken token)
    {
        var prev = _lastLoadQueryTask;
        var res = Service.Framework.Run
        (
            async () =>
            {
                await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
                _ = prev.Exception;
                var t = task(token);
                await ((Task)t).ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
                LogTaskError(t);
                return t.Result;
            },
            token
        );
        _lastLoadQueryTask = res;
        return res;
    }

    private static void Log(string message) => Service.Log.Debug($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");

    private static void LogInfo(string message) => Service.Log.Info($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");

    private static void LogCacheSegment(string action, Navmesh.CacheSegmentTelemetry segment) =>
        Log
        (
            $"缓存段 {SegmentName(segment.Kind)} {action}耗时 {segment.Duration.TotalMilliseconds:f1} ms，压缩后 {FormatMiB(segment.CompressedBytes):f2} MiB，原始 {FormatMiB(segment.UncompressedBytes):f2} MiB"
        );

    private static string SegmentName(Navmesh.CacheSegmentKind kind) => kind switch
    {
        Navmesh.CacheSegmentKind.Mesh   => "Mesh",
        Navmesh.CacheSegmentKind.Volume => "Volume",
        _                               => kind.ToString()
    };

    private static double FormatMiB(long bytes) => bytes / 1024.0 / 1024.0;

    private static void LogTaskError(Task task)
    {
        if (task.IsFaulted)
            Service.Log.Error($"[NavmeshManager] Task failed with error: {task.Exception}");
    }

    private void ReleaseRetiredState(Navmesh? navmesh, NavmeshQuery? query, string reason)
    {
        query?.ReleaseRetainedState();
        if (navmesh == null)
            return;

        if (_cacheWriteTasksByNavmesh.TryGetValue(navmesh, out var pendingWrite) && !pendingWrite.IsCompleted)
        {
            Log($"旧场景资源延后释放，等待缓存写入完成: {reason}");
            _ = pendingWrite.ContinueWith
            (
                _ => FinalizeRetiredNavmeshRelease(navmesh, reason),
                CancellationToken.None,
                TaskContinuationOptions.ExecuteSynchronously,
                TaskScheduler.Default
            );
            return;
        }

        FinalizeRetiredNavmeshRelease(navmesh, reason);
    }

    private void FinalizeRetiredNavmeshRelease(Navmesh navmesh, string reason)
    {
        navmesh.ReleaseRetainedState();
        Log($"旧场景重资源已释放: {reason}");

        if (navmesh.Volume != null)
            RequestMemoryCompaction(reason);
    }

    private void RequestMemoryCompaction(string reason)
    {
        _ = Task.Run
        (() =>
            {
                try
                {
                    GCSettings.LargeObjectHeapCompactionMode = GCLargeObjectHeapCompactionMode.CompactOnce;
                    GC.Collect(GC.MaxGeneration, GCCollectionMode.Optimized, true, true);
                    Log($"已请求大对象堆压缩: {reason}");
                }
                catch (Exception ex)
                {
                    Log($"请求内存压缩失败: {ex}");
                }
            }
        );
    }

    public void Prune(IEnumerable<Vector3> points)
    {
        if (Navmesh == null)
            throw new InvalidOperationException("无法裁剪，网格缺失");

        PruneMesh(Navmesh.Mesh, points);
    }

    private static void PruneMesh(DtNavMesh mesh, IEnumerable<Vector3> points)
    {
        var query      = new DtNavMeshQuery(mesh);
        var startPolys = CollectPruneSeedPolys(query, points).ToArray();
        Log($"裁剪起始多边形: {string.Join(", ", startPolys.Select(p => p.ToString("X")))}");
        var reachablePolys = FindReachableMeshPolys(query, startPolys);

        var pruneCount = 0;

        for (var i = 0; i < mesh.GetMaxTiles(); i++)
        {
            var t = mesh.GetTile(i);
            if (t.data?.header == null)
                continue;

            var prBase = mesh.GetPolyRefBase(t);

            for (var j = 0; j < t.data.header.polyCount; j++)
            {
                var pref = prBase | (uint)j;

                if (mesh.GetPolyFlags(pref, out var fl).Failed())
                {
                    Log($"读取多边形标记失败: {pref:X}");
                    continue;
                }

                if (reachablePolys.Contains(pref))
                {
                    if (mesh.SetPolyFlags(pref, fl & ~(int)NavmeshPolyFlags.Unreachable).Failed())
                        Log($"写入多边形标记失败: {pref:X}");
                }
                else
                {
                    pruneCount++;
                    if (mesh.SetPolyFlags(pref, fl | (int)NavmeshPolyFlags.Unreachable).Failed())
                        Log($"写入多边形标记失败: {pref:X}");
                }
            }
        }

        Log($"已裁剪不可达多边形 {pruneCount} 个");
    }

    private static IEnumerable<long> CollectPruneSeedPolys(DtNavMeshQuery query, IEnumerable<Vector3> points)
    {
        HashSet<long> result = [];

        foreach (var point in points)
        {
            foreach (var poly in FindPruneSeedPolys(query, point))
                result.Add(poly);
        }

        return result;
    }

    private static IEnumerable<long> FindPruneSeedPolys(DtNavMeshQuery query, Vector3 point)
    {
        HashSet<long> result = [];

        foreach (var poly in FindIntersectingMeshPolys(query, point, new(PruneSeedHalfExtentXZ, PruneSeedHalfExtentY, PruneSeedHalfExtentXZ)))
        {
            if (!query.ClosestPointOnPoly(poly, point.SystemToRecast(), out var closest, out _).Succeeded())
                continue;

            var closestPoint = closest.RecastToSystem();
            var dx           = closestPoint.X - point.X;
            var dz           = closestPoint.Z - point.Z;
            var horizontalSq = dx * dx        + dz * dz;
            if (horizontalSq > PruneSeedMaxHorizontalDistance * PruneSeedMaxHorizontalDistance)
                continue;

            if (MathF.Abs(closestPoint.Y - point.Y) > PruneSeedMaxVerticalDistance)
                continue;

            result.Add(poly);
        }

        if (result.Count > 0)
            return result;

        query.FindNearestPoly
            (point.SystemToRecast(), new(PruneSeedHalfExtentXZ, PruneSeedHalfExtentY, PruneSeedHalfExtentXZ), s_pruneFilter, out var nearestRef, out _, out _);
        return nearestRef != 0 ? [nearestRef] : [];
    }

    private static List<long> FindIntersectingMeshPolys(DtNavMeshQuery query, Vector3 point, Vector3 halfExtent)
    {
        var capacity = 256;

        while (true)
        {
            var refs      = new long[capacity];
            var collector = new DtCollectPolysQuery(refs, refs.Length);
            var status    = query.QueryPolygons(point.SystemToRecast(), halfExtent.SystemToRecast(), s_pruneFilter, collector);
            if (status.Failed())
                return [];

            if (!collector.Overflowed())
                return [.. refs.AsSpan(0, collector.NumCollected()).ToArray()];

            capacity *= 2;
        }
    }

    private static HashSet<long> FindReachableMeshPolys(DtNavMeshQuery query, params long[] starting)
    {
        HashSet<long> result = [];

        List<long> queue = [.. starting];
        queue.RemoveAll(static polyRef => polyRef == 0);

        var navmesh = query.GetAttachedNavMesh();

        while (queue.Count > 0)
        {
            var next = queue[^1];
            queue.RemoveAt(queue.Count - 1);

            if (!result.Add(next))
                continue;

            navmesh.GetTileAndPolyByRefUnsafe(next, out var nextTile, out var nextPoly);

            for (var i = nextPoly.firstLink; i != DtDetour.DT_NULL_LINK; i = nextTile.links[i].next)
            {
                var neighbourRef = nextTile.links[i].refs;
                if (neighbourRef != 0)
                    queue.Add(neighbourRef);
            }
        }

        return result;
    }

    private static bool InCutscene => Service.Condition[ConditionFlag.WatchingCutscene] || Service.Condition[ConditionFlag.OccupiedInCutSceneEvent];

    private static string FormatHexNumbers<T>(IEnumerable<T> nums) where T : INumber<T> =>
        string.Join('.', nums.Select(n => n.ToString("X", CultureInfo.InvariantCulture)));
}
