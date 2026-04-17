using System.Collections.Concurrent;
using System.Numerics;
using System.Runtime;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Planning;
using vnavmesh.Shared.Utilities;
using Action = System.Action;

namespace vnavmesh.Navigation.Mesh.Runtime;

// manager that loads navmesh matching current zone and performs async pathfinding queries
public sealed partial class NavmeshManager : IDisposable
{
    private readonly Config _config;

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
        (
            () =>
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
        if (Navmesh == null || Query == null)
            throw new InvalidOperationException("can't prune, mesh is missing");

        var startPolys = points.Select(pt => Query.FindNearestMeshPoly(pt));
        Log($"seeding from start polys: {string.Join(", ", startPolys.Select(p => p.ToString("X")))}");
        var reachablePolys = Query.FindReachableMeshPolys([.. startPolys]);

        var pruneCount = 0;

        for (var i = 0; i < Navmesh.Mesh.GetMaxTiles(); i++)
        {
            var t = Navmesh.Mesh.GetTile(i);
            if (t.data?.header == null)
                continue;

            var prBase = Navmesh.Mesh.GetPolyRefBase(t);

            for (var j = 0; j < t.data.header.polyCount; j++)
            {
                var pref = prBase | (uint)j;

                if (Navmesh.Mesh.GetPolyFlags(pref, out var fl).Failed())
                {
                    Log($"failed to fetch flags for {pref:X}");
                    continue;
                }

                if (reachablePolys.Contains(pref))
                {
                    if (Navmesh.Mesh.SetPolyFlags(pref, fl & ~(int)NavmeshPolyFlags.Unreachable).Failed())
                        Log($"failed to set flags for {pref:X}");
                }
                else
                {
                    pruneCount++;
                    if (Navmesh.Mesh.SetPolyFlags(pref, fl | (int)NavmeshPolyFlags.Unreachable).Failed())
                        Log($"failed to set flags for {pref:X}");
                }
            }
        }

        Log($"pruned {pruneCount} unreachable polygons");
    }
}
