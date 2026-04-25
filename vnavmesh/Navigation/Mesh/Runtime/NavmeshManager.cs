using System.Collections.Concurrent;
using System.Globalization;
using System.Numerics;
using System.Runtime;
using Dalamud.Game.ClientState.Conditions;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Client.Game;
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

public sealed class NavmeshManager : IDisposable
{
    public string        CurrentKey { get; private set; } = string.Empty;
    public Navmesh?      Navmesh    { get; private set; }
    public NavmeshQuery? Query      { get; private set; }

    public float LoadTaskProgress          => loadTaskProgress;
    public bool  PathfindInProgress        => numActivePathfinds > 0;
    public int   NumQueuedPathfindRequests => numActivePathfinds > 0 ? numActivePathfinds - 1 : 0;
    
    public event Action<Navmesh?, NavmeshQuery?>? OnNavmeshChanged;
    
    private static bool InInCutscene =>
        Service.Condition.Any(ConditionFlag.WatchingCutscene, ConditionFlag.OccupiedInCutSceneEvent);

    private static unsafe bool IsTerritoryFullyLoaded =>
        GameMain.Instance()->TerritoryLoadState == 2;

    private static readonly DtQueryDefaultFilter SPruneFilter = new();

    private readonly Config        config;
    private readonly DirectoryInfo cacheDirectory;
    
    private readonly ConcurrentDictionary<string, Task>  cacheWriteTasks          = new();
    private readonly ConcurrentDictionary<Navmesh, Task> cacheWriteTasksByNavmesh = new(ReferenceEqualityComparer.Instance);
    
    private float                    loadTaskProgress = -1;
    private CancellationTokenSource? currentCancelSource;
    private Task                     loadQueryTask;
    private int                      numActivePathfinds;
    
    public NavmeshManager(DirectoryInfo cacheDir, Config config)
    {
        this.config = config;
        
        cacheDirectory = cacheDir;
        cacheDir.Create();

        loadQueryTask = Service.Framework.Run(() => Log("加载任务开始"));
    }

    public void Dispose() =>
        ClearState();

    public void Update()
    {
        var curKey = GetCurrentKey();

        if (curKey == CurrentKey)
            return;

        if (!config.AutoLoadNavmesh)
        {
            if (CurrentKey.Length == 0)
                return;
            
            curKey = string.Empty;
        }

        Log($"场景转换。当前: '{CurrentKey}' 目标: '{curKey}'");
        
        CurrentKey = curKey;
        Reload(true);
    }

    public async Task<List<Vector3>> QueryPath
    (
        Vector3           from,
        Vector3           to,
        bool              flying,
        float             range          = 0,
        CancellationToken externalCancel = default
    ) =>
        (await QueryPathDetailed(from, to, flying, range, externalCancel)).Waypoints;

    internal Task<PostprocessedPath> QueryPathDetailed
    (
        Vector3           from,
        Vector3           to,
        bool              flying,
        float             range          = 0,
        CancellationToken externalCancel = default
    )
    {
        if (currentCancelSource == null)
            throw new Exception("无法发起查询, 导航数据未就绪");

        var combined = CancellationTokenSource.CreateLinkedTokenSource(currentCancelSource.Token, externalCancel);
        ++numActivePathfinds;
        
        return ExecuteWhenIdle
        (
            async _ =>
            {
                using var autoDisposeCombined  = combined;
                using var autoDecrementCounter = new OnDispose(() => --numActivePathfinds);
                
                Log($"发起算路。起点: {from} 终点: {to}");
                var result = await Task.Run
                             (
                                 () =>
                                 {
                                     combined.Token.ThrowIfCancellationRequested();
                                     if (Query == null)
                                         throw new Exception("无法发起算路, 导航数据构建未成功");
                                     
                                     Log($"执行算路。起点: {from:f3} 终点: {to:f3}");
                                     
                                     var plannerResult = flying
                                                             ? Query.PlanVolumePathDetailed(from, to, true, combined.Token)
                                                             : Query.PlanMeshPathDetailed(from, to, true, range, combined.Token);
                                     return Query.Postprocess(plannerResult, true, combined.Token);
                                 },
                                 combined.Token
                             );
                
                Log($"算路结束。结果: {result.Status} 路径点总数: {result.Waypoints.Count}");
                return result;
            },
            combined.Token
        );
    }

    public (Vector3 min, Vector3 max) BuildBitmap(Vector3 startingPos, string filename, float pixelSize, AABB? mapBounds = null)
    {
        if (Navmesh == null || Query == null)
            throw new InvalidOperationException("无法生成位图。导航路网数据未就绪");

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
                if (!AreInBounds(v))
                    goto cont;

                min = Vector3.Min(min, v);
                max = Vector3.Max(max, v);
            }

            polysInbounds.Add(p);

            cont: ;
        }

        var bitmap = new NavmeshBitmap(min, max, pixelSize);
        foreach (var p in polysInbounds)
            bitmap.RasterizePolygon(Navmesh.Mesh, p);
        bitmap.Save(filename);
        
        Log($"已生成位图。文件名: {filename} 中心点: {startingPos} 范围: {bitmap.MinBounds}-{bitmap.MaxBounds}");
        
        return (bitmap.MinBounds, bitmap.MaxBounds);

        bool AreInBounds(Vector3 vert)
        {
            return mapBounds is not { } aabb ||
                   vert.X >= aabb.Min.X && vert.Y >= aabb.Min.Y && vert.Z >= aabb.Min.Z && vert.X <= aabb.Max.X && vert.Y <= aabb.Max.Y && vert.Z <= aabb.Max.Z;
        }
    }

    public bool Reload(bool allowLoadFromCache)
    {
        ClearState();

        if (CurrentKey.Length <= 0)
            return true;
        
        var cts = currentCancelSource = new();
        ExecuteWhenIdle
        (
            async cancel =>
            {
                loadTaskProgress = 0;

                using var resetLoadProgress = new OnDispose(() => loadTaskProgress = -1);

                var cutsceneStart = DateTime.Now;
                while (InInCutscene)
                {
                    if ((DateTime.Now - cutsceneStart).TotalSeconds >= 5)
                    {
                        cutsceneStart = DateTime.Now;
                        Log("等待过场剧情结束");
                    }

                    await Task.Delay(100, cancel);
                }
                
                var territoryLoadStart = DateTime.Now;
                while (!IsTerritoryFullyLoaded)
                {
                    if ((DateTime.Now - territoryLoadStart).TotalSeconds >= 5)
                    {
                        territoryLoadStart = DateTime.Now;
                        Log("等待区域加载完毕");
                    }

                    await Task.Delay(100, cancel);
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
                Log($"场景快照耗时: {snapshotTimer.Value().TotalMilliseconds:f1} 毫秒");

                Log($"开始构建。键: {cacheKey} 允许重载: {allowLoadFromCache}");
                var (navmesh, cacheFile) = await Task.Run(() => BuildNavmesh(scene, cacheKey, allowLoadFromCache, cancel), cancel);
                
                Log($"加载完毕。键: {cacheKey}");
                
                Navmesh = navmesh;
                Query   = new(Navmesh, config);

                var ff = await FloodFill.GetAsync();

                if (ff.TryLookup(scene.TerritoryID, out var points))
                {
                    var pruneSeeds = points.ToArray();
                    Navmesh.DeferMeshMutation(mesh => PruneMesh(mesh, pruneSeeds));
                }

                OnNavmeshChanged?.Invoke(Navmesh, Query);
                if (cacheFile != null)
                    QueueCacheWrite(cacheKey, cacheFile, navmesh);
            },
            cts.Token
        );

        return true;
    }

    public void ClearForSceneChange()
    {
        Log("清除路径状态。场景发生切换");
        
        CurrentKey = string.Empty;
        ClearState();
    }

    internal void ReplaceMesh(Navmesh mesh)
    {
        var retiredNavmesh = Navmesh;
        var retiredQuery   = Query;
        
        Navmesh = mesh;
        Query   = new(Navmesh, config);
        
        Log("导航路网替换完成");
        
        OnNavmeshChanged?.Invoke(Navmesh, Query);
        ReleaseRetiredState(retiredNavmesh, retiredQuery, "网格替换");
    }

    private void ClearState()
    {
        if (currentCancelSource == null)
            return;

        var cts = currentCancelSource;
        currentCancelSource = null;
        cts.Cancel();
        
        var retiredNavmesh = Navmesh;
        var retiredQuery   = Query;
        Log("入队状态清理");
        
        ExecuteWhenIdle
        (
            () =>
            {
                Log("清理状态中");
                
                numActivePathfinds = 0;
                cts.Dispose();
                OnNavmeshChanged?.Invoke(null, null);
                Query   = null;
                Navmesh = null;
                ReleaseRetiredState(retiredNavmesh, retiredQuery, "场景切换卸载");
            },
            CancellationToken.None
        );
    }

    private BuildNavmeshResult BuildNavmesh(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel)
    {
        var totalTimer = StopWatchTimer.Create();
        Log($"构建任务开始。键: {cacheKey}");
        
        var customization = NavmeshCustomizationRegistry.GetForTerritory(scene.TerritoryID);
        Log($"自定义数据搜寻获取。区域: {scene.TerritoryID} 类型: {customization.GetType()}");

        var layers         = scene.FestivalLayers.ToList();
        var buildSignature = NavmeshBuilder.ComputeBuildSignature(scene, customization);
        var cache          = new FileInfo($"{cacheDirectory.FullName}/{cacheKey}.navmesh");

        if (allowLoadFromCache && TryLoadFromCache(cache, customization, buildSignature, layers, totalTimer, out var cachedResult))
            return cachedResult;

        cancel.ThrowIfCancellationRequested();

        var buildTimer          = StopWatchTimer.Create();
        var builder             = new NavmeshBuilder(scene, customization, config);
        var totalProgressWeight = Math.Max(builder.TotalEstimatedTileWeight, 1);
        
        builder.Build
        (weight =>
            {
                Interlocked.Add(ref loadTaskProgress,  0.99f * weight / totalProgressWeight);
                cancel.ThrowIfCancellationRequested();
            }
        );
        
        Log($"冷构建耗时: {buildTimer.Value().TotalMilliseconds:f1} 毫秒");

        customization.CustomizeMesh(builder.Navmesh, layers);
        var runtimeMesh = builder.Navmesh with { CustomizationApplied = true };

        if (runtimeMesh.Volume != null)
        {
            var compactTimer = StopWatchTimer.Create();
            runtimeMesh.Volume.CompactRetainedState();
            Log($"飞行体素常驻压缩耗时: {compactTimer.Value().TotalMilliseconds:f1} 毫秒");
        }

        Log($"总构建耗时: {totalTimer.Value().TotalMilliseconds:f1} 毫秒");
        Interlocked.Add(ref loadTaskProgress, 0.01f);
        return new(runtimeMesh, cache);
    }

    private static bool TryLoadFromCache
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
            result = new(null!, null);
            return false;
        }

        try
        {
            var cacheReadTimer = StopWatchTimer.Create();
            Log($"缓存加载。文件: {cache.FullName}");
            
            using var stream = new FileStream(cache.FullName, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 22, FileOptions.SequentialScan);
            using var reader = new BinaryReader(stream);
            
            var (mesh, cacheTelemetry, requiresRewrite) = Navmesh.Deserialize(reader, customization.Version, buildSignature);

            Log($"缓存已读取。耗时: {cacheReadTimer.Value().TotalMilliseconds:f1} ms");
            LogCacheSegment("读取", cacheTelemetry.Mesh);
            LogCacheSegment("读取", cacheTelemetry.Volume);
            
            if (!mesh.CustomizationApplied)
                customization.CustomizeMesh(mesh, layers);
            
            Log($"缓存命中。总耗时: {totalTimer.Value().TotalMilliseconds:f1} ms");
            result = new(mesh, requiresRewrite ? cache : null);
            return true;
        }
        catch (Exception ex)
        {
            Log($"加载缓存失败。{ex}");
            
            result = new(null!, null);
            return false;
        }
    }

    private static unsafe string GetCurrentKey()
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
            if (fest is { Id: 0, Phase: 0 })
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
        if (cacheWriteTasks.TryGetValue(cacheKey, out var existing) && !existing.IsCompleted)
        {
            Log($"后台缓存写入已在进行，跳过重复调度: {cacheKey}");
            return;
        }

        var writeTask = Task.Run(() => WriteCache(cacheKey, cache, navmesh));
        cacheWriteTasks[cacheKey]         = writeTask;
        cacheWriteTasksByNavmesh[navmesh] = writeTask;
        _ = writeTask.ContinueWith
        (
            t =>
            {
                cacheWriteTasks.TryRemove(cacheKey, out _);
                cacheWriteTasksByNavmesh.TryRemove(navmesh, out _);
                LogTaskError(t);
            },
            CancellationToken.None,
            TaskContinuationOptions.ExecuteSynchronously,
            TaskScheduler.Default
        );
    }

    private static void WriteCache(string cacheKey, FileInfo cache, Navmesh navmesh)
    {
        var timer    = StopWatchTimer.Create();
        var tempPath = $"{cache.FullName}.{Environment.ProcessId}.{Environment.CurrentManagedThreadId}.tmp";

        try
        {
            cache.Directory?.Create();
            Log($"后台写入缓存。文件: {cache.FullName}");

            var serializeTimer = StopWatchTimer.Create();
            
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
                $"后台缓存写入完成。键: {cacheKey} 总耗时: {timer.Value().TotalMilliseconds:f1} 毫秒 序列化: {serializeDuration.TotalMilliseconds:f1} 毫秒 替换: {replaceDuration.TotalMilliseconds:f1} 毫秒 大小: {sizeBytes / 1024.0 / 1024.0:f2} MB"
            );
        }
        catch (Exception ex)
        {
            Log($"后台缓存写入失败。键: {cacheKey} 错误: {ex}");
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
                // ignored
            }
        }
    }

    private void ExecuteWhenIdle(Action task, CancellationToken token)
    {
        var prev = loadQueryTask;
        loadQueryTask = Service.Framework.Run
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
        var prev = loadQueryTask;
        loadQueryTask = Service.Framework.Run
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
        var prev = loadQueryTask;
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
        loadQueryTask = res;
        return res;
    }

    private static void Log(string message) => 
        Service.Log.Debug($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");

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

        if (cacheWriteTasksByNavmesh.TryGetValue(navmesh, out var pendingWrite) && !pendingWrite.IsCompleted)
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

    private static void FinalizeRetiredNavmeshRelease(Navmesh navmesh, string reason)
    {
        navmesh.ReleaseRetainedState();
        Log($"旧场景重资源已释放: {reason}");

        if (navmesh.Volume != null)
            RequestMemoryCompaction(reason);
    }

    private static void RequestMemoryCompaction(string reason)
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

    private static HashSet<long> CollectPruneSeedPolys(DtNavMeshQuery query, IEnumerable<Vector3> points)
    {
        HashSet<long> result = [];

        foreach (var point in points)
        {
            foreach (var poly in FindPruneSeedPolys(query, point))
                result.Add(poly);
        }

        return result;
    }

    private static HashSet<long> FindPruneSeedPolys(DtNavMeshQuery query, Vector3 point)
    {
        HashSet<long> result = [];

        foreach (var poly in FindIntersectingMeshPolys(query, point, new(PRUNE_SEED_HALF_EXTENT_XZ, PRUNE_SEED_HALF_EXTENT_Y, PRUNE_SEED_HALF_EXTENT_XZ)))
        {
            if (!query.ClosestPointOnPoly(poly, point.SystemToRecast(), out var closest, out _).Succeeded())
                continue;

            var closestPoint = closest.RecastToSystem();
            var dx           = closestPoint.X - point.X;
            var dz           = closestPoint.Z - point.Z;
            var horizontalSq = dx * dx        + dz * dz;
            if (horizontalSq > PRUNE_SEED_MAX_HORIZONTAL_DISTANCE * PRUNE_SEED_MAX_HORIZONTAL_DISTANCE)
                continue;

            if (MathF.Abs(closestPoint.Y - point.Y) > PRUNE_SEED_MAX_VERTICAL_DISTANCE)
                continue;

            result.Add(poly);
        }

        if (result.Count > 0)
            return result;

        query.FindNearestPoly
        (
            point.SystemToRecast(),
            new(PRUNE_SEED_HALF_EXTENT_XZ, PRUNE_SEED_HALF_EXTENT_Y, PRUNE_SEED_HALF_EXTENT_XZ),
            SPruneFilter,
            out var nearestRef,
            out _,
            out _
        );
        return nearestRef != 0 ? [nearestRef] : [];
    }

    private static List<long> FindIntersectingMeshPolys(DtNavMeshQuery query, Vector3 point, Vector3 halfExtent)
    {
        var capacity = 256;

        while (true)
        {
            var refs      = new long[capacity];
            var collector = new DtCollectPolysQuery(refs, refs.Length);
            var status    = query.QueryPolygons(point.SystemToRecast(), halfExtent.SystemToRecast(), SPruneFilter, collector);
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

    private static string FormatHexNumbers<T>(IEnumerable<T> nums) where T : INumber<T> =>
        string.Join('.', nums.Select(n => n.ToString("X", CultureInfo.InvariantCulture)));

    private sealed record BuildNavmeshResult
    (
        Navmesh   Navmesh,
        FileInfo? CacheFile
    );

    #region 常量

    private const float PRUNE_SEED_HALF_EXTENT_XZ          = 8.0f;
    private const float PRUNE_SEED_HALF_EXTENT_Y           = 16.0f;
    private const float PRUNE_SEED_MAX_HORIZONTAL_DISTANCE = 8.0f;
    private const float PRUNE_SEED_MAX_VERTICAL_DISTANCE   = 12.0f;

    #endregion
}
