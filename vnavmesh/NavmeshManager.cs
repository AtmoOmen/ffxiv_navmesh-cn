using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;
using Dalamud.Game.ClientState.Conditions;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Lumina.Excel.Sheets;
using Navmesh.NavVolume;
using Navmesh.Utilities;
using Action = System.Action;

namespace Navmesh;

// manager that loads navmesh matching current zone and performs async pathfinding queries
public sealed class NavmeshManager : IDisposable
{
    public bool UseRaycasts      = true;
    public bool UseStringPulling = true;

    public string                                 CurrentKey { get; private set; } = "";
    public Navmesh?                               Navmesh    { get; private set; }
    public NavmeshQuery?                          Query      { get; private set; }
    public event Action<Navmesh?, NavmeshQuery?>? OnNavmeshChanged;

    public           float LoadTaskProgress => loadTaskProgress; // negative if load task is not running, otherwise in [0, 1] range
    private volatile float loadTaskProgress = -1;
    private          int   loadTaskProgressBits;

    // 路径计算进度相关字段
    public           float PathfindProgress => pathfindProgress; // negative if pathfind is not running, otherwise in [0, 1] range  
    private volatile float pathfindProgress = -1;
    private          int   pathfindProgressBits;

    private CancellationTokenSource? currentCTS;
    private Task                     lastLoadQueryTask;

    public  bool PathfindInProgress        => numActivePathfinds > 0;
    public  int  NumQueuedPathfindRequests => numActivePathfinds > 0 ? numActivePathfinds - 1 : 0;
    private int  numActivePathfinds;
    
    private readonly DirectoryInfo cacheDirectory;

    public NavmeshManager(DirectoryInfo cacheDir)
    {
        cacheDirectory = cacheDir;
        cacheDir.Create();

        lastLoadQueryTask = Service.Framework.Run(() => Log("任务已启动"));
    }

    public void Dispose()
    {
        Log("正在释放资源");
        ClearState();
    }

    public void Update()
    {
        var curKey = GetCurrentKey();
        if (curKey != CurrentKey)
        {
            // 导航网格需要重新加载
            if (!Service.Config.AutoLoadNavmesh)
            {
                if (CurrentKey.Length == 0)
                    return;  // 未加载任何内容，且禁止自动加载
                curKey = ""; // 仅卸载现有导航网格
            }

            Log($"开始由 '{CurrentKey}' 转换至 '{curKey}'");
            CurrentKey = curKey;
            Reload(true);
            // 导航网格加载进行中
        }
    }

    public bool Reload(bool allowLoadFromCache)
    {
        ClearState();
        if (CurrentKey.Length > 0)
        {
            var cts = currentCTS = new();
            ExecuteWhenIdle(async cancel =>
            {
                loadTaskProgress = 0;

                using var resetLoadProgress = new OnDispose(() => loadTaskProgress = -1);

                var waitStart = DateTime.Now;

                while (InCutscene)
                {
                    if ((DateTime.Now - waitStart).TotalSeconds >= 5)
                    {
                        waitStart = DateTime.Now;
                        Log("等待过场动画结束");
                    }

                    await Service.Framework.DelayTicks(1, cancel);
                }

                var (cacheKey, scene) = await Service.Framework.Run(() =>
                {
                    var scene = new SceneDefinition();
                    scene.FillFromActiveLayout();
                    var cacheKey = GetCacheKey(scene);
                    return (cacheKey, scene);
                }, cancel);

                Log($"开始构建导航网格 '{cacheKey}' (重载: {allowLoadFromCache})");
                var navmesh = await Task.Run(async () => await BuildNavmeshAsync(scene, cacheKey, allowLoadFromCache, cancel), cancel);
                Log($"导航网格加载完成: '{cacheKey}'");

                Navmesh = navmesh;
                Query   = new(Navmesh);
                OnNavmeshChanged?.Invoke(Navmesh, Query);
            }, cts.Token);
        }

        return true;
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static string GetPathKey(Vector3 from, Vector3 to)
    {
        // 使用四舍五入可以让邻近位置的查询复用相同的路径
        var roundedFrom = new Vector3(
            MathF.Round(from.X * 0.1f) * 10.0f,
            MathF.Round(from.Y * 0.1f) * 10.0f,
            MathF.Round(from.Z * 0.1f) * 10.0f
        );

        var roundedTo = new Vector3(
            MathF.Round(to.X * 0.1f) * 10.0f,
            MathF.Round(to.Y * 0.1f) * 10.0f,
            MathF.Round(to.Z * 0.1f) * 10.0f
        );

        return $"{roundedFrom.X},{roundedFrom.Y},{roundedFrom.Z}->{roundedTo.X},{roundedTo.Y},{roundedTo.Z}";
    }

    internal void ReplaceMesh(Navmesh mesh)
    {
        Log("导航网格已替换");
        Navmesh = mesh;
        Query   = new(Navmesh);
        OnNavmeshChanged?.Invoke(Navmesh, Query);
    }

    private static bool InCutscene =>
        Service.Condition[ConditionFlag.WatchingCutscene] || Service.Condition[ConditionFlag.OccupiedInCutSceneEvent];

    public Task<List<Vector3>> QueryPath(Vector3 from, Vector3 to, bool flying, CancellationToken externalCancel = default, float range = 0)
    {
        if (currentCTS == null)
            throw new Exception("无法开始查询, 导航数据仍在构建过程中");

        // 任务可以被内部请求（即当导航网格重新加载时）或外部取消
        var combined = CancellationTokenSource.CreateLinkedTokenSource(currentCTS.Token, externalCancel);
        ++numActivePathfinds;
        return ExecuteWhenIdle(async _ =>
        {
            using var autoDisposeCombined  = combined;
            using var autoDecrementCounter = new OnDispose(() => --numActivePathfinds);
            using var autoResetProgress    = new OnDispose(() => pathfindProgress = -1);

            // 初始化路径计算进度
            pathfindProgress = 0;

            Log($"启动从 {from} 到 {to} 的寻路");
            
            // 创建进度回调委托
            var progressCallback = UpdatePathfindProgressAtomically;
            
            var path = await Task.Run(() =>
            {
                combined.Token.ThrowIfCancellationRequested();
                if (Query == null)
                    throw new Exception("无法寻路, 导航网格构建失败");
                Log($"执行从 {from} 到 {to} 的寻路");
                return flying
                           ? Query.PathfindVolume(from, to, UseRaycasts, UseStringPulling, progressCallback, combined.Token)
                           : Query.PathfindMesh(from, to, UseRaycasts, UseStringPulling, combined.Token, range);
            }, combined.Token);
            Log($"寻路完成: {path.Count} 个路径点");

            return path;
        }, combined.Token);
    }

    // 注意: pixelSize 应为 2 的幂次方
    public (Vector3 min, Vector3 max) BuildBitmap(Vector3 startingPos, string filename, float pixelSize, AABB? mapBounds = null)
    {
        if (Navmesh == null || Query == null)
            throw new InvalidOperationException("无法构建位图, 正在创建导航信息");

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
            }

            polysInbounds.Add(p);

            cont: ;
        }

        var bitmap = new NavmeshBitmap(min, max, pixelSize);
        foreach (var p in polysInbounds) bitmap.RasterizePolygon(Navmesh.Mesh, p);
        bitmap.Save(filename);
        Service.Log.Debug($"生成导航位图 '{filename}' @ {startingPos}: {bitmap.MinBounds}-{bitmap.MaxBounds}");
        return (bitmap.MinBounds, bitmap.MaxBounds);

        bool inBounds(Vector3 vert)
        {
            return mapBounds is not { } aabb ||
                   (vert.X >= aabb.Min.X && vert.Y >= aabb.Min.Y && vert.Z >= aabb.Min.Z && vert.X <= aabb.Max.X &&
                    vert.Y <= aabb.Max.Y && vert.Z <= aabb.Max.Z);
        }
    }

    private static unsafe string GetCurrentKey()
    {
        var layout = LayoutWorld.Instance()->ActiveLayout;
        if (layout == null || layout->InitState != 7 || layout->FestivalStatus is > 0 and < 5)
            return ""; // 场景布局尚未就绪

        var filter    = LayoutUtils.FindFilter(layout);
        var filterKey = filter != null ? filter->Key : 0;

        var terrRow = Service.LuminaRow<TerritoryType>(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

        // CE always has a festival layer (i hope). the non-festival layout is briefly loaded when entering the zone, which triggers a useless mesh build since the uninitialized zone is still the same size
        if (terrRow?.TerritoryIntendedUse.RowId == 60)
        {
            var fest = layout->ActiveFestivals[0];
            if (fest is { Id: 0, Phase: 0 })
                return string.Empty;
        }

        return $"{terrRow?.Bg}//{filterKey:X}//{LayoutUtils.FestivalsString(layout->ActiveFestivals)}";
    }

    internal static unsafe string GetCacheKey(SceneDefinition scene)
    {
        // 注意: 节日活动在全局范围内激活，但大多数区域没有特定节日的图层，所以我们只在缓存键中包含真实存在的图层
        var layout    = LayoutWorld.Instance()->ActiveLayout;
        var filter    = LayoutUtils.FindFilter(layout);
        var filterKey = filter != null ? filter->Key : 0;
        var terrRow   = Service.LuminaRow<TerritoryType>(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);
        return $"{terrRow?.Bg.ToString().Replace('/', '_')}__{filterKey:X}__{string.Join('.', scene.FestivalLayers.Select(id => id.ToString("X")))}";
    }

    private void ClearState()
    {
        if (currentCTS == null)
            return; // 已经清理完毕

        var cts = currentCTS;
        currentCTS = null;
        cts.Cancel();
        Log("正在队列中等待清理状态");
        ExecuteWhenIdle(() =>
        {
            Log("正在清理状态");
            numActivePathfinds = 0;
            cts.Dispose();
            OnNavmeshChanged?.Invoke(null, null);
            Query   = null;
            Navmesh = null;
            
        }, CancellationToken.None);
    }

    private async Task<Navmesh> BuildNavmeshAsync(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel)
    {
        Log($"异步构建任务开始: '{cacheKey}'");
        var customization = NavmeshCustomizationRegistry.ForTerritory(scene.TerritoryID);
        Log($"Customization for '{scene.TerritoryID}': {customization.GetType()}");

        // 尝试读取缓存
        var cachePath = $"{cacheDirectory.FullName}/{cacheKey}.navmesh";
        if (allowLoadFromCache && File.Exists(cachePath))
        {
            var navmesh = await LoadFromCacheAsync(cachePath, customization.Version, cancel);
            if (navmesh != null) return navmesh;
        }

        cancel.ThrowIfCancellationRequested();

        loadTaskProgressBits = 0;
        var builder       = new NavmeshBuilder(scene, customization);
        var deltaProgress = 1.0f / (builder.NumTilesX * builder.NumTilesZ);

        var parallelOptions = new ParallelOptions
        {
            CancellationToken = cancel,
            MaxDegreeOfParallelism = Environment.ProcessorCount <= 8
                                         ? Math.Max(1, Environment.ProcessorCount / 2)
                                         : Math.Max(1, Environment.ProcessorCount - 1)
        };

        var tileTasks = new List<(int X, int Z)>();
        for (var z = 0; z < builder.NumTilesZ; ++z)
        for (var x = 0; x < builder.NumTilesX; ++x)
            tileTasks.Add((x, z));

        await Task.Run(() =>
        {
            Parallel.ForEach(tileTasks, parallelOptions, tile =>
            {
                builder.BuildTile(tile.X, tile.Z);

                UpdateProgressAtomically(deltaProgress);

                cancel.ThrowIfCancellationRequested();
            });
        }, cancel);
        
        customization.CustomizeMesh(builder.Navmesh.Mesh);
        builder.Navmesh.Mesh.ConnectCrossTileLinks();

        // 异步写入缓存
        await WriteToCacheAsync(builder.Navmesh, cachePath, cancel);

        return builder.Navmesh;
    }

    private void UpdateProgressAtomically(float deltaProgress)
    {
        while (true)
        {
            var currentBits  = Interlocked.CompareExchange(ref loadTaskProgressBits, 0, 0);
            var currentValue = BitConverter.Int32BitsToSingle(currentBits);

            var newValue = currentValue + deltaProgress;

            var newBits = BitConverter.SingleToInt32Bits(newValue);
            if (Interlocked.CompareExchange(ref loadTaskProgressBits, newBits, currentBits) == currentBits)
            {
                loadTaskProgress = newValue;
                break;
            }
        }
    }

    private void UpdatePathfindProgressAtomically(float progress)
    {
        while (true)
        {
            var currentBits = Interlocked.CompareExchange(ref pathfindProgressBits, 0, 0);

            // 直接设置新进度值
            var newBits = BitConverter.SingleToInt32Bits(progress);
            if (Interlocked.CompareExchange(ref pathfindProgressBits, newBits, currentBits) == currentBits)
            {
                pathfindProgress = progress;
                break;
            }
        }
    }

    private Navmesh BuildNavmesh(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel) =>
        BuildNavmeshAsync(scene, cacheKey, allowLoadFromCache, cancel).GetAwaiter().GetResult();

    private void ExecuteWhenIdle(Action task, CancellationToken token)
    {
        var prev = lastLoadQueryTask;
        lastLoadQueryTask = Service.Framework.Run(async () =>
        {
            await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
            _ = prev.Exception;
            task();
        }, token);
    }

    private void ExecuteWhenIdle(Func<CancellationToken, Task> task, CancellationToken token)
    {
        var prev = lastLoadQueryTask;
        lastLoadQueryTask = Service.Framework.Run(async () =>
        {
            await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
            _ = prev.Exception;
            var t = task(token);
            await t.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
            LogTaskError(t);
        }, token);
    }

    private Task<T> ExecuteWhenIdle<T>(Func<CancellationToken, Task<T>> task, CancellationToken token)
    {
        var prev = lastLoadQueryTask;
        var res = Service.Framework.Run(async () =>
        {
            await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
            _ = prev.Exception;
            var t = task(token);
            await ((Task)t).ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
            LogTaskError(t);
            return t.Result;
        }, token);
        lastLoadQueryTask = res;
        return res;
    }

    private static void Log(string message) =>
        Service.Log.Debug($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");

    private static void LogTaskError(Task task)
    {
        if (task.IsFaulted)
            Service.Log.Error($"[NavmeshManager] 任务执行失败: {task.Exception}");
    }

    private static async Task<Navmesh?> LoadFromCacheAsync(string cachePath, int version, CancellationToken cancel)
    {
        try
        {
            Log($"异步加载缓存: {cachePath}");
            await using var stream = new FileStream(cachePath, FileMode.Open, FileAccess.Read, FileShare.Read, 4096, true);
            using var       reader = new BinaryReader(stream);
            return await Task.Run(() => Navmesh.Deserialize(reader, version), cancel);
        }
        catch (Exception ex)
        {
            Log($"异步加载缓存失败: {ex}");
            return null;
        }
    }

    private static async Task WriteToCacheAsync(Navmesh navmesh, string cachePath, CancellationToken cancel)
    {
        try
        {
            Log($"异步写入缓存: {cachePath}");
            await using var stream = new FileStream(cachePath, FileMode.Create, FileAccess.Write, FileShare.None, 4096, true);
            await using var writer = new BinaryWriter(stream);
            await Task.Run(() => navmesh.Serialize(writer), cancel);
        }
        catch (Exception ex) { Log($"异步写入缓存失败: {ex}"); }
    }
}
