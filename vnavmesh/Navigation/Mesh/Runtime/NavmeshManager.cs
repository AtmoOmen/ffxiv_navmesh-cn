using System.Collections.Concurrent;
using System.Buffers.Binary;
using System.Diagnostics;
using System.Globalization;
using System.IO.Pipes;
using System.Numerics;
using System.Runtime;
using System.Text;
using System.Text.Json;
using Dalamud.Game.ClientState.Conditions;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Client.Game;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Bootstrap.Composition;
using vnavmesh.Configuration;
using vnavmesh.Common.Ipc;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Scene;
using vnavmesh.Common.Utilities;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Scene;
using vnavmesh.Shared.Utilities;
using Action = System.Action;

namespace vnavmesh.Navigation.Mesh.Runtime;

public sealed class NavmeshManager : IDisposable
{
    private static readonly JsonSerializerOptions ManifestJsonOptions = new()
    {
        IncludeFields = true
    };

    public string        CurrentKey { get; private set; } = string.Empty;
    public Navmesh?      Navmesh    { get; private set; }
    public NavmeshQuery? Query      { get; private set; }

    public float LoadTaskProgress          => loadTaskProgress;

    private float externalBuildProgress = -1f;
    public  float ExternalBuildProgress
    {
        get => Volatile.Read(ref externalBuildProgress);
        internal set => Volatile.Write(ref externalBuildProgress, value);
    }

    public bool  PathfindInProgress        => numActivePathfinds > 0;
    public int   NumQueuedPathfindRequests => numActivePathfinds > 0 ? numActivePathfinds - 1 : 0;
    
    public event Action<Navmesh?, NavmeshQuery?>? OnNavmeshChanged;
    
    private static bool InInCutscene =>
        Service.Condition.Any(ConditionFlag.WatchingCutscene, ConditionFlag.OccupiedInCutSceneEvent);

    private static unsafe bool IsTerritoryFullyLoaded =>
        GameMain.Instance()->TerritoryLoadState == 2;

    private static readonly DtQueryDefaultFilter SPruneFilter = new();

    private readonly Config        config;
    private readonly PluginPaths   paths;
    private readonly DirectoryInfo cacheDirectory;
    
    private readonly ConcurrentDictionary<string, Task>  cacheWriteTasks          = new();
    private readonly ConcurrentDictionary<Navmesh, Task> cacheWriteTasksByNavmesh = new(ReferenceEqualityComparer.Instance);
    
    private float                    loadTaskProgress = -1;
    private CancellationTokenSource? currentCancelSource;
    private Task                     loadQueryTask;
    private int                      numActivePathfinds;
    private int                      nextBuildSequence;
    
    public NavmeshManager(PluginPaths paths, Config config)
    {
        this.config = config;
        this.paths  = paths;
        
        cacheDirectory = paths.MeshCacheDirectory;
        cacheDirectory.Create();
        paths.WorkerStateDirectory.Create();

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

    internal Task<PostprocessedPath> QueryStraightPathDetailed
    (
        Vector3           from,
        Vector3           to,
        bool              flying,
        float             range                = 0,
        int               straightPathOptions  = 0,
        CancellationToken externalCancel       = default
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

                Log($"发起 straight path 查询。起点: {from} 终点: {to}");
                var result = await Task.Run
                             (
                                 () =>
                                 {
                                     combined.Token.ThrowIfCancellationRequested();
                                     if (Query == null)
                                         throw new Exception("无法发起 straight path 查询, 导航数据构建未成功");

                                     Log($"执行 straight path 查询。起点: {from:f3} 终点: {to:f3}");

                                     var plannerResult = flying
                                                             ? Query.PlanVolumePathDetailed(from, to, true, combined.Token)
                                                             : Query.PlanMeshPathDetailed(from, to, true, range, combined.Token);
                                     return Query.PostprocessStraightPath(plannerResult, combined.Token, straightPathOptions);
                                 },
                                 combined.Token
                             );

                Log($"straight path 查询结束。结果: {result.Status} 路径点总数: {result.Waypoints.Count}");
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
                        territoryLoadStart = DateTime.Now;

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
                var (navmesh, cacheFile) = await BuildNavmesh(scene, cacheKey, allowLoadFromCache, cancel);
                
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

    private async Task<BuildNavmeshResult> BuildNavmesh(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel)
    {
        var totalTimer = StopWatchTimer.Create();
        Log($"构建任务开始。键: {cacheKey}");
        
        var customization = NavmeshCustomizationRegistry.GetForScene(scene);
        Log($"自定义数据搜寻获取。区域: {scene.TerritoryID} 类型: {customization.GetType()}");

        var layers        = scene.FestivalLayers.ToList();
        var buildSnapshot = await CreateBuildSnapshot(scene, customization, cancel);
        var cache         = new FileInfo(Path.Combine(cacheDirectory.FullName, $"{cacheKey}.navmesh"));

        if (allowLoadFromCache && TryLoadFromCache(cache, customization, buildSnapshot.BuildSignature, layers, totalTimer, out var cachedResult))
            return cachedResult;

        cancel.ThrowIfCancellationRequested();

        var buildTimer = StopWatchTimer.Create();
        var rawFile    = await RunExternalBuild(cacheKey, buildSnapshot.Scene, buildSnapshot.Settings, cancel, true);
        Log($"外置冷构建耗时: {buildTimer.Value().TotalMilliseconds:f1} 毫秒");

        Navmesh runtimeMesh;
        await using (var stream = new FileStream(rawFile.FullName, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 22, FileOptions.SequentialScan | FileOptions.Asynchronous))
        using (var reader = new BinaryReader(stream, Encoding.UTF8, true))
        {
            var (mesh, cacheTelemetry, _) = Navmesh.Deserialize(reader, customization.Version, buildSnapshot.BuildSignature);
            LogCacheSegment("外置构建读取", cacheTelemetry.Mesh);
            LogCacheSegment("外置构建读取", cacheTelemetry.Volume);
            customization.CustomizeMesh(mesh, layers);
            runtimeMesh = mesh with { CustomizationApplied = true };
        }

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

    private async Task<BuildSnapshot> CreateBuildSnapshot(SceneDefinition scene, NavmeshCustomization customization, CancellationToken cancel) =>
        await Task.Run
        (
            () =>
            {
                cancel.ThrowIfCancellationRequested();

                var settings = customization.GetBuildSettings(scene);
                var flyable  = customization.IsFlyingSupported(scene);
                var extractor = new SceneExtractor(scene);
                customization.CustomizeScene(extractor);

                var buildScene    = extractor.ToBuildScene();
                var buildSettings = settings.ToBuildSettings(flyable, customization.Version);
                return new BuildSnapshot(buildScene, buildSettings, vnavmesh.Common.Navigation.Mesh.Build.NavmeshBuilder.ComputeBuildSignature(buildScene, buildSettings));
            },
            cancel
        );

    private async Task<FileInfo> RunExternalBuild
    (
        string                                                    cacheKey,
        BuildScene                                                scene,
        vnavmesh.Common.Navigation.Mesh.Build.NavmeshBuildSettings settings,
        CancellationToken                                         cancel,
        bool                                                      updateLoadProgress,
        Action<double>?                                           onProgress = null
    )
    {
        paths.WorkerStateDirectory.Create();

        var safeCacheKey = string.Concat(cacheKey.Select(static c => char.IsLetterOrDigit(c) || c is '-' or '_' or '.' ? c : '_'));
        var buildId      = $"{safeCacheKey}.{Environment.ProcessId}.{Interlocked.Increment(ref nextBuildSequence):X}";
        var sceneFile    = new FileInfo(Path.Combine(paths.WorkerStateDirectory.FullName, $"{buildId}.scene.bin"));
        var rawFile      = new FileInfo(Path.Combine(paths.WorkerStateDirectory.FullName, $"{buildId}.raw.navmesh"));
        var manifestFile = new FileInfo(Path.Combine(paths.WorkerStateDirectory.FullName, $"{buildId}.manifest.json"));
        var pipeName     = $"vnavmesh-build-{Environment.ProcessId}-{buildId}";

        await using (var sceneStream = new FileStream(sceneFile.FullName, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 22, FileOptions.SequentialScan | FileOptions.Asynchronous))
        using (var sceneWriter = new BinaryWriter(sceneStream, Encoding.UTF8, true))
        {
            scene.Write(sceneWriter);
            sceneWriter.Flush();
            await sceneStream.FlushAsync(cancel);
        }

        var manifest = new NavmeshBuildManifest(sceneFile.FullName, rawFile.FullName, cacheKey, settings);
        await File.WriteAllTextAsync(manifestFile.FullName, JsonSerializer.Serialize(manifest, ManifestJsonOptions), new UTF8Encoding(false), cancel);

        await using var pipe = new NamedPipeServerStream(pipeName, PipeDirection.In, 1, PipeTransmissionMode.Byte, PipeOptions.Asynchronous);
        using var process = StartWorker(pipeName, manifestFile.FullName);
        var waitForExitTask = process.WaitForExitAsync(cancel);

        try
        {
            var waitForConnectionTask = pipe.WaitForConnectionAsync(cancel);
            var connectedTask = await Task.WhenAny(waitForConnectionTask, waitForExitTask);
            if (connectedTask == waitForExitTask)
            {
                await waitForExitTask;
                throw new InvalidOperationException($"外置构建程序在连接管道前退出。退出码: {process.ExitCode}");
            }

            await waitForConnectionTask;
            NavmeshBuildResponse? response = null;
            while (response == null)
            {
                var message = await ReadBuildMessage(pipe, cancel);

                switch (message.Type)
                {
                    case "progress":
                        if (message.Progress != null)
                        {
                            if (updateLoadProgress)
                                Interlocked.Exchange(ref loadTaskProgress, Math.Clamp((float)message.Progress.Progress, 0, 0.99f));
                            onProgress?.Invoke(message.Progress.Progress);
                        }
                        break;
                    case "final":
                        response = message.Response ?? throw new InvalidOperationException("外置构建程序最终结果为空");
                        break;
                    default:
                        throw new InvalidOperationException($"外置构建程序返回未知消息: {message.Type}");
                }
            }

            if (!response.Success)
                throw new InvalidOperationException($"外置构建失败: {response.Error}");

            await waitForExitTask;
            if (process.ExitCode != 0)
                throw new InvalidOperationException($"外置构建程序退出码异常: {process.ExitCode}");

            if (string.IsNullOrWhiteSpace(response.RawNavmeshPath) || !File.Exists(response.RawNavmeshPath))
                throw new InvalidOperationException("外置构建没有生成导航网格文件");

            if (updateLoadProgress)
                Interlocked.Exchange(ref loadTaskProgress, 0.99f);
            Log($"外置构建完成。耗时: {response.DurationMs:f1} 毫秒 文件: {response.RawNavmeshPath}");
            return new(response.RawNavmeshPath);
        }
        finally
        {
            if (!process.HasExited)
            {
                try
                {
                    process.Kill(true);
                }
                catch (InvalidOperationException)
                {
                    // 已退出
                }
            }

            TryDelete(sceneFile.FullName);
            TryDelete(manifestFile.FullName);
        }
    }

    internal async Task<Navmesh> BuildExternalNavmesh
    (
        string                                                  cacheKey,
        BuildScene                                              scene,
        vnavmesh.Common.Navigation.Mesh.Build.NavmeshBuildSettings settings,
        int                                                     customizationVersion,
        string                                                  buildSignature,
        CancellationToken                                       cancel,
        Action<double>?                                         onProgress = null
    )
    {
        var rawFile = await RunExternalBuild(cacheKey, scene, settings, cancel, false, onProgress);

        try
        {
            await using var stream = new FileStream(rawFile.FullName, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 22, FileOptions.SequentialScan | FileOptions.Asynchronous);
            using var reader = new BinaryReader(stream, Encoding.UTF8, true);
            var (mesh, _, _) = Navmesh.Deserialize(reader, customizationVersion, buildSignature);
            return mesh;
        }
        finally
        {
            TryDelete(rawFile.FullName);
        }
    }

    private Process StartWorker(string pipeName, string manifestPath)
    {
        var workerPath = Path.Combine(paths.PluginDirectory.FullName, "vnavmesh.Worker.exe");
        if (!File.Exists(workerPath))
            throw new FileNotFoundException("找不到外置导航网格构建程序", workerPath);

        var startInfo = new ProcessStartInfo
        {
            FileName               = workerPath,
            WorkingDirectory       = paths.PluginDirectory.FullName,
            UseShellExecute        = false,
            CreateNoWindow         = true,
            RedirectStandardError  = true,
            RedirectStandardOutput = true
        };
        startInfo.ArgumentList.Add("--pipe");
        startInfo.ArgumentList.Add(pipeName);
        startInfo.ArgumentList.Add("--manifest");
        startInfo.ArgumentList.Add(manifestPath);

        var process = Process.Start(startInfo) ?? throw new InvalidOperationException("无法启动外置导航网格构建程序");
        _ = process.StandardOutput.ReadToEndAsync().ContinueWith(t => LogWorkerOutput("stdout", t), TaskScheduler.Default);
        _ = process.StandardError.ReadToEndAsync().ContinueWith(t => LogWorkerOutput("stderr", t), TaskScheduler.Default);
        return process;
    }

    private static async Task<NavmeshBuildMessage> ReadBuildMessage(Stream stream, CancellationToken cancel)
    {
        var lengthBuffer = new byte[sizeof(int)];
        await ReadExactlyOrThrow(stream, lengthBuffer, cancel);

        var length = BinaryPrimitives.ReadInt32LittleEndian(lengthBuffer);
        if (length is <= 0 or > 16 * 1024 * 1024)
            throw new InvalidOperationException($"外置构建程序返回消息长度异常: {length}");

        var payload = new byte[length];
        await ReadExactlyOrThrow(stream, payload, cancel);

        return JsonSerializer.Deserialize<NavmeshBuildMessage>(payload, ManifestJsonOptions) ?? throw new InvalidOperationException("外置构建程序返回消息无效");
    }

    private static async Task ReadExactlyOrThrow(Stream stream, byte[] buffer, CancellationToken cancel)
    {
        var offset = 0;
        while (offset < buffer.Length)
        {
            var read = await stream.ReadAsync(buffer.AsMemory(offset), cancel);
            if (read == 0)
                throw new InvalidOperationException("外置构建程序返回消息不完整");

            offset += read;
        }
    }

    private static void LogWorkerOutput(string streamName, Task<string> outputTask)
    {
        if (!outputTask.IsCompletedSuccessfully || string.IsNullOrWhiteSpace(outputTask.Result))
            return;

        Log($"worker {streamName}: {outputTask.Result.Trim()}");
    }

    private static void TryDelete(string path)
    {
        try
        {
            if (File.Exists(path))
                File.Delete(path);
        }
        catch
        {
            // 临时文件清理失败不影响运行
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

    private sealed record BuildSnapshot
    (
        BuildScene                                                       Scene,
        vnavmesh.Common.Navigation.Mesh.Build.NavmeshBuildSettings       Settings,
        string                                                           BuildSignature
    );

    #region 常量

    private const float PRUNE_SEED_HALF_EXTENT_XZ          = 8.0f;
    private const float PRUNE_SEED_HALF_EXTENT_Y           = 16.0f;
    private const float PRUNE_SEED_MAX_HORIZONTAL_DISTANCE = 8.0f;
    private const float PRUNE_SEED_MAX_VERTICAL_DISTANCE   = 12.0f;

    #endregion
}
