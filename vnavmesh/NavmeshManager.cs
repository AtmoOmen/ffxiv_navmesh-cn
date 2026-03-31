using Dalamud.Game.ClientState.Conditions;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;

namespace Navmesh;

// manager that loads navmesh matching current zone and performs async pathfinding queries
public sealed class NavmeshManager : IDisposable
{
	private sealed record BuildNavmeshResult(Navmesh Navmesh, FileInfo? CacheFile);

	public bool UseRaycasts = true;
	public bool UseStringPulling = true;

	public string CurrentKey { get; private set; } = ""; // unique string representing currently loaded navmesh
	public Navmesh? Navmesh { get; private set; }
	public NavmeshQuery? Query { get; private set; }
	public event Action<Navmesh?, NavmeshQuery?>? OnNavmeshChanged;

	private volatile float _loadTaskProgress = -1;
	public float LoadTaskProgress => _loadTaskProgress; // negative if load task is not running, otherwise in [0, 1] range

	private CancellationTokenSource? _currentCTS; // this is signalled when mesh is unloaded, all pathfinding tasks that use it are then cancelled
	private Task _lastLoadQueryTask; // we limit the concurrency to max 1 running task (otherwise we'd need multiple Query objects, which aren't lightweight); note that each task completes on main thread!

	private int _numActivePathfinds;
	public bool PathfindInProgress => _numActivePathfinds > 0;
	public int NumQueuedPathfindRequests => _numActivePathfinds > 0 ? _numActivePathfinds - 1 : 0;

	private DirectoryInfo _cacheDir;
	private readonly ConcurrentDictionary<string, Task> _cacheWriteTasks = new();

	public unsafe NavmeshManager(DirectoryInfo cacheDir)
	{
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
			if (!Service.Config.AutoLoadNavmesh)
			{
				if (CurrentKey.Length == 0)
					return; // nothing is loaded, and auto-load is forbidden
				curKey = ""; // just unload existing mesh
			}
			Log($"Starting transition from '{CurrentKey}' to '{curKey}'");
			CurrentKey = curKey;
			Reload(true);
			// mesh load is now in progress
		}
	}

	public bool Reload(bool allowLoadFromCache)
	{
		ClearState();
		if (CurrentKey.Length > 0)
		{
			var cts = _currentCTS = new();
			ExecuteWhenIdle(async cancel =>
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

				var snapshotTimer = Timer.Create();
				var (cacheKey, scene) = await Service.Framework.Run(() =>
				{
					var scene = new SceneDefinition();
					scene.FillFromActiveLayout();
					var cacheKey = GetCacheKey(scene);
					return (cacheKey, scene);
				}, cancel);
				Log($"场景快照耗时 {snapshotTimer.Value().TotalMilliseconds:f1} ms");

				Log($"Kicking off build for '{cacheKey}' (reload={allowLoadFromCache})");
				var buildResult = await Task.Run(() => BuildNavmesh(scene, cacheKey, allowLoadFromCache, cancel), cancel);
				var navmesh = buildResult.Navmesh;
				Log($"Mesh loaded: '{cacheKey}'");
				Navmesh = navmesh;
				Query = new(Navmesh);

				var ff = await FloodFill.GetAsync();
				if (ff.TryLookup(scene.TerritoryID, out var points))
					Prune(points);

				OnNavmeshChanged?.Invoke(Navmesh, Query);
				if (buildResult.CacheFile != null)
					QueueCacheWrite(cacheKey, buildResult.CacheFile, navmesh);
			}, cts.Token);
		}
		return true;
	}

	internal void ReplaceMesh(Navmesh mesh)
	{
		Navmesh = mesh;
		Query = new(Navmesh);
		Log($"Mesh replaced");
		OnNavmeshChanged?.Invoke(Navmesh, Query);
	}

	private static bool InCutscene => Service.Condition[ConditionFlag.WatchingCutscene] || Service.Condition[ConditionFlag.OccupiedInCutSceneEvent];

	public Task<List<Vector3>> QueryPath(Vector3 from, Vector3 to, bool flying, float range = 0, CancellationToken externalCancel = default)
	{
		if (_currentCTS == null)
			throw new Exception($"Can't initiate query - navmesh is not loaded");

		// task can be cancelled either by internal request (i.e. when navmesh is reloaded) or external
		var combined = CancellationTokenSource.CreateLinkedTokenSource(_currentCTS.Token, externalCancel);
		++_numActivePathfinds;
		return ExecuteWhenIdle(async cancel =>
		{
			using var autoDisposeCombined = combined;
			using var autoDecrementCounter = new OnDispose(() => --_numActivePathfinds);
			LogInfo($"Kicking off pathfind from {from} to {to}");
			var path = await Task.Run(() =>
			{
				combined.Token.ThrowIfCancellationRequested();
				if (Query == null)
					throw new Exception($"Can't pathfind, navmesh did not build successfully");
				Log($"Executing pathfind from {from} to {to}");
				return flying ? Query.PathfindVolume(from, to, UseRaycasts, UseStringPulling, combined.Token) : Query.PathfindMesh(from, to, UseRaycasts, UseStringPulling, range, combined.Token);
			}, combined.Token);
			Log($"Pathfinding done: {path.Count} waypoints");
			return path;
		}, combined.Token);
	}

	// note: pixelSize should be power-of-2
	public (Vector3 min, Vector3 max) BuildBitmap(Vector3 startingPos, string filename, float pixelSize, AABB? mapBounds = null)
	{
		if (Navmesh == null || Query == null)
			throw new InvalidOperationException($"Can't build bitmap - navmesh creation is in progress");

		bool inBounds(Vector3 vert) => mapBounds is not AABB aabb || vert.X >= aabb.Min.X && vert.Y >= aabb.Min.Y && vert.Z >= aabb.Min.Z && vert.X <= aabb.Max.X && vert.Y <= aabb.Max.Y && vert.Z <= aabb.Max.Z;

		var startPoly = Query.FindNearestMeshPoly(startingPos);
		var reachablePolys = Query.FindReachableMeshPolys(startPoly);

		HashSet<long> polysInbounds = [];

		Vector3 min = new(1024), max = new(-1024);
		foreach (var p in reachablePolys)
		{
			Navmesh.Mesh.GetTileAndPolyByRefUnsafe(p, out var tile, out var poly);
			for (int i = 0; i < poly.vertCount; ++i)
			{
				var v = NavmeshBitmap.GetVertex(tile, poly.verts[i]);
				if (!inBounds(v))
					goto cont;

				min = Vector3.Min(min, v);
				max = Vector3.Max(max, v);
				//Service.Log.Debug($"{p:X}.{i}= {v}");
			}

			polysInbounds.Add(p);

		cont:;
		}
		//Service.Log.Debug($"bounds: {min}-{max}");

		var bitmap = new NavmeshBitmap(min, max, pixelSize);
		foreach (var p in polysInbounds)
		{
			bitmap.RasterizePolygon(Navmesh.Mesh, p);
		}
		bitmap.Save(filename);
		Service.Log.Debug($"Generated nav bitmap '{filename}' @ {startingPos}: {bitmap.MinBounds}-{bitmap.MaxBounds}");
		return (bitmap.MinBounds, bitmap.MaxBounds);
	}

	// if non-empty string is returned, active layout is ready
	private unsafe string GetCurrentKey()
	{
		var layout = LayoutWorld.Instance()->ActiveLayout;
		if (layout == null || layout->InitState != 7 || layout->FestivalStatus is > 0 and < 5)
			return ""; // layout not ready

		var filter = LayoutUtils.FindFilter(layout);
		var filterKey = filter != null ? filter->Key : 0;

		var terrRow = Service.LuminaRow<Lumina.Excel.Sheets.TerritoryType>(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

		// CE always has a festival layer (i hope). the non-festival layout is briefly loaded when entering the zone, which triggers a useless mesh build (which is also expensive because the zone is large)
		if (terrRow?.TerritoryIntendedUse.RowId == 60)
		{
			var fest = layout->ActiveFestivals[0];
			if (fest.Id == 0 && fest.Phase == 0)
				return "";
		}

		var sgs = LayoutUtils.GetZoneSharedGroupsEnabled(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

		return $"{terrRow?.Bg}//{filterKey:X}//{LayoutUtils.FestivalsString(layout->ActiveFestivals)}//{string.Join('.', sgs)}";
	}

	internal static unsafe string GetCacheKey(SceneDefinition scene)
	{
		// note: festivals are active globally, but majority of zones don't have festival-specific layers, so we only want real ones in the cache key
		var layout = LayoutWorld.Instance()->ActiveLayout;
		var filter = LayoutUtils.FindFilter(layout);
		var filterKey = filter != null ? filter->Key : 0;
		var terrId = filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId;
		var terrRow = Service.LuminaRow<Lumina.Excel.Sheets.TerritoryType>(terrId);

		static string numbers<T>(IEnumerable<T> nums) where T : INumber<T> => string.Join('.', nums.Select(n => n.ToString("X", CultureInfo.InvariantCulture)));

		return $"{terrRow?.Bg.ToString().Replace('/', '_')}__{filterKey:X}__{numbers(scene.FestivalLayers)}__{numbers(scene.ZoneSGs)}";
	}

	private void ClearState()
	{
		if (_currentCTS == null)
			return; // already cleared

		var cts = _currentCTS;
		_currentCTS = null;
		cts.Cancel();
		Log("Queueing state clear");
		ExecuteWhenIdle(() =>
		{
			Log("Clearing state");
			_numActivePathfinds = 0;
			cts.Dispose();
			OnNavmeshChanged?.Invoke(null, null);
			Query = null;
			Navmesh = null;
		}, default);
	}

	private BuildNavmeshResult BuildNavmesh(SceneDefinition scene, string cacheKey, bool allowLoadFromCache, CancellationToken cancel)
	{
		var totalTimer = Timer.Create();
		Log($"Build task started: '{cacheKey}'");
		var customization = NavmeshCustomizationRegistry.ForTerritory(scene.TerritoryID);
		Log($"Customization for '{scene.TerritoryID}': {customization.GetType()}");

		var layers = scene.FestivalLayers.ToList();
		var buildSignature = customization.GetBuildSettings(scene).BuildSignature(customization.IsFlyingSupported(scene));

		// try reading from cache
		var cache = new FileInfo($"{_cacheDir.FullName}/{cacheKey}.navmesh");
		if (allowLoadFromCache && cache.Exists)
		{
			try
			{
				var cacheReadTimer = Timer.Create();
				Log($"Loading cache: {cache.FullName}");
				using var stream = cache.OpenRead();
				using var reader = new BinaryReader(stream);
				var cacheResult = Navmesh.Deserialize(reader, customization.Version, buildSignature);
				var mesh = cacheResult.Navmesh;
				Log($"缓存读取耗时 {cacheReadTimer.Value().TotalMilliseconds:f1} ms");
				LogCacheSegment("读取", cacheResult.Telemetry.Mesh);
				LogCacheSegment("读取", cacheResult.Telemetry.Volume);
				if (!mesh.CustomizationApplied)
					customization.CustomizeMesh(mesh, layers);
				Log($"缓存命中，总耗时 {totalTimer.Value().TotalMilliseconds:f1} ms");
				return new(mesh, null);
			}
			catch (Exception ex)
			{
				Log($"Failed to load cache: {ex}");
			}
		}
		cancel.ThrowIfCancellationRequested();

		// cache doesn't exist or can't be used for whatever reason - build navmesh from scratch
		var buildTimer = Timer.Create();
		var builder = new NavmeshBuilder(scene, customization);
		var deltaProgress = 0.99f / (builder.NumTilesX * builder.NumTilesZ);
		builder.Build(() =>
		{
			_loadTaskProgress += deltaProgress;
			cancel.ThrowIfCancellationRequested();
		});
		Log($"冷构建耗时 {buildTimer.Value().TotalMilliseconds:f1} ms");

		customization.CustomizeMesh(builder.Navmesh, layers);
		var runtimeMesh = builder.Navmesh with { CustomizationApplied = true };
		Log($"总构建耗时 {totalTimer.Value().TotalMilliseconds:f1} ms");
		deltaProgress += 0.01f;
		return new(runtimeMesh, cache);
	}

	private void QueueCacheWrite(string cacheKey, FileInfo cache, Navmesh navmesh)
	{
		if (_cacheWriteTasks.TryGetValue(cacheKey, out var existing) && !existing.IsCompleted)
		{
			Log($"后台缓存写入已在进行，跳过重复调度: {cacheKey}");
			return;
		}

		var writeTask = Task.Run(() => WriteCache(cacheKey, cache, navmesh));
		_cacheWriteTasks[cacheKey] = writeTask;
		_ = writeTask.ContinueWith(t =>
		{
			_cacheWriteTasks.TryRemove(cacheKey, out _);
			LogTaskError(t);
		}, CancellationToken.None, TaskContinuationOptions.ExecuteSynchronously, TaskScheduler.Default);
	}

	private void WriteCache(string cacheKey, FileInfo cache, Navmesh navmesh)
	{
		var timer = Timer.Create();
		var tempPath = $"{cache.FullName}.{Environment.ProcessId}.{Environment.CurrentManagedThreadId}.tmp";
		try
		{
			cache.Directory?.Create();
			Service.Log.Debug($"[vnavmesh] 后台写入缓存: {cache.FullName}");
			var serializeTimer = Timer.Create();
			Navmesh.CacheTelemetry telemetry;
			using (var stream = new FileStream(tempPath, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 22, FileOptions.SequentialScan))
			using (var writer = new BinaryWriter(stream))
			{
				telemetry = navmesh.Serialize(writer);
				writer.Flush();
				stream.Flush();
			}
			var serializeDuration = serializeTimer.Value();
			var sizeBytes = new FileInfo(tempPath).Length;

			var replaceTimer = Timer.Create();
			File.Move(tempPath, cache.FullName, true);
			var replaceDuration = replaceTimer.Value();
			LogCacheSegment("写入", telemetry.Mesh);
			LogCacheSegment("写入", telemetry.Volume);
			Log($"后台缓存写入完成 '{cacheKey}'，总耗时 {timer.Value().TotalMilliseconds:f1} ms，序列化 {serializeDuration.TotalMilliseconds:f1} ms，替换 {replaceDuration.TotalMilliseconds:f1} ms，大小 {sizeBytes / 1024.0 / 1024.0:f2} MiB");
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
		_lastLoadQueryTask = Service.Framework.Run(async () =>
		{
			await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			_ = prev.Exception;
			task();
		}, token);
	}

	private void ExecuteWhenIdle(Func<CancellationToken, Task> task, CancellationToken token)
	{
		var prev = _lastLoadQueryTask;
		_lastLoadQueryTask = Service.Framework.Run(async () =>
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
		var prev = _lastLoadQueryTask;
		var res = Service.Framework.Run(async () =>
		{
			await prev.ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			_ = prev.Exception;
			var t = task(token);
			await ((Task)t).ConfigureAwait(ConfigureAwaitOptions.SuppressThrowing | ConfigureAwaitOptions.ContinueOnCapturedContext);
			LogTaskError(t);
			return t.Result;
		}, token);
		_lastLoadQueryTask = res;
		return res;
	}

	private static void Log(string message) => Service.Log.Debug($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");
	private static void LogInfo(string message) => Service.Log.Info($"[NavmeshManager] [{Environment.CurrentManagedThreadId}] {message}");
	private static void LogCacheSegment(string action, Navmesh.CacheSegmentTelemetry segment)
	{
		Log($"缓存段 {SegmentName(segment.Kind)} {action}耗时 {segment.Duration.TotalMilliseconds:f1} ms，压缩后 {FormatMiB(segment.CompressedBytes):f2} MiB，原始 {FormatMiB(segment.UncompressedBytes):f2} MiB");
	}

	private static string SegmentName(Navmesh.CacheSegmentKind kind) => kind switch
	{
		Navmesh.CacheSegmentKind.Mesh => "Mesh",
		Navmesh.CacheSegmentKind.Volume => "Volume",
		_ => kind.ToString()
	};

	private static double FormatMiB(long bytes) => bytes / 1024.0 / 1024.0;
	private static void LogTaskError(Task task)
	{
		if (task.IsFaulted)
			Service.Log.Error($"[NavmeshManager] Task failed with error: {task.Exception}");
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
					if (Navmesh.Mesh.SetPolyFlags(pref, fl & ~Navmesh.FLAG_UNREACHABLE).Failed())
						Log($"failed to set flags for {pref:X}");
				}
				else
				{
					pruneCount++;
					if (Navmesh.Mesh.SetPolyFlags(pref, fl | Navmesh.FLAG_UNREACHABLE).Failed())
						Log($"failed to set flags for {pref:X}");
				}
			}
		}

		Log($"pruned {pruneCount} unreachable polygons");
	}
}
