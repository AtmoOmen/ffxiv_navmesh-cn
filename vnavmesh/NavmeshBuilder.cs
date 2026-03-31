using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.Extras.Jumplink;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;

namespace Navmesh;

// utility for building a navmesh from scene data
// individual tiles can be built concurrently
public class NavmeshBuilder
{
    public record struct Intermediates(RcHeightfield SolidHeightfield, RcCompactHeightfield CompactHeightfield, RcContourSet ContourSet, RcPolyMesh PolyMesh, RcPolyMeshDetail? DetailMesh);

    public sealed class BuildPhaseSummary
    {
        public required string Name { get; init; }
        public required long TotalTicks { get; init; }
        public required long AverageTicks { get; init; }
        public required long MaxTicks { get; init; }
        public required int SlowestTileX { get; init; }
        public required int SlowestTileZ { get; init; }
    }

    public sealed class SlowTileSummary
    {
        public required int TileX { get; init; }
        public required int TileZ { get; init; }
        public required long TotalTicks { get; init; }
        public required int GeometryInstanceCount { get; init; }
        public required int TerrainInstanceCount { get; init; }
        public required int PolyCount { get; init; }
        public required int VertCount { get; init; }
        public required int DetailTriCount { get; init; }
    }

    public sealed class BuildTelemetrySummary
    {
        public required long ParallelTicks { get; init; }
        public required IReadOnlyList<BuildPhaseSummary> Phases { get; init; }
        public required IReadOnlyList<SlowTileSummary> SlowTiles { get; init; }
    }

    private enum BuildPhase
    {
        RasterizeGeometry,
        RasterizeTerrain,
        BuildVolumeColumn,
        BuildJumpLinks,
        CreateDetourData,
        RecastRasterize,
        RecastBuildCompactHeightfield,
        RecastErodeArea,
        RecastBuildDistanceField,
        RecastBuildRegions,
        RecastBuildContours,
        RecastBuildPolyMesh,
        RecastBuildPolyMeshDetail,
        Count
    }

    private sealed class BuildThreadScratch
    {
        public long[] PhaseTicks { get; } = new long[(int)BuildPhase.Count];
        public Voxelizer[]? VolumeScratch;
        public Voxelizer[]? VolumeChain;

        public void Reset()
        {
            Array.Clear(PhaseTicks, 0, PhaseTicks.Length);
        }
    }

    private sealed class TileBuildInput
    {
        public required (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] GeometryInstances { get; init; }
        public required (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] TerrainInstances { get; init; }
    }

    private sealed class TileBuildResult
    {
        public required int TileX { get; init; }
        public required int TileZ { get; init; }
        public required long TotalTicks { get; init; }
        public required long[] PhaseTicks { get; init; }
        public required int GeometryInstanceCount { get; init; }
        public required int TerrainInstanceCount { get; init; }
        public required int PolyCount { get; init; }
        public required int VertCount { get; init; }
        public required int DetailTriCount { get; init; }
        public DtMeshData? MeshData;
        public VoxelMap.RootColumnBuildResult? VolumeColumn;
        public RcBuilderResult? DebugResult;
    }

    private static readonly string[] _phaseNames =
    [
        "普通几何光栅化",
        "地形与平面光栅化",
        "飞行体积列构建",
        "跳边生成",
        "Detour 数据生成",
        "Recast: 光栅化",
        "Recast: 紧凑高度场",
        "Recast: 可行走区域腐蚀",
        "Recast: 距离场",
        "Recast: 区域构建",
        "Recast: 轮廓构建",
        "Recast: 多边形网格",
        "Recast: 细节网格"
    ];

    public NavmeshSettings Settings;
    public SceneExtractor Scene;
    public Vector3 BoundsMin;
    public Vector3 BoundsMax;
    public int NumTilesX;
    public int NumTilesZ;
    public bool Flyable;
    public string BuildSignature;
    public Navmesh Navmesh; // should not be accessed while building tiles
    public BuildTelemetrySummary? LastBuildTelemetry { get; private set; }

    private readonly NavmeshCustomization _customization;
    private readonly TileBuildInput[] _tileInputs;
    private readonly ThreadLocal<BuildThreadScratch> _threadScratch = new(() => new(), true);
    private readonly float _tileWidthWorld;
    private readonly float _tileHeightWorld;
    private readonly float _invTileWidthWorld;
    private readonly float _invTileHeightWorld;

    private int _walkableClimbVoxels;
    private int _walkableHeightVoxels;
    private int _walkableRadiusVoxels;
    private float _walkableNormalThreshold;
    private int _borderSizeVoxels;
    private float _borderSizeWorld;
    private int _tileSizeXVoxels;
    private int _tileSizeZVoxels;
    private int _voxelizerNumX = 1;
    private int _voxelizerNumY = 1;
    private int _voxelizerNumZ = 1;

    public NavmeshBuilder(SceneDefinition scene, NavmeshCustomization customization)
    {
        Settings = customization.GetBuildSettings(scene);

        Flyable = customization.IsFlyingSupported(scene);
        BuildSignature = Settings.BuildSignature(Flyable);
        _customization = customization;

        var extractTimer = Timer.Create();
        Scene = new(scene);
        customization.CustomizeScene(Scene);
        var extractDuration = extractTimer.Value();

        BoundsMin = new(-1024);
        BoundsMax = new(1024);
        NumTilesX = NumTilesZ = Settings.NumTiles[0];
        Service.Log.Debug($"[NavmeshBuilder] 开始构建 {NumTilesX}x{NumTilesZ} 路网，自定义 = {customization.GetType()} v{customization.Version}");
        Service.Log.Debug($"[NavmeshBuilder] 场景提取耗时 {extractDuration.TotalMilliseconds:f1} ms");

        var navmeshParams = new DtNavMeshParams
        {
            orig = BoundsMin.SystemToRecast(),
            tileWidth = (BoundsMax.X - BoundsMin.X) / NumTilesX,
            tileHeight = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ,
            maxTiles = NumTilesX * NumTilesZ,
            maxPolys = 1 << DtNavMesh.DT_POLY_BITS
        };

        _tileWidthWorld = navmeshParams.tileWidth;
        _tileHeightWorld = navmeshParams.tileHeight;
        _invTileWidthWorld = 1.0f / _tileWidthWorld;
        _invTileHeightWorld = 1.0f / _tileHeightWorld;

        var navmesh = new DtNavMesh(navmeshParams, Settings.PolyMaxVerts);
        var volume = Flyable ? new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles) : null;
        Navmesh = new(customization.Version, BuildSignature, false, navmesh, volume);

        _walkableClimbVoxels = (int)MathF.Floor(Settings.AgentMaxClimb / Settings.CellHeight);
        _walkableHeightVoxels = (int)MathF.Ceiling(Settings.AgentHeight / Settings.CellHeight);
        _walkableRadiusVoxels = (int)MathF.Ceiling(Settings.AgentRadius / Settings.CellSize);
        _walkableNormalThreshold = Settings.AgentMaxSlopeDeg.Degrees().Cos();
        _borderSizeVoxels = 3 + _walkableRadiusVoxels;
        _borderSizeWorld = _borderSizeVoxels * Settings.CellSize;
        _tileSizeXVoxels = (int)MathF.Ceiling(navmeshParams.tileWidth / Settings.CellSize) + 2 * _borderSizeVoxels;
        _tileSizeZVoxels = (int)MathF.Ceiling(navmeshParams.tileHeight / Settings.CellSize) + 2 * _borderSizeVoxels;
        if (volume != null)
        {
            _voxelizerNumY = Settings.NumTiles[0];
            for (int i = 1; i < Settings.NumTiles.Length; ++i)
            {
                var n = Settings.NumTiles[i];
                _voxelizerNumX *= n;
                _voxelizerNumY *= n;
                _voxelizerNumZ *= n;
            }
        }

        var bucketTimer = Timer.Create();
        _tileInputs = BucketTileInputs();
        Service.Log.Debug($"[NavmeshBuilder] 瓦片分桶耗时 {bucketTimer.Value().TotalMilliseconds:f1} ms");
    }

    public void Build(Action? onTileFinished = null)
    {
        var builtTiles = BuildTileResults(false, onTileFinished);
        MergeBuiltTiles(builtTiles, false);
    }

    public List<RcBuilderResult> BuildTiles(Action? onTileFinished = null)
    {
        var builtTiles = BuildTileResults(true, onTileFinished);
        return MergeBuiltTiles(builtTiles, true);
    }

    private TileBuildInput[] BucketTileInputs()
    {
        int tileCount = NumTilesX * NumTilesZ;
        var geometryBuckets = new List<(SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)>?[tileCount];
        var terrainBuckets = new List<(SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)>?[tileCount];

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                GetTileRange(instance.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
                for (int z = minZ; z <= maxZ; ++z)
                {
                    var rowBase = z * NumTilesX;
                    for (int x = minX; x <= maxX; ++x)
                    {
                        var index = rowBase + x;
                        if ((mesh.MeshType & (SceneExtractor.MeshType.FileMesh | SceneExtractor.MeshType.CylinderMesh | SceneExtractor.MeshType.AnalyticShape)) != 0)
                            (geometryBuckets[index] ??= []).Add((mesh, instance));
                        else if ((mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0)
                            (terrainBuckets[index] ??= []).Add((mesh, instance));
                    }
                }
            }
        }

        var result = new TileBuildInput[tileCount];
        for (int i = 0; i < tileCount; ++i)
        {
            result[i] = new()
            {
                GeometryInstances = geometryBuckets[i]?.ToArray() ?? [],
                TerrainInstances = terrainBuckets[i]?.ToArray() ?? []
            };
        }
        return result;
    }

    private TileBuildResult[] BuildTileResults(bool captureIntermediates, Action? onTileFinished)
    {
        int tileCount = NumTilesX * NumTilesZ;
        int threadCount = ResolveThreadCount();
        var builtTiles = new TileBuildResult[tileCount];
        var buildTimer = Timer.Create();

        Parallel.For(0, tileCount, new ParallelOptions { MaxDegreeOfParallelism = threadCount }, tileIndex =>
        {
            int x = tileIndex % NumTilesX;
            int z = tileIndex / NumTilesX;
            builtTiles[tileIndex] = BuildTileCore(x, z, _tileInputs[tileIndex], captureIntermediates);
            onTileFinished?.Invoke();
        });

        var parallelDuration = buildTimer.Value();
        LastBuildTelemetry = SummarizeBuildTelemetry(builtTiles, parallelDuration);
        Service.Log.Debug($"[NavmeshBuilder] 并行瓦片构建耗时 {parallelDuration.TotalMilliseconds:f1} ms，线程数 = {threadCount}");
        LogBuildTelemetry(LastBuildTelemetry);
        return builtTiles;
    }

    private List<RcBuilderResult> MergeBuiltTiles(IReadOnlyList<TileBuildResult> builtTiles, bool collectIntermediates)
    {
        var mergeTimer = Timer.Create();
        List<RcBuilderResult> debugResults = collectIntermediates ? new(builtTiles.Count) : [];

        for (int tileIndex = 0; tileIndex < builtTiles.Count; ++tileIndex)
        {
            var built = builtTiles[tileIndex];
            if (built.MeshData != null)
                Navmesh.Mesh.AddTile(built.MeshData, 0, 0);

            if (Navmesh.Volume != null && built.VolumeColumn != null)
                MergeTileColumn(Navmesh.Volume, tileIndex % NumTilesX, tileIndex / NumTilesX, built.VolumeColumn);

            if (collectIntermediates && built.DebugResult != null)
                debugResults.Add(built.DebugResult);
        }

        Service.Log.Debug($"[NavmeshBuilder] 结果合并耗时 {mergeTimer.Value().TotalMilliseconds:f1} ms");
        return debugResults;
    }

    private static void MergeTileColumn(VoxelMap parent, int x, int z, VoxelMap.RootColumnBuildResult column)
    {
        var shift = parent.RootTile.Subdivision.Count;
        int ny = parent.Levels[0].NumCellsY;
        int parentIndex = parent.Levels[0].VoxelToIndex(x, 0, z);
        for (int y = 0; y < ny; ++y)
        {
            var contents = column.Contents[y];
            if ((contents & VoxelMap.VoxelOccupiedBit) == 0)
                continue;

            if ((contents & VoxelMap.VoxelIdMask) != VoxelMap.VoxelIdMask)
                contents += (ushort)shift;

            parent.RootTile.Contents[parentIndex + y] = contents;
        }
        parent.RootTile.Subdivision.AddRange(column.Subdivision);
    }

    private TileBuildResult BuildTileCore(int x, int z, TileBuildInput input, bool captureIntermediates)
    {
        var scratch = _threadScratch.Value!;
        scratch.Reset();
        var totalStart = Stopwatch.GetTimestamp();
        var telemetry = new RcContext();

        var tileBoundsMin = new Vector3(BoundsMin.X + x * _tileWidthWorld, BoundsMin.Y, BoundsMin.Z + z * _tileHeightWorld);
        var tileBoundsMax = new Vector3(tileBoundsMin.X + _tileWidthWorld, BoundsMax.Y, tileBoundsMin.Z + _tileHeightWorld);
        tileBoundsMin.X -= _borderSizeWorld;
        tileBoundsMin.Z -= _borderSizeWorld;
        tileBoundsMax.X += _borderSizeWorld;
        tileBoundsMax.Z += _borderSizeWorld;

        var shf = new RcHeightfield(_tileSizeXVoxels, _tileSizeZVoxels, tileBoundsMin.SystemToRecast(), tileBoundsMax.SystemToRecast(), Settings.CellSize, Settings.CellHeight, _borderSizeVoxels);
        var vox = Navmesh.Volume != null ? new Voxelizer(_voxelizerNumX, _voxelizerNumY, _voxelizerNumZ) : null;
        var rasterizer = new NavmeshRasterizer(shf, _walkableNormalThreshold, _walkableClimbVoxels, _walkableHeightVoxels, Settings.Filtering.HasFlag(NavmeshSettings.Filter.Interiors), vox, telemetry);

        var phaseStart = Stopwatch.GetTimestamp();
        rasterizer.Rasterize(input.GeometryInstances, SceneExtractor.MeshType.All, true, true);
        scratch.PhaseTicks[(int)BuildPhase.RasterizeGeometry] += ElapsedTimeSpanTicks(phaseStart);

        if (input.TerrainInstances.Length > 0)
        {
            phaseStart = Stopwatch.GetTimestamp();
            rasterizer.Rasterize(input.TerrainInstances, SceneExtractor.MeshType.All, false, true);
            scratch.PhaseTicks[(int)BuildPhase.RasterizeTerrain] += ElapsedTimeSpanTicks(phaseStart);
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LowHangingObstacles))
            RcFilters.FilterLowHangingWalkableObstacles(telemetry, _walkableClimbVoxels, shf);

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LedgeSpans))
            RcFilters.FilterLedgeSpans(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.WalkableLowHeightSpans))
            RcFilters.FilterWalkableLowHeightSpans(telemetry, _walkableHeightVoxels, shf);

        var chf = RcCompacts.BuildCompactHeightfield(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);
        RcAreas.ErodeWalkableArea(telemetry, _walkableRadiusVoxels, chf);

        var regionMinArea = (int)(Settings.RegionMinSize * Settings.RegionMinSize);
        var regionMergeArea = (int)(Settings.RegionMergeSize * Settings.RegionMergeSize);
        if (Settings.Partitioning == RcPartition.WATERSHED)
        {
            RcRegions.BuildDistanceField(telemetry, chf);
            RcRegions.BuildRegions(telemetry, chf, regionMinArea, regionMergeArea);
        }
        else if (Settings.Partitioning == RcPartition.MONOTONE)
        {
            RcRegions.BuildRegionsMonotone(telemetry, chf, regionMinArea, regionMergeArea);
        }
        else
        {
            RcRegions.BuildLayerRegions(telemetry, chf, regionMinArea);
        }

        var polyMaxEdgeLenVoxels = (int)(Settings.PolyMaxEdgeLen / Settings.CellSize);
        var cset = RcContours.BuildContours(telemetry, chf, Settings.PolyMaxSimplificationError, polyMaxEdgeLenVoxels, RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES);

        var pmesh = RcMeshs.BuildPolyMesh(telemetry, cset, Settings.PolyMaxVerts);
        for (int i = 0; i < pmesh.npolys; ++i)
            pmesh.flags[i] = 1;

        var detailSampleDist = Settings.DetailSampleDist < 0.9f ? 0 : Settings.CellSize * Settings.DetailSampleDist;
        var detailSampleMaxError = Settings.CellHeight * Settings.DetailMaxSampleError;
        RcPolyMeshDetail? dmesh = RcMeshDetails.BuildPolyMeshDetail(telemetry, pmesh, chf, detailSampleDist, detailSampleMaxError);

        var navmeshConfig = new DtNavMeshCreateParams
        {
            verts = pmesh.verts,
            vertCount = pmesh.nverts,
            polys = pmesh.polys,
            polyFlags = pmesh.flags,
            polyAreas = pmesh.areas,
            polyCount = pmesh.npolys,
            nvp = pmesh.nvp,

            detailMeshes = dmesh?.meshes,
            detailVerts = dmesh?.verts,
            detailVertsCount = dmesh?.nverts ?? 0,
            detailTris = dmesh?.tris,
            detailTriCount = dmesh?.ntris ?? 0,

            tileX = x,
            tileZ = z,
            tileLayer = 0,
            bmin = pmesh.bmin,
            bmax = pmesh.bmax,

            walkableHeight = Settings.AgentHeight,
            walkableRadius = Settings.AgentRadius,
            walkableClimb = Settings.AgentMaxClimb,
            cs = Settings.CellSize,
            ch = Settings.CellHeight,

            buildBvTree = true,
        };
        _customization.CustomizeSettings(navmeshConfig);

        RcBuilderResult? builderResult = null;
        JumpLinkBuilder? jumpLinkBuilder = null;
        if (captureIntermediates || Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks)
        {
            builderResult = new RcBuilderResult(x, z, shf, chf, cset, pmesh, dmesh, telemetry);
            if (Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks)
                jumpLinkBuilder = new([builderResult]);
        }

        void addConnections(List<JumpLink> links)
        {
            foreach (var link in links)
            {
                RcVec3f prev = default;
                for (var i = 0; i < link.startSamples.Length; i++)
                {
                    var p = link.startSamples[i].p;
                    var q = link.endSamples[i].p;
                    if (i == 0 || RcVecUtils.Dist2D(prev, p) > Settings.AgentRadius)
                    {
                        navmeshConfig.AddOffMeshConnection(p.RecastToSystem(), q.RecastToSystem(), Settings.AgentRadius, false);
                        prev = p;
                    }
                }
            }
        }

        if ((Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks) && jumpLinkBuilder != null)
        {
            phaseStart = Stopwatch.GetTimestamp();
            if (Settings.GenerateEdgeClimbLinks)
            {
                var cfg = new JumpLinkBuilderConfig(
                    Settings.CellSize,
                    Settings.CellHeight,
                    Settings.AgentRadius,
                    Settings.AgentHeight,
                    Settings.AgentMaxClimb,
                    Settings.GroundTolerance,
                    -Settings.AgentRadius * 0.2f,
                    Settings.CellSize + 2 * Settings.AgentRadius + Settings.ClimbDownDistance,
                    -Settings.ClimbDownMaxHeight,
                    -Settings.ClimbDownMinHeight,
                    0
                );
                addConnections(jumpLinkBuilder.Build(cfg, JumpLinkType.EDGE_CLIMB_DOWN));
            }

            if (Settings.GenerateEdgeJumpLinks)
            {
                var cfg = new JumpLinkBuilderConfig(
                    Settings.CellSize,
                    Settings.CellHeight,
                    Settings.AgentRadius,
                    Settings.AgentHeight,
                    Settings.AgentMaxClimb,
                    Settings.GroundTolerance,
                    -Settings.AgentRadius * 0.2f,
                    Settings.EdgeJumpEndDistance,
                    -Settings.EdgeJumpMaxDrop,
                    -Settings.EdgeJumpMinDrop,
                    Settings.EdgeJumpHeight
                );
                addConnections(jumpLinkBuilder.Build(cfg, JumpLinkType.EDGE_JUMP));
            }
            scratch.PhaseTicks[(int)BuildPhase.BuildJumpLinks] += ElapsedTimeSpanTicks(phaseStart);
        }

        VoxelMap.RootColumnBuildResult? volumeColumn = null;
        if (Navmesh.Volume != null && vox != null)
        {
            EnsureVolumeScratch(scratch);
            phaseStart = Stopwatch.GetTimestamp();
            volumeColumn = Navmesh.Volume.BuildRootColumn(vox, x, z, scratch.VolumeScratch!, scratch.VolumeChain!);
            scratch.PhaseTicks[(int)BuildPhase.BuildVolumeColumn] += ElapsedTimeSpanTicks(phaseStart);
        }

        phaseStart = Stopwatch.GetTimestamp();
        var navmeshData = DtNavMeshBuilder.CreateNavMeshData(navmeshConfig);
        scratch.PhaseTicks[(int)BuildPhase.CreateDetourData] += ElapsedTimeSpanTicks(phaseStart);

        AccumulateRecastTelemetry(telemetry.ToList(), scratch.PhaseTicks);

        var phaseTicks = new long[scratch.PhaseTicks.Length];
        Array.Copy(scratch.PhaseTicks, phaseTicks, phaseTicks.Length);

        var totalTicks = ElapsedTimeSpanTicks(totalStart);
        Service.Log.Debug($"[NavmeshBuilder] 瓦片 {x}x{z} 构建耗时 {TicksToMilliseconds(totalTicks):f1} ms");
        return new()
        {
            TileX = x,
            TileZ = z,
            TotalTicks = totalTicks,
            PhaseTicks = phaseTicks,
            GeometryInstanceCount = input.GeometryInstances.Length,
            TerrainInstanceCount = input.TerrainInstances.Length,
            PolyCount = pmesh.npolys,
            VertCount = pmesh.nverts,
            DetailTriCount = dmesh?.ntris ?? 0,
            MeshData = navmeshData,
            VolumeColumn = volumeColumn,
            DebugResult = captureIntermediates ? builderResult : null
        };
    }

    private static void AccumulateRecastTelemetry(IReadOnlyList<RcTelemetryTick> telemetry, long[] phaseTicks)
    {
        foreach (var tick in telemetry)
        {
            if (tick.Key.StartsWith("RC_TIMER_RASTERIZE_", StringComparison.Ordinal))
            {
                phaseTicks[(int)BuildPhase.RecastRasterize] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD))
            {
                phaseTicks[(int)BuildPhase.RecastBuildCompactHeightfield] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_ERODE_AREA))
            {
                phaseTicks[(int)BuildPhase.RecastErodeArea] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_DISTANCEFIELD))
            {
                phaseTicks[(int)BuildPhase.RecastBuildDistanceField] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_REGIONS))
            {
                phaseTicks[(int)BuildPhase.RecastBuildRegions] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_CONTOURS))
            {
                phaseTicks[(int)BuildPhase.RecastBuildContours] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_POLYMESH))
            {
                phaseTicks[(int)BuildPhase.RecastBuildPolyMesh] += tick.Ticks;
            }
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_POLYMESHDETAIL))
            {
                phaseTicks[(int)BuildPhase.RecastBuildPolyMeshDetail] += tick.Ticks;
            }
        }
    }

    private static BuildTelemetrySummary SummarizeBuildTelemetry(IReadOnlyList<TileBuildResult> builtTiles, TimeSpan parallelDuration)
    {
        List<BuildPhaseSummary> phases = [];
        int tileCount = builtTiles.Count;
        for (int phaseIndex = 0; phaseIndex < (int)BuildPhase.Count; ++phaseIndex)
        {
            long totalTicks = 0;
            long maxTicks = 0;
            int slowestTileX = -1;
            int slowestTileZ = -1;
            foreach (var tile in builtTiles)
            {
                var ticks = tile.PhaseTicks[phaseIndex];
                totalTicks += ticks;
                if (ticks > maxTicks)
                {
                    maxTicks = ticks;
                    slowestTileX = tile.TileX;
                    slowestTileZ = tile.TileZ;
                }
            }

            if (totalTicks == 0)
                continue;

            phases.Add(new()
            {
                Name = _phaseNames[phaseIndex],
                TotalTicks = totalTicks,
                AverageTicks = totalTicks / Math.Max(tileCount, 1),
                MaxTicks = maxTicks,
                SlowestTileX = slowestTileX,
                SlowestTileZ = slowestTileZ
            });
        }

        List<SlowTileSummary> slowTiles = [];
        foreach (var tile in builtTiles.OrderByDescending(t => t.TotalTicks).Take(5))
        {
            slowTiles.Add(new()
            {
                TileX = tile.TileX,
                TileZ = tile.TileZ,
                TotalTicks = tile.TotalTicks,
                GeometryInstanceCount = tile.GeometryInstanceCount,
                TerrainInstanceCount = tile.TerrainInstanceCount,
                PolyCount = tile.PolyCount,
                VertCount = tile.VertCount,
                DetailTriCount = tile.DetailTriCount
            });
        }

        return new()
        {
            ParallelTicks = parallelDuration.Ticks,
            Phases = phases,
            SlowTiles = slowTiles
        };
    }

    private static void LogBuildTelemetry(BuildTelemetrySummary telemetry)
    {
        Service.Log.Debug("[NavmeshBuilder] 阶段统计（总计 / 单瓦片均值 / 最慢瓦片）");
        foreach (var phase in telemetry.Phases)
        {
            Service.Log.Debug($"[NavmeshBuilder] {phase.Name}: 总计 {TicksToMilliseconds(phase.TotalTicks):f1} ms，均值 {TicksToMilliseconds(phase.AverageTicks):f2} ms，最慢瓦片 {phase.SlowestTileX}x{phase.SlowestTileZ} = {TicksToMilliseconds(phase.MaxTicks):f1} ms");
        }

        if (telemetry.SlowTiles.Count > 0)
        {
            var slowestTiles = string.Join("，", telemetry.SlowTiles.Select(tile => $"{tile.TileX}x{tile.TileZ} = {TicksToMilliseconds(tile.TotalTicks):f1} ms"));
            Service.Log.Debug($"[NavmeshBuilder] 最慢瓦片 Top {telemetry.SlowTiles.Count}: {slowestTiles}");
        }
        foreach (var tile in telemetry.SlowTiles)
        {
            Service.Log.Debug($"[NavmeshBuilder] 慢瓦片 {tile.TileX}x{tile.TileZ}: 几何实例 {tile.GeometryInstanceCount}，地形实例 {tile.TerrainInstanceCount}，Poly {tile.PolyCount}，Vert {tile.VertCount}，DetailTri {tile.DetailTriCount}");
        }
    }

    private void EnsureVolumeScratch(BuildThreadScratch scratch)
    {
        if (Navmesh.Volume == null || scratch.VolumeScratch != null)
            return;

        var levels = Navmesh.Volume.Levels;
        scratch.VolumeScratch = new Voxelizer[levels.Length - 1];
        scratch.VolumeChain = new Voxelizer[levels.Length];

        int nx = _voxelizerNumX;
        int ny = _voxelizerNumY;
        int nz = _voxelizerNumZ;
        for (int i = levels.Length - 1; i > 0; --i)
        {
            nx /= levels[i].NumCellsX;
            ny /= levels[i].NumCellsY;
            nz /= levels[i].NumCellsZ;
            scratch.VolumeScratch[i - 1] = new Voxelizer(nx, ny, nz, true);
        }
    }

    private int ResolveThreadCount()
    {
        var maxThreads = Environment.ProcessorCount;
        var wantedThreads = Service.Config.BuildMaxCores;
        int threadCount = wantedThreads <= 0 ? maxThreads + wantedThreads : wantedThreads;
        return Math.Clamp(threadCount, 1, maxThreads);
    }

    private void GetTileRange(AABB bounds, out int minX, out int maxX, out int minZ, out int maxZ)
    {
        minX = (int)MathF.Floor((bounds.Min.X - _borderSizeWorld - BoundsMin.X) * _invTileWidthWorld);
        maxX = (int)MathF.Floor((bounds.Max.X + _borderSizeWorld - BoundsMin.X) * _invTileWidthWorld);
        minZ = (int)MathF.Floor((bounds.Min.Z - _borderSizeWorld - BoundsMin.Z) * _invTileHeightWorld);
        maxZ = (int)MathF.Floor((bounds.Max.Z + _borderSizeWorld - BoundsMin.Z) * _invTileHeightWorld);

        minX = Math.Clamp(minX, 0, NumTilesX - 1);
        maxX = Math.Clamp(maxX, 0, NumTilesX - 1);
        minZ = Math.Clamp(minZ, 0, NumTilesZ - 1);
        maxZ = Math.Clamp(maxZ, 0, NumTilesZ - 1);
    }

    private static long ElapsedTimeSpanTicks(long startTimestamp)
        => (long)((Stopwatch.GetTimestamp() - startTimestamp) * (double)TimeSpan.TicksPerSecond / Stopwatch.Frequency);

    private static double TicksToMilliseconds(long ticks)
        => ticks / (double)TimeSpan.TicksPerMillisecond;
}
