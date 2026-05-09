using System.Diagnostics;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.Extras.Jumplink;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Models;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Common.Utilities;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Models;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Build;

using static DtDetour;

// utility for building a navmesh from scene data
// individual tiles can be built concurrently
public class NavmeshBuilder
{
    private readonly Config _config;
    private const    float  WorldBoundsMin = -1024f;
    private const    float  WorldBoundsMax = 1024f;

    public record struct Intermediates
    (
        RcHeightfield        SolidHeightfield,
        RcCompactHeightfield CompactHeightfield,
        RcContourSet         ContourSet,
        RcPolyMesh           PolyMesh,
        RcPolyMeshDetail?    DetailMesh
    );

    public sealed class BuildPhaseSummary
    {
        public required string Name              { get; init; }
        public required long   TotalTicks        { get; init; }
        public required long   AverageTicks      { get; init; }
        public required long   MaxTicks          { get; init; }
        public required int    SlowestTileX      { get; init; }
        public required int    SlowestTileZ      { get; init; }
        public required double ShareOfPhaseTicks { get; init; }
    }

    public sealed class SlowTileSummary
    {
        public required int  TileX               { get; init; }
        public required int  TileZ               { get; init; }
        public required long TotalTicks          { get; init; }
        public required int  GeometryJobCount    { get; init; }
        public required int  TerrainJobCount     { get; init; }
        public required int  UniqueJobCount      { get; init; }
        public required int  PrimitiveCount      { get; init; }
        public required int  EstimatedSpanWeight { get; init; }
        public required int  PreCompactSpanCount { get; init; }
        public required int  PolyCount           { get; init; }
        public required int  VertCount           { get; init; }
        public required int  DetailTriCount      { get; init; }
    }

    public sealed class BuildTelemetrySummary
    {
        public required int                              ConfiguredBuildMaxCores { get; init; }
        public required int                              MaxAvailableCores       { get; init; }
        public required int                              ThreadCount             { get; init; }
        public required long                             ParallelTicks           { get; init; }
        public required long                             AggregatedPhaseTicks    { get; init; }
        public required int                              UniqueRasterJobCount    { get; init; }
        public required double                           JobCoverageMultiplier   { get; init; }
        public required long                             PreparedTerrainBytes    { get; init; }
        public required IReadOnlyList<BuildPhaseSummary> Phases                  { get; init; }
        public required IReadOnlyList<SlowTileSummary>   SlowTiles               { get; init; }
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
        public long[]                           PhaseTicks { get; } = new long[(int)BuildPhase.Count];
        public Voxelizer?                       VolumeRoot;
        public NavmeshRasterizer.ScratchBuffers Rasterizer { get; } = new();

        public void Reset()
        {
            Array.Clear(PhaseTicks, 0, PhaseTicks.Length);
            VolumeRoot?.Clear();
        }
    }

    private sealed class TileBuildInput
    {
        public required int GeometryJobStart    { get; init; }
        public required int GeometryJobCount    { get; init; }
        public required int TerrainJobStart     { get; init; }
        public required int TerrainJobCount     { get; init; }
        public required int PrimitiveCount      { get; init; }
        public required int EstimatedSpanWeight { get; init; }
    }

    private sealed class TileBuildResult
    {
        public required int                          TileX               { get; init; }
        public required int                          TileZ               { get; init; }
        public required long                         TotalTicks          { get; init; }
        public required long[]                       PhaseTicks          { get; init; }
        public required int                          GeometryJobCount    { get; init; }
        public required int                          TerrainJobCount     { get; init; }
        public required int                          PrimitiveCount      { get; init; }
        public required int                          UniqueJobCount      { get; init; }
        public required int                          EstimatedSpanWeight { get; init; }
        public required int                          PreCompactSpanCount { get; init; }
        public required int                          PolyCount           { get; init; }
        public required int                          VertCount           { get; init; }
        public required int                          DetailTriCount      { get; init; }
        public required int                          GeneratedClimbLinks { get; init; }
        public required int                          GeneratedJumpLinks  { get; init; }
        public          DtMeshData?                  MeshData;
        public          VolumeRootColumnBuildResult? VolumeColumn;
        public          RcBuilderResult?             DebugResult;
    }

    private static readonly string[] PhaseNames =
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

    public NavmeshSettings        Settings;
    public SceneExtractor         Scene;
    public Vector3                BoundsMin;
    public Vector3                BoundsMax;
    public int                    NumTilesX;
    public int                    NumTilesZ;
    public bool                   Flyable;
    public string                 BuildSignature;
    public Navmesh                Navmesh; // should not be accessed while building tiles
    public BuildTelemetrySummary? LastBuildTelemetry       { get; private set; }
    public long                   TotalEstimatedTileWeight { get; }

    private readonly NavmeshCustomization            _customization;
    private readonly TileBuildInput[]                _tileInputs;
    private readonly int[]                           _tileBuildOrder;
    private readonly RasterJob[]                     _geometryJobs;
    private readonly RasterJob[]                     _terrainJobs;
    private readonly ThreadLocal<BuildThreadScratch> _threadScratch = new(() => new(), true);
    private readonly float                           _tileWidthWorld;
    private readonly float                           _tileHeightWorld;
    private readonly float                           _invTileWidthWorld;
    private readonly float                           _invTileHeightWorld;
    private readonly int                             _uniqueRasterJobCount;
    private readonly int                             _totalRasterJobReferences;
    private readonly long                            _preparedTerrainBytes;

    private int   _walkableClimbVoxels;
    private int   _walkableHeightVoxels;
    private int   _walkableRadiusVoxels;
    private float _walkableNormalThreshold;
    private int   _borderSizeVoxels;
    private float _borderSizeWorld;
    private int   _tileSizeXVoxels;
    private int   _tileSizeZVoxels;
    private int   _voxelizerNumX = 1;
    private int   _voxelizerNumY = 1;
    private int   _voxelizerNumZ = 1;

    public static string ComputeBuildSignature(SceneDefinition scene, NavmeshCustomization customization)
    {
        var settings = customization.GetBuildSettings(scene);
        var flyable  = customization.IsFlyingSupported(scene);
        var extract  = new SceneExtractor(scene);
        customization.CustomizeScene(extract);
        var groundTiles = ResolveGroundTileCount(extract, settings);
        return $"{settings.BuildSignature(flyable)}GroundTiles={groundTiles};";
    }

    public NavmeshBuilder(SceneDefinition scene, NavmeshCustomization customization, Config config)
    {
        _config  = config;
        Settings = customization.GetBuildSettings(scene);

        Flyable        = customization.IsFlyingSupported(scene);
        BuildSignature = Settings.BuildSignature(Flyable);
        _customization = customization;

        var extractTimer = StopWatchTimer.Create();
        Scene = new(scene);
        customization.CustomizeScene(Scene);
        var extractDuration = extractTimer.Value();

        BoundsMin      = new(WorldBoundsMin);
        BoundsMax      = new(WorldBoundsMax);
        NumTilesX      = NumTilesZ = ResolveGroundTileCount(Scene, Settings);
        BuildSignature = $"{BuildSignature}GroundTiles={NumTilesX};";
        Service.Log.Debug($"[NavmeshBuilder] 开始构建 {NumTilesX}x{NumTilesZ} 路网，自定义 = {customization.GetType()} v{customization.Version}");
        Service.Log.Debug($"[NavmeshBuilder] 场景提取耗时 {extractDuration.TotalMilliseconds:f1} ms");

        var navmeshParams = new DtNavMeshParams
        {
            orig       = BoundsMin.SystemToRecast(),
            tileWidth  = (BoundsMax.X - BoundsMin.X) / NumTilesX,
            tileHeight = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ,
            maxTiles   = NumTilesX                   * NumTilesZ,
            maxPolys   = 1 << DT_POLY_BITS
        };

        _tileWidthWorld     = navmeshParams.tileWidth;
        _tileHeightWorld    = navmeshParams.tileHeight;
        _invTileWidthWorld  = 1.0f / _tileWidthWorld;
        _invTileHeightWorld = 1.0f / _tileHeightWorld;

        var navmesh = new DtNavMesh();
        navmesh.Init(navmeshParams, Settings.PolyMaxVerts);
        var volumeTiles = new int[Settings.VolumeTiles.Length + 1];
        volumeTiles[0] = NumTilesX;
        Array.Copy(Settings.VolumeTiles, 0, volumeTiles, 1, Settings.VolumeTiles.Length);
        var volume = Flyable ? new VoxelMap(BoundsMin, BoundsMax, volumeTiles) : null;
        Navmesh = new(customization.Version, BuildSignature, false, navmesh, volume);

        _walkableClimbVoxels     = (int)MathF.Floor(Settings.AgentMaxClimb / Settings.CellHeight);
        _walkableHeightVoxels    = (int)MathF.Ceiling(Settings.AgentHeight / Settings.CellHeight);
        _walkableRadiusVoxels    = (int)MathF.Ceiling(Settings.AgentRadius / Settings.CellSize);
        _walkableNormalThreshold = Settings.AgentMaxSlopeDeg.Degrees().Cos();
        _borderSizeVoxels        = 3 + _walkableRadiusVoxels;
        _borderSizeWorld         = _borderSizeVoxels * Settings.CellSize;
        _tileSizeXVoxels         = (int)MathF.Ceiling(navmeshParams.tileWidth  / Settings.CellSize) + 2 * _borderSizeVoxels;
        _tileSizeZVoxels         = (int)MathF.Ceiling(navmeshParams.tileHeight / Settings.CellSize) + 2 * _borderSizeVoxels;

        if (volume != null)
        {
            _voxelizerNumX = 1;
            _voxelizerNumY = NumTilesX;
            _voxelizerNumZ = 1;

            foreach (var n in Settings.VolumeTiles)
            {
                _voxelizerNumX *= n;
                _voxelizerNumY *= n;
                _voxelizerNumZ *= n;
            }
        }

        var bucketTimer    = StopWatchTimer.Create();
        var bucketedInputs = BucketTileInputs();
        _tileInputs               = bucketedInputs.Inputs;
        _tileBuildOrder           = bucketedInputs.TileBuildOrder;
        _geometryJobs             = bucketedInputs.GeometryJobs;
        _terrainJobs              = bucketedInputs.TerrainJobs;
        _uniqueRasterJobCount     = bucketedInputs.UniqueRasterJobCount;
        _totalRasterJobReferences = bucketedInputs.TotalRasterJobReferences;
        _preparedTerrainBytes     = bucketedInputs.PreparedTerrainBytes;
        TotalEstimatedTileWeight  = bucketedInputs.TotalEstimatedTileWeight;
        Service.Log.Debug($"[NavmeshBuilder] 瓦片分桶耗时 {bucketTimer.Value().TotalMilliseconds:f1} ms");
    }

    public void Build(Action<int>? onTileFinished = null)
    {
        var builtTiles = BuildTileResults(false, onTileFinished);
        MergeBuiltTiles(builtTiles, false);
    }

    public List<RcBuilderResult> BuildTiles(Action<int>? onTileFinished = null)
    {
        var builtTiles = BuildTileResults(true, onTileFinished);
        return MergeBuiltTiles(builtTiles, true);
    }

    private static int ResolveGroundTileCount(SceneExtractor scene, NavmeshSettings settings)
    {
        var min   = new Vector3(float.MaxValue);
        var max   = new Vector3(float.MinValue);
        var found = false;

        foreach (var mesh in scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                min   = Vector3.Min(min, instance.WorldBounds.Min);
                max   = Vector3.Max(max, instance.WorldBounds.Max);
                found = true;
            }
        }

        if (!found)
            return 16;

        var occupiedSpan    = MathF.Max(max.X - min.X, max.Z - min.Z);
        var targetTileCount = (int)MathF.Ceiling(occupiedSpan / settings.GroundTileSize);
        targetTileCount = Math.Clamp(targetTileCount, 1, settings.GroundTileCountMax);
        var pow2 = 1;
        while (pow2 < targetTileCount)
            pow2 <<= 1;

        return Math.Clamp(pow2, 1, settings.GroundTileCountMax);
    }

    private static void AccumulateRecastTelemetry(IReadOnlyList<RcTelemetryTick> telemetry, long[] phaseTicks)
    {
        foreach (var tick in telemetry)
        {
            if (tick.Key.StartsWith("RC_TIMER_RASTERIZE_", StringComparison.Ordinal)) phaseTicks[(int)BuildPhase.RecastRasterize]                  += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_COMPACTHEIGHTFIELD)) phaseTicks[(int)BuildPhase.RecastBuildCompactHeightfield] += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_ERODE_AREA)) phaseTicks[(int)BuildPhase.RecastErodeArea]                             += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_DISTANCEFIELD)) phaseTicks[(int)BuildPhase.RecastBuildDistanceField]           += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_REGIONS)) phaseTicks[(int)BuildPhase.RecastBuildRegions]                       += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_CONTOURS)) phaseTicks[(int)BuildPhase.RecastBuildContours]                     += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_POLYMESH)) phaseTicks[(int)BuildPhase.RecastBuildPolyMesh]                     += tick.Ticks;
            else if (tick.Key == nameof(RcTimerLabel.RC_TIMER_BUILD_POLYMESHDETAIL)) phaseTicks[(int)BuildPhase.RecastBuildPolyMeshDetail]         += tick.Ticks;
        }
    }

    private static BuildTelemetrySummary SummarizeBuildTelemetry
    (
        IReadOnlyList<TileBuildResult> builtTiles,
        TimeSpan                       parallelDuration,
        int                            configuredBuildMaxCores,
        int                            maxAvailableCores,
        int                            threadCount,
        int                            uniqueRasterJobCount,
        int                            totalRasterJobReferences,
        long                           preparedTerrainBytes
    )
    {
        List<BuildPhaseSummary> phases               = [];
        var                     tileCount            = builtTiles.Count;
        long                    aggregatedPhaseTicks = 0;

        for (var phaseIndex = 0; phaseIndex < (int)BuildPhase.Count; ++phaseIndex)
            foreach (var tile in builtTiles)
                aggregatedPhaseTicks += tile.PhaseTicks[phaseIndex];

        for (var phaseIndex = 0; phaseIndex < (int)BuildPhase.Count; ++phaseIndex)
        {
            long totalTicks   = 0;
            long maxTicks     = 0;
            var  slowestTileX = -1;
            var  slowestTileZ = -1;

            foreach (var tile in builtTiles)
            {
                var ticks = tile.PhaseTicks[phaseIndex];
                totalTicks += ticks;

                if (ticks > maxTicks)
                {
                    maxTicks     = ticks;
                    slowestTileX = tile.TileX;
                    slowestTileZ = tile.TileZ;
                }
            }

            if (totalTicks == 0)
                continue;

            phases.Add
            (
                new()
                {
                    Name              = PhaseNames[phaseIndex],
                    TotalTicks        = totalTicks,
                    AverageTicks      = totalTicks / Math.Max(tileCount, 1),
                    MaxTicks          = maxTicks,
                    SlowestTileX      = slowestTileX,
                    SlowestTileZ      = slowestTileZ,
                    ShareOfPhaseTicks = aggregatedPhaseTicks > 0 ? totalTicks / (double)aggregatedPhaseTicks : 0
                }
            );
        }

        List<SlowTileSummary> slowTiles = [];

        foreach (var tile in builtTiles.OrderByDescending(t => t.TotalTicks).Take(5))
        {
            slowTiles.Add
            (
                new()
                {
                    TileX               = tile.TileX,
                    TileZ               = tile.TileZ,
                    TotalTicks          = tile.TotalTicks,
                    GeometryJobCount    = tile.GeometryJobCount,
                    TerrainJobCount     = tile.TerrainJobCount,
                    UniqueJobCount      = tile.UniqueJobCount,
                    PrimitiveCount      = tile.PrimitiveCount,
                    EstimatedSpanWeight = tile.EstimatedSpanWeight,
                    PreCompactSpanCount = tile.PreCompactSpanCount,
                    PolyCount           = tile.PolyCount,
                    VertCount           = tile.VertCount,
                    DetailTriCount      = tile.DetailTriCount
                }
            );
        }

        return new()
        {
            ConfiguredBuildMaxCores = configuredBuildMaxCores,
            MaxAvailableCores       = maxAvailableCores,
            ThreadCount             = threadCount,
            ParallelTicks           = parallelDuration.Ticks,
            AggregatedPhaseTicks    = aggregatedPhaseTicks,
            UniqueRasterJobCount    = uniqueRasterJobCount,
            JobCoverageMultiplier   = uniqueRasterJobCount > 0 ? totalRasterJobReferences / (double)uniqueRasterJobCount : 0,
            PreparedTerrainBytes    = preparedTerrainBytes,
            Phases                  = phases,
            SlowTiles               = slowTiles
        };
    }

    private static void LogBuildTelemetry(BuildTelemetrySummary telemetry)
    {
        Service.Log.Debug
        (
            $"[NavmeshBuilder] 构建线程信息：配置核心数 = {telemetry.ConfiguredBuildMaxCores}，可用核心数 = {telemetry.MaxAvailableCores}，实际线程数 = {telemetry.ThreadCount}"
        );
        Service.Log.Debug
        (
            $"[NavmeshBuilder] Raster job 统计：唯一 job 数 = {telemetry.UniqueRasterJobCount}，覆盖倍率 = {telemetry.JobCoverageMultiplier:f2}，地形预处理缓存 = {telemetry.PreparedTerrainBytes / 1024.0 / 1024.0:f2} MiB"
        );
        Service.Log.Debug("[NavmeshBuilder] 阶段统计（总计 / 单瓦片均值 / 最慢瓦片）");

        foreach (var phase in telemetry.Phases)
        {
            Service.Log.Debug
            (
                $"[NavmeshBuilder] {phase.Name}: 总计 {TicksToMilliseconds(phase.TotalTicks):f1} ms，占比 {phase.ShareOfPhaseTicks:P1}，均值 {TicksToMilliseconds(phase.AverageTicks):f2} ms，最慢瓦片 {phase.SlowestTileX}x{phase.SlowestTileZ} = {TicksToMilliseconds(phase.MaxTicks):f1} ms"
            );
        }

        var detailPhase = telemetry.Phases.FirstOrDefault(phase => phase.Name == "Recast: 细节网格");
        if (detailPhase != null && detailPhase.ShareOfPhaseTicks >= 0.30)
            Service.Log.Information("[NavmeshBuilder] 细节网格耗时占比较高，如仅需快速验证，可启用“快速构建（关闭细节网格）”");

        if (telemetry.SlowTiles.Count > 0)
        {
            var slowestTiles = string.Join("，", telemetry.SlowTiles.Select(tile => $"{tile.TileX}x{tile.TileZ} = {TicksToMilliseconds(tile.TotalTicks):f1} ms"));
            Service.Log.Debug($"[NavmeshBuilder] 最慢瓦片 Top {telemetry.SlowTiles.Count}: {slowestTiles}");
        }

        foreach (var tile in telemetry.SlowTiles)
        {
            Service.Log.Debug
            (
                $"[NavmeshBuilder] 慢瓦片 {tile.TileX}x{tile.TileZ}: 几何 job {tile.GeometryJobCount}，地形 job {tile.TerrainJobCount}，唯一 job {tile.UniqueJobCount}，Primitive {tile.PrimitiveCount}，预估 span 权重 {tile.EstimatedSpanWeight}，紧凑前 span {tile.PreCompactSpanCount}，Poly {tile.PolyCount}，Vert {tile.VertCount}，DetailTri {tile.DetailTriCount}"
            );
        }
    }


    private TileBuildResult[] BuildTileResults(bool captureIntermediates, Action<int>? onTileFinished)
    {
        var tileCount     = NumTilesX * NumTilesZ;
        var maxThreads    = Environment.ProcessorCount;
        var wantedThreads = 0;
        var threadCount   = Math.Clamp(wantedThreads <= 0 ? maxThreads + wantedThreads : wantedThreads, 1, maxThreads);
        var builtTiles    = new TileBuildResult[tileCount];
        var buildTimer    = StopWatchTimer.Create();
        var nextIndex     = -1;

        Parallel.For
        (
            0,
            threadCount,
            new ParallelOptions { MaxDegreeOfParallelism = threadCount },
            _ =>
            {
                while (true)
                {
                    var orderIndex = Interlocked.Increment(ref nextIndex);
                    if ((uint)orderIndex >= (uint)tileCount)
                        break;

                    var tileIndex = _tileBuildOrder[orderIndex];
                    var input     = _tileInputs[tileIndex];
                    var x         = tileIndex % NumTilesX;
                    var z         = tileIndex / NumTilesX;
                    builtTiles[tileIndex] = BuildTileCore(x, z, input, captureIntermediates, onTileFinished);
                }
            }
        );

        var parallelDuration = buildTimer.Value();
        LastBuildTelemetry = SummarizeBuildTelemetry
        (
            builtTiles,
            parallelDuration,
            wantedThreads,
            Environment.ProcessorCount,
            threadCount,
            _uniqueRasterJobCount,
            _totalRasterJobReferences,
            _preparedTerrainBytes
        );
        Service.Log.Debug
        (
            $"[NavmeshBuilder] 并行瓦片构建耗时 {parallelDuration.TotalMilliseconds:f1} ms，配置核心数 = {wantedThreads}，可用核心数 = {Environment.ProcessorCount}，实际线程数 = {threadCount}"
        );
        LogBuildTelemetry(LastBuildTelemetry);
        return builtTiles;
    }

    private List<RcBuilderResult> MergeBuiltTiles(IReadOnlyList<TileBuildResult> builtTiles, bool collectIntermediates)
    {
        var                   mergeTimer   = StopWatchTimer.Create();
        List<RcBuilderResult> debugResults = collectIntermediates ? new(builtTiles.Count) : [];
        var                   climbLinks   = 0;
        var                   jumpLinks    = 0;

        for (var tileIndex = 0; tileIndex < builtTiles.Count; ++tileIndex)
        {
            var built = builtTiles[tileIndex];
            climbLinks += built.GeneratedClimbLinks;
            jumpLinks  += built.GeneratedJumpLinks;
            if (built.MeshData != null)
                Navmesh.Mesh.AddTile(built.MeshData, 0, 0, out _);

            if (Navmesh.Volume != null && built.VolumeColumn != null)
            {
                var parent      = Navmesh.Volume;
                var shift       = parent.RootTile.SubdivisionCount;
                var cellYCount  = parent.Levels[0].NumCellsY;
                var parentIndex = parent.Levels[0].VoxelToIndex(tileIndex % NumTilesX, 0, tileIndex / NumTilesX);

                for (var y = 0; y < cellYCount; ++y)
                {
                    var contents = built.VolumeColumn.Contents[y];
                    if ((contents & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                        continue;

                    if ((contents & VoxelMap.VOXEL_ID_MASK) != VoxelMap.VOXEL_ID_MASK)
                        contents += (ushort)shift;

                    parent.RootTile.Contents[parentIndex + y] = contents;
                }

                parent.RootTile.AddSubdivisions(built.VolumeColumn.Subdivision);
            }

            if (collectIntermediates && built.DebugResult != null)
                debugResults.Add(built.DebugResult);
        }

        Navmesh.GeneratedClimbDownLinkCount = climbLinks;
        Navmesh.GeneratedEdgeJumpLinkCount  = jumpLinks;
        Service.Log.Debug($"[NavmeshBuilder] 结果合并耗时 {mergeTimer.Value().TotalMilliseconds:f1} ms");
        return debugResults;
    }

    private TileBuildResult BuildTileCore(int x, int z, TileBuildInput input, bool captureIntermediates, Action<int>? onTileProgress)
    {
        var scratch = _threadScratch.Value!;
        scratch.Reset();
        if (Navmesh.Volume != null && scratch.VolumeRoot == null)
            scratch.VolumeRoot = new Voxelizer(_voxelizerNumX, _voxelizerNumY, _voxelizerNumZ);
        var totalStart          = Stopwatch.GetTimestamp();
        var telemetry           = new RcContext();
        var geometryJobs        = _geometryJobs.AsSpan(input.GeometryJobStart, input.GeometryJobCount);
        var terrainJobs         = _terrainJobs.AsSpan(input.TerrainJobStart, input.TerrainJobCount);
        var totalProgressBudget = Math.Max(input.EstimatedSpanWeight, 1);
        var geometryShare       = geometryJobs.IsEmpty ? 0 : 10;
        var terrainShare        = terrainJobs.IsEmpty ? 0 : 50;
        var volumeShare         = Navmesh.Volume != null ? 12 : 0;
        var linkShare           = Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks ? 2 : 0;
        var detourShare         = 1;
        var recastShare         = 100           - geometryShare - terrainShare - volumeShare - linkShare   - detourShare;
        var activeShareSum      = geometryShare + terrainShare  + volumeShare  + linkShare   + detourShare + recastShare;
        var geometryBudget      = geometryShare == 0 ? 0 : totalProgressBudget * geometryShare / activeShareSum;
        var terrainBudget       = terrainShare  == 0 ? 0 : totalProgressBudget * terrainShare  / activeShareSum;
        var volumeBudget        = volumeShare   == 0 ? 0 : totalProgressBudget * volumeShare   / activeShareSum;
        var linkBudget          = linkShare     == 0 ? 0 : totalProgressBudget * linkShare     / activeShareSum;
        var detourBudget        = detourShare   == 0 ? 0 : totalProgressBudget * detourShare   / activeShareSum;
        var recastBudget        = totalProgressBudget - geometryBudget - terrainBudget - volumeBudget - linkBudget - detourBudget;
        var reportedProgress    = 0;

        void ReportProgress(int delta)
        {
            if (delta <= 0 || onTileProgress == null)
                return;

            reportedProgress += delta;
            onTileProgress(delta);
        }

        Action<RasterJob>? CreateRasterProgressReporter(ReadOnlySpan<RasterJob> jobs, int budget, out Action finish)
        {
            finish = static () => { };
            if (budget <= 0 || onTileProgress == null || jobs.IsEmpty)
                return null;

            long totalCost = 0;
            foreach (ref readonly var job in jobs)
                totalCost += Math.Max(EstimateSpanWeight(job.PrimitiveCount, job.VertexCount, job.TerrainLike, 1), 1);

            if (totalCost <= 0)
                return null;

            long consumedCost = 0;
            var  emitted      = 0;
            finish = () =>
            {
                var remainder = budget - emitted;
                if (remainder <= 0)
                    return;

                emitted += remainder;
                ReportProgress(remainder);
            };

            return job =>
            {
                consumedCost += Math.Max(EstimateSpanWeight(job.PrimitiveCount, job.VertexCount, job.TerrainLike, 1), 1);
                var target = (int)(consumedCost * budget / totalCost);
                var delta  = target - emitted;
                if (delta <= 0)
                    return;

                emitted += delta;
                ReportProgress(delta);
            };
        }

        var tileBoundsMin = new Vector3(BoundsMin.X     + x * _tileWidthWorld, BoundsMin.Y, BoundsMin.Z     + z * _tileHeightWorld);
        var tileBoundsMax = new Vector3(tileBoundsMin.X + _tileWidthWorld,     BoundsMax.Y, tileBoundsMin.Z + _tileHeightWorld);
        tileBoundsMin.X -= _borderSizeWorld;
        tileBoundsMin.Z -= _borderSizeWorld;
        tileBoundsMax.X += _borderSizeWorld;
        tileBoundsMax.Z += _borderSizeWorld;

        var shf = new RcHeightfield
        (
            _tileSizeXVoxels,
            _tileSizeZVoxels,
            tileBoundsMin.SystemToRecast(),
            tileBoundsMax.SystemToRecast(),
            Settings.CellSize,
            Settings.CellHeight,
            _borderSizeVoxels
        );
        var vox = scratch.VolumeRoot;
        var rasterizer = new NavmeshRasterizer
        (
            shf,
            _walkableNormalThreshold,
            _walkableClimbVoxels,
            _walkableHeightVoxels,
            Settings.Filtering.HasFlag(NavmeshSettings.Filter.Interiors),
            vox,
            telemetry,
            scratch.Rasterizer,
            x,
            z
        );

        var geometryProgress     = CreateRasterProgressReporter(geometryJobs, geometryBudget, out var finishGeometryProgress);
        var terrainProgress      = CreateRasterProgressReporter(terrainJobs,  terrainBudget,  out var finishTerrainProgress);
        var recastCompactBudget  = recastBudget                                                      * 38 / 100;
        var recastFilterBudget   = recastBudget                                                      * 10 / 100;
        var recastDistanceBudget = Settings.Partitioning == RcPartition.WATERSHED ? recastBudget * 8 / 100 : 0;
        var recastRegionBudget   = recastBudget                                                      * 24 / 100;
        var recastContourBudget  = recastBudget                                                      * 10 / 100;
        var recastPolyBudget     = recastBudget - recastCompactBudget - recastFilterBudget - recastDistanceBudget - recastRegionBudget - recastContourBudget;

        var phaseStart = Stopwatch.GetTimestamp();
        rasterizer.Rasterize(geometryJobs, true, true, geometryProgress);
        scratch.PhaseTicks[(int)BuildPhase.RasterizeGeometry] += ElapsedTimeSpanTicks(phaseStart);
        finishGeometryProgress();

        if (!terrainJobs.IsEmpty)
        {
            phaseStart = Stopwatch.GetTimestamp();
            rasterizer.Rasterize(terrainJobs, false, true, terrainProgress);
            scratch.PhaseTicks[(int)BuildPhase.RasterizeTerrain] += ElapsedTimeSpanTicks(phaseStart);
            finishTerrainProgress();
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LowHangingObstacles))
            RcFilters.FilterLowHangingWalkableObstacles(telemetry, _walkableClimbVoxels, shf);

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LedgeSpans))
            RcFilters.FilterLedgeSpans(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.WalkableLowHeightSpans))
            RcFilters.FilterWalkableLowHeightSpans(telemetry, _walkableHeightVoxels, shf);

        var preCompactSpanCount = 0;
        for (var cell = 0; cell < shf.spans.Length; ++cell)
        for (var span = shf.spans[cell]; span != null; span = span.next)
            if (span.area != 0)
                preCompactSpanCount++;

        var chf = RcCompacts.BuildCompactHeightfield(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);
        ReportProgress(recastCompactBudget);
        RcAreas.ErodeWalkableArea(telemetry, _walkableRadiusVoxels, chf);
        RcAreas.MedianFilterWalkableArea(telemetry, chf);
        ReportProgress(recastFilterBudget);

        var regionMinArea   = (int)(Settings.RegionMinSize   * Settings.RegionMinSize);
        var regionMergeArea = (int)(Settings.RegionMergeSize * Settings.RegionMergeSize);

        if (Settings.Partitioning == RcPartition.WATERSHED)
        {
            RcRegions.BuildDistanceField(telemetry, chf);
            ReportProgress(recastDistanceBudget);
            RcRegions.BuildRegions(telemetry, chf, regionMinArea, regionMergeArea);
        }
        else if (Settings.Partitioning == RcPartition.MONOTONE) RcRegions.BuildRegionsMonotone(telemetry, chf, regionMinArea, regionMergeArea);
        else RcRegions.BuildLayerRegions(telemetry, chf, regionMinArea);

        ReportProgress(recastRegionBudget);

        var effectivePolyMaxEdgeLen = Settings.PolyMaxEdgeLen > 0 ? Settings.PolyMaxEdgeLen : Settings.AgentRadius * 8f;
        var polyMaxEdgeLenVoxels    = (int)(effectivePolyMaxEdgeLen / Settings.CellSize);
        var cset = RcContours.BuildContours
            (telemetry, chf, Settings.PolyMaxSimplificationError, polyMaxEdgeLenVoxels, RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES);
        ReportProgress(recastContourBudget);

        var pmesh = RcMeshs.BuildPolyMesh(telemetry, cset, Settings.PolyMaxVerts);

        for (var i = 0; i < pmesh.npolys; ++i)
        {
            var area = pmesh.areas[i] == 0 ? NavmeshArea.Null : NavmeshArea.Ground;
            pmesh.areas[i] = (int)area;
            pmesh.flags[i] = area == NavmeshArea.Null ? (int)NavmeshPolyFlags.None : (int)NavmeshPolyFlags.Ground;
        }

        var               detailSampleDist     = Settings.FastBuild || Settings.DetailSampleDist < 0.9f ? 0 : Settings.CellSize * Settings.DetailSampleDist;
        var               detailSampleMaxError = Settings.CellHeight * Settings.DetailMaxSampleError;
        RcPolyMeshDetail? dmesh                = null;
        if (detailSampleDist > 0 && pmesh.npolys > 0)
            dmesh = RcMeshDetails.BuildPolyMeshDetail(telemetry, pmesh, chf, detailSampleDist, detailSampleMaxError);
        ReportProgress(recastPolyBudget);

        var navmeshConfig = new DtNavMeshCreateParams
        {
            verts            = pmesh.verts,
            vertCount        = pmesh.nverts,
            polys            = pmesh.polys,
            polyFlags        = pmesh.flags,
            polyAreas        = pmesh.areas,
            polyCount        = pmesh.npolys,
            nvp              = pmesh.nvp,
            detailMeshes     = dmesh?.meshes,
            detailVerts      = dmesh?.verts,
            detailVertsCount = dmesh?.nverts ?? 0,
            detailTris       = dmesh?.tris,
            detailTriCount   = dmesh?.ntris ?? 0,
            tileX            = x,
            tileZ            = z,
            tileLayer        = 0,
            bmin             = pmesh.bmin,
            bmax             = pmesh.bmax,
            walkableHeight   = Settings.AgentHeight,
            walkableRadius   = Settings.AgentRadius,
            walkableClimb    = Settings.AgentMaxClimb,
            cs               = Settings.CellSize,
            ch               = Settings.CellHeight,
            buildBvTree      = true
        };
        _customization.CustomizeSettings(navmeshConfig);

        RcBuilderResult?   builderResult       = null;
        DtJumpLinkBuilder? jumpLinkBuilder     = null;
        var                generatedClimbLinks = 0;
        var                generatedJumpLinks  = 0;

        if (captureIntermediates || Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks)
        {
            builderResult = new RcBuilderResult(x, z, shf, chf, cset, pmesh, dmesh, telemetry);
            if (Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks)
                jumpLinkBuilder = new([builderResult]);
        }

        HashSet<(int Kind, int StartX, int StartY, int StartZ, int EndX, int EndY, int EndZ)> acceptedLinks = [];

        int addConnections(List<DtJumpLink> links, NavmeshArea area, NavmeshPolyFlags flags, NavmeshOffMeshKind kind)
        {
            var acceptedCount = 0;

            foreach (var link in links)
            {
                RcVec3f prev = default;

                for (var i = 0; i < link.startSamples.Length; i++)
                {
                    var p = link.startSamples[i].p;
                    var q = link.endSamples[i].p;

                    if (i != 0 && RcVec.Dist2D(prev, p) <= Settings.AgentRadius)
                        continue;

                    var start                = p.RecastToSystem();
                    var end                  = q.RecastToSystem();
                    var delta                = end               - start;
                    var horizontalDistanceSq = delta.X * delta.X + delta.Z * delta.Z;
                    var verticalDistanceAbs  = MathF.Abs(delta.Y);
                    if (horizontalDistanceSq <= Settings.AgentRadius * Settings.AgentRadius * 4 && verticalDistanceAbs <= Settings.AgentMaxClimb * 1.25f)
                        continue;

                    if (verticalDistanceAbs > Settings.EdgeJumpMaxDrop + Settings.AgentHeight)
                        continue;

                    var key = ((int)kind, Quantize(start.X), Quantize(start.Y), Quantize(start.Z), Quantize(end.X), Quantize(end.Y), Quantize(end.Z));
                    if (!acceptedLinks.Add(key))
                        continue;

                    {
                        navmeshConfig.AddOffMeshConnection(start, end, Settings.AgentRadius, false, 0, area, flags, kind);
                        prev = p;
                        acceptedCount++;
                    }
                }
            }

            return acceptedCount;
        }

        static int Quantize(float value)
        {
            return (int)MathF.Round(value * 4f);
        }

        if ((Settings.GenerateEdgeClimbLinks || Settings.GenerateEdgeJumpLinks) && jumpLinkBuilder != null)
        {
            phaseStart = Stopwatch.GetTimestamp();

            if (Settings.GenerateEdgeClimbLinks)
            {
                var cfg = new DtJumpLinkBuilderConfig
                (
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
                generatedClimbLinks = addConnections
                (
                    jumpLinkBuilder.Build(cfg, DtJumpLinkType.EDGE_CLIMB_DOWN),
                    NavmeshArea.GeneratedClimbDown,
                    NavmeshPolyFlags.GeneratedClimbDown,
                    NavmeshOffMeshKind.GeneratedClimbDown
                );
            }

            if (Settings.GenerateEdgeJumpLinks)
            {
                var cfg = new DtJumpLinkBuilderConfig
                (
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
                generatedJumpLinks = addConnections
                (
                    jumpLinkBuilder.Build(cfg, DtJumpLinkType.EDGE_JUMP),
                    NavmeshArea.GeneratedEdgeJump,
                    NavmeshPolyFlags.GeneratedEdgeJump,
                    NavmeshOffMeshKind.GeneratedEdgeJump
                );
            }

            scratch.PhaseTicks[(int)BuildPhase.BuildJumpLinks] += ElapsedTimeSpanTicks(phaseStart);
            ReportProgress(linkBudget);
        }

        VolumeRootColumnBuildResult? volumeColumn = null;

        if (Navmesh.Volume != null && vox != null)
        {
            phaseStart                                            =  Stopwatch.GetTimestamp();
            volumeColumn                                          =  Navmesh.Volume.BuildRootColumn(vox, x, z);
            scratch.PhaseTicks[(int)BuildPhase.BuildVolumeColumn] += ElapsedTimeSpanTicks(phaseStart);
            ReportProgress(volumeBudget);
        }

        phaseStart = Stopwatch.GetTimestamp();
        var navmeshData = DtNavMeshBuilder.CreateNavMeshData(navmeshConfig);
        scratch.PhaseTicks[(int)BuildPhase.CreateDetourData] += ElapsedTimeSpanTicks(phaseStart);
        ReportProgress(detourBudget);

        AccumulateRecastTelemetry(telemetry.ToList(), scratch.PhaseTicks);

        var phaseTicks = new long[scratch.PhaseTicks.Length];
        Array.Copy(scratch.PhaseTicks, phaseTicks, phaseTicks.Length);

        var totalTicks = ElapsedTimeSpanTicks(totalStart);
        ReportProgress(totalProgressBudget - reportedProgress);
        return new()
        {
            TileX               = x,
            TileZ               = z,
            TotalTicks          = totalTicks,
            PhaseTicks          = phaseTicks,
            GeometryJobCount    = input.GeometryJobCount,
            TerrainJobCount     = input.TerrainJobCount,
            PrimitiveCount      = input.PrimitiveCount,
            UniqueJobCount      = input.GeometryJobCount + input.TerrainJobCount,
            EstimatedSpanWeight = input.EstimatedSpanWeight,
            PreCompactSpanCount = preCompactSpanCount,
            PolyCount           = pmesh.npolys,
            VertCount           = pmesh.nverts,
            DetailTriCount      = dmesh?.ntris ?? 0,
            GeneratedClimbLinks = generatedClimbLinks,
            GeneratedJumpLinks  = generatedJumpLinks,
            MeshData            = navmeshData,
            VolumeColumn        = volumeColumn,
            DebugResult         = captureIntermediates ? builderResult : null
        };
    }

    private static long ElapsedTimeSpanTicks(long startTimestamp)
        => (long)((Stopwatch.GetTimestamp() - startTimestamp) * (double)TimeSpan.TicksPerSecond / Stopwatch.Frequency);

    private static double TicksToMilliseconds(long ticks)
        => ticks / (double)TimeSpan.TicksPerMillisecond;


    private (TileBuildInput[] Inputs, int[] TileBuildOrder, RasterJob[] GeometryJobs, RasterJob[] TerrainJobs, int UniqueRasterJobCount, int TotalRasterJobReferences
        , long PreparedTerrainBytes, long TotalEstimatedTileWeight)
        BucketTileInputs()
    {
        var             tileCount                = NumTilesX * NumTilesZ;
        var             geometryCounts           = new int[tileCount];
        var             terrainCounts            = new int[tileCount];
        var             primitiveCounts          = new int[tileCount];
        var             spanWeights              = new int[tileCount];
        List<RasterJob> geometryJobs             = [];
        List<RasterJob> terrainJobs              = [];
        var             totalRasterJobReferences = 0;
        long            preparedTerrainBytes     = 0;

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                foreach (var part in mesh.Parts)
                {
                    var worldBounds    = TransformBounds(instance.WorldTransform, part.LocalBounds);
                    var primitiveCount = part.Primitives.Count;
                    var vertexCount    = part.Vertices.Count;
                    if (primitiveCount == 0 || vertexCount == 0)
                        continue;

                    GetTileRange(worldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
                    var                      terrainLike     = (mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0;
                    var                      coverage        = (maxX - minX + 1) * (maxZ - minZ + 1);
                    var                      spanWeight      = EstimateSpanWeight(primitiveCount, vertexCount, terrainLike, coverage);
                    PreparedTerrainGeometry? preparedTerrain = null;

                    if (terrainLike)
                    {
                        preparedTerrain      =  PrepareTerrainGeometry(part, instance, minX, maxX, minZ, maxZ);
                        preparedTerrainBytes += preparedTerrain.MemoryBytes;
                    }

                    var job = new RasterJob
                    {
                        MeshType        = mesh.MeshType,
                        Part            = part,
                        Instance        = instance,
                        WorldBounds     = worldBounds,
                        PrimitiveCount  = primitiveCount,
                        VertexCount     = vertexCount,
                        TerrainLike     = terrainLike,
                        PreparedTerrain = preparedTerrain
                    };

                    for (var z = minZ; z <= maxZ; ++z)
                    {
                        var rowBase = z * NumTilesX;

                        for (var x = minX; x <= maxX; ++x)
                        {
                            var index = rowBase + x;
                            if (terrainLike)
                                ++terrainCounts[index];
                            else
                                ++geometryCounts[index];
                            primitiveCounts[index] += primitiveCount;
                            spanWeights[index]     += spanWeight;
                            ++totalRasterJobReferences;
                        }
                    }

                    if (terrainLike)
                        terrainJobs.Add(job);
                    else
                        geometryJobs.Add(job);
                }
            }
        }

        var  result                   = new TileBuildInput[tileCount];
        var  geometryTotal            = 0;
        var  terrainTotal             = 0;
        long totalEstimatedTileWeight = 0;

        for (var i = 0; i < tileCount; ++i)
        {
            result[i] = new()
            {
                GeometryJobStart    = geometryTotal,
                GeometryJobCount    = geometryCounts[i],
                TerrainJobStart     = terrainTotal,
                TerrainJobCount     = terrainCounts[i],
                PrimitiveCount      = primitiveCounts[i],
                EstimatedSpanWeight = spanWeights[i]
            };
            geometryTotal            += geometryCounts[i];
            terrainTotal             += terrainCounts[i];
            totalEstimatedTileWeight += Math.Max(spanWeights[i], 1);
        }

        var tileBuildOrder = new int[tileCount];
        for (var i = 0; i < tileCount; ++i)
            tileBuildOrder[i] = i;
        Array.Sort
        (
            tileBuildOrder,
            (lhs, rhs) =>
            {
                var leftWeight  = spanWeights[lhs];
                var rightWeight = spanWeights[rhs];
                var compare     = rightWeight.CompareTo(leftWeight);
                return compare != 0 ? compare : lhs.CompareTo(rhs);
            }
        );

        var geometryJobsByTile = new RasterJob[geometryTotal];
        var terrainJobsByTile  = new RasterJob[terrainTotal];
        var geometryOffsets    = new int[tileCount];
        var terrainOffsets     = new int[tileCount];

        for (var i = 0; i < tileCount; ++i)
        {
            geometryOffsets[i] = result[i].GeometryJobStart;
            terrainOffsets[i]  = result[i].TerrainJobStart;
        }

        foreach (var job in geometryJobs)
        {
            GetTileRange(job.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);

            for (var z = minZ; z <= maxZ; ++z)
            {
                var rowBase = z * NumTilesX;
                for (var x = minX; x <= maxX; ++x)
                    geometryJobsByTile[geometryOffsets[rowBase + x]++] = job;
            }
        }

        foreach (var job in terrainJobs)
        {
            GetTileRange(job.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);

            for (var z = minZ; z <= maxZ; ++z)
            {
                var rowBase = z * NumTilesX;
                for (var x = minX; x <= maxX; ++x)
                    terrainJobsByTile[terrainOffsets[rowBase + x]++] = job;
            }
        }

        return (result, tileBuildOrder, geometryJobsByTile, terrainJobsByTile, geometryJobs.Count + terrainJobs.Count, totalRasterJobReferences,
                   preparedTerrainBytes, totalEstimatedTileWeight);
    }

    private static int EstimateSpanWeight(int primitiveCount, int vertexCount, bool terrainLike, int coverage)
        => primitiveCount * (terrainLike ? 12 : 4) + vertexCount * (terrainLike ? 3 : 1) + coverage * (terrainLike ? 16 : 4);

    private PreparedTerrainGeometry PrepareTerrainGeometry
        (SceneExtractor.MeshPart part, SceneExtractor.MeshInstance instance, int minTileX, int maxTileX, int minTileZ, int maxTileZ)
    {
        var prepared   = NavmeshRasterizer.PrepareTerrainGeometry(part, instance);
        var primitives = part.PrimitiveSpan;
        if (primitives.IsEmpty)
            return prepared;

        var primitiveInfos            = GC.AllocateUninitializedArray<PreparedTerrainGeometry.PrimitiveInfo>(primitives.Length);
        var walkableNormalThresholdSq = _walkableNormalThreshold * _walkableNormalThreshold;
        var includeVolume             = Navmesh.Volume != null;

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            ref readonly var primitive = ref primitives[primitiveIndex];
            var              offset1   = primitive.V1 * 3;
            var              offset2   = primitive.V2 * 3;
            var              offset3   = primitive.V3 * 3;
            var              v1x       = prepared.WorldVertexTriples[offset1];
            var              v1y       = prepared.WorldVertexTriples[offset1 + 1];
            var              v1z       = prepared.WorldVertexTriples[offset1 + 2];
            var              v2x       = prepared.WorldVertexTriples[offset2];
            var              v2y       = prepared.WorldVertexTriples[offset2 + 1];
            var              v2z       = prepared.WorldVertexTriples[offset2 + 2];
            var              v3x       = prepared.WorldVertexTriples[offset3];
            var              v3y       = prepared.WorldVertexTriples[offset3 + 1];
            var              v3z       = prepared.WorldVertexTriples[offset3 + 2];
            var              v12x      = v2x             - v1x;
            var              v12y      = v2y             - v1y;
            var              v12z      = v2z             - v1z;
            var              v13x      = v3x             - v1x;
            var              v13y      = v3y             - v1y;
            var              v13z      = v3z             - v1z;
            var              crossX    = v12y   * v13z   - v12z   * v13y;
            var              crossY    = v12z   * v13x   - v12x   * v13z;
            var              crossZ    = v12x   * v13y   - v12y   * v13x;
            var              lenSq     = crossX * crossX + crossY * crossY + crossZ * crossZ;

            if (lenSq == 0)
            {
                primitiveInfos[primitiveIndex] = new
                    (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PreparedTerrainGeometry.PrimitiveInfo.BuildFlags(false, false, false, false));
                continue;
            }

            var flags           = primitive.Flags & ~instance.ForceClearPrimFlags | instance.ForceSetPrimFlags;
            var realSolid       = !flags.HasFlag(SceneExtractor.PrimitiveFlags.FlyThrough);
            var unwalkableSlope = crossY <= 0 || crossY * crossY < walkableNormalThresholdSq * lenSq;
            var unwalkable = flags.HasFlag
                                 (SceneExtractor.PrimitiveFlags.ForceUnwalkable) ||
                             unwalkableSlope                                     ||
                             includeVolume                                           &&
                             flags.HasFlag(SceneExtractor.PrimitiveFlags.Unlandable) &&
                             !flags.HasFlag(SceneExtractor.PrimitiveFlags.ForceWalkable);
            var projected  = crossY != 0;
            var planeGradX = projected ? -crossX / crossY : 0;
            var planeGradZ = projected ? -crossZ / crossY : 0;
            var planeBias  = projected ? v1y - planeGradX * v1x - planeGradZ * v1z : 0;
            primitiveInfos[primitiveIndex] = new
            (
                v1x,
                v1z,
                v12x,
                v12z,
                v13x,
                v13z,
                Math.Min(v1x, Math.Min(v2x, v3x)),
                Math.Max(v1x, Math.Max(v2x, v3x)),
                Math.Min(v1z, Math.Min(v2z, v3z)),
                Math.Max(v1z, Math.Max(v2z, v3z)),
                planeGradX,
                planeGradZ,
                planeBias,
                projected ? -1.0f / crossY : 0,
                unwalkable ? 0 : RcRecast.RC_WALKABLE_AREA,
                PreparedTerrainGeometry.PrimitiveInfo.BuildFlags(realSolid, crossY > 0, projected, true)
            );
        }

        prepared.PrimitiveInfos = primitiveInfos;
        var tileCountX   = maxTileX - minTileX + 1;
        var tileCountZ   = maxTileZ - minTileZ + 1;
        var bucketCount  = tileCountX * tileCountZ;
        var writeOffsets = new int[bucketCount];

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            if (!primitiveInfos[primitiveIndex].Valid)
                continue;

            GetPrimitiveTileRange(primitiveInfos[primitiveIndex], out var primitiveMinX, out var primitiveMaxX, out var primitiveMinZ, out var primitiveMaxZ);

            for (var z = primitiveMinZ; z <= primitiveMaxZ; ++z)
            {
                var rowBase = (z - minTileZ) * tileCountX;
                for (var x = primitiveMinX; x <= primitiveMaxX; ++x)
                    ++writeOffsets[rowBase + x - minTileX];
            }
        }

        var offsets = GC.AllocateUninitializedArray<int>(bucketCount + 1);
        for (var i = 0; i < bucketCount; ++i)
            offsets[i + 1] = offsets[i] + writeOffsets[i];

        var primitiveIndices = GC.AllocateUninitializedArray<int>(offsets[bucketCount]);
        Array.Copy(offsets, writeOffsets, bucketCount);

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            if (!primitiveInfos[primitiveIndex].Valid)
                continue;

            GetPrimitiveTileRange(primitiveInfos[primitiveIndex], out var primitiveMinX, out var primitiveMaxX, out var primitiveMinZ, out var primitiveMaxZ);

            for (var z = primitiveMinZ; z <= primitiveMaxZ; ++z)
            {
                var rowBase = (z - minTileZ) * tileCountX;

                for (var x = primitiveMinX; x <= primitiveMaxX; ++x)
                {
                    var cellIndex = rowBase + x - minTileX;
                    primitiveIndices[writeOffsets[cellIndex]++] = primitiveIndex;
                }
            }
        }

        prepared.TileMinX             = minTileX;
        prepared.TileMinZ             = minTileZ;
        prepared.TileCountX           = tileCountX;
        prepared.TileCountZ           = tileCountZ;
        prepared.TilePrimitiveOffsets = offsets;
        prepared.TilePrimitiveIndices = primitiveIndices;
        return prepared;
    }

    private void GetTileRange(AABB bounds, out int minX, out int maxX, out int minZ, out int maxZ) =>
        GetTileRange(bounds.Min.X, bounds.Max.X, bounds.Min.Z, bounds.Max.Z, out minX, out maxX, out minZ, out maxZ);

    private void GetTileRange(Matrix4x3 worldTransform, AABB localBounds, out int minX, out int maxX, out int minZ, out int maxZ) =>
        GetTileRange(TransformBounds(worldTransform, localBounds), out minX, out maxX, out minZ, out maxZ);

    private void GetPrimitiveTileRange(PreparedTerrainGeometry.PrimitiveInfo primitiveInfo, out int minX, out int maxX, out int minZ, out int maxZ) =>
        GetTileRange(primitiveInfo.MinX, primitiveInfo.MaxX, primitiveInfo.MinZ, primitiveInfo.MaxZ, out minX, out maxX, out minZ, out maxZ);

    private void GetTileRange(float minWorldX, float maxWorldX, float minWorldZ, float maxWorldZ, out int minX, out int maxX, out int minZ, out int maxZ)
    {
        minX = (int)MathF.Floor((minWorldX                    - _borderSizeWorld - BoundsMin.X) * _invTileWidthWorld);
        maxX = (int)MathF.Floor((maxWorldX + _borderSizeWorld - BoundsMin.X)                    * _invTileWidthWorld);
        minZ = (int)MathF.Floor((minWorldZ                    - _borderSizeWorld - BoundsMin.Z) * _invTileHeightWorld);
        maxZ = (int)MathF.Floor((maxWorldZ + _borderSizeWorld - BoundsMin.Z)                    * _invTileHeightWorld);

        minX = Math.Clamp(minX, 0, NumTilesX - 1);
        maxX = Math.Clamp(maxX, 0, NumTilesX - 1);
        minZ = Math.Clamp(minZ, 0, NumTilesZ - 1);
        maxZ = Math.Clamp(maxZ, 0, NumTilesZ - 1);
    }

    private static AABB TransformBounds(Matrix4x3 worldTransform, AABB localBounds)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = axisX      * localCenter.X + axisY      * localCenter.Y + axisZ      * localCenter.Z + worldTransform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        return new() { Min = center                  - extent, Max = center       + extent };
    }

    private static Vector3 Abs(Vector3 value) => new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));
}
