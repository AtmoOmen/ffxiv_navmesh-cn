using System.Numerics;
using DotRecast.Detour;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Models;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Build;

using static DotRecast.Detour.DtDetour;

// utility for building a navmesh from scene data
// individual tiles can be built concurrently
public partial class NavmeshBuilder
{
    private readonly Config _config;
    private const float WorldBoundsMin = -1024f;
    private const float WorldBoundsMax = 1024f;

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
        public required int  TileX                 { get; init; }
        public required int  TileZ                 { get; init; }
        public required long TotalTicks            { get; init; }
        public required int  GeometryJobCount      { get; init; }
        public required int  TerrainJobCount       { get; init; }
        public required int  UniqueJobCount        { get; init; }
        public required int  PrimitiveCount        { get; init; }
        public required int  EstimatedSpanWeight   { get; init; }
        public required int  PreCompactSpanCount   { get; init; }
        public required int  PolyCount             { get; init; }
        public required int  VertCount             { get; init; }
        public required int  DetailTriCount        { get; init; }
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
        public required int GeometryJobStart     { get; init; }
        public required int GeometryJobCount     { get; init; }
        public required int TerrainJobStart      { get; init; }
        public required int TerrainJobCount      { get; init; }
        public required int PrimitiveCount       { get; init; }
        public required int EstimatedSpanWeight  { get; init; }
    }

    private sealed class TileBuildResult
    {
        public required int                             TileX                 { get; init; }
        public required int                             TileZ                 { get; init; }
        public required long                            TotalTicks            { get; init; }
        public required long[]                          PhaseTicks            { get; init; }
        public required int                             GeometryJobCount      { get; init; }
        public required int                             TerrainJobCount       { get; init; }
        public required int                             PrimitiveCount        { get; init; }
        public required int                             UniqueJobCount        { get; init; }
        public required int                             EstimatedSpanWeight   { get; init; }
        public required int                             PreCompactSpanCount   { get; init; }
        public required int                             PolyCount             { get; init; }
        public required int                             VertCount             { get; init; }
        public required int                             DetailTriCount        { get; init; }
        public required int                             GeneratedClimbLinks   { get; init; }
        public required int                             GeneratedJumpLinks    { get; init; }
        public          DtMeshData?                     MeshData;
        public          VoxelMap.RootColumnBuildResult? VolumeColumn;
        public          RcBuilderResult?                DebugResult;
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

    public NavmeshSettings        Settings;
    public SceneExtractor         Scene;
    public Vector3                BoundsMin;
    public Vector3                BoundsMax;
    public int                    NumTilesX;
    public int                    NumTilesZ;
    public bool                   Flyable;
    public string                 BuildSignature;
    public Navmesh                Navmesh; // should not be accessed while building tiles
    public BuildTelemetrySummary? LastBuildTelemetry { get; private set; }

    private readonly NavmeshCustomization                                               _customization;
    private readonly TileBuildInput[]                         _tileInputs;
    private readonly int[]                                    _tileBuildOrder;
    private readonly RasterJob[]                              _geometryJobs;
    private readonly RasterJob[]                              _terrainJobs;
    private readonly ThreadLocal<BuildThreadScratch>                                    _threadScratch = new(() => new(), true);
    private readonly float                                                              _tileWidthWorld;
    private readonly float                                                              _tileHeightWorld;
    private readonly float                                                              _invTileWidthWorld;
    private readonly float                                                              _invTileHeightWorld;
    private readonly int                                                                _uniqueRasterJobCount;
    private readonly int                                                                _totalRasterJobReferences;
    private readonly long                                                               _preparedTerrainBytes;

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

        BoundsMin = new(WorldBoundsMin);
        BoundsMax = new(WorldBoundsMax);
        NumTilesX = NumTilesZ = ResolveGroundTileCount(Scene, Settings);
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
        var volume  = Flyable ? new VoxelMap(BoundsMin, BoundsMax, volumeTiles) : null;
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
        _tileInputs              = bucketedInputs.Inputs;
        _tileBuildOrder          = bucketedInputs.TileBuildOrder;
        _geometryJobs            = bucketedInputs.GeometryJobs;
        _terrainJobs             = bucketedInputs.TerrainJobs;
        _uniqueRasterJobCount    = bucketedInputs.UniqueRasterJobCount;
        _totalRasterJobReferences = bucketedInputs.TotalRasterJobReferences;
        _preparedTerrainBytes    = bucketedInputs.PreparedTerrainBytes;
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

    private static int ResolveGroundTileCount(SceneExtractor scene, NavmeshSettings settings)
    {
        var min = new Vector3(float.MaxValue);
        var max = new Vector3(float.MinValue);
        var found = false;

        foreach (var mesh in scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                min = Vector3.Min(min, instance.WorldBounds.Min);
                max = Vector3.Max(max, instance.WorldBounds.Max);
                found = true;
            }
        }

        if (!found)
            return 16;

        var occupiedSpan = MathF.Max(max.X - min.X, max.Z - min.Z);
        var targetTileCount = (int)MathF.Ceiling(occupiedSpan / settings.GroundTileSize);
        targetTileCount = Math.Clamp(targetTileCount, 1, settings.GroundTileCountMax);
        var pow2 = 1;
        while (pow2 < targetTileCount)
            pow2 <<= 1;

        return Math.Clamp(pow2, 1, settings.GroundTileCountMax);
    }
}
