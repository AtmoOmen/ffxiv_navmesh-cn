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
        public required string Name         { get; init; }
        public required long   TotalTicks   { get; init; }
        public required long   AverageTicks { get; init; }
        public required long   MaxTicks     { get; init; }
        public required int    SlowestTileX { get; init; }
        public required int    SlowestTileZ { get; init; }
    }

    public sealed class SlowTileSummary
    {
        public required int  TileX                 { get; init; }
        public required int  TileZ                 { get; init; }
        public required long TotalTicks            { get; init; }
        public required int  GeometryInstanceCount { get; init; }
        public required int  TerrainInstanceCount  { get; init; }
        public required int  PolyCount             { get; init; }
        public required int  VertCount             { get; init; }
        public required int  DetailTriCount        { get; init; }
    }

    public sealed class BuildTelemetrySummary
    {
        public required long                             ParallelTicks { get; init; }
        public required IReadOnlyList<BuildPhaseSummary> Phases        { get; init; }
        public required IReadOnlyList<SlowTileSummary>   SlowTiles     { get; init; }
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
        public required int GeometryStart { get; init; }
        public required int GeometryCount { get; init; }
        public required int TerrainStart  { get; init; }
        public required int TerrainCount  { get; init; }
    }

    private sealed class TileBuildResult
    {
        public required int                             TileX                 { get; init; }
        public required int                             TileZ                 { get; init; }
        public required long                            TotalTicks            { get; init; }
        public required long[]                          PhaseTicks            { get; init; }
        public required int                             GeometryInstanceCount { get; init; }
        public required int                             TerrainInstanceCount  { get; init; }
        public required int                             PolyCount             { get; init; }
        public required int                             VertCount             { get; init; }
        public required int                             DetailTriCount        { get; init; }
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
    private readonly TileBuildInput[]                                                   _tileInputs;
    private readonly int[]                                                              _tileBuildOrder;
    private readonly (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] _geometryInstances;
    private readonly (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] _terrainInstances;
    private readonly ThreadLocal<BuildThreadScratch>                                    _threadScratch = new(() => new(), true);
    private readonly float                                                              _tileWidthWorld;
    private readonly float                                                              _tileHeightWorld;
    private readonly float                                                              _invTileWidthWorld;
    private readonly float                                                              _invTileHeightWorld;

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

        BoundsMin = new(-1024);
        BoundsMax = new(1024);
        NumTilesX = NumTilesZ = Settings.NumTiles[0];
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
        var volume  = Flyable ? new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles) : null;
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
            _voxelizerNumY = Settings.NumTiles[0];

            for (var i = 1; i < Settings.NumTiles.Length; ++i)
            {
                var n = Settings.NumTiles[i];
                _voxelizerNumX *= n;
                _voxelizerNumY *= n;
                _voxelizerNumZ *= n;
            }
        }

        var bucketTimer    = StopWatchTimer.Create();
        var bucketedInputs = BucketTileInputs();
        _tileInputs        = bucketedInputs.Inputs;
        _tileBuildOrder    = bucketedInputs.TileBuildOrder;
        _geometryInstances = bucketedInputs.GeometryInstances;
        _terrainInstances  = bucketedInputs.TerrainInstances;
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
}
