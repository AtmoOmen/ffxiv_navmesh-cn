using System;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Recast;
using Navmesh.NavVolume;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Navmesh.Utilities;

namespace Navmesh;

// utility for building a navmesh from scene data
// individual tiles can be built concurrently
public class NavmeshBuilder
{
    public record struct Intermediates(
        RcHeightfield        SolidHeightfield,
        RcCompactHeightfield CompactHeightfield,
        RcContourSet         ContourSet,
        RcPolyMesh           PolyMesh,
        RcPolyMeshDetail?    DetailMesh);

    public RcContext       Telemetry = new();
    public NavmeshSettings Settings;
    public SceneExtractor  Scene;
    public Vector3         BoundsMin;
    public Vector3         BoundsMax;
    public int             NumTilesX;
    public int             NumTilesZ;
    public Navmesh         Navmesh; // should not be accessed while building tiles

    private NavmeshCustomization customization;

    private int   walkableClimbVoxels;
    private int   walkableHeightVoxels;
    private int   walkableRadiusVoxels;
    private float walkableNormalThreshold;
    private int   borderSizeVoxels;
    private float borderSizeWorld;
    private int   tileSizeXVoxels;
    private int   tileSizeZVoxels;
    private int   voxelizerNumX = 1;
    private int   voxelizerNumY = 1;
    private int   voxelizerNumZ = 1;

    public NavmeshBuilder(SceneDefinition scene, NavmeshCustomization customization)
    {
        Settings = customization.Settings;
        var flyable = customization.IsFlyingSupported(scene);
        this.customization = customization;

        // load all meshes
        Scene = new(scene);
        customization.CustomizeScene(Scene);

        BoundsMin = new(-1024);
        BoundsMax = new(1024);
        NumTilesX = NumTilesZ = Settings.NumTiles[0];
        Service.Log.Debug($"starting building {NumTilesX}x{NumTilesZ} navmesh, customization = {customization.GetType()} v{customization.Version}");

        // create empty navmesh
        var navmeshParams = new DtNavMeshParams
        {
            orig       = BoundsMin.SystemToRecast(),
            tileWidth  = (BoundsMax.X - BoundsMin.X) / NumTilesX,
            tileHeight = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ,
            maxTiles   = NumTilesX                   * NumTilesZ,
            maxPolys   = 1 << DtNavMesh.DT_POLY_BITS
        };

        var navmesh = new DtNavMesh(navmeshParams, Settings.PolyMaxVerts);
        var volume  = flyable ? new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles) : null;
        Navmesh = new(customization.Version, navmesh, volume);

        // calculate derived parameters
        walkableClimbVoxels     = (int)MathF.Floor(Settings.AgentMaxClimb / Settings.CellHeight);
        walkableHeightVoxels    = (int)MathF.Ceiling(Settings.AgentHeight / Settings.CellHeight);
        walkableRadiusVoxels    = (int)MathF.Ceiling(Settings.AgentRadius / Settings.CellSize);
        walkableNormalThreshold = Settings.AgentMaxSlopeDeg.Degrees().Cos();
        borderSizeVoxels        = 3 + walkableRadiusVoxels;
        borderSizeWorld         = borderSizeVoxels * Settings.CellSize;
        tileSizeXVoxels         = (int)MathF.Ceiling(navmeshParams.tileWidth  / Settings.CellSize) + (2 * borderSizeVoxels);
        tileSizeZVoxels         = (int)MathF.Ceiling(navmeshParams.tileHeight / Settings.CellSize) + (2 * borderSizeVoxels);
        if (volume != null)
        {
            voxelizerNumY = Settings.NumTiles[0];
            for (var i = 1; i < Settings.NumTiles.Length; ++i)
            {
                var n = Settings.NumTiles[i];
                voxelizerNumX *= n;
                voxelizerNumY *= n;
                voxelizerNumZ *= n;
            }
        }
    }

    public List<((int X, int Z), Intermediates Intermediates)> BuildTiles(Action? onTileFinished = null)
    {
        var tiles = new List<((int X, int Z), Intermediates Intermediates)>();
        var tasks = new List<((int, int), Task<(DtMeshData?, Voxelizer?, Intermediates)>)>();

        for (var z = 0; z < NumTilesZ; z++)
        {
            for (var x = 0; x < NumTilesX; x++)
            {
                var z0 = z;
                var x0 = x;
                tasks.Add(((x0, z0), Task.Run(() =>
                {
                    var data = BuildTile(x0, z0);
                    onTileFinished?.Invoke();
                    return data;
                })));
            }
        }

        Task.WaitAll(tasks.Select(t => t.Item2));

        foreach (var ((x, z), t) in tasks)
        {
            var (tile, vox, i) = t.Result;
            tiles.Add(((x, z), i));
            if (tile != null)
                Navmesh.Mesh.AddTile(tile, 0, 0);
            if (vox != null)
                Navmesh.Volume?.Build(vox, x, z);
        }

        return tiles;
    }

    // this can be called concurrently; returns intermediate data that can be discarded if not used
    public (DtMeshData?, Voxelizer?, Intermediates) BuildTile(int x, int z)
    {
        var timer = Timer.Create();

        // 0. calculate tile bounds
        // we expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile
        // this is done in order to make sure that the navmesh tiles connect correctly at the borders, and the obstacles close to the border work correctly with the dilation process
        // no polygons (or contours) will be created on the border area
        var widthWorld    = Navmesh.Mesh.GetParams().tileWidth;
        var heightWorld   = Navmesh.Mesh.GetParams().tileHeight;
        var tileBoundsMin = new Vector3(BoundsMin.X     + (x * widthWorld), BoundsMin.Y, BoundsMin.Z     + (z * heightWorld));
        var tileBoundsMax = new Vector3(tileBoundsMin.X + widthWorld,       BoundsMax.Y, tileBoundsMin.Z + heightWorld);
        tileBoundsMin.X -= borderSizeWorld;
        tileBoundsMin.Z -= borderSizeWorld;
        tileBoundsMax.X += borderSizeWorld;
        tileBoundsMax.Z += borderSizeWorld;

        // 1. voxelize raw geometry
        // this creates a 'solid heightfield', which is a grid of sorted linked lists of spans
        // each span contains an 'area id', which is either walkable (if normal is good) or not (otherwise); areas outside spans contains no geometry at all
        var shf = new RcHeightfield(tileSizeXVoxels, tileSizeZVoxels, tileBoundsMin.SystemToRecast(), tileBoundsMax.SystemToRecast(), Settings.CellSize,
                                    Settings.CellHeight, borderSizeVoxels);
        var vox = Navmesh.Volume != null ? new Voxelizer(voxelizerNumX, voxelizerNumY, voxelizerNumZ) : null;
        var rasterizer = new NavmeshRasterizer(shf, walkableNormalThreshold, walkableClimbVoxels, walkableHeightVoxels,
                                               Settings.Filtering.HasFlag(NavmeshSettings.Filter.Interiors), vox, Telemetry);
        rasterizer.Rasterize(Scene, SceneExtractor.MeshType.FileMesh | SceneExtractor.MeshType.CylinderMesh | SceneExtractor.MeshType.AnalyticShape, true,
                             true);                                                                                        // rasterize normal geometry
        rasterizer.Rasterize(Scene, SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane, false, true); // rasterize terrain and bounding planes

        // 2. perform a bunch of postprocessing on a heightfield
        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LowHangingObstacles))
        {
            // mark non-walkable spans as walkable if their maximum is within climb distance of the span below
            // this allows climbing stairs, walking over curbs, etc
            RcFilters.FilterLowHangingWalkableObstacles(Telemetry, walkableClimbVoxels, shf);
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LedgeSpans))
        {
            // mark 'ledge' spans as non-walkable - spans that have too large height distance to the neighbour
            // this reduces the impact of voxelization error
            RcFilters.FilterLedgeSpans(Telemetry, walkableHeightVoxels, walkableClimbVoxels, shf);
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.WalkableLowHeightSpans))
        {
            // mark walkable spans of very low height (smaller than agent height) as non-walkable (TODO: do we still need it?)
            RcFilters.FilterWalkableLowHeightSpans(Telemetry, walkableHeightVoxels, shf);
        }

        // 3. create a 'compact heightfield' structure
        // this is very similar to a normal heightfield, except that spans are now stored in a single array, and grid cells just contain consecutive ranges
        // this also contains connectivity data (links to neighbouring cells)
        // note that spans from null areas are not added to the compact heightfield
        // also note that for each span, y is equal to the solid span's smax (makes sense - in solid, walkable voxel is one containing walkable geometry, so free area is 'above')
        // h is not really used beyond connectivity calculations (it's a distance to the next span - potentially of null area - or to maxheight)
        var chf = RcCompacts.BuildCompactHeightfield(Telemetry, walkableHeightVoxels, walkableClimbVoxels, shf);

        // 4. mark spans that are too close to unwalkable as unwalkable, to account for actor's non-zero radius
        // this changes area of some spans from walkable to non-walkable
        // note that before this step, compact heightfield has no non-walkable spans
        RcAreas.ErodeWalkableArea(Telemetry, walkableRadiusVoxels, chf);
        // note: this is the good time to mark convex poly areas with custom area ids

        // 5. build connected regions; this assigns region ids to spans in the compact heightfield
        // there are different algorithms with different tradeoffs
        var regionMinArea   = (int)(Settings.RegionMinSize   * Settings.RegionMinSize);
        var regionMergeArea = (int)(Settings.RegionMergeSize * Settings.RegionMergeSize);
        switch (Settings.Partitioning)
        {
            case RcPartition.WATERSHED:
                RcRegions.BuildDistanceField(Telemetry, chf);
                RcRegions.BuildRegions(Telemetry, chf, regionMinArea, regionMergeArea);
                break;
            case RcPartition.MONOTONE:
                RcRegions.BuildRegionsMonotone(Telemetry, chf, regionMinArea, regionMergeArea);
                break;
            default:
                RcRegions.BuildLayerRegions(Telemetry, chf, regionMinArea);
                break;
        }

        // 6. build contours around regions, then simplify them to reduce vertex count
        // contour set is just a list of contours, each of which is (when projected to XZ plane) a simple non-convex polygon that belong to a single region with a single area id
        var polyMaxEdgeLenVoxels = (int)(Settings.PolyMaxEdgeLen / Settings.CellSize);
        var cset = RcContours.BuildContours(Telemetry, chf, Settings.PolyMaxSimplificationError, polyMaxEdgeLenVoxels,
                                            RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES);

        // 7. triangulate contours to build a mesh of convex polygons with adjacency information
        var pmesh = RcMeshs.BuildPolyMesh(Telemetry, cset, Settings.PolyMaxVerts);
        for (var i = 0; i < pmesh.npolys; ++i)
            pmesh.flags[i] = 1;

        // 8. split polygonal mesh into triangular mesh with correct height
        // this step is optional
        var detailSampleDist     = Settings.DetailSampleDist < 0.9f ? 0 : Settings.CellSize * Settings.DetailSampleDist;
        var detailSampleMaxError = Settings.CellHeight * Settings.DetailMaxSampleError;
        var dmesh                = RcMeshDetails.BuildPolyMeshDetail(Telemetry, pmesh, chf, detailSampleDist, detailSampleMaxError);

        // 9. create detour navmesh data
        var navmeshConfig = new DtNavMeshCreateParams
        {
            verts     = pmesh.verts,
            vertCount = pmesh.nverts,
            polys     = pmesh.polys,
            polyFlags = pmesh.flags,
            polyAreas = pmesh.areas,
            polyCount = pmesh.npolys,
            nvp       = pmesh.nvp,

            detailMeshes     = dmesh?.meshes,
            detailVerts      = dmesh?.verts,
            detailVertsCount = dmesh?.nverts ?? 0,
            detailTris       = dmesh?.tris,
            detailTriCount   = dmesh?.ntris ?? 0,

            tileX     = x,
            tileZ     = z,
            tileLayer = 0, // TODO: do we care to use layers?..
            bmin      = pmesh.bmin,
            bmax      = pmesh.bmax,

            walkableHeight = Settings.AgentHeight,
            walkableRadius = Settings.AgentRadius,
            walkableClimb  = Settings.AgentMaxClimb,
            cs             = Settings.CellSize,
            ch             = Settings.CellHeight,

            buildBvTree = true // TODO: false if using layers?
        };
        customization.CustomizeSettings(navmeshConfig);
        var navmeshData = DtNavMeshBuilder.CreateNavMeshData(navmeshConfig);

        //// 10. add tile to the navmesh
        //if (navmeshData != null)
        //{
        //    lock (Navmesh.Mesh)
        //    {
        //        Navmesh.Mesh.AddTile(navmeshData, 0, 0);
        //    }
        //}

        //// 11. build nav volume data
        //// TODO: keep local 1x1x16 voxel map, and just merge under lock
        //if (Navmesh.Volume != null && vox != null)
        //{
        //    lock (Navmesh.Volume)
        //    {
        //        Navmesh.Volume.Build(vox, x, z);
        //    }
        //}

        Service.Log.Debug($"构建导航区块 {x}x{z} 完成 (所用时间: {timer.Value().TotalMilliseconds}ms)");
        return (navmeshData, vox, new(shf, chf, cset, pmesh, dmesh));
    }
}
