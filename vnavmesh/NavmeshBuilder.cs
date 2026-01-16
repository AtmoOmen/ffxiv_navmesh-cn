using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.Extras.Jumplink;
using DotRecast.Recast;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;

namespace Navmesh;

// utility for building a navmesh from scene data
// individual tiles can be built concurrently
public class NavmeshBuilder
{
    public record struct Intermediates(RcHeightfield SolidHeightfield, RcCompactHeightfield CompactHeightfield, RcContourSet ContourSet, RcPolyMesh PolyMesh, RcPolyMeshDetail? DetailMesh);

    public RcContext Telemetry = new();
    public NavmeshSettings Settings;
    public SceneExtractor Scene;
    public Vector3 BoundsMin;
    public Vector3 BoundsMax;
    public int NumTilesX;
    public int NumTilesZ;
    public Navmesh Navmesh; // should not be accessed while building tiles

    private NavmeshCustomization customization;

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
        var navmeshParams = new DtNavMeshParams();
        navmeshParams.orig = BoundsMin.SystemToRecast();
        navmeshParams.tileWidth = (BoundsMax.X - BoundsMin.X) / NumTilesX;
        navmeshParams.tileHeight = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ;
        navmeshParams.maxTiles = NumTilesX * NumTilesZ;
        navmeshParams.maxPolys = 1 << DtNavMesh.DT_POLY_BITS;

        var navmesh = new DtNavMesh(navmeshParams, Settings.PolyMaxVerts);
        var volume = flyable ? new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles) : null;
        Navmesh = new(customization.Version, navmesh, volume);

        // calculate derived parameters
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
    }

    // TODO this is kinda more complicated than it needs to be because we're trying to maintain tile order in the output mesh
    public List<RcBuilderResult> BuildTiles(Action? onTileFinished = null)
    {
        int tileCount = NumTilesX * NumTilesZ;
        var tileCounts = new int[tileCount];

        float tileWidth = (BoundsMax.X - BoundsMin.X) / NumTilesX;
        float tileHeight = (BoundsMax.Z - BoundsMin.Z) / NumTilesZ;
        float invTileWidth = 1.0f / tileWidth;
        float invTileHeight = 1.0f / tileHeight;

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var inst in mesh.Instances)
            {
                int minX = (int)MathF.Floor((inst.WorldBounds.Min.X - _borderSizeWorld - BoundsMin.X) * invTileWidth);
                int maxX = (int)MathF.Floor((inst.WorldBounds.Max.X + _borderSizeWorld - BoundsMin.X) * invTileWidth);
                int minZ = (int)MathF.Floor((inst.WorldBounds.Min.Z - _borderSizeWorld - BoundsMin.Z) * invTileHeight);
                int maxZ = (int)MathF.Floor((inst.WorldBounds.Max.Z + _borderSizeWorld - BoundsMin.Z) * invTileHeight);

                minX = Math.Clamp(minX, 0, NumTilesX - 1);
                maxX = Math.Clamp(maxX, 0, NumTilesX - 1);
                minZ = Math.Clamp(minZ, 0, NumTilesZ - 1);
                maxZ = Math.Clamp(maxZ, 0, NumTilesZ - 1);

                for (int z = minZ; z <= maxZ; ++z)
                {
                    for (int x = minX; x <= maxX; ++x)
                    {
                        tileCounts[z * NumTilesX + x]++;
                    }
                }
            }
        }

        var tileInstances = new List<(SceneExtractor.Mesh, SceneExtractor.MeshInstance)>[tileCount];
        for (int i = 0; i < tileCount; ++i)
            tileInstances[i] = new(tileCounts[i]);

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var inst in mesh.Instances)
            {
                int minX = (int)MathF.Floor((inst.WorldBounds.Min.X - _borderSizeWorld - BoundsMin.X) * invTileWidth);
                int maxX = (int)MathF.Floor((inst.WorldBounds.Max.X + _borderSizeWorld - BoundsMin.X) * invTileWidth);
                int minZ = (int)MathF.Floor((inst.WorldBounds.Min.Z - _borderSizeWorld - BoundsMin.Z) * invTileHeight);
                int maxZ = (int)MathF.Floor((inst.WorldBounds.Max.Z + _borderSizeWorld - BoundsMin.Z) * invTileHeight);

                minX = Math.Clamp(minX, 0, NumTilesX - 1);
                maxX = Math.Clamp(maxX, 0, NumTilesX - 1);
                minZ = Math.Clamp(minZ, 0, NumTilesZ - 1);
                maxZ = Math.Clamp(maxZ, 0, NumTilesZ - 1);

                for (int z = minZ; z <= maxZ; ++z)
                {
                    for (int x = minX; x <= maxX; ++x)
                    {
                        tileInstances[z * NumTilesX + x].Add((mesh, inst));
                    }
                }
            }
        }

        int threadCount;
        var maxThreads = Environment.ProcessorCount;
        var wantedThreads = Service.Config.BuildMaxCores;
        if (wantedThreads <= 0)
            threadCount = maxThreads + wantedThreads;
        else
            threadCount = wantedThreads;
        threadCount = Math.Clamp(threadCount, 1, maxThreads);

        var tileData = new DtMeshData?[tileCount];
        var volumeData = Navmesh.Volume != null ? new VoxelMap?[tileCount] : null;
        var resultData = new RcBuilderResult[tileCount];

        Parallel.For(0, tileCount, new ParallelOptions { MaxDegreeOfParallelism = threadCount }, (tileIndex) =>
        {
            int x = tileIndex % NumTilesX;
            int z = tileIndex / NumTilesX;
            var instances = tileInstances[tileIndex];
            var (tile, vox, rc) = BuildTile(x, z, instances);

            tileData[tileIndex] = tile;
            resultData[tileIndex] = rc;

            if (volumeData != null && vox != null)
            {
                var tileVolume = new VoxelMap(BoundsMin, BoundsMax, Settings.NumTiles);
                tileVolume.Build(vox, x, z);
                volumeData[tileIndex] = tileVolume;
            }

            onTileFinished?.Invoke();
        });

        var results = new List<RcBuilderResult>(tileCount);
        for (int tileIndex = 0; tileIndex < tileCount; ++tileIndex)
        {
            var tile = tileData[tileIndex];
            if (tile != null)
                Navmesh.Mesh.AddTile(tile, 0, 0);

            if (Navmesh.Volume != null && volumeData != null)
            {
                var tileVolume = volumeData[tileIndex];
                if (tileVolume != null)
                    MergeTile(Navmesh.Volume, tileIndex % NumTilesX, tileIndex / NumTilesX, tileVolume);
            }

            results.Add(resultData[tileIndex]);
        }

        return results;
    }

    private static void MergeTile(VoxelMap parent, int x, int z, VoxelMap child)
    {
        var shift = parent.RootTile.Subdivision.Count;

        int ny = parent.Levels[0].NumCellsY;
        int parentIndex = parent.Levels[0].VoxelToIndex(x, 0, z);
        int childIndex = child.Levels[0].VoxelToIndex(x, 0, z);
        for (int y = 0; y < ny; ++y)
        {
            var contents = child.RootTile.Contents[childIndex + y];
            if ((contents & VoxelMap.VoxelOccupiedBit) == 0)
                continue; // empty

            if ((contents & VoxelMap.VoxelIdMask) != VoxelMap.VoxelIdMask)
                contents += (ushort)shift;

            parent.RootTile.Contents[parentIndex + y] = contents;
        }
        parent.RootTile.Subdivision.AddRange(child.RootTile.Subdivision);
    }

    // this can be called concurrently; returns intermediate data that can be discarded if not used
    public (DtMeshData?, Voxelizer?, RcBuilderResult) BuildTile(int x, int z)
    {
        return BuildTile(x, z, IterateAllInstances());
    }

    private IEnumerable<(SceneExtractor.Mesh, SceneExtractor.MeshInstance)> IterateAllInstances()
    {
        foreach (var mesh in Scene.Meshes.Values)
            foreach (var inst in mesh.Instances)
                yield return (mesh, inst);
    }

    public (DtMeshData?, Voxelizer?, RcBuilderResult) BuildTile(int x, int z, IEnumerable<(SceneExtractor.Mesh, SceneExtractor.MeshInstance)> instances)
    {
        var timer = Timer.Create();

        // 0. calculate tile bounds
        // we expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile
        // this is done in order to make sure that the navmesh tiles connect correctly at the borders, and the obstacles close to the border work correctly with the dilation process
        // no polygons (or contours) will be created on the border area
        float widthWorld = Navmesh.Mesh.GetParams().tileWidth;
        float heightWorld = Navmesh.Mesh.GetParams().tileHeight;
        var tileBoundsMin = new Vector3(BoundsMin.X + x * widthWorld, BoundsMin.Y, BoundsMin.Z + z * heightWorld);
        var tileBoundsMax = new Vector3(tileBoundsMin.X + widthWorld, BoundsMax.Y, tileBoundsMin.Z + heightWorld);
        tileBoundsMin.X -= _borderSizeWorld;
        tileBoundsMin.Z -= _borderSizeWorld;
        tileBoundsMax.X += _borderSizeWorld;
        tileBoundsMax.Z += _borderSizeWorld;

        // 1. voxelize raw geometry
        // this creates a 'solid heightfield', which is a grid of sorted linked lists of spans
        // each span contains an 'area id', which is either walkable (if normal is good) or not (otherwise); areas outside spans contains no geometry at all
        var shf = new RcHeightfield(_tileSizeXVoxels, _tileSizeZVoxels, tileBoundsMin.SystemToRecast(), tileBoundsMax.SystemToRecast(), Settings.CellSize, Settings.CellHeight, _borderSizeVoxels);
        var vox = Navmesh.Volume != null ? new Voxelizer(_voxelizerNumX, _voxelizerNumY, _voxelizerNumZ) : null;
        var rasterizer = new NavmeshRasterizer(shf, _walkableNormalThreshold, _walkableClimbVoxels, _walkableHeightVoxels, Settings.Filtering.HasFlag(NavmeshSettings.Filter.Interiors), vox, Telemetry);
        rasterizer.Rasterize(instances, SceneExtractor.MeshType.FileMesh | SceneExtractor.MeshType.CylinderMesh | SceneExtractor.MeshType.AnalyticShape, true, true); // rasterize normal geometry
        rasterizer.Rasterize(instances, SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane, false, true); // rasterize terrain and bounding planes

        // 2. perform a bunch of postprocessing on a heightfield
        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LowHangingObstacles))
        {
            // mark non-walkable spans as walkable if their maximum is within climb distance of the span below
            // this allows climbing stairs, walking over curbs, etc
            RcFilters.FilterLowHangingWalkableObstacles(Telemetry, _walkableClimbVoxels, shf);
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LedgeSpans))
        {
            // mark 'ledge' spans as non-walkable - spans that have too large height distance to the neighbour
            // this reduces the impact of voxelization error
            RcFilters.FilterLedgeSpans(Telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.WalkableLowHeightSpans))
        {
            // mark walkable spans of very low height (smaller than agent height) as non-walkable (TODO: do we still need it?)
            RcFilters.FilterWalkableLowHeightSpans(Telemetry, _walkableHeightVoxels, shf);
        }

        // 3. create a 'compact heightfield' structure
        // this is very similar to a normal heightfield, except that spans are now stored in a single array, and grid cells just contain consecutive ranges
        // this also contains connectivity data (links to neighbouring cells)
        // note that spans from null areas are not added to the compact heightfield
        // also note that for each span, y is equal to the solid span's smax (makes sense - in solid, walkable voxel is one containing walkable geometry, so free area is 'above')
        // h is not really used beyond connectivity calculations (it's a distance to the next span - potentially of null area - or to maxheight)
        var chf = RcCompacts.BuildCompactHeightfield(Telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);

        // 4. mark spans that are too close to unwalkable as unwalkable, to account for actor's non-zero radius
        // this changes area of some spans from walkable to non-walkable
        // note that before this step, compact heightfield has no non-walkable spans
        RcAreas.ErodeWalkableArea(Telemetry, _walkableRadiusVoxels, chf);
        // note: this is the good time to mark convex poly areas with custom area ids

        // 5. build connected regions; this assigns region ids to spans in the compact heightfield
        // there are different algorithms with different tradeoffs
        var regionMinArea = (int)(Settings.RegionMinSize * Settings.RegionMinSize);
        var regionMergeArea = (int)(Settings.RegionMergeSize * Settings.RegionMergeSize);
        if (Settings.Partitioning == RcPartition.WATERSHED)
        {
            RcRegions.BuildDistanceField(Telemetry, chf);
            RcRegions.BuildRegions(Telemetry, chf, regionMinArea, regionMergeArea);
        }
        else if (Settings.Partitioning == RcPartition.MONOTONE)
        {
            RcRegions.BuildRegionsMonotone(Telemetry, chf, regionMinArea, regionMergeArea);
        }
        else
        {
            RcRegions.BuildLayerRegions(Telemetry, chf, regionMinArea);
        }

        // 6. build contours around regions, then simplify them to reduce vertex count
        // contour set is just a list of contours, each of which is (when projected to XZ plane) a simple non-convex polygon that belong to a single region with a single area id
        var polyMaxEdgeLenVoxels = (int)(Settings.PolyMaxEdgeLen / Settings.CellSize);
        var cset = RcContours.BuildContours(Telemetry, chf, Settings.PolyMaxSimplificationError, polyMaxEdgeLenVoxels, RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES);

        // 7. triangulate contours to build a mesh of convex polygons with adjacency information
        var pmesh = RcMeshs.BuildPolyMesh(Telemetry, cset, Settings.PolyMaxVerts);
        for (int i = 0; i < pmesh.npolys; ++i)
            pmesh.flags[i] = 1;

        // 8. split polygonal mesh into triangular mesh with correct height
        // this step is optional
        var detailSampleDist = Settings.DetailSampleDist < 0.9f ? 0 : Settings.CellSize * Settings.DetailSampleDist;
        var detailSampleMaxError = Settings.CellHeight * Settings.DetailMaxSampleError;
        RcPolyMeshDetail? dmesh = RcMeshDetails.BuildPolyMeshDetail(Telemetry, pmesh, chf, detailSampleDist, detailSampleMaxError);

        // 9. create detour navmesh data
        var navmeshConfig = new DtNavMeshCreateParams()
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
            tileLayer = 0, // TODO: do we care to use layers?..
            bmin = pmesh.bmin,
            bmax = pmesh.bmax,

            walkableHeight = Settings.AgentHeight,
            walkableRadius = Settings.AgentRadius,
            walkableClimb = Settings.AgentMaxClimb,
            cs = Settings.CellSize,
            ch = Settings.CellHeight,

            buildBvTree = true, // TODO: false if using layers?
        };
        customization.CustomizeSettings(navmeshConfig);

        var builderResult = new RcBuilderResult(x, z, shf, chf, cset, pmesh, dmesh, Telemetry);
        var bl = new JumpLinkBuilder([builderResult]);

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
            addConnections(bl.Build(cfg, JumpLinkType.EDGE_CLIMB_DOWN));
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
            addConnections(bl.Build(cfg, JumpLinkType.EDGE_JUMP));
        }

        var navmeshData = DtNavMeshBuilder.CreateNavMeshData(navmeshConfig);

        Service.Log.Debug($"built navmesh tile {x}x{z} in {timer.Value().TotalMilliseconds}ms");
        return (navmeshData, vox, builderResult);
    }
}
