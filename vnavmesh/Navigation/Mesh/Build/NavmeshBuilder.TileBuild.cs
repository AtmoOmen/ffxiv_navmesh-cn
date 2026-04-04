using System.Diagnostics;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.Extras.Jumplink;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
    private TileBuildResult[] BuildTileResults(bool captureIntermediates, Action? onTileFinished)
    {
        var tileCount   = NumTilesX * NumTilesZ;
        var threadCount = ResolveThreadCount();
        var builtTiles  = new TileBuildResult[tileCount];
        var buildTimer  = StopWatchTimer.Create();

        Parallel.ForEach
        (
            _tileBuildOrder,
            new ParallelOptions { MaxDegreeOfParallelism = threadCount },
            tileIndex =>
            {
                var x = tileIndex % NumTilesX;
                var z = tileIndex / NumTilesX;
                builtTiles[tileIndex] = BuildTileCore(x, z, _tileInputs[tileIndex], captureIntermediates);
                onTileFinished?.Invoke();
            }
        );

        var parallelDuration = buildTimer.Value();
        LastBuildTelemetry = SummarizeBuildTelemetry(builtTiles, parallelDuration);
        Service.Log.Debug($"[NavmeshBuilder] 并行瓦片构建耗时 {parallelDuration.TotalMilliseconds:f1} ms，线程数 = {threadCount}");
        LogBuildTelemetry(LastBuildTelemetry);
        return builtTiles;
    }

    private List<RcBuilderResult> MergeBuiltTiles(IReadOnlyList<TileBuildResult> builtTiles, bool collectIntermediates)
    {
        var                   mergeTimer   = StopWatchTimer.Create();
        List<RcBuilderResult> debugResults = collectIntermediates ? new(builtTiles.Count) : [];

        for (var tileIndex = 0; tileIndex < builtTiles.Count; ++tileIndex)
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
        var shift       = parent.RootTile.Subdivision.Count;
        var ny          = parent.Levels[0].NumCellsY;
        var parentIndex = parent.Levels[0].VoxelToIndex(x, 0, z);

        for (var y = 0; y < ny; ++y)
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
        EnsureVolumeScratch(scratch);
        var totalStart        = Stopwatch.GetTimestamp();
        var telemetry         = new RcContext();
        var geometryInstances = _geometryInstances.AsSpan(input.GeometryStart, input.GeometryCount);
        var terrainInstances  = _terrainInstances.AsSpan(input.TerrainStart, input.TerrainCount);

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
            scratch.Rasterizer
        );

        var phaseStart = Stopwatch.GetTimestamp();
        rasterizer.Rasterize(geometryInstances, SceneExtractor.MeshType.All, true, true);
        scratch.PhaseTicks[(int)BuildPhase.RasterizeGeometry] += ElapsedTimeSpanTicks(phaseStart);

        if (!terrainInstances.IsEmpty)
        {
            phaseStart = Stopwatch.GetTimestamp();
            rasterizer.Rasterize(terrainInstances, SceneExtractor.MeshType.All, false, true);
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

        var regionMinArea   = (int)(Settings.RegionMinSize   * Settings.RegionMinSize);
        var regionMergeArea = (int)(Settings.RegionMergeSize * Settings.RegionMergeSize);

        if (Settings.Partitioning == RcPartition.WATERSHED)
        {
            RcRegions.BuildDistanceField(telemetry, chf);
            RcRegions.BuildRegions(telemetry, chf, regionMinArea, regionMergeArea);
        }
        else if (Settings.Partitioning == RcPartition.MONOTONE) RcRegions.BuildRegionsMonotone(telemetry, chf, regionMinArea, regionMergeArea);
        else RcRegions.BuildLayerRegions(telemetry, chf, regionMinArea);

        var polyMaxEdgeLenVoxels = (int)(Settings.PolyMaxEdgeLen / Settings.CellSize);
        var cset = RcContours.BuildContours
            (telemetry, chf, Settings.PolyMaxSimplificationError, polyMaxEdgeLenVoxels, RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES);

        var pmesh = RcMeshs.BuildPolyMesh(telemetry, cset, Settings.PolyMaxVerts);
        for (var i = 0; i < pmesh.npolys; ++i)
            pmesh.flags[i] = 1;

        var detailSampleDist     = Settings.DetailSampleDist < 0.9f ? 0 : Settings.CellSize * Settings.DetailSampleDist;
        var detailSampleMaxError = Settings.CellHeight * Settings.DetailMaxSampleError;
        var dmesh                = RcMeshDetails.BuildPolyMeshDetail(telemetry, pmesh, chf, detailSampleDist, detailSampleMaxError);

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

        RcBuilderResult? builderResult   = null;
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
                        navmeshConfig.AddOffMeshConnection(p.RecastToSystem(), q.RecastToSystem(), Settings.AgentRadius);
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
                var cfg = new JumpLinkBuilderConfig
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
                addConnections(jumpLinkBuilder.Build(cfg, JumpLinkType.EDGE_CLIMB_DOWN));
            }

            if (Settings.GenerateEdgeJumpLinks)
            {
                var cfg = new JumpLinkBuilderConfig
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
                addConnections(jumpLinkBuilder.Build(cfg, JumpLinkType.EDGE_JUMP));
            }

            scratch.PhaseTicks[(int)BuildPhase.BuildJumpLinks] += ElapsedTimeSpanTicks(phaseStart);
        }

        VoxelMap.RootColumnBuildResult? volumeColumn = null;

        if (Navmesh.Volume != null && vox != null)
        {
            phaseStart                                            =  Stopwatch.GetTimestamp();
            volumeColumn                                          =  Navmesh.Volume.BuildRootColumn(vox, x, z);
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
            TileX                 = x,
            TileZ                 = z,
            TotalTicks            = totalTicks,
            PhaseTicks            = phaseTicks,
            GeometryInstanceCount = input.GeometryCount,
            TerrainInstanceCount  = input.TerrainCount,
            PolyCount             = pmesh.npolys,
            VertCount             = pmesh.nverts,
            DetailTriCount        = dmesh?.ntris ?? 0,
            MeshData              = navmeshData,
            VolumeColumn          = volumeColumn,
            DebugResult           = captureIntermediates ? builderResult : null
        };
    }

    private static long ElapsedTimeSpanTicks(long startTimestamp)
        => (long)((Stopwatch.GetTimestamp() - startTimestamp) * (double)TimeSpan.TicksPerSecond / Stopwatch.Frequency);

    private static double TicksToMilliseconds(long ticks)
        => ticks / (double)TimeSpan.TicksPerMillisecond;
}
