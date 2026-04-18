using System.Diagnostics;
using System.Numerics;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.Extras.Jumplink;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
    private TileBuildResult[] BuildTileResults(bool captureIntermediates, Action<int>? onTileFinished)
    {
        var tileCount   = NumTilesX * NumTilesZ;
        var threadCount = ResolveThreadCount();
        var builtTiles  = new TileBuildResult[tileCount];
        var buildTimer  = StopWatchTimer.Create();
        var nextIndex   = -1;

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
            _config.BuildMaxCores,
            Environment.ProcessorCount,
            threadCount,
            _uniqueRasterJobCount,
            _totalRasterJobReferences,
            _preparedTerrainBytes
        );
        Service.Log.Debug
        (
            $"[NavmeshBuilder] 并行瓦片构建耗时 {parallelDuration.TotalMilliseconds:f1} ms，配置核心数 = {_config.BuildMaxCores}，可用核心数 = {Environment.ProcessorCount}，实际线程数 = {threadCount}"
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
                MergeTileColumn(Navmesh.Volume, tileIndex % NumTilesX, tileIndex / NumTilesX, built.VolumeColumn);

            if (collectIntermediates && built.DebugResult != null)
                debugResults.Add(built.DebugResult);
        }

        Navmesh.GeneratedClimbDownLinkCount = climbLinks;
        Navmesh.GeneratedEdgeJumpLinkCount  = jumpLinks;
        Service.Log.Debug($"[NavmeshBuilder] 结果合并耗时 {mergeTimer.Value().TotalMilliseconds:f1} ms");
        return debugResults;
    }

    private static void MergeTileColumn(VoxelMap parent, int x, int z, VoxelMap.RootColumnBuildResult column)
    {
        var shift       = parent.RootTile.SubdivisionCount;
        var ny          = parent.Levels[0].NumCellsY;
        var parentIndex = parent.Levels[0].VoxelToIndex(x, 0, z);

        for (var y = 0; y < ny; ++y)
        {
            var contents = column.Contents[y];
            if ((contents & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                continue;

            if ((contents & VoxelMap.VOXEL_ID_MASK) != VoxelMap.VOXEL_ID_MASK)
                contents += (ushort)shift;

            parent.RootTile.Contents[parentIndex + y] = contents;
        }

        parent.RootTile.AddSubdivisions(column.Subdivision);
    }

    private TileBuildResult BuildTileCore(int x, int z, TileBuildInput input, bool captureIntermediates, Action<int>? onTileProgress)
    {
        var scratch = _threadScratch.Value!;
        scratch.Reset();
        EnsureVolumeScratch(scratch);
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

        var preCompactSpanCount = CountWalkableSpans(shf);
        var chf                 = RcCompacts.BuildCompactHeightfield(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);
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

        VoxelMap.RootColumnBuildResult? volumeColumn = null;

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

    private static int CountWalkableSpans(RcHeightfield heightfield)
    {
        var spanCount = 0;
        var cellCount = heightfield.width * heightfield.height;

        for (var cellIndex = 0; cellIndex < cellCount; ++cellIndex)
        for (var span = heightfield.spans[cellIndex]; span != null; span = span.next)
            if (span.area != RcRecast.RC_NULL_AREA)
                ++spanCount;

        return spanCount;
    }

    private static double TicksToMilliseconds(long ticks)
        => ticks / (double)TimeSpan.TicksPerMillisecond;
}
