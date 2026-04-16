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
            if ((contents & VoxelMap.VoxelOccupiedBit) == 0)
                continue;

            if ((contents & VoxelMap.VoxelIdMask) != VoxelMap.VoxelIdMask)
                contents += (ushort)shift;

            parent.RootTile.Contents[parentIndex + y] = contents;
        }

        parent.RootTile.AddSubdivisions(column.Subdivision);
    }

    private TileBuildResult BuildTileCore(int x, int z, TileBuildInput input, bool captureIntermediates)
    {
        var scratch = _threadScratch.Value!;
        scratch.Reset();
        EnsureVolumeScratch(scratch);
        var totalStart   = Stopwatch.GetTimestamp();
        var telemetry    = new RcContext();
        var geometryJobs = _geometryJobs.AsSpan(input.GeometryJobStart, input.GeometryJobCount);
        var terrainJobs  = _terrainJobs.AsSpan(input.TerrainJobStart, input.TerrainJobCount);

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
        rasterizer.Rasterize(geometryJobs, true, true);
        scratch.PhaseTicks[(int)BuildPhase.RasterizeGeometry] += ElapsedTimeSpanTicks(phaseStart);

        if (!terrainJobs.IsEmpty)
        {
            phaseStart = Stopwatch.GetTimestamp();
            rasterizer.Rasterize(terrainJobs, false, true);
            scratch.PhaseTicks[(int)BuildPhase.RasterizeTerrain] += ElapsedTimeSpanTicks(phaseStart);
        }

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LowHangingObstacles))
            RcFilters.FilterLowHangingWalkableObstacles(telemetry, _walkableClimbVoxels, shf);

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.LedgeSpans))
            RcFilters.FilterLedgeSpans(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);

        if (Settings.Filtering.HasFlag(NavmeshSettings.Filter.WalkableLowHeightSpans))
            RcFilters.FilterWalkableLowHeightSpans(telemetry, _walkableHeightVoxels, shf);

        var preCompactSpanCount = CountWalkableSpans(shf);
        var chf                 = RcCompacts.BuildCompactHeightfield(telemetry, _walkableHeightVoxels, _walkableClimbVoxels, shf);
        RcAreas.ErodeWalkableArea(telemetry, _walkableRadiusVoxels, chf);
        RcAreas.MedianFilterWalkableArea(telemetry, chf);

        var regionMinArea   = (int)(Settings.RegionMinSize   * Settings.RegionMinSize);
        var regionMergeArea = (int)(Settings.RegionMergeSize * Settings.RegionMergeSize);

        if (Settings.Partitioning == RcPartition.WATERSHED)
        {
            RcRegions.BuildDistanceField(telemetry, chf);
            RcRegions.BuildRegions(telemetry, chf, regionMinArea, regionMergeArea);
        }
        else if (Settings.Partitioning == RcPartition.MONOTONE) RcRegions.BuildRegionsMonotone(telemetry, chf, regionMinArea, regionMergeArea);
        else RcRegions.BuildLayerRegions(telemetry, chf, regionMinArea);

        var effectivePolyMaxEdgeLen = Settings.PolyMaxEdgeLen > 0 ? Settings.PolyMaxEdgeLen : Settings.AgentRadius * 8f;
        var polyMaxEdgeLenVoxels = (int)(effectivePolyMaxEdgeLen / Settings.CellSize);
        var cset = RcContours.BuildContours
            (telemetry, chf, Settings.PolyMaxSimplificationError, polyMaxEdgeLenVoxels, RcBuildContoursFlags.RC_CONTOUR_TESS_WALL_EDGES);

        var pmesh = RcMeshs.BuildPolyMesh(telemetry, cset, Settings.PolyMaxVerts);
        for (var i = 0; i < pmesh.npolys; ++i)
        {
            var area = pmesh.areas[i] == 0 ? NavmeshArea.Null : NavmeshArea.Ground;
            pmesh.areas[i] = (int)area;
            pmesh.flags[i] = area == NavmeshArea.Null ? (int)NavmeshPolyFlags.None : (int)NavmeshPolyFlags.Ground;
        }

        var detailSampleDist     = Settings.FastBuild || Settings.DetailSampleDist < 0.9f ? 0 : Settings.CellSize * Settings.DetailSampleDist;
        var detailSampleMaxError = Settings.CellHeight * Settings.DetailMaxSampleError;
        RcPolyMeshDetail? dmesh  = null;
        if (detailSampleDist > 0 && pmesh.npolys > 0)
            dmesh = RcMeshDetails.BuildPolyMeshDetail(telemetry, pmesh, chf, detailSampleDist, detailSampleMaxError);

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
        DtJumpLinkBuilder? jumpLinkBuilder = null;
        var generatedClimbLinks = 0;
        var generatedJumpLinks  = 0;

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

                    var start = p.RecastToSystem();
                    var end   = q.RecastToSystem();
                    var delta = end - start;
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

        static int Quantize(float value) => (int)MathF.Round(value * 4f);

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
        return new()
        {
            TileX                 = x,
            TileZ                 = z,
            TotalTicks            = totalTicks,
            PhaseTicks            = phaseTicks,
            GeometryJobCount      = input.GeometryJobCount,
            TerrainJobCount       = input.TerrainJobCount,
            PrimitiveCount        = input.PrimitiveCount,
            UniqueJobCount        = input.GeometryJobCount + input.TerrainJobCount,
            EstimatedSpanWeight   = input.EstimatedSpanWeight,
            PreCompactSpanCount   = preCompactSpanCount,
            PolyCount             = pmesh.npolys,
            VertCount             = pmesh.nverts,
            DetailTriCount        = dmesh?.ntris ?? 0,
            GeneratedClimbLinks   = generatedClimbLinks,
            GeneratedJumpLinks    = generatedJumpLinks,
            MeshData              = navmeshData,
            VolumeColumn          = volumeColumn,
            DebugResult           = captureIntermediates ? builderResult : null
        };
    }

    private static long ElapsedTimeSpanTicks(long startTimestamp)
        => (long)((Stopwatch.GetTimestamp() - startTimestamp) * (double)TimeSpan.TicksPerSecond / Stopwatch.Frequency);

    private static int CountWalkableSpans(RcHeightfield heightfield)
    {
        var spanCount  = 0;
        var cellCount  = heightfield.width * heightfield.height;

        for (var cellIndex = 0; cellIndex < cellCount; ++cellIndex)
        {
            for (var span = heightfield.spans[cellIndex]; span != null; span = span.next)
            {
                if (span.area != DotRecast.Recast.RcRecast.RC_NULL_AREA)
                    ++spanCount;
            }
        }

        return spanCount;
    }

    private static double TicksToMilliseconds(long ticks)
        => ticks / (double)TimeSpan.TicksPerMillisecond;
}
