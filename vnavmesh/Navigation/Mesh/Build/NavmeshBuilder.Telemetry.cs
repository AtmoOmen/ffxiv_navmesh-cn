using DotRecast.Core;
using vnavmesh.Bootstrap;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
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
        List<BuildPhaseSummary> phases    = [];
        var                     tileCount = builtTiles.Count;
        long                    aggregatedPhaseTicks = 0;

        for (var phaseIndex = 0; phaseIndex < (int)BuildPhase.Count; ++phaseIndex)
        {
            foreach (var tile in builtTiles)
                aggregatedPhaseTicks += tile.PhaseTicks[phaseIndex];
        }

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
                    Name         = PhaseNames[phaseIndex],
                    TotalTicks   = totalTicks,
                    AverageTicks = totalTicks / Math.Max(tileCount, 1),
                    MaxTicks     = maxTicks,
                    SlowestTileX = slowestTileX,
                    SlowestTileZ = slowestTileZ,
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
}
