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

    private static BuildTelemetrySummary SummarizeBuildTelemetry(IReadOnlyList<TileBuildResult> builtTiles, TimeSpan parallelDuration)
    {
        List<BuildPhaseSummary> phases    = [];
        var                     tileCount = builtTiles.Count;

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
                    Name         = _phaseNames[phaseIndex],
                    TotalTicks   = totalTicks,
                    AverageTicks = totalTicks / Math.Max(tileCount, 1),
                    MaxTicks     = maxTicks,
                    SlowestTileX = slowestTileX,
                    SlowestTileZ = slowestTileZ
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
                    TileX                 = tile.TileX,
                    TileZ                 = tile.TileZ,
                    TotalTicks            = tile.TotalTicks,
                    GeometryInstanceCount = tile.GeometryInstanceCount,
                    TerrainInstanceCount  = tile.TerrainInstanceCount,
                    PolyCount             = tile.PolyCount,
                    VertCount             = tile.VertCount,
                    DetailTriCount        = tile.DetailTriCount
                }
            );
        }

        return new()
        {
            ParallelTicks = parallelDuration.Ticks,
            Phases        = phases,
            SlowTiles     = slowTiles
        };
    }

    private static void LogBuildTelemetry(BuildTelemetrySummary telemetry)
    {
        Service.Log.Debug("[NavmeshBuilder] 阶段统计（总计 / 单瓦片均值 / 最慢瓦片）");

        foreach (var phase in telemetry.Phases)
        {
            Service.Log.Debug
            (
                $"[NavmeshBuilder] {phase.Name}: 总计 {TicksToMilliseconds(phase.TotalTicks):f1} ms，均值 {TicksToMilliseconds(phase.AverageTicks):f2} ms，最慢瓦片 {phase.SlowestTileX}x{phase.SlowestTileZ} = {TicksToMilliseconds(phase.MaxTicks):f1} ms"
            );
        }

        if (telemetry.SlowTiles.Count > 0)
        {
            var slowestTiles = string.Join("，", telemetry.SlowTiles.Select(tile => $"{tile.TileX}x{tile.TileZ} = {TicksToMilliseconds(tile.TotalTicks):f1} ms"));
            Service.Log.Debug($"[NavmeshBuilder] 最慢瓦片 Top {telemetry.SlowTiles.Count}: {slowestTiles}");
        }

        foreach (var tile in telemetry.SlowTiles)
        {
            Service.Log.Debug
            (
                $"[NavmeshBuilder] 慢瓦片 {tile.TileX}x{tile.TileZ}: 几何实例 {tile.GeometryInstanceCount}，地形实例 {tile.TerrainInstanceCount}，Poly {tile.PolyCount}，Vert {tile.VertCount}，DetailTri {tile.DetailTriCount}"
            );
        }
    }
}
