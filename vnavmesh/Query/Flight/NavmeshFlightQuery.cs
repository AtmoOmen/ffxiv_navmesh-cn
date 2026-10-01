using System.Numerics;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Common.Extensions;
using vnavmesh.Common.Utils;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Enums;
using vnavmesh.Query.Flight.Models;
using vnavmesh.Query.Ground;
using vnavmesh.Query.Models;

namespace vnavmesh.Query.Flight;

internal sealed class NavmeshFlightQuery
{
    private readonly NavmeshQuery       query;
    private readonly NavmeshGroundQuery groundQuery;

    internal NavmeshFlightQuery
    (
        NavmeshQuery       query,
        NavmeshGroundQuery groundQuery
    )
    {
        this.query       = query;
        this.groundQuery = groundQuery;
    }

    internal PlannerResult PlanVolumePathDetailed
    (
        Vector3           from,
        Vector3           to,
        CancellationToken cancel,
        Vector3?          avoidCenter = null,
        float             avoidRadius = 0
    )
    {
        if (query.VolumeQuery == null)
        {
            Service.Log.Error("体素导航体未构建，无法执行飞行算路");
            return CreateFlightFailure(to);
        }

        var volumeQuery    = query.VolumeQuery!;
        var volume         = volumeQuery.Volume;
        var locateTimer    = StopWatchTimer.Create();
        var startLocate    = query.FindNearestVolumeVoxelSurfaceAware(from);
        var endLocate      = query.FindNearestVolumeVoxelSurfaceAware(to);
        var startVoxel     = startLocate.Voxel;
        var endVoxel       = endLocate.Voxel;
        var locateDuration = locateTimer.Value();
        Service.Log.Debug($"[算路] 飞行体素 {startVoxel:X} -> {endVoxel:X}");

        if (startVoxel == VoxelMap.INVALID_VOXEL || endVoxel == VoxelMap.INVALID_VOXEL)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 无法定位空体素");
            return CreateFlightFailure(to);
        }

        var requestedStartLeaf  = volume.FindLeafVoxel(from);
        var requestedTargetLeaf = volume.FindLeafVoxel(to);
        var safeStart = !startLocate.UsedSurfaceAnchor && requestedStartLeaf.empty && requestedStartLeaf.voxel == startVoxel ?
                            from :
                            startLocate.SafePoint;
        var safeDestination = !endLocate.UsedSurfaceAnchor && requestedTargetLeaf.empty && requestedTargetLeaf.voxel == endVoxel ?
                                  to :
                                  endLocate.SafePoint;
        var safeDestinationAdjusted = Vector3.DistanceSquared(safeDestination, to) > 0.000001f;

        // 体积图会把一串窄洞逐个堵死，飞行只能到达开阔空腔里的位置。
        // 终点落在封闭空腔时，沿地面路径分段拼接：开阔段飞行，封闭段走地面。
        if (IsVolumeRegionSmall(safeDestination, VOLUME_GOAL_FLOOD_LIMIT, from, out var floodedCells, out var sealedNearest) &&
            !IsSealedNearestAtStart(sealedNearest, from))
        {
            Service.Log.Debug($"[算路] 飞行终点所在的体积空腔只有 {floodedCells} 格，判定为封闭");

            // 起点落得到地面时靠地面路径分段，落不到地面才从体积图上逐层穿洞
            if (TryResolveGroundPoint(from) is not null)
            {
                if (TryBuildSegmentedVolumePath(from, to, cancel, avoidCenter, avoidRadius, out var segmentedResult))
                    return segmentedResult;
            }
            else if (TryBuildSealedGoalFallback(from, to, safeDestination, cancel, avoidCenter, avoidRadius, out var sealedResult))
                return sealedResult;

            // 终点在体积图上被隔离，地面又到不了，体积搜索只会去穷尽可达空间
            Service.Log.Warning("[算路] 分段与穿层方案均失败，终点在体积图上不可达，返回 Partial");
            return CreateFlightUnreachable(from, to);
        }

        var searchTimer = StopWatchTimer.Create();
        var voxelPath = volumeQuery.FindPath
            (startVoxel, endVoxel, safeStart, safeDestination, false, cancel, avoidCenter, avoidRadius);
        var telemetry = volumeQuery.LastTelemetry;

        Service.Log.Debug
        (
            $"[算路] 飞行路径查询完成：空体素定位耗时 = {locateDuration.TotalSeconds:f3} 秒，主体搜索耗时 = {searchTimer.Value().TotalSeconds:f3} 秒，细层访问节点 = {telemetry.VisitedNodes}，粗层扩展节点 = {telemetry.CoarseExpandedNodes}，生成节点 = {telemetry.GeneratedNodes}，LoS 检查 = {telemetry.LineOfSightChecks}，LoS 命中 = {telemetry.LineOfSightHits}，开放表峰值 = {telemetry.PeakOpenListSize}，终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，搜索轮次 = {telemetry.SearchAttempts}，启发式权重 = {telemetry.HeuristicWeight:f2}，路径点 = {voxelPath.Count}，起点修正 = {(Vector3.DistanceSquared(safeStart, from) > 0.000001f ? "是" : "否")}，安全终点修正 = {(safeDestinationAdjusted ? "是" : "否")}"
        );

        if (voxelPath.Count == 0)
        {
            Service.Log.Error
            (
                $"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 体素路径为空，终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，访问节点 = {telemetry.VisitedNodes}"
            );
            return CreateFlightFailure(to);
        }

        List<Vector3> rawWaypoints = new(voxelPath.Count);
        foreach (var step in voxelPath)
            rawWaypoints.Add(step.p);

        if (telemetry.Termination != VolumeSearchTermination.ReachedGoal)
        {
            var partialDestination = rawWaypoints[^1];
            var nearGoalThreshold  = ComputeNearGoalThreshold(volume);
            var distanceToGoal     = Vector3.Distance(partialDestination, safeDestination);
            var aligned            = Vector3.DistanceSquared(partialDestination, safeDestination) <= 0.000001f;

            if (distanceToGoal <= nearGoalThreshold && (aligned || CanFlyDirectlyBetween(partialDestination, safeDestination)))
            {
                Service.Log.Debug
                (
                    $"[算路] 飞行体素搜索近终点视为完成：终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，路径终点 = {partialDestination:f3}，安全终点 = {safeDestination:f3}，距离 = {distanceToGoal:f3}，阈值 = {nearGoalThreshold:f3}"
                );

                if (!aligned)
                    rawWaypoints.Add(safeDestination);
            }
            else
            {
                // 体积搜索在步数触顶时停下，停点不保证贴着门框，但它确实是飞行到达过的位置。
                // 让停点落到地面，剩下的一段交给地面寻路，只让穿门那一段落地。
                Service.Log.Warning
                (
                    $"飞行体素搜索未抵达终点：终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，请求空体素终点 = {safeDestination:f3}，当前终点 = {partialDestination:f3}，距离 = {distanceToGoal:f3}，阈值 = {nearGoalThreshold:f3}，改为接地面续算"
                );

                if (!requestedTargetLeaf.empty &&
                    TryBuildFlightGroundTransitionResult(from, to, partialDestination, rawWaypoints, cancel, avoidCenter, avoidRadius, out var fallbackResult))
                    return fallbackResult;

                return new()
                {
                    Status               = PathfindStatus.Partial,
                    RequestedMode        = MovementMode.Flight,
                    RequestedDestination = to,
                    FinalDestination     = partialDestination,
                    DestinationTolerance = 0,
                    Segments =
                    [
                        new()
                        {
                            MovementMode           = MovementMode.Flight,
                            SegmentKind            = MovementSegmentKind.FlightTraverse,
                            AllowVerticalControl   = true,
                            GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                            TraversalStartPosition = from,
                            StartPosition          = from,
                            EndPosition            = partialDestination,
                            Points                 = [.. rawWaypoints]
                        }
                    ]
                };
            }
        }

        if (!requestedTargetLeaf.empty &&
            TryBuildFlightGroundTransitionResult(from, to, safeDestination, rawWaypoints, cancel, avoidCenter, avoidRadius, out var hybridResult))
            return hybridResult;

        var finalDestination    = safeDestination;
        var destinationAdjusted = safeDestinationAdjusted;
        var landingPoint        = TryResolveFlightLandingPoint(to, safeDestination);
        var completionTolerance = MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(to, safeDestination));

        if (landingPoint is { } resolvedLandingPoint)
        {
            finalDestination = resolvedLandingPoint;
            var leafVerticalTolerance = MathF.Max(volumeQuery.Volume.Levels[^1].CellSize.Y, query.ConfigData.PathTolerance);
            completionTolerance = MathF.Max(query.ConfigData.PathTolerance, MathF.Max(HorizontalDistanceXZ(to, resolvedLandingPoint), leafVerticalTolerance));
            destinationAdjusted = Vector3.Distance(resolvedLandingPoint, to) > completionTolerance;

            if (rawWaypoints.Count == 0 || Vector3.DistanceSquared(rawWaypoints[^1], resolvedLandingPoint) > 0.000001f)
                rawWaypoints.Add(resolvedLandingPoint);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行终点解析：请求终点 = {to:f3}，空体素终点 = {safeDestination:f3}，落地点 = {(landingPoint is { } lp ? lp.ToString("f3") : "无")}，最终终点 = {finalDestination:f3}，落地吸附 = {(landingPoint != null ? "是" : "否")}"
        );

        var finalDestinationTolerance = landingPoint is { } resolvedLanding ?
                                            MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(to, resolvedLanding)) :
                                            0;

        return new()
        {
            Status = destinationAdjusted ?
                         PathfindStatus.Partial :
                         PathfindStatus.Complete,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = to,
            FinalDestination     = finalDestination,
            DestinationTolerance = finalDestinationTolerance,
            Segments =
            [
                new()
                {
                    MovementMode           = MovementMode.Flight,
                    SegmentKind            = MovementSegmentKind.FlightTraverse,
                    AllowVerticalControl   = true,
                    GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                    TraversalStartPosition = from,
                    StartPosition          = from,
                    EndPosition            = finalDestination,
                    Points                 = [.. rawWaypoints]
                }
            ]
        };
    }

    // 终点被体积图上的小空腔关住时（例如门内的房间），两个方向的搜索都会把预算耗在逼近它上。
    // 这里不去寻路，只在叶体素网格上洪泛给定位置所在的分量：能在上限内穷尽，说明它被封闭住了。
    private bool IsVolumeRegionSmall
    (
        Vector3                   point,
        int                       limit,
        Vector3                   towards,
        out int                   cells,
        out (int X, int Y, int Z) nearest
    )
    {
        cells   = 0;
        nearest = default;

        if (query.VolumeQuery == null)
            return false;

        var volume   = query.VolumeQuery.Volume;
        var cellSize = volume.Levels[^1].CellSize;
        var origin   = volume.RootTile.BoundsMin;

        var (sx, sy, sz) = ToLeafCell(point, origin, cellSize);
        var seeded = false;

        // 采样点常常贴在地面上，取整后的格子可能落在脚下；向上找到第一个空格子作为种子
        for (var attempt = 0; attempt < VOLUME_SEED_LIFT_ATTEMPTS; ++attempt, ++sy)
        {
            var seed = origin + (new Vector3(sx + 0.5f, sy + 0.5f, sz + 0.5f) * cellSize);
            if (!volume.FindLeafVoxel(seed).empty)
                continue;

            seeded = true;
            break;
        }

        if (!seeded)
        {
            cells = 0;
            return true;
        }

        var startCell = (X: sx, Y: sy, Z: sz);
        var visited   = new HashSet<(int X, int Y, int Z)> { startCell };
        var queue     = new Queue<(int X, int Y, int Z)>();
        queue.Enqueue(startCell);

        var bestDistance = float.MaxValue;
        nearest = startCell;

        while (queue.Count > 0)
        {
            var (cx, cy, cz) = queue.Dequeue();

            var center   = origin + (new Vector3(cx + 0.5f, cy + 0.5f, cz + 0.5f) * cellSize);
            var distance = Vector3.DistanceSquared(center, towards);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                nearest      = (cx, cy, cz);
            }

            foreach (var (dx, dy, dz) in LeafNeighbours())
            {
                var next = (X: cx + dx, Y: cy + dy, Z: cz + dz);
                if (visited.Contains(next))
                    continue;

                var world = origin + (new Vector3(next.X + 0.5f, next.Y + 0.5f, next.Z + 0.5f) * cellSize);
                if (!volume.FindLeafVoxel(world).empty)
                    continue;

                visited.Add(next);

                if (visited.Count > limit)
                {
                    cells = visited.Count;
                    return false;
                }

                queue.Enqueue(next);
            }
        }

        cells = visited.Count;
        return true;
    }

    private static (int X, int Y, int Z) ToLeafCell
    (
        Vector3 point,
        Vector3 origin,
        Vector3 cellSize
    )
    {
        var local = (point - origin) / cellSize;
        return ((int)MathF.Floor(local.X), (int)MathF.Floor(local.Y), (int)MathF.Floor(local.Z));
    }

    private static IEnumerable<(int, int, int)> LeafNeighbours()
    {
        yield return (1, 0, 0);
        yield return (-1, 0, 0);
        yield return (0, 1, 0);
        yield return (0, -1, 0);
        yield return (0, 0, 1);
        yield return (0, 0, -1);
    }

    private Vector3 LeafCellCenter
    (
        (int X, int Y, int Z) cell
    )
    {
        var volume = query.VolumeQuery!.Volume;
        return volume.RootTile.BoundsMin + (new Vector3(cell.X + 0.5f, cell.Y + 0.5f, cell.Z + 0.5f) * volume.Levels[^1].CellSize);
    }

    // 分量已经覆盖到起点所在的那一格，说明起点与终点本来就在同一个空腔里，不必绕行地面
    private bool IsSealedNearestAtStart
    (
        (int X, int Y, int Z) cell,
        Vector3               start
    )
    {
        var cellSize      = query.VolumeQuery!.Volume.Levels[^1].CellSize;
        var coverDistance = MathF.Max(cellSize.X, MathF.Max(cellSize.Y, cellSize.Z));
        return Vector3.Distance(LeafCellCenter(cell), start) <= coverDistance;
    }

    // 沿地面路径扫描出"开阔段 / 封闭段"的交替序列。开阔段说明那里的体积空腔连着起点一侧的大空间，
    // 可以直接飞过去；封闭段是被体积图堵住的窄洞或门，只能走地面。
    private List<(Vector3 Start, Vector3 End, bool Open)> ScanOpenRuns
    (
        IReadOnlyList<long> corridor,
        Vector3             start,
        Vector3             end
    )
    {
        List<(Vector3 Point, bool Open)> samples = [(start, !IsVolumeRegionSmall(start, VOLUME_OPEN_FLOOD_LIMIT, start, out _, out _))];

        var previous  = start;
        var lastProbe = start;

        foreach (var polyRef in corridor)
        {
            if (!query.MeshQuery.ClosestPointOnPoly(polyRef, previous.ToRecast(), out var closest, out _).Succeeded())
                break;

            var point = closest.ToSystem();
            previous = point;

            if (Vector3.Distance(point, lastProbe) < VOLUME_RETREAT_SAMPLE_STEP)
                continue;

            lastProbe = point;
            samples.Add((point, !IsVolumeRegionSmall(point, VOLUME_OPEN_FLOOD_LIMIT, point, out _, out _)));
        }

        samples.Add((end, false));

        List<(Vector3 Start, Vector3 End, bool Open)> runs    = [];
        var                                           current = samples[0];

        for (var i = 1; i < samples.Count; ++i)
        {
            if (samples[i].Open == current.Open)
                continue;

            runs.Add((current.Point, samples[i - 1].Point, current.Open));
            current = samples[i];
        }

        runs.Add((current.Point, samples[^1].Point, current.Open));
        return runs;
    }

    // 起点可能停在半空中，地面寻路只有正负五码的定位范围；这里放宽垂直范围把它落到地面上。
    // 下方本来就没有地面时返回 null，由调用方改走体积图上的穿层方案。
    private Vector3? TryResolveGroundPoint
    (
        Vector3 point
    )
    {
        var polyRef = query.FindNearestMeshPoly(point, VOLUME_GROUND_SEARCH_XZ, VOLUME_GROUND_SEARCH_Y, false);
        if (polyRef == 0)
            return null;

        return query.MeshQuery.ClosestPointOnPoly(polyRef, point.ToRecast(), out var closest, out _).Succeeded() ?
                   closest.ToSystem() :
                   null;
    }

    // 下方没有地面时拿不到地面路径，改从体积图本身找门：从终点的封闭分量出发，
    // 逐层穿过被体积图堵住的窄洞，直到某一层与起点连通，那里就是飞行能到达的接地点。
    private bool TryBuildSealedGoalFallback
    (
        Vector3           from,
        Vector3           to,
        Vector3           safeDestination,
        CancellationToken cancel,
        Vector3?          avoidCenter,
        float             avoidRadius,
        out PlannerResult result
    )
    {
        result = null!;

        var volumeQuery = query.VolumeQuery;
        if (volumeQuery == null)
            return false;

        var volume = volumeQuery.Volume;
        var cursor = safeDestination;

        for (var hop = 0; hop < VOLUME_SEALED_HOP_LIMIT; ++hop)
        {
            IsVolumeRegionSmall(cursor, VOLUME_GOAL_FLOOD_LIMIT, from, out _, out var nearest);

            // 只有真的贴到起点所在的空间才算连通。"空腔很大"并不等于连通，
            // 起点那一侧可能还隔着别的被体积图堵住的窄洞。
            var nearestPoint = LeafCellCenter(nearest);

            if (Vector3.Distance(nearestPoint, from) <= VOLUME_HANDOFF_REACH_DISTANCE)
            {
                if (!TryResolveOpenVoxel(from,   out var fromVoxel,    out var fromPoint) ||
                    !TryResolveOpenVoxel(cursor, out var handoffVoxel, out var handoffPoint))
                    return false;

                // 地面到不了终点时（终点在浮岛这类只能飞抵的位置），接上的地面段会停在地面能靠近的最近处；
                // 先判地面再飞，可以省掉一次注定到不了终点的飞行搜索
                var ground = groundQuery.PlanMeshPathDetailed(handoffPoint, to, 0, cancel, avoidCenter, avoidRadius);

                if (ground.Status != PathfindStatus.Complete || ground.Segments.Count == 0)
                {
                    Service.Log.Warning($"[算路] 地面路径未到终点（{ground.Status}），穿层方案放弃");
                    return false;
                }

                var flight = volumeQuery.FindPath
                    (fromVoxel, handoffVoxel, fromPoint, handoffPoint, false, cancel, avoidCenter, avoidRadius);

                if (flight.Count == 0)
                    return false;

                List<Vector3> points = new(flight.Count);
                foreach (var step in flight)
                    points.Add(step.p);

                points.Add(handoffPoint);

                List<PlannerPathSegment> segments =
                [
                    new()
                    {
                        MovementMode           = MovementMode.Flight,
                        SegmentKind            = MovementSegmentKind.FlightTraverse,
                        AllowVerticalControl   = true,
                        GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                        TraversalStartPosition = from,
                        StartPosition          = from,
                        EndPosition            = handoffPoint,
                        Points                 = points
                    }
                ];
                segments.AddRange(ground.Segments);

                Service.Log.Debug($"[算路] 体积图上穿过 {hop} 层封闭空腔后取得接地点 {handoffPoint:f3}");

                result = new()
                {
                    Status               = ground.Status,
                    RequestedMode        = MovementMode.Flight,
                    RequestedDestination = to,
                    FinalDestination     = ground.FinalDestination,
                    DestinationTolerance = MathF.Max
                        (ground.DestinationTolerance, MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(to, ground.FinalDestination))),
                    Segments = segments
                };
                return true;
            }

            if (!TryStepOutsideRegion(volume, nearest, from, out var next))
                return false;

            cursor = next;
        }

        return false;
    }

    // 从封闭分量中离起点最近的一格出发，按"朝向起点"的优先顺序逐格外探，取紧邻的另一侧空体素
    private bool TryStepOutsideRegion
    (
        VoxelMap              volume,
        (int X, int Y, int Z) cell,
        Vector3               towards,
        out Vector3           next
    )
    {
        next = default;

        var cellSize = volume.Levels[^1].CellSize;
        var origin   = volume.RootTile.BoundsMin;
        var center   = origin  + (new Vector3(cell.X + 0.5f, cell.Y + 0.5f, cell.Z + 0.5f) * cellSize);
        var delta    = towards - center;

        (int X, int Y, int Z)[] directions =
        [
            (Math.Sign(delta.X), 0, 0),
            (0, 0, Math.Sign(delta.Z)),
            (0, Math.Sign(delta.Y), 0),
            (0, 0, -Math.Sign(delta.Z)),
            (0, -Math.Sign(delta.Y), 0),
            (-Math.Sign(delta.X), 0, 0)
        ];

        foreach (var (dx, dy, dz) in directions)
        {
            if (dx == 0 && dy == 0 && dz == 0)
                continue;

            var probe = cell;

            for (var step = 0; step < VOLUME_SEALED_STEP_LIMIT; ++step)
            {
                probe = (X: probe.X + dx, Y: probe.Y + dy, Z: probe.Z + dz);

                var world = origin + (new Vector3(probe.X + 0.5f, probe.Y + 0.5f, probe.Z + 0.5f) * cellSize);
                if (!volume.FindLeafVoxel(world).empty)
                    continue;

                next = world;
                return true;
            }
        }

        return false;
    }

    // 采样点贴在地面上，向上找到的第一个空体素必然与它同处一侧空腔，
    // 用它当飞行端点可以避开"落到被堵住的那一侧"。
    private bool TryResolveOpenVoxel
    (
        Vector3     point,
        out ulong   voxel,
        out Vector3 safePoint
    )
    {
        voxel     = VoxelMap.INVALID_VOXEL;
        safePoint = point;

        if (query.VolumeQuery == null)
            return false;

        var volume   = query.VolumeQuery.Volume;
        var cellSize = volume.Levels[^1].CellSize;
        var origin   = volume.RootTile.BoundsMin;

        var (sx, sy, sz) = ToLeafCell(point, origin, cellSize);

        for (var attempt = 0; attempt < VOLUME_SEED_LIFT_ATTEMPTS; ++attempt, ++sy)
        {
            var seed = origin + (new Vector3(sx + 0.5f, sy + 0.5f, sz + 0.5f) * cellSize);
            var leaf = volume.FindLeafVoxel(seed);

            if (!leaf.empty || leaf.voxel == VoxelMap.INVALID_VOXEL)
                continue;

            voxel     = leaf.voxel;
            safePoint = seed;
            return true;
        }

        return false;
    }

    private bool TryBuildSegmentedVolumePath
    (
        Vector3           from,
        Vector3           to,
        CancellationToken cancel,
        Vector3?          avoidCenter,
        float             avoidRadius,
        out PlannerResult result
    )
    {
        result = null!;

        var volumeQuery = query.VolumeQuery;
        if (volumeQuery == null)
            return false;

        // 起点可能停在半空中，地面寻路只有正负五码的定位范围；先沿垂直方向放宽搜索把它落到地面上
        if (TryResolveGroundPoint(from) is not { } groundStart)
        {
            Service.Log.Warning($"[算路] 无法把起点 {from:f3} 落到地面上，分段拼接放弃");
            return false;
        }

        var ground = groundQuery.PlanMeshPathDetailed(groundStart, to, 0, cancel, avoidCenter, avoidRadius);

        if (!ground.Succeeded || ground.Segments.Count == 0)
        {
            Service.Log.Warning($"[算路] 地面路径不可用（起点 {groundStart:f3}），分段拼接放弃");
            return false;
        }

        // 地面本身就被隔断时，用它分段只会拼出一条到不了终点的路，交给穿层方案
        if (ground.Status != PathfindStatus.Complete)
        {
            Service.Log.Warning($"[算路] 地面路径不完整（{ground.Status}），分段拼接放弃");
            return false;
        }

        var corridor = ground.Segments[0].Corridor;
        if (corridor.Count == 0)
            return false;

        var runs = ScanOpenRuns(corridor, ground.Segments[0].StartPosition, to);
        if (runs.Count == 0)
            return false;

        List<PlannerPathSegment> segments   = [];
        var                      cursor     = from;
        var                      lastStatus = ground.Status;

        for (var i = 0; i < runs.Count; ++i)
        {
            var run = runs[i];

            if (run.Open)
            {
                // 飞行段两端都取所在采样点正上方的空体素，保证落在开阔侧而不是被堵住的洞内
                if (!TryResolveOpenVoxel(cursor,  out var fromVoxel, out var fromPoint) ||
                    !TryResolveOpenVoxel(run.End, out var toVoxel,   out var toPoint))
                    return false;

                var flight = query.VolumeQuery!.FindPath
                    (fromVoxel, toVoxel, fromPoint, toPoint, false, cancel, avoidCenter, avoidRadius);

                if (flight.Count == 0)
                    return false;

                List<Vector3> points = new(flight.Count);
                foreach (var step in flight)
                    points.Add(step.p);

                points.Add(toPoint);

                segments.Add
                (
                    new()
                    {
                        MovementMode           = MovementMode.Flight,
                        SegmentKind            = MovementSegmentKind.FlightTraverse,
                        AllowVerticalControl   = true,
                        GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                        TraversalStartPosition = cursor,
                        StartPosition          = cursor,
                        EndPosition            = run.End,
                        Points                 = points
                    }
                );

                cursor = run.End;
                continue;
            }

            // 地面段一直延伸到下一个开阔段的起点，这样紧接其后的飞行段两端都落在开阔侧
            var groundEnd = i + 1 < runs.Count ?
                                runs[i + 1].Start :
                                run.End;
            var groundRun = groundQuery.PlanMeshPathDetailed(cursor, groundEnd, 0, cancel, avoidCenter, avoidRadius);

            if (!groundRun.Succeeded || groundRun.Segments.Count == 0)
                return false;

            lastStatus = groundRun.Status;
            segments.AddRange(groundRun.Segments);
            cursor = groundEnd;
        }

        if (segments.Count == 0)
            return false;

        Service.Log.Debug($"[算路] 飞行终点被封闭，拼接出 {segments.Count} 段路径，其中开阔段 {runs.Count(r => r.Open)} 个");

        result = new()
        {
            Status               = lastStatus,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = to,
            FinalDestination     = ground.FinalDestination,
            DestinationTolerance = MathF.Max
                (ground.DestinationTolerance, MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(to, ground.FinalDestination))),
            Segments = segments
        };
        return true;
    }

    // 体积图上能否从一点直线飞到另一点；用来判断"直接吸附到终点"会不会从门框这类薄壁上穿过去
    private bool CanFlyDirectlyBetween
    (
        Vector3 left,
        Vector3 right
    )
    {
        if (query.VolumeQuery == null)
            return false;

        var volume    = query.VolumeQuery.Volume;
        var leftLeaf  = volume.FindLeafVoxel(left);
        var rightLeaf = volume.FindLeafVoxel(right);

        if (!leftLeaf.empty                           ||
            !rightLeaf.empty                          ||
            leftLeaf.voxel  == VoxelMap.INVALID_VOXEL ||
            rightLeaf.voxel == VoxelMap.INVALID_VOXEL)
            return false;

        return VoxelSearch.LineOfSight(volume, leftLeaf.voxel, rightLeaf.voxel, left, right);
    }

    private Vector3? TryResolveFlightLandingPoint
    (
        Vector3 requestedTarget,
        Vector3 safeDestination
    )
    {
        var toleranceFloor = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
        if (query.VolumeQuery == null)
            return null;

        var landingLeafSize = query.VolumeQuery.Volume.Levels[^1].CellSize;
        var landingSearchExtent = new Vector3
        (
            MathF.Max(landingLeafSize.X, landingLeafSize.Z),
            MathF.Max(landingLeafSize.Y, toleranceFloor),
            MathF.Max(landingLeafSize.X, landingLeafSize.Z)
        );
        var landingPoint = query.FindPointOnFloor
                               (requestedTarget, landingSearchExtent.X) ??
                           query.FindNearestPointOnMesh(requestedTarget, landingSearchExtent.X, landingSearchExtent.Y);
        if (landingPoint is not { } resolved)
            return TryResolveVolumeLandingPoint(requestedTarget, safeDestination, landingLeafSize);

        var requestHorizontalDistance = HorizontalDistanceXZ(resolved, requestedTarget);
        if (requestHorizontalDistance > landingSearchExtent.X)
            return TryResolveVolumeLandingPoint(requestedTarget, safeDestination, landingLeafSize);

        var safeHorizontalDistance = HorizontalDistanceXZ(resolved, safeDestination);
        if (safeHorizontalDistance > HorizontalDistanceXZ(safeDestination, requestedTarget) + toleranceFloor)
            return TryResolveVolumeLandingPoint(requestedTarget, safeDestination, landingLeafSize);

        var verticalDrop = safeDestination.Y - resolved.Y;
        if (verticalDrop < -query.ConfigData.PathTolerance || verticalDrop > MathF.Abs(safeDestination.Y - requestedTarget.Y) + landingSearchExtent.Y)
            return TryResolveVolumeLandingPoint(requestedTarget, safeDestination, landingLeafSize);

        return resolved;
    }

    private Vector3? TryResolveVolumeLandingPoint
    (
        Vector3 requestedTarget,
        Vector3 safeDestination,
        Vector3 landingLeafSize
    )
    {
        var volume   = query.VolumeQuery!.Volume;
        var safeLeaf = volume.FindLeafVoxel(safeDestination);
        if (!safeLeaf.empty || safeLeaf.voxel == VoxelMap.INVALID_VOXEL)
            return null;

        var belowPoint = safeDestination - new Vector3(0, MathF.Max(landingLeafSize.Y, float.Epsilon), 0);
        var belowLeaf  = volume.FindLeafVoxel(belowPoint);
        if (belowLeaf.empty)
            return null;

        var horizontalDistance = HorizontalDistanceXZ(requestedTarget, safeDestination);
        if (horizontalDistance > MathF.Max(landingLeafSize.X, landingLeafSize.Z))
            return null;

        if (volume.TryGetSurfaceTop(belowLeaf.voxel, out var surfaceTopY))
            return new Vector3(safeDestination.X, surfaceTopY + query.ConfigData.PathTolerance, safeDestination.Z);

        return safeDestination;
    }

    private Vector3? TryBuildFlightGroundApproachPoint
    (
        Vector3 safeFlightDestination,
        Vector3 transitionPoint,
        Vector3 groundLeadTarget,
        Vector3 requestedTarget
    )
    {
        var toleranceFloor      = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
        var horizontalGap       = HorizontalDistanceXZ(safeFlightDestination, transitionPoint);
        var transitionTolerance = MathF.Max(toleranceFloor, MathF.Abs(safeFlightDestination.Y - transitionPoint.Y));
        if (horizontalGap > transitionTolerance)
            return null;

        var verticalDrop = safeFlightDestination.Y - transitionPoint.Y;
        if (verticalDrop <= toleranceFloor)
            return null;

        // 落差只有一两格体素时直接下降即可。此时再绕到目标反方向会让飞行段飞过落地点后折回来，
        // 路径上出现明显的弯折，而这么小的落差本来也不需要斜向进场。
        var leafHeight = query.VolumeQuery?.Volume.Levels[^1].CellSize.Y ?? 0f;
        if (verticalDrop <= MathF.Max(toleranceFloor, leafHeight * 2f))
            return null;

        var leadDelta = new Vector2(groundLeadTarget.X - transitionPoint.X, groundLeadTarget.Z - transitionPoint.Z);
        if (leadDelta.LengthSquared() <= 0.000001f)
            leadDelta = new Vector2(requestedTarget.X - transitionPoint.X, requestedTarget.Z - transitionPoint.Z);
        if (leadDelta.LengthSquared() <= 0.000001f)
            return null;

        leadDelta = Vector2.Normalize(leadDelta);
        var approachHorizontal = Math.Clamp(verticalDrop, transitionTolerance, HorizontalDistanceXZ(requestedTarget, transitionPoint));
        var candidate = new Vector3
        (
            transitionPoint.X - (leadDelta.X  * approachHorizontal),
            transitionPoint.Y + (verticalDrop * (approachHorizontal / (approachHorizontal + verticalDrop))),
            transitionPoint.Z - (leadDelta.Y  * approachHorizontal)
        );

        var approachLocate = query.FindNearestVolumeVoxelSurfaceAware(candidate, transitionTolerance, MathF.Max(toleranceFloor, verticalDrop));
        var approachVoxel  = approachLocate.Voxel;
        if (approachVoxel == VoxelMap.INVALID_VOXEL)
            return candidate;

        return approachLocate.SafePoint;
    }

    private void TrimFlightWaypointsForGroundTransition
    (
        List<Vector3> flightWaypoints,
        Vector3       approachPoint
    )
    {
        if (query.VolumeQuery == null || flightWaypoints.Count < 2)
            return;

        var volume       = query.VolumeQuery.Volume;
        var approachLeaf = volume.FindLeafVoxel(approachPoint);
        if (!approachLeaf.empty || approachLeaf.voxel == VoxelMap.INVALID_VOXEL)
            return;

        while (flightWaypoints.Count >= 2)
        {
            var previousPoint = flightWaypoints[^2];
            var previousLeaf  = volume.FindLeafVoxel(previousPoint);
            if (!previousLeaf.empty || previousLeaf.voxel == VoxelMap.INVALID_VOXEL)
                break;

            if (!VoxelSearch.LineOfSight(volume, previousLeaf.voxel, approachLeaf.voxel, previousPoint, approachPoint))
                break;

            flightWaypoints.RemoveAt(flightWaypoints.Count - 1);
        }
    }

    private bool TryBuildFlightGroundTransitionResult
    (
        Vector3           requestedStart,
        Vector3           requestedTarget,
        Vector3           safeFlightDestination,
        List<Vector3>     rawFlightWaypoints,
        CancellationToken cancel,
        Vector3?          avoidCenter,
        float             avoidRadius,
        out PlannerResult result
    )
    {
        var toleranceFloor = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);

        if (query.FindNearestMeshPoly(requestedTarget, allowUnreachable: false) == 0)
        {
            result = null!;
            return false;
        }

        // 接地点可能在停点下方十几码的空中，地面寻路需要先把它投影到地面，
        // 而进场过渡仍要用空中的停点来算下降段，否则飞行段会先停在停点再垂直掉下去。
        var groundStart = query.FindNearestMeshPoly(safeFlightDestination, allowUnreachable: false) != 0 ?
                              safeFlightDestination :
                              query.FindPointOnFloor(safeFlightDestination);

        if (groundStart is not { } resolvedGroundStart)
        {
            result = null!;
            return false;
        }

        var groundResult = groundQuery.PlanMeshPathDetailed(resolvedGroundStart, requestedTarget, 0, cancel, avoidCenter, avoidRadius);

        // 地面到不了目标时（目标在浮岛这类只能飞抵的位置），接上的地面段会停在地面能靠近的最近处
        if (groundResult.Status != PathfindStatus.Complete || groundResult.Segments.Count == 0)
        {
            result = null!;
            return false;
        }

        var transitionPoint = groundResult.Segments[0].StartPosition;
        var approachPoint = TryBuildFlightGroundApproachPoint(safeFlightDestination, transitionPoint, groundResult.Segments[0].EndPosition, requestedTarget);
        List<Vector3> flightWaypoints = [.. rawFlightWaypoints];

        if (approachPoint is { } resolvedApproachPoint)
        {
            TrimFlightWaypointsForGroundTransition(flightWaypoints, resolvedApproachPoint);
            if (flightWaypoints.Count == 0 || Vector3.DistanceSquared(flightWaypoints[^1], resolvedApproachPoint) > 0.000001f)
                flightWaypoints.Add(resolvedApproachPoint);
        }

        if (flightWaypoints.Count == 0 || Vector3.DistanceSquared(flightWaypoints[^1], transitionPoint) > 0.000001f)
            flightWaypoints.Add(transitionPoint);

        List<PlannerPathSegment> segments =
        [
            new()
            {
                MovementMode           = MovementMode.Flight,
                SegmentKind            = MovementSegmentKind.FlightTraverse,
                AllowVerticalControl   = true,
                GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                TraversalStartPosition = requestedStart,
                StartPosition          = requestedStart,
                EndPosition            = transitionPoint,
                Points                 = flightWaypoints
            }
        ];
        foreach (var segment in groundResult.Segments)
            segments.Add(segment);

        var transitionAdjusted = Vector3.Distance
                                     (safeFlightDestination, transitionPoint) >
                                 MathF.Max(toleranceFloor, MathF.Abs(safeFlightDestination.Y - transitionPoint.Y));
        Service.Log.Debug
        (
            $"[算路] 飞行接地面续算：空体素终点 = {safeFlightDestination:f3}，近地点 = {(approachPoint is { } ap ? ap.ToString("f3") : "无")}，桥接点 = {transitionPoint:f3}，桥接修正 = {(transitionAdjusted ? "是" : "否")}，地面结果 = {groundResult.Status}，地面段数 = {groundResult.Segments.Count}"
        );

        var destinationTolerance = MathF.Max
            (groundResult.DestinationTolerance, MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(requestedTarget, groundResult.FinalDestination)));

        result = new()
        {
            Status               = groundResult.Status,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = requestedTarget,
            FinalDestination     = groundResult.FinalDestination,
            DestinationTolerance = destinationTolerance,
            Segments             = segments
        };
        return true;
    }

    private static PlannerResult CreateFlightFailure
    (
        Vector3 destination
    ) =>
        new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = destination,
            FinalDestination     = destination,
            DestinationTolerance = 0
        };

    // 终点被体积图关在与起点不连通的空腔里，且地面也到不了，此时没有可行的飞行路径
    private static PlannerResult CreateFlightUnreachable
    (
        Vector3 start,
        Vector3 destination
    ) =>
        new()
        {
            Status               = PathfindStatus.Partial,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = destination,
            FinalDestination     = start,
            DestinationTolerance = 0,
            Segments =
            [
                new()
                {
                    MovementMode           = MovementMode.Flight,
                    SegmentKind            = MovementSegmentKind.FlightTraverse,
                    AllowVerticalControl   = true,
                    GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                    TraversalStartPosition = start,
                    StartPosition          = start,
                    EndPosition            = start,
                    Points                 = [start]
                }
            ]
        };

    private static string GetLogVolumeSearchTermination
    (
        VolumeSearchTermination termination
    ) => termination switch
    {
        VolumeSearchTermination.ReachedGoal       => "达到终点",
        VolumeSearchTermination.SearchExhausted   => "搜索穷尽",
        VolumeSearchTermination.StepBudgetReached => "步数触顶",
        _                                         => "未知"
    };

    private static float ComputeNearGoalThreshold
    (
        VoxelMap volume
    )
    {
        var l1CellSize  = volume.Levels[1].CellSize;
        var maxL1Extent = MathF.Max(l1CellSize.X, MathF.Max(l1CellSize.Y, l1CellSize.Z));
        return maxL1Extent * 2f;
    }

    private static float HorizontalDistanceXZ
    (
        Vector3 left,
        Vector3 right
    )
    {
        var dx = left.X             - right.X;
        var dz = left.Z             - right.Z;
        return MathF.Sqrt((dx * dx) + (dz * dz));
    }

    #region 常量

    private const int   VOLUME_GOAL_FLOOD_LIMIT       = 50_000;
    private const int   VOLUME_OPEN_FLOOD_LIMIT       = 10_000;
    private const int   VOLUME_SEED_LIFT_ATTEMPTS     = 8;
    private const int   VOLUME_SEALED_HOP_LIMIT       = 24;
    private const int   VOLUME_SEALED_STEP_LIMIT      = 64;
    private const float VOLUME_HANDOFF_REACH_DISTANCE = 32f;
    private const float VOLUME_RETREAT_SAMPLE_STEP    = 16f;
    private const float VOLUME_GROUND_SEARCH_XZ       = 8f;
    private const float VOLUME_GROUND_SEARCH_Y        = 256f;

    #endregion
}
