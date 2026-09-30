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
    private const int   VOLUME_GOAL_FLOOD_LIMIT        = 50_000;
    private const float VOLUME_GOAL_RETREAT_DISTANCE   = 24f;

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

        // 终点被体积图上的小空腔关住时（例如门内的房间），正向搜索会把全部预算耗在逼近它的路上。
        // 判定封闭后把飞行目标退到地面路径上，飞行段只需要飞到门口，剩下的一小段落地走完。
        var searchVoxel = endVoxel;
        var searchGoal  = safeDestination;
        var retreated   = false;

        if (IsVolumeGoalSealed(safeDestination, out var floodedCells))
        {
            Service.Log.Debug($"[算路] 飞行终点所在的体积空腔只有 {floodedCells} 格，判定为封闭");

            if (TryResolveRetreatGoal(from, to, cancel, out var retreatGoal))
            {
                var retreatLocate = query.FindNearestVolumeVoxelSurfaceAware(retreatGoal);

                if (retreatLocate.Voxel != VoxelMap.INVALID_VOXEL)
                {
                    searchVoxel = retreatLocate.Voxel;
                    searchGoal  = retreatLocate.SafePoint;
                    retreated   = true;
                    Service.Log.Debug($"[算路] 飞行目标退至 {searchGoal:f3}");
                }
                else
                {
                    Service.Log.Warning($"[算路] 退让点 {retreatGoal:f3} 附近没有空体素，维持原飞行目标");
                }
            }
            else
            {
                Service.Log.Warning("[算路] 无法从地面路径解析退让点，维持原飞行目标");
            }
        }

        var searchTimer = StopWatchTimer.Create();
        var voxelPath   = volumeQuery.FindPath
            (startVoxel, searchVoxel, safeStart, searchGoal, false, cancel, avoidCenter, avoidRadius);
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

        if (retreated && telemetry.Termination == VolumeSearchTermination.ReachedGoal)
        {
            if (!requestedTargetLeaf.empty &&
                TryBuildFlightGroundTransitionResult(from, to, searchGoal, rawWaypoints, cancel, avoidCenter, avoidRadius, out var retreatedResult))
                return retreatedResult;
        }

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
    // 这里不去寻路，只在叶体素网格上洪泛终点所在的分量：能在上限内穷尽，就说明它被封闭住了。
    private bool IsVolumeGoalSealed
    (
        Vector3 goalPoint,
        out int floodedCells
    )
    {
        floodedCells = 0;

        if (query.VolumeQuery == null)
            return false;

        var volume   = query.VolumeQuery.Volume;
        var cellSize = volume.Levels[^1].CellSize;
        var origin   = volume.RootTile.BoundsMin;

        var startCell = ToLeafCell(goalPoint, origin, cellSize);
        var visited   = new HashSet<(int X, int Y, int Z)> { startCell };
        var queue     = new Queue<(int X, int Y, int Z)>();
        queue.Enqueue(startCell);

        while (queue.Count > 0)
        {
            var (cx, cy, cz) = queue.Dequeue();

            foreach (var (dx, dy, dz) in LeafNeighbours())
            {
                var next = (X: cx + dx, Y: cy + dy, Z: cz + dz);
                if (visited.Contains(next))
                    continue;

                var world = origin + (new Vector3(next.X + 0.5f, next.Y + 0.5f, next.Z + 0.5f) * cellSize);
                if (!volume.FindLeafVoxel(world).empty)
                    continue;

                visited.Add(next);

                if (visited.Count > VOLUME_GOAL_FLOOD_LIMIT)
                {
                    floodedCells = visited.Count;
                    return false;
                }

                queue.Enqueue(next);
            }
        }

        floodedCells = visited.Count;
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

    private bool TryResolveRetreatGoal
    (
        Vector3           from,
        Vector3           to,
        CancellationToken cancel,
        out Vector3       retreatGoal
    )
    {
        retreatGoal = default;

        var ground = groundQuery.PlanMeshPathDetailed(from, to, 0, cancel);
        if (!ground.Succeeded || ground.Segments.Count == 0)
            return false;

        var segment  = ground.Segments[0];
        var corridor = segment.Corridor;
        if (corridor.Count == 0)
            return false;

        // 从终点端沿走廊逐个多边形往回退一段，让飞行目标避开门口这类体积上不可达的位置
        var remaining = VOLUME_GOAL_RETREAT_DISTANCE;
        var previous  = segment.EndPosition;

        for (var i = corridor.Count - 1; i >= 0; --i)
        {
            if (!query.MeshQuery.ClosestPointOnPoly(corridor[i], previous.ToRecast(), out var closest, out _).Succeeded())
                break;

            var point    = closest.ToSystem();
            var distance = Vector3.Distance(previous, point);

            if (distance >= remaining)
            {
                retreatGoal = Vector3.Lerp(previous, point, remaining / distance);
                return true;
            }

            remaining -= distance;
            previous   = point;
        }

        return false;
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

        if (!leftLeaf.empty || !rightLeaf.empty ||
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

        if (!groundResult.Succeeded || groundResult.Segments.Count == 0)
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
}
