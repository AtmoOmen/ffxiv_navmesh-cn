using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Common.Utilities;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation;

internal sealed class NavmeshFlightQuery
{
    private readonly NavmeshQuery       query;
    private readonly NavmeshGroundQuery groundQuery;

    internal NavmeshFlightQuery(NavmeshQuery query, NavmeshGroundQuery groundQuery)
    {
        this.query       = query;
        this.groundQuery = groundQuery;
    }

    internal PlannerResult PlanVolumePathDetailed(Vector3 from, Vector3 to, CancellationToken cancel)
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

        var requestedStartLeaf = volume.FindLeafVoxel(from);
        var requestedTargetLeaf = volume.FindLeafVoxel(to);
        var safeStart = !startLocate.UsedSurfaceAnchor && requestedStartLeaf.empty && requestedStartLeaf.voxel == startVoxel
                            ? from
                            : startLocate.SafePoint;
        var safeDestination = !endLocate.UsedSurfaceAnchor && requestedTargetLeaf.empty && requestedTargetLeaf.voxel == endVoxel
                                  ? to
                                  : endLocate.SafePoint;
        var safeDestinationAdjusted = Vector3.DistanceSquared(safeDestination, to) > 0.000001f;
        var searchTimer = StopWatchTimer.Create();
        var voxelPath = volumeQuery.FindPath
            (startVoxel, endVoxel, safeStart, safeDestination, false, cancel);
        var telemetry = volumeQuery.LastTelemetry;

        Service.Log.Debug
        (
            $"[算路] 飞行路径查询完成：空体素定位耗时 = {locateDuration.TotalSeconds:f3} 秒，主体搜索耗时 = {searchTimer.Value().TotalSeconds:f3} 秒，访问节点 = {telemetry.VisitedNodes}，生成节点 = {telemetry.GeneratedNodes}，LoS 检查 = {telemetry.LineOfSightChecks}，LoS 命中 = {telemetry.LineOfSightHits}，开放表峰值 = {telemetry.PeakOpenListSize}，终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，搜索轮次 = {telemetry.SearchAttempts}，启发式权重 = {telemetry.HeuristicWeight:f2}，路径点 = {voxelPath.Count}，起点修正 = {(Vector3.DistanceSquared(safeStart, from) > 0.000001f ? "是" : "否")}，安全终点修正 = {(safeDestinationAdjusted ? "是" : "否")}"
        );
        var flightDebug = volumeQuery.LastPathDebug;

        if (voxelPath.Count == 0 && flightDebug is { CoarsePath.Count: > 0 })
        {
            Service.Log.Warning("[算路] 飞行体素调试：返回粗层 best-effort 叠加，不生成正式路线");
            return new()
            {
                Status               = PathfindStatus.Partial,
                RequestedMode        = MovementMode.Flight,
                RequestedDestination = to,
                FinalDestination     = to,
                DestinationTolerance = 0,
                Segments =
                [
                    new()
                    {
                        MovementMode           = MovementMode.Flight,
                        SegmentKind            = MovementSegmentKind.FlightTraverse,
                        AllowVerticalControl   = true,
                        ReachabilitySource     = PathReachabilitySource.Volume,
                        GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
                        TraversalStartPosition = from,
                        StartPosition          = from,
                        EndPosition            = to,
                        Points                 = [],
                        FlightPathDebug        = flightDebug
                    }
                ]
            };
        }

        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 体素路径为空，终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，访问节点 = {telemetry.VisitedNodes}");
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

            if (distanceToGoal <= nearGoalThreshold)
            {
                Service.Log.Debug
                (
                    $"[算路] 飞行体素搜索近终点视为完成：终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，路径终点 = {partialDestination:f3}，安全终点 = {safeDestination:f3}，距离 = {distanceToGoal:f3}，阈值 = {nearGoalThreshold:f3}"
                );

                if (Vector3.DistanceSquared(partialDestination, safeDestination) > 0.000001f)
                    rawWaypoints.Add(safeDestination);
            }
            else
            {
                Service.Log.Warning
                (
                    $"飞行体素搜索未抵达终点：终止 = {GetLogVolumeSearchTermination(telemetry.Termination)}，请求空体素终点 = {safeDestination:f3}，当前终点 = {partialDestination:f3}，距离 = {distanceToGoal:f3}，阈值 = {nearGoalThreshold:f3}"
                );

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
                            MovementMode         = MovementMode.Flight,
                            SegmentKind          = MovementSegmentKind.FlightTraverse,
                            AllowVerticalControl = true,
                            ReachabilitySource   = PathReachabilitySource.Volume,
                            GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                            TraversalStartPosition = from,
                            StartPosition        = from,
                            EndPosition          = partialDestination,
                            Points               = [.. rawWaypoints],
                            FlightPathDebug      = flightDebug
                        }
                    ]
                };
            }
        }

        if (!requestedTargetLeaf.empty &&
            TryBuildFlightGroundTransitionResult(from, to, safeDestination, rawWaypoints, cancel, out var hybridResult))
            return hybridResult;

        var finalDestination    = safeDestination;
        var destinationAdjusted = safeDestinationAdjusted;
        var landingPoint        = TryResolveFlightLandingPoint(to, safeDestination);
        var completionTolerance = MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(to, safeDestination));

        if (landingPoint is { } resolvedLandingPoint)
        {
            finalDestination    = resolvedLandingPoint;
            completionTolerance = MathF.Max(query.ConfigData.PathTolerance, HorizontalDistanceXZ(to, resolvedLandingPoint));
            destinationAdjusted = Vector3.Distance(resolvedLandingPoint, to) > completionTolerance;

            if (rawWaypoints.Count == 0 || Vector3.DistanceSquared(rawWaypoints[^1], resolvedLandingPoint) > 0.000001f)
                rawWaypoints.Add(resolvedLandingPoint);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行终点解析：请求终点 = {to:f3}，空体素终点 = {safeDestination:f3}，落地点 = {(landingPoint is { } lp ? lp.ToString("f3") : "无")}，最终终点 = {finalDestination:f3}，落地吸附 = {(landingPoint != null ? "是" : "否")}"
        );

        var finalDestinationTolerance = landingPoint != null ? completionTolerance : 0;

        return new()
        {
            Status               = destinationAdjusted ? PathfindStatus.Partial : PathfindStatus.Complete,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = to,
            FinalDestination     = finalDestination,
            DestinationTolerance = finalDestinationTolerance,
            Segments =
            [
                new()
                {
                    MovementMode         = MovementMode.Flight,
                    SegmentKind          = MovementSegmentKind.FlightTraverse,
                    AllowVerticalControl = true,
                    ReachabilitySource   = PathReachabilitySource.Volume,
                    GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                    TraversalStartPosition = from,
                    StartPosition        = from,
                    EndPosition          = finalDestination,
                    Points               = [.. rawWaypoints],
                    FlightPathDebug      = flightDebug
                }
            ]
        };
    }

    private Vector3? TryResolveFlightLandingPoint(Vector3 requestedTarget, Vector3 safeDestination)
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
                               (requestedTarget, landingSearchExtent.X, allowUnreachable: true) ??
                           query.FindNearestPointOnMesh(requestedTarget, landingSearchExtent.X, landingSearchExtent.Y, allowUnreachable: true);
        if (landingPoint is not { } resolved)
            return null;

        var requestHorizontalDistance = HorizontalDistanceXZ(resolved, requestedTarget);
        if (requestHorizontalDistance > landingSearchExtent.X)
            return null;

        var safeHorizontalDistance = HorizontalDistanceXZ(resolved, safeDestination);
        if (safeHorizontalDistance > HorizontalDistanceXZ(safeDestination, requestedTarget) + toleranceFloor)
            return null;

        var verticalDrop = safeDestination.Y - resolved.Y;
        if (verticalDrop < -query.ConfigData.PathTolerance || verticalDrop > MathF.Abs(safeDestination.Y - requestedTarget.Y) + landingSearchExtent.Y)
            return null;

        return resolved;
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
        if (verticalDrop <= transitionTolerance)
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
            transitionPoint.X - leadDelta.X  * approachHorizontal,
            transitionPoint.Y + verticalDrop * (approachHorizontal / (approachHorizontal + verticalDrop)),
            transitionPoint.Z - leadDelta.Y  * approachHorizontal
        );

        var approachLocate = query.FindNearestVolumeVoxelSurfaceAware(candidate, transitionTolerance, MathF.Max(toleranceFloor, verticalDrop));
        var approachVoxel = approachLocate.Voxel;
        if (approachVoxel == VoxelMap.INVALID_VOXEL)
            return candidate;

        return approachLocate.SafePoint;
    }

    private void TrimFlightWaypointsForGroundTransition(List<Vector3> flightWaypoints, Vector3 approachPoint)
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
        out PlannerResult result
    )
    {
        var toleranceFloor = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
        var groundResult   = groundQuery.PlanMeshPathDetailed(safeFlightDestination, requestedTarget, 0, cancel);

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
                MovementMode         = MovementMode.Flight,
                SegmentKind          = MovementSegmentKind.FlightTraverse,
                AllowVerticalControl = true,
                ReachabilitySource   = PathReachabilitySource.Volume,
                GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                TraversalStartPosition = requestedStart,
                StartPosition        = requestedStart,
                EndPosition          = transitionPoint,
                Points               = flightWaypoints,
                FlightPathDebug      = query.VolumeQuery?.LastPathDebug
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

    private static PlannerResult CreateFlightFailure(Vector3 destination) =>
        new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = destination,
            FinalDestination     = destination,
            DestinationTolerance = 0
        };

    private static string GetLogVolumeSearchTermination(VolumeSearchTermination termination) => termination switch
    {
        VolumeSearchTermination.ReachedGoal       => "达到终点",
        VolumeSearchTermination.SearchExhausted   => "搜索穷尽",
        VolumeSearchTermination.StepBudgetReached => "步数触顶",
        _                                         => "未知"
    };

    private static float ComputeNearGoalThreshold(VoxelMap volume)
    {
        var l1CellSize = volume.Levels[1].CellSize;
        var maxL1Extent = MathF.Max(l1CellSize.X, MathF.Max(l1CellSize.Y, l1CellSize.Z));
        return maxL1Extent * 2f;
    }

    private static float HorizontalDistanceXZ(Vector3 left, Vector3 right)
    {
        var dx = left.X           - right.X;
        var dz = left.Z           - right.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
    }

}
