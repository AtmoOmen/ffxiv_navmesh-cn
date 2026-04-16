using System.Numerics;
using vnavmesh.Bootstrap;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Query;

public partial class NavmeshQuery
{
    private const float FlightLandingProbeHalfExtentXZ      = 1f;
    private const float FlightLandingProbeHalfExtentY       = 2f;
    private const float FlightLandingMaxRequestHorizontal   = 1f;
    private const float FlightLandingMaxSafeHorizontal      = 1.5f;
    private const float FlightLandingMaxSafeVerticalDrop    = 8f;
    private const float FlightLandingCompletionSlack        = 0.25f;
    private const float FlightGroundTransitionSlack         = 0.25f;
    private const float FlightGroundApproachTriggerHorizontal = 0.5f;
    private const float FlightGroundApproachMinHorizontal     = 1.5f;
    private const float FlightGroundApproachMaxHorizontal     = 4f;
    private const float FlightGroundApproachHeightRatio       = 0.7f;

    internal PlannerResult PlanVolumePathDetailed(Vector3 from, Vector3 to, bool useRaycast, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("体素导航体未构建，无法执行飞行算路");
            return CreateFlightFailure(to);
        }

        var volume        = VolumeQuery.Volume;
        var locateTimer    = StopWatchTimer.Create();
        var startVoxel     = FindNearestVolumeVoxel(from);
        var endVoxel       = FindNearestVolumeVoxel(to);
        var locateDuration = locateTimer.Value();
        Service.Log.Debug($"[算路] 飞行体素 {startVoxel:X} -> {endVoxel:X}");

        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 无法定位空体素");
            return CreateFlightFailure(to);
        }

        var requestedStartLeaf  = volume.FindLeafVoxel(from);
        var requestedTargetLeaf = volume.FindLeafVoxel(to);
        var safeStart           = requestedStartLeaf.empty && requestedStartLeaf.voxel == startVoxel ? from : VoxelSearch.FindClosestVoxelPoint(volume, startVoxel, from);
        var safeDestination     = requestedTargetLeaf.empty && requestedTargetLeaf.voxel == endVoxel ? to : VoxelSearch.FindClosestVoxelPoint(volume, endVoxel, to);
        var safeDestinationAdjusted = Vector3.DistanceSquared(safeDestination, to) > 0.000001f;
        var searchTimer = StopWatchTimer.Create();
        var voxelPath = VolumeQuery.FindPath
            (startVoxel, endVoxel, safeStart, safeDestination, useRaycast, false, cancel);
        var telemetry = VolumeQuery.LastTelemetry;

        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 体素路径为空");
            return CreateFlightFailure(to);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行路径查询完成：空体素定位耗时 = {locateDuration.TotalSeconds:f3} 秒，主体搜索耗时 = {searchTimer.Value().TotalSeconds:f3} 秒，访问节点 = {telemetry.VisitedNodes}，生成节点 = {telemetry.GeneratedNodes}，LoS 检查 = {telemetry.LineOfSightChecks}，LoS 命中 = {telemetry.LineOfSightHits}，开放表峰值 = {telemetry.PeakOpenListSize}，路径点 = {voxelPath.Count}，起点修正 = {(Vector3.DistanceSquared(safeStart, from) > 0.000001f ? "是" : "否")}，安全终点修正 = {(safeDestinationAdjusted ? "是" : "否")}"
        );

        List<Vector3> rawWaypoints = new(voxelPath.Count);
        foreach (var step in voxelPath)
            rawWaypoints.Add(step.p);

        var finalDestination    = safeDestination;
        var destinationAdjusted = safeDestinationAdjusted;
        var landingPoint        = !requestedTargetLeaf.empty ? TryResolveFlightLandingPoint(to, safeDestination) : null;
        var completionTolerance = MathF.Max(_config.PathTolerance, FlightLandingCompletionSlack);

        if (landingPoint is { } resolvedLandingPoint)
        {
            finalDestination = resolvedLandingPoint;
            destinationAdjusted = Vector3.Distance(resolvedLandingPoint, to) > completionTolerance;

            if (rawWaypoints.Count == 0 || Vector3.DistanceSquared(rawWaypoints[^1], resolvedLandingPoint) > 0.000001f)
                rawWaypoints.Add(resolvedLandingPoint);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行终点解析：请求终点 = {to:f3}，空体素终点 = {safeDestination:f3}，落地点 = {(landingPoint is { } lp ? lp.ToString("f3") : "无")}，最终终点 = {finalDestination:f3}，落地吸附 = {(landingPoint != null ? "是" : "否")}"
        );

        if (landingPoint == null &&
            !requestedTargetLeaf.empty &&
            TryBuildFlightGroundTransitionResult(from, to, safeDestination, rawWaypoints, useRaycast, cancel, out var hybridResult))
            return hybridResult;

        return new()
        {
            Status               = destinationAdjusted ? PathfindStatus.Partial : PathfindStatus.Complete,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = to,
            FinalDestination     = finalDestination,
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
                    StartPosition        = from,
                    EndPosition          = finalDestination,
                    Points               = [.. rawWaypoints]
                }
            ]
        };
    }

    private Vector3? TryResolveFlightLandingPoint(Vector3 requestedTarget, Vector3 safeDestination)
    {
        var landingPoint = FindPointOnFloor(requestedTarget, FlightLandingProbeHalfExtentXZ)
            ?? FindNearestPointOnMesh(requestedTarget, FlightLandingProbeHalfExtentXZ, FlightLandingProbeHalfExtentY);
        if (landingPoint is not { } resolved)
            return null;

        var requestHorizontalDistance = HorizontalDistanceXZ(resolved, requestedTarget);
        if (requestHorizontalDistance > FlightLandingMaxRequestHorizontal)
            return null;

        var safeHorizontalDistance = HorizontalDistanceXZ(resolved, safeDestination);
        if (safeHorizontalDistance > FlightLandingMaxSafeHorizontal)
            return null;

        var verticalDrop = safeDestination.Y - resolved.Y;
        if (verticalDrop < -_config.PathTolerance || verticalDrop > FlightLandingMaxSafeVerticalDrop)
            return null;

        return resolved;
    }

    private static float HorizontalDistanceXZ(Vector3 left, Vector3 right)
    {
        var dx = left.X - right.X;
        var dz = left.Z - right.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
    }

    private Vector3? TryBuildFlightGroundApproachPoint(Vector3 safeFlightDestination, Vector3 transitionPoint, Vector3 groundLeadTarget, Vector3 requestedTarget)
    {
        var horizontalGap = HorizontalDistanceXZ(safeFlightDestination, transitionPoint);
        if (horizontalGap > FlightGroundApproachTriggerHorizontal)
            return null;

        var verticalDrop = safeFlightDestination.Y - transitionPoint.Y;
        if (verticalDrop <= FlightGroundTransitionSlack)
            return null;

        var leadDelta = new Vector2(groundLeadTarget.X - transitionPoint.X, groundLeadTarget.Z - transitionPoint.Z);
        if (leadDelta.LengthSquared() <= 0.000001f)
            leadDelta = new Vector2(requestedTarget.X - transitionPoint.X, requestedTarget.Z - transitionPoint.Z);
        if (leadDelta.LengthSquared() <= 0.000001f)
            return null;

        leadDelta = Vector2.Normalize(leadDelta);
        var approachHorizontal = Math.Clamp(verticalDrop * 0.75f, FlightGroundApproachMinHorizontal, FlightGroundApproachMaxHorizontal);
        var candidate = new Vector3
        (
            transitionPoint.X - leadDelta.X * approachHorizontal,
            transitionPoint.Y + verticalDrop * FlightGroundApproachHeightRatio,
            transitionPoint.Z - leadDelta.Y * approachHorizontal
        );

        var approachVoxel = FindNearestVolumeVoxel(candidate, FlightGroundApproachMinHorizontal, MathF.Max(1f, verticalDrop * 0.5f));
        if (approachVoxel == VoxelMap.InvalidVoxel)
            return candidate;

        return VoxelSearch.FindClosestVoxelPoint(VolumeQuery!.Volume, approachVoxel, candidate);
    }

    private bool TryBuildFlightGroundTransitionResult
    (
        Vector3           requestedStart,
        Vector3           requestedTarget,
        Vector3           safeFlightDestination,
        List<Vector3>     rawFlightWaypoints,
        bool              useRaycast,
        CancellationToken cancel,
        out PlannerResult result
    )
    {
        var groundResult = PlanMeshPathDetailed(safeFlightDestination, requestedTarget, useRaycast, 0, cancel);
        if (!groundResult.Succeeded || groundResult.Segments.Count == 0)
        {
            result = default!;
            return false;
        }

        var transitionPoint = groundResult.Segments[0].StartPosition;
        var approachPoint   = TryBuildFlightGroundApproachPoint(safeFlightDestination, transitionPoint, groundResult.Segments[0].EndPosition, requestedTarget);
        List<Vector3> flightWaypoints = [.. rawFlightWaypoints];
        if (approachPoint is { } resolvedApproachPoint &&
            (flightWaypoints.Count == 0 || Vector3.DistanceSquared(flightWaypoints[^1], resolvedApproachPoint) > 0.000001f))
            flightWaypoints.Add(resolvedApproachPoint);
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
                StartPosition        = requestedStart,
                EndPosition          = transitionPoint,
                Points               = flightWaypoints
            }
        ];
        foreach (var segment in groundResult.Segments)
            segments.Add(segment);

        var transitionAdjusted = Vector3.Distance(safeFlightDestination, transitionPoint) > FlightGroundTransitionSlack;
        Service.Log.Debug
        (
            $"[算路] 飞行接地面续算：空体素终点 = {safeFlightDestination:f3}，近地点 = {(approachPoint is { } ap ? ap.ToString("f3") : "无")}，桥接点 = {transitionPoint:f3}，桥接修正 = {(transitionAdjusted ? "是" : "否")}，地面结果 = {groundResult.Status}，地面段数 = {groundResult.Segments.Count}"
        );

        result = new()
        {
            Status               = groundResult.Status,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = requestedTarget,
            FinalDestination     = groundResult.FinalDestination,
            DestinationTolerance = groundResult.DestinationTolerance,
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

}
