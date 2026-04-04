using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Planning;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Query;

public partial class NavmeshQuery
{
    private PlannerResult LogMeshFailure(Vector3 from, Vector3 to, long startRef, long endRef, long lastPoly, float range, string reason)
    {
        var lastPolyText = lastPoly != 0 ? lastPoly.ToString("X") : "<none>";
        Service.Log.Error($"地面算路失败：起点 = {from:f3}，请求终点 = {to:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPolyText}，容差 = {range:f3}，原因 = {reason}");
        return new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = to,
            FinalDestination     = to,
            DestinationTolerance = range
        };
    }

    private void LogMeshResult(PlannerResult result, Vector3 from, long startRef, long endRef, long lastPoly, float range, TimeSpan duration)
    {
        var diagnostic = BuildSeamDiagnostic(from, result.RequestedDestination, result.FinalDestination);
        var message =
            $"地面算路完成：状态 = {result.Status}，起点 = {from:f3}，请求终点 = {result.RequestedDestination:f3}，实际终点 = {result.FinalDestination:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPoly:X}，容差 = {range:f3}，耗时 = {duration.TotalSeconds:f3} 秒，粗路径段 = {result.Segments.Count}";

        if (result.Status == PathfindStatus.Partial)
            message +=
                $"，起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}";

        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Service.Log.Warning(message);

                if (diagnostic.IsSuspectedTileSeamCutoff)
                {
                    Service.Log.Warning
                    (
                        $"[SuspectedTileSeamCutoff] 疑似区块接缝截断：起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}"
                    );
                }

                break;
            case PathfindStatus.Failed:
                Service.Log.Error(message);
                break;
            default:
                Service.Log.Debug(message);
                break;
        }
    }

    private bool TryRepairShortGroundGap
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        CancellationToken cancel,
        out PlannerResult repairedResult
    )
    {
        repairedResult = default!;

        var partialEnd = partialCandidate.FinalDestination;
        var repairCandidates = FindIntersectingMeshPolys
            (partialEnd, new(ShortGapRepairSearchHalfExtentXZ, ShortGapRepairSearchHalfExtentY, ShortGapRepairSearchHalfExtentXZ));
        MeshPathCandidate? bestResumeCandidate = null;
        List<string>       candidateLogs       = [];

        foreach (var poly in repairCandidates)
        {
            if (poly == 0 || poly == partialCandidate.LastPoly || partialCandidate.Corridor.Contains(poly))
                continue;

            cancel.ThrowIfCancellationRequested();

            var projectedPoint = FindNearestPointOnMeshPoly(partialEnd, poly);
            if (projectedPoint == null)
                continue;

            var bridgePoint              = projectedPoint.Value;
            var bridgeDelta              = bridgePoint - partialEnd;
            var bridgeHorizontalDistance = new Vector2(bridgeDelta.X, bridgeDelta.Z).Length();
            var bridgeVerticalDistance   = MathF.Abs(bridgeDelta.Y);
            if (bridgeHorizontalDistance > ShortGapRepairMaxBridgeDistance || bridgeVerticalDistance > ShortGapRepairMaxVerticalDelta)
                continue;

            var corridor = new List<long>();
            var status   = MeshQuery.FindPath(poly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, ref corridor, opt);

            if (status.Failed() || corridor.Count == 0)
            {
                candidateLogs.Add($"{poly:X}: 失败 ({status})");
                continue;
            }

            var resumeCandidate = BuildMeshPathCandidate
            (
                new(poly, bridgePoint, bridgeHorizontalDistance * bridgeHorizontalDistance, bridgePoint.Y - partialEnd.Y),
                corridor,
                status,
                requestedEndPos,
                requestedTarget,
                endRef,
                range
            );

            if (resumeCandidate.ResultStatus == PathfindStatus.Failed)
            {
                candidateLogs.Add($"{poly:X}: 失败（无法投影终点）");
                continue;
            }

            candidateLogs.Add($"{poly:X}: {resumeCandidate.ResultStatus}，桥接距离 {bridgeHorizontalDistance:f3}，高差 {bridgeVerticalDistance:f3}");
            if (bestResumeCandidate == null || IsBetterStartPathCandidate(resumeCandidate, bestResumeCandidate.Value, requestedTarget))
                bestResumeCandidate = resumeCandidate;

            if (resumeCandidate.ResultStatus == PathfindStatus.Complete)
                break;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] Partial 终点附近候选评估：{string.Join(" | ", candidateLogs)}");

        if (bestResumeCandidate == null)
            return false;

        var bridgePointToAdd = bestResumeCandidate.Value.StartPoint;
        var mergedWaypoints = new List<Vector3>
        {
            partialCandidate.StartPoint
        };
        mergedWaypoints.AddRange(partialCandidate.Corridor.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()));
        mergedWaypoints.Add(partialCandidate.FinalDestination);
        mergedWaypoints.Add(bridgePointToAdd);
        mergedWaypoints.AddRange(bestResumeCandidate.Value.Corridor.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()));
        mergedWaypoints.Add(bestResumeCandidate.Value.FinalDestination);

        Service.Log.Warning
            ($"[算路] 已触发短距补桥：partial 终点 = {partialCandidate.FinalDestination:f3}，桥接点 = {bridgePointToAdd:f3}，桥接后结果 = {bestResumeCandidate.Value.ResultStatus}");
        repairedResult = new()
        {
            Status               = bestResumeCandidate.Value.ResultStatus,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = requestedTarget,
            FinalDestination     = bestResumeCandidate.Value.FinalDestination,
            DestinationTolerance = range,
            Segments =
            [
                new()
                {
                    MovementMode         = MovementMode.Ground,
                    SegmentKind          = MovementSegmentKind.GroundTraverse,
                    AllowVerticalControl = false,
                    ReachabilitySource   = PathReachabilitySource.Mesh,
                    GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                    StartPosition        = partialCandidate.StartPoint,
                    EndPosition          = bestResumeCandidate.Value.FinalDestination,
                    Points               = [.. mergedWaypoints]
                }
            ]
        };
        return true;
    }

    private SeamDiagnostic BuildSeamDiagnostic(Vector3 from, Vector3 requested, Vector3 actual)
    {
        var startTile        = ToTileCoord(FindMeshTile(from));
        var requestedTile    = ToTileCoord(FindMeshTile(requested));
        var actualTile       = ToTileCoord(FindMeshTile(actual));
        var gap              = Vector3.Distance(requested, actual);
        var boundaryDistance = DistanceToNearestTileBoundary(actual);
        var isNearbyTile     = Math.Abs(requestedTile.X - actualTile.X) <= 1 && Math.Abs(requestedTile.Z - actualTile.Z) <= 1;
        return new
        (
            startTile,
            requestedTile,
            actualTile,
            boundaryDistance,
            boundaryDistance <= SuspectedTileSeamBoundaryMaxDistance,
            isNearbyTile,
            gap <= SuspectedTileSeamGapMaxDistance
        );
    }

    private float DistanceToNearestTileBoundary(Vector3 position)
    {
        var (tileX, tileZ)     = FindMeshTile(position);
        var (tileMin, tileMax) = GetMeshTileBounds(tileX, tileZ);
        var distMinX = MathF.Abs(position.X - tileMin.X);
        var distMaxX = MathF.Abs(tileMax.X  - position.X);
        var distMinZ = MathF.Abs(position.Z - tileMin.Z);
        var distMaxZ = MathF.Abs(tileMax.Z  - position.Z);
        return MathF.Min(MathF.Min(distMinX, distMaxX), MathF.Min(distMinZ, distMaxZ));
    }

    private static int ResultStatusRank(PathfindStatus status) => status switch
    {
        PathfindStatus.Complete           => 0,
        PathfindStatus.ReachedWithinRange => 1,
        PathfindStatus.Partial            => 2,
        _                                 => 3
    };

    private static bool NearlyEqual(float left, float right) => MathF.Abs(left - right) <= 0.0001f;

    private static TileCoord ToTileCoord((int x, int z) tile) => new(tile.x, tile.z);
}
