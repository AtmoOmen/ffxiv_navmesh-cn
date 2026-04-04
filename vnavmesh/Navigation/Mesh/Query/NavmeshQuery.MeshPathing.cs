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
    internal PlannerResult PlanMeshPathDetailed(Vector3 from, Vector3 to, bool useRaycast, float range, CancellationToken cancel)
    {
        var requestedStartRef = FindNearestMeshPoly(from);
        var endRef            = FindNearestMeshPoly(to);
        if (requestedStartRef == 0 || endRef == 0)
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法在导航网格上找到起点或终点多边形");

        var timer = StopWatchTimer.Create();
        LastPath.Clear();
        var opt = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0,
            useRaycast ? 5 : 0
        );
        var            randomness = _config.RandomnessMultiplier;
        IDtQueryFilter filter     = randomness > 0 ? _randomnessFilter : _teleportFilter;

        if (randomness > 0)
        {
            _randomnessFilter.RandomnessMultiplier = randomness;
            _randomnessFilter.RandomSeed           = (ulong)Random.Shared.NextInt64();
        }

        var requestedEndPos    = to.SystemToRecast();
        var bestStartCandidate = ResolveBestStartPathCandidate(from, to, requestedStartRef, endRef, requestedEndPos, filter, opt, range, cancel);
        if (bestStartCandidate == null)
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");

        var pathCandidate = bestStartCandidate.Value;
        var startRef      = pathCandidate.StartRef;
        LastPath.AddRange(pathCandidate.Corridor);
        var pathStatus = pathCandidate.QueryStatus;
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}（原始起点候选 = {requestedStartRef:X}）");
        Service.Log.Debug($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {pathStatus}，路径 = {string.Join(", ", LastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();

        var lastPoly     = pathCandidate.LastPoly;
        var resultStatus = pathCandidate.ResultStatus;

        if (resultStatus == PathfindStatus.Partial &&
            TryRepairShortGroundGap(pathCandidate, to, endRef, requestedEndPos, filter, opt, range, cancel, out var repairedResult))
        {
            LogMeshResult(repairedResult, from, startRef, endRef, lastPoly, range, timer.Value());
            return repairedResult;
        }

        var result = new PlannerResult
        {
            Status               = resultStatus,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = to,
            FinalDestination     = pathCandidate.FinalDestination,
            DestinationTolerance = range,
            Segments =
            [
                new()
                {
                    MovementMode         = MovementMode.Ground,
                    SegmentKind          = MovementSegmentKind.GroundTraverse,
                    AllowVerticalControl = false,
                    ReachabilitySource   = PathReachabilitySource.Mesh,
                    GeometryKind         = PlannerSegmentGeometryKind.MeshCorridor,
                    StartPosition        = pathCandidate.StartPoint,
                    EndPosition          = pathCandidate.FinalDestination,
                    Corridor             = [.. LastPath]
                }
            ]
        };
        LogMeshResult(result, from, startRef, endRef, lastPoly, range, timer.Value());
        return result;
    }

    private MeshPathCandidate? ResolveBestStartPathCandidate
    (
        Vector3           from,
        Vector3           to,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        CancellationToken cancel
    )
    {
        var candidates = CollectStartPolyCandidates(from, requestedStartRef);
        if (candidates.Count == 0)
            return null;

        MeshPathCandidate? best          = null;
        List<string>       candidateLogs = [];

        foreach (var candidate in candidates.Take(MaxStartPolyCandidatesToEvaluate))
        {
            cancel.ThrowIfCancellationRequested();

            var corridor = new List<long>();
            var status   = MeshQuery.FindPath(candidate.PolyRef, endRef, candidate.ProjectedPoint.SystemToRecast(), requestedEndPos, filter, ref corridor, opt);

            if (status.Failed() || corridor.Count == 0)
            {
                candidateLogs.Add($"{candidate.PolyRef:X}: 失败 ({status})");
                continue;
            }

            var pathCandidate = BuildMeshPathCandidate(candidate, corridor, status, requestedEndPos, to, endRef, range);

            if (pathCandidate.ResultStatus == PathfindStatus.Failed)
            {
                candidateLogs.Add($"{candidate.PolyRef:X}: 失败（无法投影最终可达点）");
                continue;
            }

            candidateLogs.Add
            (
                $"{candidate.PolyRef:X}: {pathCandidate.ResultStatus}，距目标 {MathF.Sqrt(pathCandidate.DistanceToRequestedTargetSq(to)):f3}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}"
            );

            if (best == null || IsBetterStartPathCandidate(pathCandidate, best.Value, to))
                best = pathCandidate;

            if (pathCandidate.ResultStatus == PathfindStatus.Complete)
                break;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] 起点候选评估：{string.Join(" | ", candidateLogs)}");

        if (best is { } selected)
        {
            Service.Log.Debug($"[算路] 已选起点多边形 {selected.StartRef:X}，投影点 = {selected.StartPoint:f3}，结果 = {selected.ResultStatus}");
            if (selected.StartRef != requestedStartRef)
                Service.Log.Warning($"[算路] 已修正起点多边形：原始候选 = {requestedStartRef:X}，选中 = {selected.StartRef:X}");
        }

        return best;
    }

    private List<MeshPolyCandidate> CollectStartPolyCandidates(Vector3 from, long requestedStartRef)
    {
        HashSet<long>           seen       = [];
        List<MeshPolyCandidate> candidates = [];

        foreach (var poly in FindIntersectingMeshPolys(from, new(StartPolyCandidateHalfExtentXZ, StartPolyCandidateHalfExtentY, StartPolyCandidateHalfExtentXZ)))
            TryAddCandidate(poly, false);

        TryAddCandidate(requestedStartRef, true);

        candidates.Sort
        (static (a, b) =>
            {
                var bucketA = a.IsTooFarAbove ? 1 : 0;
                var bucketB = b.IsTooFarAbove ? 1 : 0;
                var cmp     = bucketA.CompareTo(bucketB);
                if (cmp != 0)
                    return cmp;

                cmp = a.VerticalDistanceAbs.CompareTo(b.VerticalDistanceAbs);
                if (cmp != 0)
                    return cmp;

                cmp = a.HorizontalDistanceSq.CompareTo(b.HorizontalDistanceSq);
                if (cmp != 0)
                    return cmp;

                return a.PolyRef.CompareTo(b.PolyRef);
            }
        );

        return candidates;

        void TryAddCandidate(long poly, bool forceInclude)
        {
            if (poly == 0 || !seen.Add(poly))
                return;

            var projected = FindNearestPointOnMeshPoly(from, poly);
            if (projected == null)
                return;

            var point                = projected.Value;
            var dx                   = point.X - from.X;
            var dz                   = point.Z - from.Z;
            var horizontalDistanceSq = dx * dx + dz * dz;
            var verticalDistance     = point.Y - from.Y;

            if (!forceInclude)
            {
                if (horizontalDistanceSq > StartPolyCandidateMaxHorizontalDistance * StartPolyCandidateMaxHorizontalDistance)
                    return;
                if (MathF.Abs(verticalDistance) > StartPolyCandidateMaxVerticalDistance)
                    return;
            }

            candidates.Add(new(poly, point, horizontalDistanceSq, verticalDistance));
        }
    }

    private MeshPathCandidate BuildMeshPathCandidate
        (MeshPolyCandidate startCandidate, List<long> corridor, DtStatus status, RcVec3f requestedEndPos, Vector3 requestedTarget, long endRef, float range)
    {
        var     lastPoly     = corridor[^1];
        var     resultStatus = PathfindStatus.Complete;
        Vector3 finalDestination;

        if (status.IsPartial() || lastPoly != endRef)
        {
            var closestStatus = MeshQuery.ClosestPointOnPoly(lastPoly, requestedEndPos, out var projectedEndPos, out _);
            if (closestStatus.Failed())
                return new(startCandidate, status, PathfindStatus.Failed, startCandidate.ProjectedPoint, corridor);

            finalDestination = projectedEndPos.RecastToSystem();
            resultStatus = range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }
        else finalDestination = requestedTarget;

        return new(startCandidate, status, resultStatus, finalDestination, corridor);
    }

    private static bool IsBetterStartPathCandidate(MeshPathCandidate candidate, MeshPathCandidate currentBest, Vector3 requestedTarget)
    {
        var rankCandidate = ResultStatusRank(candidate.ResultStatus);
        var rankCurrent   = ResultStatusRank(currentBest.ResultStatus);
        if (rankCandidate != rankCurrent)
            return rankCandidate < rankCurrent;

        var candidateDistance = candidate.DistanceToRequestedTargetSq(requestedTarget);
        var currentDistance   = currentBest.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(candidateDistance, currentDistance))
            return candidateDistance < currentDistance;

        if (!NearlyEqual(candidate.StartCandidate.VerticalDistanceAbs, currentBest.StartCandidate.VerticalDistanceAbs))
            return candidate.StartCandidate.VerticalDistanceAbs < currentBest.StartCandidate.VerticalDistanceAbs;

        if (!NearlyEqual(candidate.StartCandidate.HorizontalDistanceSq, currentBest.StartCandidate.HorizontalDistanceSq))
            return candidate.StartCandidate.HorizontalDistanceSq < currentBest.StartCandidate.HorizontalDistanceSq;

        return candidate.StartRef < currentBest.StartRef;
    }
}
