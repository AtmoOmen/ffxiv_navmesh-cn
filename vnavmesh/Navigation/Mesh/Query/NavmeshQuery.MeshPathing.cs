using System.Numerics;
using System.Threading.Tasks;
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
        var requestedEndRef   = FindNearestMeshPoly(to);
        var endCandidate      = ResolveBestEndPathCandidate(to, requestedEndRef);
        if (endCandidate == null)
            return LogMeshFailure(from, to, requestedStartRef, requestedEndRef, 0, range, "无法为终点选择可用的导航多边形");

        var resolvedDestination = endCandidate.Value.ProjectedPoint;
        var endRef              = endCandidate.Value.PolyRef;

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

        var requestedEndPos    = resolvedDestination.SystemToRecast();
        var bestStartCandidate = ResolveBestStartPathCandidate(from, to, requestedStartRef, endRef, requestedEndPos, filter, opt, range, cancel);
        if (bestStartCandidate == null)
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");

        var pathCandidate = bestStartCandidate.Value;
        var startRef      = pathCandidate.StartRef;
        LastPath.AddRange(pathCandidate.Corridor);
        var pathStatus = pathCandidate.QueryStatus;
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}（原始起点候选 = {requestedStartRef:X}）");
        Service.Log.Debug($"[算路] 地面终点解析：原始终点候选 = {requestedEndRef:X}，选中 = {endRef:X}，终点投影 = {resolvedDestination:f3}");
        Service.Log.Debug($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {pathStatus}，路径 = {string.Join(", ", LastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();

        var lastPoly     = pathCandidate.LastPoly;
        var resultStatus = pathCandidate.ResultStatus;

        if (resultStatus == PathfindStatus.Partial &&
            TryRepairShortGroundGap(pathCandidate, to, endRef, requestedEndPos, filter, opt, range, cancel, out var repairedResult, out var repairedLastPoly))
        {
            LogMeshResult(repairedResult, from, startRef, endRef, repairedLastPoly, range, timer.Value());
            return repairedResult;
        }

        var result = BuildGroundPlannerResult(to, range, resultStatus, pathCandidate.FinalDestination, [BuildGroundMeshCorridorSegment(pathCandidate)]);
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

        var candidateBatch = candidates.Take(MaxStartPolyCandidatesToEvaluate).ToArray();

        MeshPathCandidate? best                = null;
        MeshPathCandidate? requestedSuccessful = null;
        MeshPathCandidate? requestedLocked     = null;
        List<string>       candidateLogs       = [];
        var                requestedFailed     = false;

        var evaluations = EvaluateStartPathCandidates(candidateBatch, to, endRef, requestedEndPos, filter, opt, range, cancel);
        foreach (var evaluation in evaluations)
        {
            if (!string.IsNullOrEmpty(evaluation.Log))
                candidateLogs.Add(evaluation.Log);

            if (evaluation.RequestedFailed)
                requestedFailed = true;

            if (evaluation.PathCandidate is not { } pathCandidate)
                continue;

            if (pathCandidate.IsRequestedStart)
                requestedSuccessful = pathCandidate;

            if (pathCandidate.IsRequestedStart && pathCandidate.IsPointOverPoly)
                requestedLocked = pathCandidate;

            if (best == null || IsBetterStartPathCandidate(pathCandidate, best.Value, to))
                best = pathCandidate;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] 起点候选评估：{string.Join(" | ", candidateLogs)}");

        if (requestedLocked is { } locked)
        {
            LogStartCandidateDecision(locked, to, requestedStartRef, requestedSuccessful, requestedFailed, true);
            return locked;
        }

        if (best is { } selected)
            LogStartCandidateDecision(selected, to, requestedStartRef, requestedSuccessful, requestedFailed, false);

        return best;
    }

    private StartCandidateEvaluation[] EvaluateStartPathCandidates
    (
        IReadOnlyList<MeshPolyCandidate> candidates,
        Vector3                          requestedTarget,
        long                             endRef,
        RcVec3f                          requestedEndPos,
        IDtQueryFilter                   filter,
        DtFindPathOption                 opt,
        float                            range,
        CancellationToken                cancel
    )
    {
        var result = new StartCandidateEvaluation[candidates.Count];
        if (candidates.Count == 0)
            return result;

        if (candidates.Count > 1 && Environment.ProcessorCount > 1)
        {
            Parallel.For
            (
                0,
                candidates.Count,
                new ParallelOptions { CancellationToken = cancel, MaxDegreeOfParallelism = Math.Min(Environment.ProcessorCount, candidates.Count) },
                i => result[i] = EvaluateStartPathCandidate(candidates[i], requestedTarget, endRef, requestedEndPos, filter, opt, range)
            );
        }
        else
        {
            for (var i = 0; i < candidates.Count; ++i)
            {
                cancel.ThrowIfCancellationRequested();
                result[i] = EvaluateStartPathCandidate(candidates[i], requestedTarget, endRef, requestedEndPos, filter, opt, range);
            }
        }

        return result;
    }

    private StartCandidateEvaluation EvaluateStartPathCandidate
    (
        MeshPolyCandidate candidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range
    )
    {
        var query    = new DtNavMeshQuery(MeshQuery.GetAttachedNavMesh());
        var corridor = new List<long>();
        var status   = query.FindPath(candidate.PolyRef, endRef, candidate.ProjectedPoint.SystemToRecast(), requestedEndPos, filter, ref corridor, opt);

        if (status.Failed() || corridor.Count == 0)
        {
            return new
            (
                null,
                $"{candidate.PolyRef:X}: 失败 ({status})，requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}",
                candidate.IsRequestedStart
            );
        }

        var pathCandidate = BuildMeshPathCandidate(query, candidate, corridor, status, requestedEndPos, requestedTarget, endRef, range);
        if (pathCandidate.ResultStatus == PathfindStatus.Failed)
        {
            return new
            (
                null,
                $"{candidate.PolyRef:X}: 失败（无法投影最终可达点），requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}",
                candidate.IsRequestedStart
            );
        }

        return new
        (
            pathCandidate,
            $"{candidate.PolyRef:X}: {pathCandidate.ResultStatus}，requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}，距目标 {MathF.Sqrt(pathCandidate.DistanceToRequestedTargetSq(requestedTarget)):f3}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}",
            false
        );
    }

    private List<MeshPolyCandidate> CollectStartPolyCandidates(Vector3 from, long requestedStartRef)
    {
        Dictionary<long, int>   candidateIndices = [];
        List<MeshPolyCandidate> candidates       = [];

        foreach (var poly in FindIntersectingMeshPolys(from, new(StartPolyCandidateHalfExtentXZ, StartPolyCandidateHalfExtentY, StartPolyCandidateHalfExtentXZ)))
            TryAddCandidate(poly, false);

        TryAddCandidate(requestedStartRef, true);

        candidates.Sort
        (static (a, b) =>
            {
                if (a.IsRequestedStart != b.IsRequestedStart)
                    return b.IsRequestedStart.CompareTo(a.IsRequestedStart);

                if (a.IsPointOverPoly != b.IsPointOverPoly)
                    return b.IsPointOverPoly.CompareTo(a.IsPointOverPoly);

                var supportCmp = b.SupportProbeHits.CompareTo(a.SupportProbeHits);
                if (supportCmp != 0)
                    return supportCmp;

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
            if (poly == 0)
                return;

            if (candidateIndices.TryGetValue(poly, out var existingIndex))
            {
                if (forceInclude && !candidates[existingIndex].IsRequestedStart)
                    candidates[existingIndex] = candidates[existingIndex] with { IsRequestedStart = true };
                return;
            }

            if (!TryClosestPointOnPolyWithFlags(from, poly, out var point, out var isPointOverPoly))
                return;

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

            var supportHits = CountStartSupportProbeHits(from, poly);
            var candidate   = new MeshPolyCandidate(poly, point, horizontalDistanceSq, verticalDistance, forceInclude, isPointOverPoly, supportHits);
            candidateIndices.Add(poly, candidates.Count);
            candidates.Add(candidate);
        }
    }

    private int CountStartSupportProbeHits(Vector3 from, long poly)
    {
        var matchDistanceSq = StartSupportMatchDistance * StartSupportMatchDistance;
        var step            = MathF.Tau / StartSupportProbeCount;
        var hits            = 0;

        for (var i = 0; i < StartSupportProbeCount; i++)
        {
            var angle = step * i;
            var probe = new Vector3(from.X + MathF.Cos(angle) * StartSupportProbeRadius, from.Y, from.Z + MathF.Sin(angle) * StartSupportProbeRadius);
            if (!TryClosestPointOnPolyWithFlags(probe, poly, out var probeClosest, out var probeOverPoly))
                continue;

            if (probeOverPoly)
            {
                hits++;
                continue;
            }

            var dx = probeClosest.X - probe.X;
            var dz = probeClosest.Z - probe.Z;
            if (dx * dx + dz * dz <= matchDistanceSq)
                hits++;
        }

        return hits;
    }

    private readonly record struct StartCandidateEvaluation
    (
        MeshPathCandidate? PathCandidate,
        string             Log,
        bool               RequestedFailed
    );

    private MeshPathCandidate BuildMeshPathCandidate
        (MeshPolyCandidate startCandidate, List<long> corridor, DtStatus status, RcVec3f requestedEndPos, Vector3 requestedTarget, long endRef, float range)
        => BuildMeshPathCandidate(MeshQuery, startCandidate, corridor, status, requestedEndPos, requestedTarget, endRef, range);

    private static MeshPathCandidate BuildMeshPathCandidate
        (DtNavMeshQuery query, MeshPolyCandidate startCandidate, List<long> corridor, DtStatus status, RcVec3f requestedEndPos, Vector3 requestedTarget, long endRef, float range)
    {
        var     lastPoly     = corridor[^1];
        var     resultStatus = PathfindStatus.Complete;
        Vector3 finalDestination;

        if (status.IsPartial() || lastPoly != endRef)
        {
            var closestStatus = query.ClosestPointOnPoly(lastPoly, requestedEndPos, out var projectedEndPos, out _);
            if (closestStatus.Failed())
                return new(startCandidate, status, PathfindStatus.Failed, startCandidate.ProjectedPoint, corridor);

            finalDestination = projectedEndPos.RecastToSystem();
            resultStatus = range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }
        else finalDestination = requestedEndPos.RecastToSystem();

        return new(startCandidate, status, resultStatus, finalDestination, corridor);
    }

    private MeshEndCandidate? ResolveBestEndPathCandidate(Vector3 requestedTarget, long requestedEndRef)
    {
        var candidates = CollectEndPolyCandidates(requestedTarget, requestedEndRef);
        if (candidates.Count == 0)
            return null;

        MeshEndCandidate? best          = null;
        List<string>      candidateLogs = [];

        foreach (var candidate in candidates)
        {
            candidateLogs.Add
            (
                $"{candidate.PolyRef:X}: requested = {(candidate.IsRequestedEnd ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}"
            );

            if (best == null || IsBetterEndCandidate(candidate, best.Value))
                best = candidate;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] 终点候选评估：{string.Join(" | ", candidateLogs)}");

        return best;
    }

    private List<MeshEndCandidate> CollectEndPolyCandidates(Vector3 requestedTarget, long requestedEndRef)
    {
        Dictionary<long, int> candidateIndices = [];
        List<MeshEndCandidate> candidates = [];

        foreach (var poly in FindIntersectingMeshPolys(requestedTarget, new(EndPolyCandidateHalfExtentXZ, EndPolyCandidateHalfExtentY, EndPolyCandidateHalfExtentXZ)))
            TryAddCandidate(poly, false);

        TryAddCandidate(requestedEndRef, true);

        candidates.Sort
        (
            static (a, b) =>
            {
                if (a.IsPointOverPoly != b.IsPointOverPoly)
                    return b.IsPointOverPoly.CompareTo(a.IsPointOverPoly);

                if (a.IsAboveTarget != b.IsAboveTarget)
                    return a.IsAboveTarget.CompareTo(b.IsAboveTarget);

                var cmp = a.VerticalDistanceAbs.CompareTo(b.VerticalDistanceAbs);
                if (cmp != 0)
                    return cmp;

                cmp = a.HorizontalDistanceSq.CompareTo(b.HorizontalDistanceSq);
                if (cmp != 0)
                    return cmp;

                if (a.IsRequestedEnd != b.IsRequestedEnd)
                    return b.IsRequestedEnd.CompareTo(a.IsRequestedEnd);

                return a.PolyRef.CompareTo(b.PolyRef);
            }
        );

        return candidates;

        void TryAddCandidate(long poly, bool forceInclude)
        {
            if (poly == 0)
                return;

            if (candidateIndices.TryGetValue(poly, out var existingIndex))
            {
                if (forceInclude && !candidates[existingIndex].IsRequestedEnd)
                    candidates[existingIndex] = candidates[existingIndex] with { IsRequestedEnd = true };
                return;
            }

            if (!TryClosestPointOnPolyWithFlags(requestedTarget, poly, out var point, out var isPointOverPoly))
                return;

            var dx                   = point.X - requestedTarget.X;
            var dz                   = point.Z - requestedTarget.Z;
            var horizontalDistanceSq = dx * dx + dz * dz;
            var verticalDistance     = point.Y - requestedTarget.Y;

            if (!forceInclude)
            {
                if (horizontalDistanceSq > EndPolyCandidateMaxHorizontalDistance * EndPolyCandidateMaxHorizontalDistance)
                    return;
                if (MathF.Abs(verticalDistance) > EndPolyCandidateMaxVerticalDistance)
                    return;
            }

            candidateIndices.Add(poly, candidates.Count);
            candidates.Add(new(poly, point, horizontalDistanceSq, verticalDistance, forceInclude, isPointOverPoly));
        }
    }

    private static bool IsBetterEndCandidate(MeshEndCandidate candidate, MeshEndCandidate currentBest)
    {
        if (candidate.IsPointOverPoly != currentBest.IsPointOverPoly)
            return candidate.IsPointOverPoly;

        if (candidate.IsAboveTarget != currentBest.IsAboveTarget)
            return !candidate.IsAboveTarget;

        if (!NearlyEqual(candidate.VerticalDistanceAbs, currentBest.VerticalDistanceAbs))
            return candidate.VerticalDistanceAbs < currentBest.VerticalDistanceAbs;

        if (!NearlyEqual(candidate.HorizontalDistanceSq, currentBest.HorizontalDistanceSq))
            return candidate.HorizontalDistanceSq < currentBest.HorizontalDistanceSq;

        if (candidate.IsRequestedEnd != currentBest.IsRequestedEnd)
            return candidate.IsRequestedEnd;

        return candidate.PolyRef < currentBest.PolyRef;
    }

    private static bool IsBetterStartPathCandidate(MeshPathCandidate candidate, MeshPathCandidate currentBest, Vector3 requestedTarget)
    {
        var candidateRequestedOver = candidate.IsRequestedStart && candidate.IsPointOverPoly;
        var currentRequestedOver   = currentBest.IsRequestedStart && currentBest.IsPointOverPoly;
        if (candidateRequestedOver != currentRequestedOver)
            return candidateRequestedOver;

        if (candidate.IsPointOverPoly != currentBest.IsPointOverPoly)
            return candidate.IsPointOverPoly;

        if (candidate.SupportProbeHits != currentBest.SupportProbeHits)
            return candidate.SupportProbeHits > currentBest.SupportProbeHits;

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

    private static PlannerResult BuildGroundPlannerResult
    (
        Vector3                           requestedTarget,
        float                             range,
        PathfindStatus                    status,
        Vector3                           finalDestination,
        IReadOnlyList<PlannerPathSegment> segments
    ) =>
        new()
        {
            Status               = status,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = requestedTarget,
            FinalDestination     = finalDestination,
            DestinationTolerance = range,
            Segments             = segments
        };

    private static PlannerPathSegment BuildGroundMeshCorridorSegment(MeshPathCandidate candidate) =>
        new()
        {
            MovementMode         = MovementMode.Ground,
            SegmentKind          = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl = false,
            ReachabilitySource   = PathReachabilitySource.Mesh,
            GeometryKind         = PlannerSegmentGeometryKind.MeshCorridor,
            StartPosition        = candidate.StartPoint,
            EndPosition          = candidate.FinalDestination,
            Corridor             = [.. candidate.Corridor]
        };

    private static PlannerPathSegment BuildGroundDiscreteSegment(Vector3 startPosition, params Vector3[] points) =>
        new()
        {
            MovementMode         = MovementMode.Ground,
            SegmentKind          = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl = false,
            ReachabilitySource   = PathReachabilitySource.Mesh,
            GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
            StartPosition        = startPosition,
            EndPosition          = points.Length > 0 ? points[^1] : startPosition,
            Points               = [.. points]
        };
}
