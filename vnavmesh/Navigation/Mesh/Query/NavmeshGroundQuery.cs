using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Utilities;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Navigation.Mesh.Query;

using static DtDetour;

internal sealed class NavmeshGroundQuery
{
    private readonly NavmeshQuery query;

    private long groundQueryCount;
    private long partialGroundQueryCount;
    private long suspectedTileSeamCutoffCount;
    private long anyAnglePreferredCount;
    private long classicFallbackCount;
    private long startReplacementCount;
    private long endReplacementCount;

    internal NavmeshGroundQuery(NavmeshQuery query) =>
        this.query = query;

    internal GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        new()
        {
            GroundQueries               = Interlocked.Read(ref groundQueryCount),
            PartialQueries              = Interlocked.Read(ref partialGroundQueryCount),
            SuspectedTileSeamCutoffs    = Interlocked.Read(ref suspectedTileSeamCutoffCount),
            AnyAnglePreferred           = Interlocked.Read(ref anyAnglePreferredCount),
            ClassicFallbacks            = Interlocked.Read(ref classicFallbackCount),
            StartReplacements           = Interlocked.Read(ref startReplacementCount),
            EndReplacements             = Interlocked.Read(ref endReplacementCount),
            GeneratedClimbLinksAccepted = query.NavmeshData.GeneratedClimbDownLinkCount,
            GeneratedJumpLinksAccepted  = query.NavmeshData.GeneratedEdgeJumpLinkCount
        };

    internal PlannerResult PlanMeshPathDetailed(Vector3 from, Vector3 to, bool useRaycast, float range, CancellationToken cancel)
    {
        Interlocked.Increment(ref groundQueryCount);
        var                    requestedStartRef   = query.FindNearestMeshPoly(from);
        var                    requestedEndRef     = query.FindNearestMeshPoly(to);
        Dictionary<long, int>  endCandidateIndices = [];
        List<MeshEndCandidate> endCandidates       = [];

        foreach (var poly in query.FindIntersectingMeshPolys
                     (to, new(END_POLY_CANDIDATE_HALF_EXTENT_XZ, END_POLY_CANDIDATE_HALF_EXTENT_Y, END_POLY_CANDIDATE_HALF_EXTENT_XZ)))
            TryAddEndCandidate(poly, false);

        TryAddEndCandidate(requestedEndRef, true);
        endCandidates.Sort
        (static (a, b) =>
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

        MeshEndCandidate? endCandidate     = null;
        List<string>      endCandidateLogs = [];

        foreach (var candidate in endCandidates)
        {
            endCandidateLogs.Add
            (
                $"{candidate.PolyRef:X}: requested = {(candidate.IsRequestedEnd ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}"
            );

            if (endCandidate == null)
            {
                endCandidate = candidate;
                continue;
            }

            var currentBest = endCandidate.Value;
            var isBetterCandidate =
                candidate.IsPointOverPoly != currentBest.IsPointOverPoly                       ? candidate.IsPointOverPoly :
                candidate.IsAboveTarget   != currentBest.IsAboveTarget                         ? !candidate.IsAboveTarget :
                !NearlyEqual(candidate.VerticalDistanceAbs,  currentBest.VerticalDistanceAbs)  ? candidate.VerticalDistanceAbs < currentBest.VerticalDistanceAbs :
                !NearlyEqual(candidate.HorizontalDistanceSq, currentBest.HorizontalDistanceSq) ? candidate.HorizontalDistanceSq < currentBest.HorizontalDistanceSq :
                candidate.IsRequestedEnd != currentBest.IsRequestedEnd ? candidate.IsRequestedEnd :
                                                                         candidate.PolyRef < currentBest.PolyRef;

            if (isBetterCandidate)
                endCandidate = candidate;
        }

        if (endCandidateLogs.Count > 0)
            Service.Log.Debug($"[算路] 终点候选评估：{string.Join(" | ", endCandidateLogs)}");

        if (endCandidate is { } selectedEndCandidate && selectedEndCandidate.PolyRef != requestedEndRef)
            Interlocked.Increment(ref endReplacementCount);

        if (endCandidate == null)
            return LogMeshFailure(from, to, requestedStartRef, requestedEndRef, 0, range, "无法为终点选择可用的导航多边形");

        var resolvedDestination = endCandidate.Value.ProjectedPoint;
        var endRef              = endCandidate.Value.PolyRef;

        var timer           = StopWatchTimer.Create();
        var requestedEndPos = resolvedDestination.SystemToRecast();
        var prunedAttempt   = ExecuteGroundPathAttempt(from, to, requestedStartRef, endRef, requestedEndPos, query.GroundAreaFilter, useRaycast, range, cancel);
        var selectedAttempt = prunedAttempt;

        if (prunedAttempt == null || prunedAttempt.Value.Result.Status == PathfindStatus.Partial)
        {
            var unprunedAttempt = ExecuteGroundPathAttempt
            (
                from,
                to,
                requestedStartRef,
                endRef,
                requestedEndPos,
                query.GroundAreaFilterIgnoringUnreachable,
                useRaycast,
                range,
                cancel
            );

            var shouldUseFallback = false;

            if (unprunedAttempt is { } fallbackAttempt)
            {
                shouldUseFallback = selectedAttempt == null;

                if (!shouldUseFallback && selectedAttempt is { } currentAttempt)
                {
                    var fallbackRank = GetResultStatusRank(fallbackAttempt.Result.Status);
                    var currentRank  = GetResultStatusRank(currentAttempt.Result.Status);
                    shouldUseFallback = fallbackRank       != currentRank
                                            ? fallbackRank < currentRank
                                            : IsBetterStartPathCandidate(fallbackAttempt.Candidate, currentAttempt.Candidate, to);
                }
            }

            if (unprunedAttempt is { } fallbackAttemptToApply && shouldUseFallback)
            {
                var initialAttempt = selectedAttempt is { } currentAttempt
                                         ? $"{currentAttempt.Candidate.StartRef:X}/{currentAttempt.Candidate.ResultStatus}/requested={(currentAttempt.Candidate.IsRequestedStart ? 1 : 0)}/overPoly={(currentAttempt.Candidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(currentAttempt.Candidate.DistanceToRequestedTargetSq(to)):f3}/结果={currentAttempt.Result.Status}/最后={currentAttempt.LastPoly:X}"
                                         : "<none>";
                var fallbackAttemptText =
                    $"{fallbackAttemptToApply.Candidate.StartRef:X}/{fallbackAttemptToApply.Candidate.ResultStatus}/requested={(fallbackAttemptToApply.Candidate.IsRequestedStart ? 1 : 0)}/overPoly={(fallbackAttemptToApply.Candidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(fallbackAttemptToApply.Candidate.DistanceToRequestedTargetSq(to)):f3}/结果={fallbackAttemptToApply.Result.Status}/最后={fallbackAttemptToApply.LastPoly:X}";
                Service.Log.Warning($"[算路] 已改用忽略裁剪标记的回退结果：首轮 = {initialAttempt}，回退 = {fallbackAttemptText}");
                selectedAttempt = fallbackAttemptToApply;
            }
        }

        if (selectedAttempt is not { } attempt)
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");

        query.LastPath.Clear();
        var pathCandidate = attempt.Candidate;
        var startRef      = attempt.StartRef;
        if (pathCandidate.QueryMode == GroundQueryMode.AnyAngle)
            Interlocked.Increment(ref anyAnglePreferredCount);
        query.LastPath.AddRange(pathCandidate.Corridor);
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}（原始起点候选 = {requestedStartRef:X}，查询模式 = {pathCandidate.QueryMode}）");
        Service.Log.Debug($"[算路] 地面终点解析：原始终点候选 = {requestedEndRef:X}，选中 = {endRef:X}，终点投影 = {resolvedDestination:f3}");
        Service.Log.Debug
            ($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {attempt.QueryStatus}，路径 = {string.Join(", ", query.LastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();
        LogMeshResult(attempt.Result, from, startRef, endRef, attempt.LastPoly, range, timer.Value());
        return attempt.Result;

        void TryAddEndCandidate(long poly, bool forceInclude)
        {
            if (poly == 0)
                return;

            if (endCandidateIndices.TryGetValue(poly, out var existingIndex))
            {
                if (forceInclude && !endCandidates[existingIndex].IsRequestedEnd)
                    endCandidates[existingIndex] = endCandidates[existingIndex] with { IsRequestedEnd = true };
                return;
            }

            if (!TryClosestPointOnPolyWithFlags(to, poly, out var point, out var isPointOverPoly))
                return;

            var dx                   = point.X - to.X;
            var dz                   = point.Z - to.Z;
            var horizontalDistanceSq = dx * dx + dz * dz;
            var verticalDistance     = point.Y - to.Y;

            if (!forceInclude)
            {
                if (horizontalDistanceSq > END_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE * END_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE)
                    return;
                if (MathF.Abs(verticalDistance) > END_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE)
                    return;
            }

            endCandidateIndices.Add(poly, endCandidates.Count);
            endCandidates.Add(new(poly, point, horizontalDistanceSq, verticalDistance, forceInclude, isPointOverPoly));
        }
    }

    private bool TryClosestPointOnPolyWithFlags(Vector3 point, long poly, out Vector3 closestPoint, out bool isOverPoly)
    {
        if (query.MeshQuery.ClosestPointOnPoly(poly, point.SystemToRecast(), out var closest, out isOverPoly).Succeeded())
        {
            closestPoint = closest.RecastToSystem();
            return true;
        }

        closestPoint = default;
        isOverPoly   = false;
        return false;
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
        GroundQueryMode   queryMode,
        float             range,
        CancellationToken cancel
    )
    {
        var selection = EvaluateBestStartPathCandidate
        (
            from,
            to,
            requestedStartRef,
            endRef,
            requestedEndPos,
            filter,
            opt,
            queryMode,
            range,
            cancel,
            false,
            true
        );

        if (selection.Selected == null || selection.Selected.Value.ResultStatus == PathfindStatus.Partial)
        {
            var expandedSelection = EvaluateBestStartPathCandidate
            (
                from,
                to,
                requestedStartRef,
                endRef,
                requestedEndPos,
                filter,
                opt,
                queryMode,
                range,
                cancel,
                true,
                false
            );

            if (expandedSelection.Selected is { } expandedCandidate &&
                (selection.Selected == null || IsBetterStartPathCandidate(expandedCandidate, selection.Selected.Value, to)))
            {
                var firstAttempt = selection.Selected is { } currentSelection
                                       ? $"{currentSelection.StartRef:X}/{currentSelection.ResultStatus}/requested={(currentSelection.IsRequestedStart ? 1 : 0)}/overPoly={(currentSelection.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(currentSelection.DistanceToRequestedTargetSq(to)):f3}"
                                       : "<none>";
                var expandedAttempt =
                    $"{expandedCandidate.StartRef:X}/{expandedCandidate.ResultStatus}/requested={(expandedCandidate.IsRequestedStart ? 1 : 0)}/overPoly={(expandedCandidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(expandedCandidate.DistanceToRequestedTargetSq(to)):f3}";
                Service.Log.Warning
                (
                    $"[算路] 起点扩搜改判：查询模式 = {queryMode}，首轮 = {firstAttempt}，扩搜 = {expandedAttempt}，扩搜候选 = {expandedSelection.EvaluatedCandidates}/{expandedSelection.TotalCandidates}"
                );
                selection = expandedSelection;
            }
        }

        if (selection.Selected is { } selected)
            LogStartCandidateDecision(selected, to, requestedStartRef, selection.RequestedSuccessful, selection.RequestedFailed, selection.LockedByRequested);

        return selection.Selected;
    }

    private StartCandidateSelection EvaluateBestStartPathCandidate
    (
        Vector3           from,
        Vector3           requestedTarget,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        GroundQueryMode   queryMode,
        float             range,
        CancellationToken cancel,
        bool              expandedSearch,
        bool              allowRequestedLock
    )
    {
        var candidates = CollectStartPolyCandidates(from, requestedStartRef, expandedSearch);
        if (candidates.Count == 0)
            return new(null, null, false, false, 0, 0);

        var evaluationLimit = expandedSearch ? EXPANDED_MAX_START_POLY_CANDIDATES_TO_EVALUATE : MAX_START_POLY_CANDIDATES_TO_EVALUATE;
        var candidateBatch  = candidates.Take(evaluationLimit).ToArray();

        MeshPathCandidate? best                = null;
        MeshPathCandidate? requestedSuccessful = null;
        MeshPathCandidate? requestedLocked     = null;
        List<string>       candidateLogs       = [];
        var                requestedFailed     = false;

        var evaluations = EvaluateStartPathCandidates(candidateBatch, requestedTarget, endRef, requestedEndPos, filter, opt, queryMode, range, cancel);

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

            if (allowRequestedLock &&
                pathCandidate is { IsRequestedStart: true, IsPointOverPoly: true, ResultStatus: PathfindStatus.Complete or PathfindStatus.ReachedWithinRange })
                requestedLocked = pathCandidate;

            if (best == null || IsBetterStartPathCandidate(pathCandidate, best.Value, requestedTarget))
                best = pathCandidate;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] {(expandedSearch ? "起点扩搜候选评估" : "起点候选评估")}：{string.Join(" | ", candidateLogs)}");

        var selected = allowRequestedLock && requestedLocked != null ? requestedLocked : best;
        return new(selected, requestedSuccessful, requestedFailed, allowRequestedLock && requestedLocked != null, candidates.Count, candidateBatch.Length);
    }

    private StartCandidateEvaluation[] EvaluateStartPathCandidates
    (
        IReadOnlyList<MeshPolyCandidate> candidates,
        Vector3                          requestedTarget,
        long                             endRef,
        RcVec3f                          requestedEndPos,
        IDtQueryFilter                   filter,
        DtFindPathOption                 opt,
        GroundQueryMode                  queryMode,
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
                i => result[i] = EvaluateStartPathCandidate(candidates[i], requestedTarget, endRef, requestedEndPos, filter, opt, queryMode, range)
            );
        }
        else
        {
            for (var i = 0; i < candidates.Count; ++i)
            {
                cancel.ThrowIfCancellationRequested();
                result[i] = EvaluateStartPathCandidate(candidates[i], requestedTarget, endRef, requestedEndPos, filter, opt, queryMode, range);
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
        GroundQueryMode   queryMode,
        float             range
    )
    {
        var candidateQuery = new DtNavMeshQuery(query.MeshQuery.GetAttachedNavMesh());
        var status = FindPath
        (
            candidateQuery,
            candidate.PolyRef,
            endRef,
            candidate.ProjectedPoint.SystemToRecast(),
            requestedEndPos,
            filter,
            opt,
            out var corridor
        );

        if (status.Failed() || corridor.Count == 0)
        {
            return new
            (
                null,
                $"{candidate.PolyRef:X}: 失败 ({status})，requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}",
                candidate.IsRequestedStart
            );
        }

        var pathCandidate = BuildMeshPathCandidate
        (
            candidateQuery,
            candidate,
            corridor,
            status,
            requestedEndPos,
            requestedTarget,
            endRef,
            queryMode,
            range
        );

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
            $"{candidate.PolyRef:X}: {pathCandidate.ResultStatus}，requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}，路径长 {pathCandidate.PathLength:f3}，距目标 {MathF.Sqrt(pathCandidate.DistanceToRequestedTargetSq(requestedTarget)):f3}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}",
            false
        );
    }

    private List<MeshPolyCandidate> CollectStartPolyCandidates(Vector3 from, long requestedStartRef, bool expandedSearch)
    {
        Dictionary<long, int> candidateIndices = [];
        List<MeshPolyCandidate> candidates = [];
        var halfExtentXZ = expandedSearch ? EXPANDED_START_POLY_CANDIDATE_HALF_EXTENT_XZ : START_POLY_CANDIDATE_HALF_EXTENT_XZ;
        var halfExtentY = expandedSearch ? EXPANDED_START_POLY_CANDIDATE_HALF_EXTENT_Y : START_POLY_CANDIDATE_HALF_EXTENT_Y;
        var maxHorizontalDistance = expandedSearch ? EXPANDED_START_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE : START_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE;
        var maxVerticalDistance = expandedSearch ? EXPANDED_START_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE : START_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE;

        foreach (var poly in query.FindIntersectingMeshPolys(from, new(halfExtentXZ, halfExtentY, halfExtentXZ)))
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
                if (horizontalDistanceSq > maxHorizontalDistance * maxHorizontalDistance)
                    return;
                if (MathF.Abs(verticalDistance) > maxVerticalDistance)
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
        var matchDistanceSq = START_SUPPORT_MATCH_DISTANCE * START_SUPPORT_MATCH_DISTANCE;
        var step            = MathF.Tau                    / START_SUPPORT_PROBE_COUNT;
        var hits            = 0;

        for (var i = 0; i < START_SUPPORT_PROBE_COUNT; i++)
        {
            var angle = step * i;
            var probe = new Vector3(from.X + MathF.Cos(angle) * START_SUPPORT_PROBE_RADIUS, from.Y, from.Z + MathF.Sin(angle) * START_SUPPORT_PROBE_RADIUS);
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

    private void LogStartCandidateDecision
    (
        MeshPathCandidate  selected,
        Vector3            requestedTarget,
        long               requestedStartRef,
        MeshPathCandidate? requestedSuccessful,
        bool               requestedFailed,
        bool               lockedByRequested
    )
    {
        Service.Log.Debug
        (
            $"[算路] 已选起点多边形 {selected.StartRef:X}，投影点 = {selected.StartPoint:f3}，结果 = {selected.ResultStatus}，requested = {(selected.IsRequestedStart ? 1 : 0)}，overPoly = {(selected.IsPointOverPoly ? 1 : 0)}，supportHits = {selected.SupportProbeHits}"
        );

        if (selected.StartRef == requestedStartRef)
            return;

        Interlocked.Increment(ref startReplacementCount);

        var reason = "综合排序更优";

        if (lockedByRequested)
            reason = "锁定请求起点";
        else if (requestedFailed)
            reason = "请求起点不可达";
        else if (requestedSuccessful is { } requested)
        {
            if (selected.ResultStatus != requested.ResultStatus)
                reason = $"状态更优: {requested.ResultStatus} -> {selected.ResultStatus}";
            else if (selected.IsPointOverPoly != requested.IsPointOverPoly)
                reason = selected.IsPointOverPoly ? "命中更贴脚的多边形" : "保持非贴脚候选";
            else if (!NearlyEqual(selected.StartCandidate.VerticalDistanceAbs, requested.StartCandidate.VerticalDistanceAbs))
                reason = $"垂直偏差更小: {requested.StartCandidate.VerticalDistanceAbs:f3} -> {selected.StartCandidate.VerticalDistanceAbs:f3}";
            else if (!NearlyEqual(selected.DistanceToRequestedTargetSq(requestedTarget), requested.DistanceToRequestedTargetSq(requestedTarget)))
            {
                reason =
                    $"距目标更近: {MathF.Sqrt(requested.DistanceToRequestedTargetSq(requestedTarget)):f3} -> {MathF.Sqrt(selected.DistanceToRequestedTargetSq(requestedTarget)):f3}";
            }
            else if (!NearlyEqual(selected.PathLength, requested.PathLength))
                reason = $"路径更短: {requested.PathLength:f3} -> {selected.PathLength:f3}";
            else if (selected.WeightedLinkPenalty != requested.WeightedLinkPenalty)
                reason = $"跳链代价更低: {requested.WeightedLinkPenalty} -> {selected.WeightedLinkPenalty}";
            else if (!NearlyEqual(selected.StartCandidate.HorizontalDistanceSq, requested.StartCandidate.HorizontalDistanceSq))
                reason = $"水平偏移更小: {MathF.Sqrt(requested.StartCandidate.HorizontalDistanceSq):f3} -> {MathF.Sqrt(selected.StartCandidate.HorizontalDistanceSq):f3}";
            else if (selected.SupportProbeHits > requested.SupportProbeHits)
                reason = $"支撑探针命中更多: {requested.SupportProbeHits} -> {selected.SupportProbeHits}";
            else if (selected.QueryMode != requested.QueryMode)
                reason = $"查询模式更优: {requested.QueryMode} -> {selected.QueryMode}";
        }

        Service.Log.Warning
        (
            $"[算路] 起点候选改判：原始 = {requestedStartRef:X}，选中 = {selected.StartRef:X}，原因 = {reason}，最终距目标 = {MathF.Sqrt(selected.DistanceToRequestedTargetSq(requestedTarget)):f3}"
        );
    }

    private bool TryRepairGroundGap
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        CancellationToken cancel,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        if (TryRepairGroundGapByRaycast(partialCandidate, requestedTarget, requestedEndPos, filter, range, out repairedResult, out repairedLastPoly))
            return true;

        if (TryRepairGroundGapByMoveAlongSurface
                (partialCandidate, requestedTarget, endRef, requestedEndPos, filter, opt, range, out repairedResult, out repairedLastPoly))
            return true;

        return TryRepairGroundGapByNearbyContinuation
            (partialCandidate, requestedTarget, endRef, requestedEndPos, filter, opt, range, cancel, out repairedResult, out repairedLastPoly);
    }

    private bool TryRepairGroundGapByRaycast
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        float             range,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        Span<long> visited = stackalloc long[MAX_PATH_POLYS];
        var status = query.MeshQuery.Raycast
        (
            partialCandidate.LastPoly,
            partialCandidate.FinalDestination.SystemToRecast(),
            requestedEndPos,
            filter,
            out var hitT,
            out _,
            visited,
            out var visitedCount,
            visited.Length
        );
        if (status.Failed() || hitT < 0.99f)
            return false;

        repairedLastPoly = visitedCount > 0 ? visited[visitedCount - 1] : partialCandidate.LastPoly;
        repairedResult = BuildGroundPlannerResult
        (
            requestedTarget,
            range,
            range > 0 && Vector3.Distance(partialCandidate.FinalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Complete,
            requestedTarget,
            [BuildGroundMeshCorridorSegment(partialCandidate), BuildGroundDiscreteSegment(partialCandidate.FinalDestination, requestedTarget)]
        );
        Service.Log.Warning($"[算路] Partial 修复命中 Raycast：起点 = {partialCandidate.FinalDestination:f3}，终点 = {requestedTarget:f3}，最后多边形 = {repairedLastPoly:X}");
        return true;
    }

    private bool TryRepairGroundGapByMoveAlongSurface
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        Span<long> visited = stackalloc long[32];
        var status = query.MeshQuery.MoveAlongSurface
        (
            partialCandidate.LastPoly,
            partialCandidate.FinalDestination.SystemToRecast(),
            requestedEndPos,
            filter,
            out var moved,
            visited,
            out var visitedCount,
            visited.Length
        );
        if (status.Failed() || visitedCount == 0)
            return false;

        var movedPoint   = moved.RecastToSystem();
        var movedDelta   = movedPoint - partialCandidate.FinalDestination;
        var movedPoly    = visited[visitedCount - 1];
        var movedDistSq  = movedDelta.X * movedDelta.X + movedDelta.Z * movedDelta.Z;
        var verticalDist = MathF.Abs(movedDelta.Y);
        if (movedDistSq <= 0.000001f || verticalDist > SHORT_GAP_REPAIR_MAX_VERTICAL_DELTA)
            return false;

        if (Vector3.Distance(movedPoint, requestedTarget) <= MathF.Max(range, 0.15f))
        {
            repairedLastPoly = movedPoly;
            repairedResult = BuildGroundPlannerResult
            (
                requestedTarget,
                range,
                range > 0 ? PathfindStatus.ReachedWithinRange : PathfindStatus.Complete,
                movedPoint,
                [BuildGroundMeshCorridorSegment(partialCandidate), BuildGroundDiscreteSegment(partialCandidate.FinalDestination, movedPoint)]
            );
            Service.Log.Warning($"[算路] Partial 修复命中 MoveAlongSurface：终点已贴近目标，桥接点 = {movedPoint:f3}，最后多边形 = {repairedLastPoly:X}");
            return true;
        }

        if (!TryClosestPointOnPolyWithFlags(movedPoint, movedPoly, out var bridgePoint, out var isPointOverPoly))
            return false;

        var resumeStatus = FindPath(query.MeshQuery, movedPoly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);
        if (resumeStatus.Failed() || corridor.Count == 0)
            return false;

        var resumeCandidate = BuildMeshPathCandidate
        (
            new
            (
                movedPoly,
                bridgePoint,
                Vector3.DistanceSquared(bridgePoint, partialCandidate.FinalDestination),
                bridgePoint.Y - partialCandidate.FinalDestination.Y,
                false,
                isPointOverPoly,
                CountStartSupportProbeHits(partialCandidate.FinalDestination, movedPoly)
            ),
            corridor,
            resumeStatus,
            requestedEndPos,
            requestedTarget,
            endRef,
            GroundQueryMode.Classic,
            range
        );
        if (resumeCandidate.ResultStatus == PathfindStatus.Failed)
            return false;

        repairedLastPoly = resumeCandidate.LastPoly;
        List<PlannerPathSegment> segments = [BuildGroundMeshCorridorSegment(partialCandidate)];
        if (Vector3.DistanceSquared(partialCandidate.FinalDestination, bridgePoint) > 0.000001f)
            segments.Add(BuildGroundDiscreteSegment(partialCandidate.FinalDestination, bridgePoint));
        segments.Add(BuildGroundMeshCorridorSegment(resumeCandidate));
        repairedResult = BuildGroundPlannerResult(requestedTarget, range, resumeCandidate.ResultStatus, resumeCandidate.FinalDestination, segments);
        Service.Log.Warning($"[算路] Partial 修复命中 MoveAlongSurface + 续算：桥接点 = {bridgePoint:f3}，结果 = {resumeCandidate.ResultStatus}，最后多边形 = {repairedLastPoly:X}");
        return true;
    }

    private bool TryRepairGroundGapByNearbyContinuation
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        CancellationToken cancel,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        var partialEnd = partialCandidate.FinalDestination;
        var repairCandidates = query.FindIntersectingMeshPolys
            (partialEnd, new(SHORT_GAP_REPAIR_SEARCH_HALF_EXTENT_XZ, SHORT_GAP_REPAIR_SEARCH_HALF_EXTENT_Y, SHORT_GAP_REPAIR_SEARCH_HALF_EXTENT_XZ));
        MeshPathCandidate? bestResumeCandidate = null;
        List<string>       candidateLogs       = [];

        foreach (var poly in repairCandidates)
        {
            if (poly == 0 || poly == partialCandidate.LastPoly || partialCandidate.Corridor.Contains(poly))
                continue;

            cancel.ThrowIfCancellationRequested();

            if (!TryClosestPointOnPolyWithFlags(partialEnd, poly, out var bridgePoint, out var isPointOverPoly))
                continue;

            var bridgeDelta              = bridgePoint - partialEnd;
            var bridgeHorizontalDistance = new Vector2(bridgeDelta.X, bridgeDelta.Z).Length();
            var bridgeVerticalDistance   = MathF.Abs(bridgeDelta.Y);
            if (bridgeHorizontalDistance > SHORT_GAP_REPAIR_MAX_BRIDGE_DISTANCE || bridgeVerticalDistance > SHORT_GAP_REPAIR_MAX_VERTICAL_DELTA)
                continue;

            var status = FindPath(query.MeshQuery, poly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);

            if (status.Failed() || corridor.Count == 0)
            {
                candidateLogs.Add($"{poly:X}: 失败 ({status})");
                continue;
            }

            var resumeCandidate = BuildMeshPathCandidate
            (
                new
                (
                    poly,
                    bridgePoint,
                    bridgeHorizontalDistance * bridgeHorizontalDistance,
                    bridgePoint.Y - partialEnd.Y,
                    false,
                    isPointOverPoly,
                    CountStartSupportProbeHits(partialEnd, poly)
                ),
                corridor,
                status,
                requestedEndPos,
                requestedTarget,
                endRef,
                GroundQueryMode.Classic,
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
        repairedLastPoly = bestResumeCandidate.Value.LastPoly;
        List<PlannerPathSegment> segments = [BuildGroundMeshCorridorSegment(partialCandidate)];
        if (Vector3.DistanceSquared(partialCandidate.FinalDestination, bridgePointToAdd) > 0.000001f)
            segments.Add(BuildGroundDiscreteSegment(partialCandidate.FinalDestination, bridgePointToAdd));
        segments.Add(BuildGroundMeshCorridorSegment(bestResumeCandidate.Value));

        Service.Log.Warning
        (
            $"[算路] 已触发短距补桥：partial 终点 = {partialCandidate.FinalDestination:f3}，桥接点 = {bridgePointToAdd:f3}，桥接后结果 = {bestResumeCandidate.Value.ResultStatus}，段数 = {segments.Count}，结果来源 = 短距补桥 + 续算，最后可达 = {repairedLastPoly:X}"
        );
        repairedResult = BuildGroundPlannerResult
        (
            requestedTarget,
            range,
            bestResumeCandidate.Value.ResultStatus,
            bestResumeCandidate.Value.FinalDestination,
            segments
        );
        return true;
    }

    private GroundPathAttempt? ExecuteGroundPathAttempt
    (
        Vector3           from,
        Vector3           to,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        bool              useRaycast,
        float             range,
        CancellationToken cancel
    )
    {
        var anyAngleOption = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0,
            useRaycast ? 5 : 0
        );
        var classicOption = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            0,
            0
        );
        var anyAngleCandidate = ResolveBestStartPathCandidate
        (
            from,
            to,
            requestedStartRef,
            endRef,
            requestedEndPos,
            filter,
            anyAngleOption,
            GroundQueryMode.AnyAngle,
            range,
            cancel
        );
        MeshPathCandidate? classicCandidate = null;

        var fallbackDistanceThreshold = MathF.Max(range, 0.35f);

        if (useRaycast                                                                                                      ||
            anyAngleCandidate                                       == null                                                 ||
            anyAngleCandidate.Value.ResultStatus                    != PathfindStatus.Complete                              ||
            anyAngleCandidate.Value.DistanceToRequestedTargetSq(to) > fallbackDistanceThreshold * fallbackDistanceThreshold ||
            anyAngleCandidate.Value.WeightedLinkPenalty             >= 4)
        {
            Interlocked.Increment(ref classicFallbackCount);
            classicCandidate = ResolveBestStartPathCandidate
            (
                from,
                to,
                requestedStartRef,
                endRef,
                requestedEndPos,
                filter,
                classicOption,
                GroundQueryMode.Classic,
                range,
                cancel
            );
        }

        MeshPathCandidate? bestStartCandidate;
        if (anyAngleCandidate == null)
            bestStartCandidate = classicCandidate;
        else if (classicCandidate == null)
            bestStartCandidate = anyAngleCandidate;
        else
            bestStartCandidate = IsBetterStartPathCandidate(anyAngleCandidate.Value, classicCandidate.Value, to) ? anyAngleCandidate : classicCandidate;
        if (bestStartCandidate is not { } pathCandidate)
            return null;

        var lastPoly     = pathCandidate.LastPoly;
        var resultStatus = pathCandidate.ResultStatus;
        var result       = BuildGroundPlannerResult(to, range, resultStatus, pathCandidate.FinalDestination, [BuildGroundMeshCorridorSegment(pathCandidate)]);

        if (resultStatus == PathfindStatus.Partial &&
            TryRepairGroundGap(pathCandidate, to, endRef, requestedEndPos, filter, classicOption, range, cancel, out var repairedResult, out var repairedLastPoly))
        {
            result   = repairedResult;
            lastPoly = repairedLastPoly;
        }

        return new(pathCandidate, result, pathCandidate.StartRef, lastPoly, pathCandidate.QueryStatus);
    }

    private MeshPathCandidate BuildMeshPathCandidate
    (
        MeshPolyCandidate startCandidate,
        List<long>        corridor,
        DtStatus          status,
        RcVec3f           requestedEndPos,
        Vector3           requestedTarget,
        long              endRef,
        GroundQueryMode   queryMode,
        float             range
    ) => BuildMeshPathCandidate(query.MeshQuery, startCandidate, corridor, status, requestedEndPos, requestedTarget, endRef, queryMode, range);

    private static MeshPathCandidate BuildMeshPathCandidate
    (
        DtNavMeshQuery    query,
        MeshPolyCandidate startCandidate,
        List<long>        corridor,
        DtStatus          status,
        RcVec3f           requestedEndPos,
        Vector3           requestedTarget,
        long              endRef,
        GroundQueryMode   queryMode,
        float             range
    )
    {
        var     lastPoly     = corridor[^1];
        var     resultStatus = PathfindStatus.Complete;
        Vector3 finalDestination;

        if (status.IsPartial() || lastPoly != endRef)
        {
            var closestStatus = query.ClosestPointOnPoly(lastPoly, requestedEndPos, out var projectedEndPos, out _);
            if (closestStatus.Failed())
                return new(startCandidate, status, PathfindStatus.Failed, startCandidate.ProjectedPoint, corridor, queryMode, float.MaxValue, int.MaxValue, 0, 0);

            finalDestination = projectedEndPos.RecastToSystem();
            resultStatus = range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }
        else finalDestination = requestedEndPos.RecastToSystem();

        var pathLength = EstimatePathLength(query, startCandidate.ProjectedPoint.SystemToRecast(), finalDestination.SystemToRecast(), corridor);
        CountPathSemantics(query.GetAttachedNavMesh(), corridor, out var weightedLinkPenalty, out var offMeshTransitionCount, out var areaCrossingCount);
        return new
        (
            startCandidate,
            status,
            resultStatus,
            finalDestination,
            corridor,
            queryMode,
            pathLength,
            weightedLinkPenalty,
            offMeshTransitionCount,
            areaCrossingCount
        );
    }

    private static float EstimatePathLength(DtNavMeshQuery query, RcVec3f startPos, RcVec3f endPos, List<long> corridor)
    {
        if (corridor.Count == 0)
            return float.MaxValue;

        var straightPath = new DtStraightPath[256];
        var status       = query.FindStraightPath(startPos, endPos, [.. corridor], corridor.Count, straightPath, out var count, straightPath.Length, 0);

        if (status.Succeeded() && count > 0)
        {
            var total    = 0f;
            var previous = startPos;

            for (var i = 0; i < count; i++)
            {
                total    += RcVec3f.Distance(previous, straightPath[i].pos);
                previous =  straightPath[i].pos;
            }

            return total;
        }

        var      navmesh        = query.GetAttachedNavMesh();
        var      fallback       = RcVec3f.Distance(startPos, endPos);
        RcVec3f? previousCenter = null;

        foreach (var polyRef in corridor)
        {
            var center = navmesh.GetPolyCenter(polyRef);
            if (previousCenter is { } prev)
                fallback += RcVec3f.Distance(prev, center);
            previousCenter = center;
        }

        return fallback;
    }

    private static void CountPathSemantics
    (
        DtNavMesh           navmesh,
        IReadOnlyList<long> corridor,
        out int             weightedLinkPenalty,
        out int             offMeshTransitionCount,
        out int             areaCrossingCount
    )
    {
        weightedLinkPenalty    = 0;
        offMeshTransitionCount = 0;
        areaCrossingCount      = 0;
        NavmeshArea? previousArea = null;

        foreach (var polyRef in corridor)
        {
            navmesh.GetTileAndPolyByRefUnsafe(polyRef, out _, out var poly);
            var area = (NavmeshArea)poly.GetArea();
            if (previousArea is { } lastArea && lastArea != area)
                areaCrossingCount++;

            switch (area)
            {
                case NavmeshArea.GeneratedClimbDown:
                    weightedLinkPenalty += 2;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.GeneratedEdgeJump:
                    weightedLinkPenalty += 4;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.ManualOffMesh:
                    weightedLinkPenalty += 1;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.Teleport:
                    weightedLinkPenalty += 1;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.ClientPath:
                    weightedLinkPenalty += 3;
                    offMeshTransitionCount++;
                    break;
            }

            previousArea = area;
        }
    }

    private static DtStatus FindPath
    (
        DtNavMeshQuery   query,
        long             startRef,
        long             endRef,
        RcVec3f          startPos,
        RcVec3f          endPos,
        IDtQueryFilter   filter,
        DtFindPathOption opt,
        out List<long>   corridor
    )
    {
        var buffer = new long[MAX_PATH_POLYS];
        corridor = [];

        if (opt.options != 0)
        {
            var status = query.InitSlicedFindPath(startRef, endRef, startPos, endPos, filter, opt.options);
            if (status.Failed())
                return status;

            do
            {
                status = query.UpdateSlicedFindPath(MAX_PATH_POLYS, out _);
            }
            while (status.InProgress());

            if (status.Failed())
                return status;

            status   = query.FinalizeSlicedFindPath(buffer, out var count, buffer.Length);
            corridor = [.. buffer.AsSpan(0, count).ToArray()];
            return status;
        }

        var directStatus = query.FindPath(startRef, endRef, startPos, endPos, filter, buffer, out var directCount, buffer.Length);
        corridor = [.. buffer.AsSpan(0, directCount).ToArray()];
        return directStatus;
    }

    private static bool IsBetterStartPathCandidate(MeshPathCandidate candidate, MeshPathCandidate currentBest, Vector3 requestedTarget)
    {
        var rankCandidate = GetResultStatusRank(candidate.ResultStatus);
        var rankCurrent   = GetResultStatusRank(currentBest.ResultStatus);
        if (rankCandidate != rankCurrent)
            return rankCandidate < rankCurrent;

        var candidateRequestedOver = candidate is { IsRequestedStart: true, IsPointOverPoly: true };
        var currentRequestedOver   = currentBest is { IsRequestedStart: true, IsPointOverPoly: true };
        if (candidateRequestedOver != currentRequestedOver)
            return candidateRequestedOver;

        if (candidate.IsPointOverPoly != currentBest.IsPointOverPoly)
            return candidate.IsPointOverPoly;

        if (!NearlyEqual(candidate.StartCandidate.VerticalDistanceAbs, currentBest.StartCandidate.VerticalDistanceAbs))
            return candidate.StartCandidate.VerticalDistanceAbs < currentBest.StartCandidate.VerticalDistanceAbs;

        var candidateDistance = candidate.DistanceToRequestedTargetSq(requestedTarget);
        var currentDistance   = currentBest.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(candidateDistance, currentDistance))
            return candidateDistance < currentDistance;

        if (!NearlyEqual(candidate.PathLength, currentBest.PathLength))
            return candidate.PathLength < currentBest.PathLength;

        if (candidate.WeightedLinkPenalty != currentBest.WeightedLinkPenalty)
            return candidate.WeightedLinkPenalty < currentBest.WeightedLinkPenalty;

        if (!NearlyEqual(candidate.StartCandidate.HorizontalDistanceSq, currentBest.StartCandidate.HorizontalDistanceSq))
            return candidate.StartCandidate.HorizontalDistanceSq < currentBest.StartCandidate.HorizontalDistanceSq;

        if (candidate.QueryMode != currentBest.QueryMode)
            return candidate.QueryMode == GroundQueryMode.AnyAngle;

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

    private static PlannerResult LogMeshFailure(Vector3 from, Vector3 to, long startRef, long endRef, long lastPoly, float range, string reason)
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
        var startTile     = new TileCoord(query.FindMeshTile(from).X,                        query.FindMeshTile(from).Z);
        var requestedTile = new TileCoord(query.FindMeshTile(result.RequestedDestination).X, query.FindMeshTile(result.RequestedDestination).Z);
        var actualTile    = new TileCoord(query.FindMeshTile(result.FinalDestination).X,     query.FindMeshTile(result.FinalDestination).Z);
        var (tileX, tileZ)     = query.FindMeshTile(result.FinalDestination);
        var (tileMin, tileMax) = query.GetMeshTileBounds(tileX, tileZ);
        var distanceToNearestBoundary = MathF.Min
        (
            MathF.Min(MathF.Abs(result.FinalDestination.X - tileMin.X), MathF.Abs(tileMax.X - result.FinalDestination.X)),
            MathF.Min(MathF.Abs(result.FinalDestination.Z - tileMin.Z), MathF.Abs(tileMax.Z - result.FinalDestination.Z))
        );
        var meshParams = query.MeshQuery.GetAttachedNavMesh().GetParams();
        var seamBoundaryTolerance = MathF.Max
                                        (MathF.Min(meshParams.tileWidth, meshParams.tileHeight), MathF.Max(query.ConfigData.PathTolerance, float.Epsilon)) *
                                    0.5f;
        var seamGapTolerance = MathF.Max(range, seamBoundaryTolerance);
        var diagnostic = new SeamDiagnostic
        (
            startTile,
            requestedTile,
            actualTile,
            distanceToNearestBoundary,
            distanceToNearestBoundary <= seamBoundaryTolerance,
            Math.Abs(requestedTile.X - actualTile.X) <= 1 && Math.Abs(requestedTile.Z - actualTile.Z) <= 1,
            Vector3.Distance(result.RequestedDestination, result.FinalDestination) <= seamGapTolerance
        );
        var message =
            $"地面算路完成：状态 = {result.Status}，起点 = {from:f3}，请求终点 = {result.RequestedDestination:f3}，实际终点 = {result.FinalDestination:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPoly:X}，容差 = {range:f3}，耗时 = {duration.TotalSeconds:f3} 秒，粗路径段 = {result.Segments.Count}";

        if (result.Status == PathfindStatus.Partial)
        {
            message +=
                $"，起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}";
        }

        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Interlocked.Increment(ref partialGroundQueryCount);
                Service.Log.Warning(message);

                if (diagnostic.IsSuspectedTileSeamCutoff)
                {
                    Interlocked.Increment(ref suspectedTileSeamCutoffCount);
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

    private static bool NearlyEqual(float left, float right) =>
        MathF.Abs(left - right) <= 0.0001f;

    private static int GetResultStatusRank(PathfindStatus status) => status switch
    {
        PathfindStatus.Complete           => 0,
        PathfindStatus.ReachedWithinRange => 1,
        PathfindStatus.Partial            => 2,
        _                                 => 3
    };

    private static float HorizontalDistanceXZ(Vector3 left, Vector3 right)
    {
        var dx = left.X           - right.X;
        var dz = left.Z           - right.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
    }

    private enum GroundQueryMode
    {
        AnyAngle,
        Classic
    }

    private readonly record struct GroundPathAttempt
    (
        MeshPathCandidate Candidate,
        PlannerResult     Result,
        long              StartRef,
        long              LastPoly,
        DtStatus          QueryStatus
    );

    private readonly record struct StartCandidateEvaluation
    (
        MeshPathCandidate? PathCandidate,
        string             Log,
        bool               RequestedFailed
    );

    private readonly record struct StartCandidateSelection
    (
        MeshPathCandidate? Selected,
        MeshPathCandidate? RequestedSuccessful,
        bool               RequestedFailed,
        bool               LockedByRequested,
        int                TotalCandidates,
        int                EvaluatedCandidates
    );

    private readonly record struct TileCoord
    (
        int X,
        int Z
    )
    {
        public override string ToString() => $"{X}x{Z}";
    }

    private readonly record struct SeamDiagnostic
    (
        TileCoord StartTile,
        TileCoord RequestedTile,
        TileCoord FinalTile,
        float     DistanceToNearestBoundary,
        bool      IsNearTileBoundary,
        bool      IsNearbyTile,
        bool      IsShortGap
    )
    {
        public bool IsSuspectedTileSeamCutoff => IsNearTileBoundary && IsNearbyTile && IsShortGap;
    }

    private readonly record struct MeshPolyCandidate
    (
        long    PolyRef,
        Vector3 ProjectedPoint,
        float   HorizontalDistanceSq,
        float   VerticalDelta,
        bool    IsRequestedStart,
        bool    IsPointOverPoly,
        int     SupportProbeHits
    )
    {
        public float VerticalDistanceAbs => MathF.Abs(VerticalDelta);
        public bool  IsTooFarAbove       => VerticalDelta > START_POLY_CANDIDATE_ABOVE_TOLERANCE;
    }

    private readonly record struct MeshPathCandidate
    (
        MeshPolyCandidate StartCandidate,
        DtStatus          QueryStatus,
        PathfindStatus    ResultStatus,
        Vector3           FinalDestination,
        List<long>        Corridor,
        GroundQueryMode   QueryMode,
        float             PathLength,
        int               WeightedLinkPenalty,
        int               OffMeshTransitionCount,
        int               AreaCrossingCount
    )
    {
        public long    StartRef         => StartCandidate.PolyRef;
        public Vector3 StartPoint       => StartCandidate.ProjectedPoint;
        public long    LastPoly         => Corridor.Count > 0 ? Corridor[^1] : 0;
        public bool    IsRequestedStart => StartCandidate.IsRequestedStart;
        public bool    IsPointOverPoly  => StartCandidate.IsPointOverPoly;
        public int     SupportProbeHits => StartCandidate.SupportProbeHits;

        public float DistanceToRequestedTargetSq(Vector3 requestedTarget) => Vector3.DistanceSquared(FinalDestination, requestedTarget);
    }

    internal sealed class GroundPathDiagnosticsSnapshot
    {
        public required long GroundQueries               { get; init; }
        public required long PartialQueries              { get; init; }
        public required long SuspectedTileSeamCutoffs    { get; init; }
        public required long AnyAnglePreferred           { get; init; }
        public required long ClassicFallbacks            { get; init; }
        public required long StartReplacements           { get; init; }
        public required long EndReplacements             { get; init; }
        public required long GeneratedClimbLinksAccepted { get; init; }
        public required long GeneratedJumpLinksAccepted  { get; init; }
    }

    private readonly record struct MeshEndCandidate
    (
        long    PolyRef,
        Vector3 ProjectedPoint,
        float   HorizontalDistanceSq,
        float   VerticalDelta,
        bool    IsRequestedEnd,
        bool    IsPointOverPoly
    )
    {
        public float VerticalDistanceAbs => MathF.Abs(VerticalDelta);
        public bool  IsAboveTarget       => VerticalDelta > END_POLY_CANDIDATE_ABOVE_TOLERANCE;
    }

    internal sealed class GoalRadiusHeuristic
    (
        float tolerance
    ) : IDtQueryHeuristic
    {
        float IDtQueryHeuristic.GetCost(RcVec3f neighbourPos, RcVec3f endPos)
        {
            var dist = RcVec3f.Distance(neighbourPos, endPos) * DtDefaultQueryHeuristic.H_SCALE;
            return dist < tolerance ? -1 : dist;
        }
    }

    internal sealed class GroundAreaCostFilter
    (
        bool excludeUnreachable = true
    ) : IDtQueryFilter
    {
        private readonly DtQueryDefaultFilter _filter = new
            ((int)NavmeshPolyFlags.AllTraversable, excludeUnreachable ? (int)NavmeshPolyFlags.Unreachable : 0, CreateAreaCosts());

        private static float[] CreateAreaCosts()
        {
            var costs = new float[DT_MAX_AREAS];
            Array.Fill(costs, 1f);
            costs[(int)NavmeshArea.Null]               = float.MaxValue;
            costs[(int)NavmeshArea.Ground]             = 1.0f;
            costs[(int)NavmeshArea.GeneratedClimbDown] = 1.8f;
            costs[(int)NavmeshArea.GeneratedEdgeJump]  = 2.6f;
            costs[(int)NavmeshArea.ManualOffMesh]      = 1.35f;
            costs[(int)NavmeshArea.Teleport]           = 1.15f;
            costs[(int)NavmeshArea.ClientPath]         = 3.0f;
            return costs;
        }

        public float GetCost
        (
            RcVec3f    pa,
            RcVec3f    pb,
            long       prevRef,
            DtMeshTile prevTile,
            DtPoly     prevPoly,
            long       curRef,
            DtMeshTile curTile,
            DtPoly     curPoly,
            long       nextRef,
            DtMeshTile nextTile,
            DtPoly     nextPoly
        )
            => _filter.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => _filter.PassFilter(refs, tile, poly);
    }

    #region 常量

    private const float START_POLY_CANDIDATE_HALF_EXTENT_XZ                   = 5.0f;
    private const float START_POLY_CANDIDATE_HALF_EXTENT_Y                    = 6.0f;
    private const float START_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE          = 4.0f;
    private const float START_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE            = 3.0f;
    private const float START_POLY_CANDIDATE_ABOVE_TOLERANCE                  = 0.75f;
    private const float START_SUPPORT_PROBE_RADIUS                            = 0.35f;
    private const int   START_SUPPORT_PROBE_COUNT                             = 8;
    private const float START_SUPPORT_MATCH_DISTANCE                          = 0.20f;
    private const int   MAX_START_POLY_CANDIDATES_TO_EVALUATE                 = 8;
    private const float EXPANDED_START_POLY_CANDIDATE_HALF_EXTENT_XZ          = 8.0f;
    private const float EXPANDED_START_POLY_CANDIDATE_HALF_EXTENT_Y           = 16.0f;
    private const float EXPANDED_START_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE = 8.0f;
    private const float EXPANDED_START_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE   = 12.0f;
    private const int   EXPANDED_MAX_START_POLY_CANDIDATES_TO_EVALUATE        = 24;
    private const float END_POLY_CANDIDATE_HALF_EXTENT_XZ                     = 5.0f;
    private const float END_POLY_CANDIDATE_HALF_EXTENT_Y                      = 8.0f;
    private const float END_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE            = 4.0f;
    private const float END_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE              = 8.0f;
    private const float END_POLY_CANDIDATE_ABOVE_TOLERANCE                    = 0.25f;
    private const float SHORT_GAP_REPAIR_SEARCH_HALF_EXTENT_XZ                = 4.0f;
    private const float SHORT_GAP_REPAIR_SEARCH_HALF_EXTENT_Y                 = 2.5f;
    private const float SHORT_GAP_REPAIR_MAX_BRIDGE_DISTANCE                  = 3.5f;
    private const float SHORT_GAP_REPAIR_MAX_VERTICAL_DELTA                   = 1.0f;
    private const int   MAX_PATH_POLYS                                        = 4096;

    #endregion
}
