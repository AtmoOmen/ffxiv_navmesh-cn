using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Utilities;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Navigation.Mesh.Query;

using static DtDetour;

internal sealed class NavmeshGroundQuery
(
    NavmeshQuery query
)
{
    private long groundQueryCount;
    private long partialGroundQueryCount;
    private long suspectedTileSeamCutoffCount;
    private long startReplacementCount;
    private long endReplacementCount;

    internal PlannerResult PlanMeshPathDetailed(Vector3 from, Vector3 to, float range, CancellationToken cancel)
    {
        Interlocked.Increment(ref groundQueryCount);
        var requestedStartRef = query.FindNearestMeshPoly(from);
        var requestedEndRef   = query.FindNearestMeshPoly(to);
        if (!TrySelectEndCandidate(to, requestedEndRef, out var endCandidate))
            return LogMeshFailure(from, to, requestedStartRef, requestedEndRef, 0, range, "无法为终点选择可用的导航多边形");

        var timer           = StopWatchTimer.Create();
        var requestedEndPos = endCandidate.ProjectedPoint.SystemToRecast();
        var endRef          = endCandidate.PolyRef;
        var pathCandidate   = ResolveBestStartPathCandidate(from, to, requestedStartRef, endRef, requestedEndPos, range, cancel);

        if (pathCandidate is not { } selected)
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");

        var startRef     = selected.StartRef;
        var lastPoly     = selected.LastPoly;
        var resultStatus = selected.ResultStatus;
        var result       = BuildGroundPlannerResult(to, range, resultStatus, selected.FinalDestination, [BuildGroundMeshCorridorSegment(selected, from)]);

        if (resultStatus == PathfindStatus.Partial &&
            TryRepairGroundGapByTileBoundaryBridge(selected, from, to, endRef, range, out var repairedResult, out var repairedLastPoly))
        {
            result   = repairedResult;
            lastPoly = repairedLastPoly;
        }

        query.LastPath.Clear();
        query.LastPath.AddRange(selected.Corridor);
        Service.Log.Debug($"[算路] 起点 = {from:f3}，终点 = {to:f3}，多边形 {startRef:X} -> {endRef:X}（原始起点 = {requestedStartRef:X}）");
        Service.Log.Debug($"[算路] 终点解析：原始 = {requestedEndRef:X}，选中 = {endRef:X}，投影 = {endCandidate.ProjectedPoint:f3}");
        Service.Log.Debug
            ($"[算路] 耗时 {timer.Value().TotalSeconds:f3} 秒，路径 = {string.Join(", ", query.LastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();
        LogMeshResult(result, from, startRef, endRef, lastPoly, range, timer.Value());
        return result;

        void LogMeshResult(PlannerResult result, Vector3 from, long startRef, long endRef, long lastPoly, float range, TimeSpan duration)
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
    }

    internal GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        new()
        {
            GroundQueries               = Interlocked.Read(ref groundQueryCount),
            PartialQueries              = Interlocked.Read(ref partialGroundQueryCount),
            SuspectedTileSeamCutoffs    = Interlocked.Read(ref suspectedTileSeamCutoffCount),
            StartReplacements           = Interlocked.Read(ref startReplacementCount),
            EndReplacements             = Interlocked.Read(ref endReplacementCount),
            GeneratedClimbLinksAccepted = query.NavmeshData.GeneratedClimbDownLinkCount,
            GeneratedJumpLinksAccepted  = query.NavmeshData.GeneratedEdgeJumpLinkCount
        };

    private bool TrySelectEndCandidate(Vector3 requestedTarget, long requestedEndRef, out MeshEndCandidate selectedCandidate)
    {
        var candidates = CollectEndCandidates(requestedTarget, requestedEndRef);

        if (candidates.Count == 0)
        {
            selectedCandidate = default;
            return false;
        }

        Service.Log.Debug($"[算路] 终点候选评估：{string.Join(" | ", candidates.Select(DescribeEndCandidate))}");

        selectedCandidate = candidates[0];
        if (selectedCandidate.PolyRef != requestedEndRef)
            Interlocked.Increment(ref endReplacementCount);

        return true;
        
        static string DescribeEndCandidate(MeshEndCandidate candidate) =>
            $"{candidate.PolyRef:X}: requested = {(candidate.IsRequestedEnd ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}";
    }

    private List<MeshEndCandidate> CollectEndCandidates(Vector3 requestedTarget, long requestedEndRef)
    {
        Dictionary<long, int>  candidateIndices = [];
        List<MeshEndCandidate> candidates       = [];

        foreach (var poly in query.FindIntersectingMeshPolys
                     (requestedTarget, new(END_POLY_CANDIDATE_HALF_EXTENT_XZ, END_POLY_CANDIDATE_HALF_EXTENT_Y, END_POLY_CANDIDATE_HALF_EXTENT_XZ)))
            TryAddEndCandidate(poly, false);

        TryAddEndCandidate(requestedEndRef, true);
        candidates.Sort(CompareEndCandidate);
        return candidates;

        void TryAddEndCandidate(long poly, bool forceInclude)
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
                if (horizontalDistanceSq > END_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE * END_POLY_CANDIDATE_MAX_HORIZONTAL_DISTANCE)
                    return;
                if (MathF.Abs(verticalDistance) > END_POLY_CANDIDATE_MAX_VERTICAL_DISTANCE)
                    return;
            }

            candidateIndices.Add(poly, candidates.Count);
            candidates.Add(new(poly, point, horizontalDistanceSq, verticalDistance, forceInclude, isPointOverPoly));
        }
    }

    private static int CompareEndCandidate(MeshEndCandidate left, MeshEndCandidate right)
    {
        if (left.IsPointOverPoly != right.IsPointOverPoly)
            return right.IsPointOverPoly.CompareTo(left.IsPointOverPoly);

        if (left.IsAboveTarget != right.IsAboveTarget)
            return left.IsAboveTarget.CompareTo(right.IsAboveTarget);

        var cmp = left.VerticalDistanceAbs.CompareTo(right.VerticalDistanceAbs);
        if (cmp != 0)
            return cmp;

        cmp = left.HorizontalDistanceSq.CompareTo(right.HorizontalDistanceSq);
        if (cmp != 0)
            return cmp;

        if (left.IsRequestedEnd != right.IsRequestedEnd)
            return right.IsRequestedEnd.CompareTo(left.IsRequestedEnd);

        return left.PolyRef.CompareTo(right.PolyRef);
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
        float             range,
        CancellationToken cancel
    )
    {
        var selection = EvaluateBestStartPathCandidate(from, to, requestedStartRef, endRef, requestedEndPos, range, cancel, false);
        selection = TryUpgradeStartSelectionWithExpandedSearch(selection, from, to, requestedStartRef, endRef, requestedEndPos, range, cancel);

        if (selection.Selected is { } selected)
            LogStartCandidateDecision(selected, to, requestedStartRef, selection.RequestedSuccessful, selection.RequestedFailed, selection.LockedByRequested);

        return selection.Selected;

        void LogStartCandidateDecision
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
                $"[算路] 已选起点多边形 {selected.StartRef:X}，投影点 = {selected.StartCandidate.ProjectedPoint:f3}，输出起点 = {selected.StartPoint:f3}，结果 = {selected.ResultStatus}，requested = {(selected.IsRequestedStart ? 1 : 0)}，overPoly = {(selected.IsPointOverPoly ? 1 : 0)}，supportHits = {selected.SupportProbeHits}"
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
                else if (!NearlyEqual(selected.DistanceToRequestedTargetSq(requestedTarget), requested.DistanceToRequestedTargetSq(requestedTarget)))
                {
                    reason =
                        $"距目标更近: {MathF.Sqrt(requested.DistanceToRequestedTargetSq(requestedTarget)):f3} -> {MathF.Sqrt(selected.DistanceToRequestedTargetSq(requestedTarget)):f3}";
                }
                else if (!NearlyEqual(selected.PathLength, requested.PathLength))
                    reason = $"路径更短: {requested.PathLength:f3} -> {selected.PathLength:f3}";
                else if (selected.IsPointOverPoly != requested.IsPointOverPoly)
                    reason = selected.IsPointOverPoly ? "命中更贴脚的多边形" : "保持非贴脚候选";
                else if (!NearlyEqual(selected.StartCandidate.VerticalDistanceAbs, requested.StartCandidate.VerticalDistanceAbs))
                    reason = $"垂直偏差更小: {requested.StartCandidate.VerticalDistanceAbs:f3} -> {selected.StartCandidate.VerticalDistanceAbs:f3}";
                else if (selected.WeightedLinkPenalty != requested.WeightedLinkPenalty)
                    reason = $"跳链代价更低: {requested.WeightedLinkPenalty} -> {selected.WeightedLinkPenalty}";
                else if (!NearlyEqual(selected.StartCandidate.HorizontalDistanceSq, requested.StartCandidate.HorizontalDistanceSq))
                {
                    reason =
                        $"水平偏移更小: {MathF.Sqrt(requested.StartCandidate.HorizontalDistanceSq):f3} -> {MathF.Sqrt(selected.StartCandidate.HorizontalDistanceSq):f3}";
                }
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
    }

    private StartCandidateSelection TryUpgradeStartSelectionWithExpandedSearch
    (
        StartCandidateSelection currentSelection,
        Vector3                 from,
        Vector3                 requestedTarget,
        long                    requestedStartRef,
        long                    endRef,
        RcVec3f                 requestedEndPos,
        float                   range,
        CancellationToken       cancel
    )
    {
        if (currentSelection.LockedByRequested)
            return currentSelection;

        if (currentSelection.Selected is { ResultStatus: not PathfindStatus.Partial })
            return currentSelection;

        var expandedSelection = EvaluateBestStartPathCandidate
        (
            from,
            requestedTarget,
            requestedStartRef,
            endRef,
            requestedEndPos,
            range,
            cancel,
            true
        );

        if (expandedSelection.Selected is not { } expandedCandidate)
            return currentSelection;

        if (currentSelection.Selected is { } selected &&
            !IsBetterStartPathCandidate(expandedCandidate, selected, requestedTarget))
            return currentSelection;

        LogExpandedStartSelectionOverride(currentSelection.Selected, expandedCandidate, expandedSelection, requestedTarget);
        return expandedSelection;

        static void LogExpandedStartSelectionOverride
        (
            MeshPathCandidate?      firstSelection,
            MeshPathCandidate       expandedCandidate,
            StartCandidateSelection expandedSelection,
            Vector3                 requestedTarget
        )
        {
            var firstAttempt    = firstSelection is { } selected ? DescribeStartPathCandidateDecision(selected, requestedTarget) : "<none>";
            var expandedAttempt = DescribeStartPathCandidateDecision(expandedCandidate, requestedTarget);
            Service.Log.Warning
            (
                $"[算路] 起点扩搜改判：首轮 = {firstAttempt}，扩搜 = {expandedAttempt}，扩搜候选 = {expandedSelection.EvaluatedCandidates}/{expandedSelection.TotalCandidates}"
            );
        }
    }

    private StartCandidateSelection EvaluateBestStartPathCandidate
    (
        Vector3           from,
        Vector3           requestedTarget,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        float             range,
        CancellationToken cancel,
        bool              expandedSearch
    )
    {
        var candidates = CollectStartPolyCandidates(from, requestedStartRef, expandedSearch);
        if (candidates.Count == 0)
            return new(null, null, false, false, 0, 0);

        var evaluationLimit = expandedSearch ? EXPANDED_MAX_START_POLY_CANDIDATES_TO_EVALUATE : MAX_START_POLY_CANDIDATES_TO_EVALUATE;
        var candidateBatch  = candidates.Take(evaluationLimit).ToArray();

        MeshPathCandidate? best                = null;
        MeshPathCandidate? requestedSuccessful = null;
        List<string>       candidateLogs       = [];
        var                requestedFailed     = false;

        var evaluations = EvaluateStartPathCandidates(candidateBatch, from, requestedTarget, endRef, requestedEndPos, range, cancel);

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

            if (best == null || IsBetterStartPathCandidate(pathCandidate, best.Value, requestedTarget))
                best = pathCandidate;
        }

        var lockedByRequested = false;

        if (requestedSuccessful is { } requestedCandidate && ShouldLockRequestedStartCandidate(requestedCandidate))
        {
            best              = requestedCandidate;
            lockedByRequested = true;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] {(expandedSearch ? "起点扩搜候选评估" : "起点候选评估")}：{string.Join(" | ", candidateLogs)}");

        return new(best, requestedSuccessful, requestedFailed, lockedByRequested, candidates.Count, candidateBatch.Length);
    }

    private StartCandidateEvaluation[] EvaluateStartPathCandidates
    (
        IReadOnlyList<MeshPolyCandidate> candidates,
        Vector3                          requestedStart,
        Vector3                          requestedTarget,
        long                             endRef,
        RcVec3f                          requestedEndPos,
        float                            range,
        CancellationToken                cancel
    )
    {
        var result = new StartCandidateEvaluation[candidates.Count];
        if (candidates.Count == 0)
            return result;

        var filter = query.GroundAreaFilterIgnoringUnreachable;

        if (candidates.Count > 1 && Environment.ProcessorCount > 1)
        {
            Parallel.For
            (
                0,
                candidates.Count,
                new ParallelOptions { CancellationToken = cancel, MaxDegreeOfParallelism = Math.Min(Environment.ProcessorCount, candidates.Count) },
                i => result[i] = EvaluateStartPathCandidate(candidates[i], requestedStart, requestedTarget, endRef, requestedEndPos, filter, range)
            );
        }
        else
        {
            for (var i = 0; i < candidates.Count; ++i)
            {
                cancel.ThrowIfCancellationRequested();
                result[i] = EvaluateStartPathCandidate(candidates[i], requestedStart, requestedTarget, endRef, requestedEndPos, filter, range);
            }
        }

        return result;
    }

    private StartCandidateEvaluation EvaluateStartPathCandidate
    (
        MeshPolyCandidate candidate,
        Vector3           requestedStart,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
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
            requestedStart,
            corridor,
            status,
            requestedEndPos,
            requestedTarget,
            endRef,
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

        int CountStartSupportProbeHits(Vector3 start, long poly)
        {
            const float MATCH_DISTANCE_SQ = START_SUPPORT_MATCH_DISTANCE * START_SUPPORT_MATCH_DISTANCE;
            const float STEP              = MathF.Tau                    / START_SUPPORT_PROBE_COUNT;
            var         hits              = 0;

            for (var i = 0; i < START_SUPPORT_PROBE_COUNT; i++)
            {
                var angle = STEP * i;
                var probe = new Vector3(start.X + MathF.Cos(angle) * START_SUPPORT_PROBE_RADIUS, start.Y, start.Z + MathF.Sin(angle) * START_SUPPORT_PROBE_RADIUS);
                if (!TryClosestPointOnPolyWithFlags(probe, poly, out var probeClosest, out var probeOverPoly))
                    continue;

                if (probeOverPoly)
                {
                    hits++;
                    continue;
                }

                var dx = probeClosest.X - probe.X;
                var dz = probeClosest.Z - probe.Z;
                if (dx * dx + dz * dz <= MATCH_DISTANCE_SQ)
                    hits++;
            }

            return hits;
        }
    }

    private static string DescribeStartPathCandidateDecision(MeshPathCandidate candidate, Vector3 requestedTarget) =>
        $"{candidate.StartRef:X}/{candidate.ResultStatus}/requested={(candidate.IsRequestedStart ? 1 : 0)}/overPoly={(candidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(candidate.DistanceToRequestedTargetSq(requestedTarget)):f3}";

    private bool TryRepairGroundGapByTileBoundaryBridge
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedStart,
        Vector3           requestedTarget,
        long              endRef,
        float             range,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        var partialEnd = partialCandidate.FinalDestination;
        if (!TryResolveTileBoundaryBridgePoint
                (partialCandidate.LastPoly, endRef, partialEnd, requestedTarget, out var bridgePoint, out var bridgeH, out var bridgeV, out var remainingDistance))
            return false;

        repairedLastPoly = endRef;
        List<PlannerPathSegment> segments = [BuildGroundMeshCorridorSegment(partialCandidate, requestedStart)];
        if (Vector3.DistanceSquared(partialCandidate.FinalDestination, bridgePoint) > 0.000001f)
            segments.Add(BuildGroundDiscreteSegment(partialCandidate.FinalDestination, bridgePoint));

        Service.Log.Warning
        (
            $"[算路] Tile 边界桥接：partial 终点 = {partialEnd:f3}，桥接点 = {bridgePoint:f3}，桥接水平距 {bridgeH:f3}，高差 {bridgeV:f3}，剩余距离 {remainingDistance:f3}"
        );
        repairedResult = BuildGroundPlannerResult(requestedTarget, range, PathfindStatus.Complete, requestedTarget, segments);
        return true;
    }

    private bool TryResolveTileBoundaryBridgePoint
    (
        long        lastPoly,
        long        endRef,
        Vector3     partialEnd,
        Vector3     requestedTarget,
        out Vector3 bridgePoint,
        out float   bridgeHorizontalDistance,
        out float   bridgeVerticalDistance,
        out float   remainingDistance
    )
    {
        bridgePoint              = default;
        bridgeHorizontalDistance = 0;
        bridgeVerticalDistance   = 0;
        remainingDistance        = Vector3.Distance(partialEnd, requestedTarget);

        if (remainingDistance > TILE_BOUNDARY_MAX_BRIDGE_DISTANCE)
            return false;

        var navmesh = query.MeshQuery.GetAttachedNavMesh();
        navmesh.GetTileAndPolyByRef(lastPoly, out var lastTile, out _);
        navmesh.GetTileAndPolyByRef(endRef,   out var endTile,  out _);
        if (lastTile == null || endTile == null)
            return false;

        var dx = Math.Abs(lastTile.data.header.x - endTile.data.header.x);
        var dz = Math.Abs(lastTile.data.header.y - endTile.data.header.y);
        if (dx > 1 || dz > 1)
            return false;

        if (!TryClosestPointOnPolyWithFlags(partialEnd, endRef, out bridgePoint, out _))
            return false;

        var bridgeDelta = bridgePoint - partialEnd;
        bridgeHorizontalDistance = new Vector2(bridgeDelta.X, bridgeDelta.Z).Length();
        bridgeVerticalDistance   = MathF.Abs(bridgeDelta.Y);
        return bridgeHorizontalDistance <= TILE_BOUNDARY_MAX_BRIDGE_DISTANCE && bridgeVerticalDistance <= TILE_BOUNDARY_MAX_VERTICAL_DELTA;
    }

    private static MeshPathCandidate BuildMeshPathCandidate
    (
        DtNavMeshQuery    query,
        MeshPolyCandidate startCandidate,
        Vector3           requestedStart,
        List<long>        corridor,
        DtStatus          status,
        RcVec3f           requestedEndPos,
        Vector3           requestedTarget,
        long              endRef,
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
            {
                return new
                (
                    startCandidate,
                    status,
                    PathfindStatus.Failed,
                    startCandidate.ProjectedPoint,
                    startCandidate.ProjectedPoint,
                    corridor,
                    GroundQueryMode.Classic,
                    float.MaxValue,
                    int.MaxValue,
                    0,
                    0
                );
            }

            finalDestination = projectedEndPos.RecastToSystem();
            resultStatus = range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }
        else finalDestination = requestedEndPos.RecastToSystem();

        var startPoint = ResolvePreferredOffMeshStartPoint(query, requestedStart, startCandidate, corridor, finalDestination);
        var pathLength = EstimatePathLength(query, startPoint.SystemToRecast(), finalDestination.SystemToRecast(), corridor);
        CountPathSemantics(query.GetAttachedNavMesh(), corridor, out var weightedLinkPenalty, out var offMeshTransitionCount, out var areaCrossingCount);

        return new
        (
            startCandidate,
            status,
            resultStatus,
            startPoint,
            finalDestination,
            corridor,
            GroundQueryMode.Classic,
            pathLength,
            weightedLinkPenalty,
            offMeshTransitionCount,
            areaCrossingCount
        );

        static void CountPathSemantics
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
    }

    private static Vector3 ResolvePreferredOffMeshStartPoint
    (
        DtNavMeshQuery      query,
        Vector3             requestedStart,
        MeshPolyCandidate   startCandidate,
        IReadOnlyList<long> corridor,
        Vector3             finalDestination
    )
    {
        if (startCandidate.IsPointOverPoly || corridor.Count == 0)
            return startCandidate.ProjectedPoint;

        query.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(startCandidate.PolyRef, out var tile, out var poly);
        if (poly.GetPolyType() != DtPolyTypes.DT_POLYTYPE_GROUND || poly.vertCount < 3)
            return startCandidate.ProjectedPoint;

        var lookaheadPoint = ResolveOffMeshLookaheadPoint(query, startCandidate.ProjectedPoint, finalDestination, corridor);
        var desiredDelta   = new Vector2(lookaheadPoint.X - requestedStart.X, lookaheadPoint.Z - requestedStart.Z);
        if (desiredDelta.LengthSquared() <= OFF_MESH_DIRECTION_MIN_DISTANCE_SQ)
            return startCandidate.ProjectedPoint;

        var desiredDirection = Vector2.Normalize(desiredDelta);
        var projectedPoint   = startCandidate.ProjectedPoint;
        var bestPoint        = projectedPoint;
        var bestAlignment    = DirectionalAlignmentXZ(requestedStart, projectedPoint, desiredDirection);

        for (var i = 0; i < poly.vertCount; ++i)
        {
            var vertex    = NavmeshBitmap.GetVertex(tile, poly.verts[i]);
            var alignment = DirectionalAlignmentXZ(requestedStart, vertex, desiredDirection);
            if (alignment <= bestAlignment + OFF_MESH_VERTEX_ALIGNMENT_IMPROVEMENT_MIN)
                continue;

            if (alignment                                       > bestAlignment + 0.0001f ||
                Vector3.DistanceSquared(vertex, projectedPoint) < Vector3.DistanceSquared(bestPoint, projectedPoint))
            {
                bestPoint     = vertex;
                bestAlignment = alignment;
            }
        }

        return bestPoint;
    }

    private static Vector3 ResolveOffMeshLookaheadPoint
    (
        DtNavMeshQuery      query,
        Vector3             startPoint,
        Vector3             finalDestination,
        IReadOnlyList<long> corridor
    )
    {
        if (TryFindStraightPath
            (
                query,
                startPoint.SystemToRecast(),
                finalDestination.SystemToRecast(),
                corridor,
                DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS,
                out var straightPath,
                out var count
            ))
        {
            for (var i = 0; i < count; ++i)
            {
                var point = straightPath[i].pos.RecastToSystem();
                if (Vector3.DistanceSquared(point, startPoint) > OFF_MESH_LOOKAHEAD_POINT_MIN_DISTANCE_SQ)
                    return point;
            }
        }

        if (corridor.Count > 1)
            return query.GetAttachedNavMesh().GetPolyCenter(corridor[1]).RecastToSystem();

        return finalDestination;
    }

    private static float DirectionalAlignmentXZ(Vector3 origin, Vector3 point, Vector2 desiredDirection)
    {
        var delta = new Vector2(point.X - origin.X, point.Z - origin.Z);
        if (delta.LengthSquared() <= OFF_MESH_DIRECTION_MIN_DISTANCE_SQ)
            return -1f;

        delta = Vector2.Normalize(delta);
        return Vector2.Dot(delta, desiredDirection);
    }

    private static float EstimatePathLength(DtNavMeshQuery query, RcVec3f startPos, RcVec3f endPos, List<long> corridor)
    {
        if (corridor.Count == 0)
            return float.MaxValue;

        if (TryFindStraightPath(query, startPos, endPos, corridor, 0, out var straightPath, out var count))
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

    private static bool TryFindStraightPath
    (
        DtNavMeshQuery       query,
        RcVec3f              startPos,
        RcVec3f              endPos,
        IReadOnlyList<long>  corridor,
        int                  straightPathOptions,
        out DtStraightPath[] straightPath,
        out int              count
    )
    {
        straightPath = new DtStraightPath[256];
        count        = 0;

        if (corridor.Count == 0)
            return false;

        var status = query.FindStraightPath(startPos, endPos, [.. corridor], corridor.Count, straightPath, out count, straightPath.Length, straightPathOptions);
        return status.Succeeded() && count > 0;
    }

    private static DtStatus FindPath
    (
        DtNavMeshQuery query,
        long           startRef,
        long           endRef,
        RcVec3f        startPos,
        RcVec3f        endPos,
        IDtQueryFilter filter,
        out List<long> corridor
    )
    {
        var buffer = new long[MAX_PATH_POLYS];
        corridor = [];
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

        var candidateDistance = candidate.DistanceToRequestedTargetSq(requestedTarget);
        var currentDistance   = currentBest.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(candidateDistance, currentDistance))
            return candidateDistance < currentDistance;

        if (!NearlyEqual(candidate.PathLength, currentBest.PathLength))
            return candidate.PathLength < currentBest.PathLength;

        if (candidate.IsPointOverPoly != currentBest.IsPointOverPoly)
            return candidate.IsPointOverPoly;

        if (!NearlyEqual(candidate.StartCandidate.VerticalDistanceAbs, currentBest.StartCandidate.VerticalDistanceAbs))
            return candidate.StartCandidate.VerticalDistanceAbs < currentBest.StartCandidate.VerticalDistanceAbs;

        var candidateRequestedOver = candidate is { IsRequestedStart: true, IsPointOverPoly: true };
        var currentRequestedOver   = currentBest is { IsRequestedStart: true, IsPointOverPoly: true };
        if (candidateRequestedOver != currentRequestedOver)
            return candidateRequestedOver;

        if (candidate.WeightedLinkPenalty != currentBest.WeightedLinkPenalty)
            return candidate.WeightedLinkPenalty < currentBest.WeightedLinkPenalty;

        if (!NearlyEqual(candidate.StartCandidate.HorizontalDistanceSq, currentBest.StartCandidate.HorizontalDistanceSq))
            return candidate.StartCandidate.HorizontalDistanceSq < currentBest.StartCandidate.HorizontalDistanceSq;

        return candidate.StartRef < currentBest.StartRef;
    }

    private static bool ShouldLockRequestedStartCandidate(MeshPathCandidate candidate) =>
        candidate.IsRequestedStart &&
        (
            !candidate.IsPointOverPoly ||
            candidate.ResultStatus is PathfindStatus.Complete or PathfindStatus.ReachedWithinRange
        );

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

    private static PlannerPathSegment BuildGroundMeshCorridorSegment(MeshPathCandidate candidate, Vector3 traversalStartPosition) =>
        new()
        {
            MovementMode           = MovementMode.Ground,
            SegmentKind            = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl   = false,
            ReachabilitySource     = PathReachabilitySource.Mesh,
            GeometryKind           = PlannerSegmentGeometryKind.MeshCorridor,
            TraversalStartPosition = traversalStartPosition,
            StartPosition          = candidate.StartPoint,
            EndPosition            = candidate.FinalDestination,
            Corridor               = [.. candidate.Corridor]
        };

    private static PlannerPathSegment BuildGroundDiscreteSegment(Vector3 startPosition, params Vector3[] points) =>
        new()
        {
            MovementMode           = MovementMode.Ground,
            SegmentKind            = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl   = false,
            ReachabilitySource     = PathReachabilitySource.Mesh,
            GeometryKind           = PlannerSegmentGeometryKind.DiscretePoints,
            TraversalStartPosition = startPosition,
            StartPosition          = startPosition,
            EndPosition            = points.Length > 0 ? points[^1] : startPosition,
            Points                 = [.. points]
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

    private static bool NearlyEqual(float left, float right) =>
        MathF.Abs(left - right) <= 0.0001f;

    private static int GetResultStatusRank(PathfindStatus status) => status switch
    {
        PathfindStatus.Complete           => 0,
        PathfindStatus.ReachedWithinRange => 1,
        PathfindStatus.Partial            => 2,
        _                                 => 3
    };


    private enum GroundQueryMode
    {
        AnyAngle,
        Classic
    }

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
        Vector3           StartPoint,
        Vector3           FinalDestination,
        List<long>        Corridor,
        GroundQueryMode   QueryMode,
        float             PathLength,
        int               WeightedLinkPenalty,
        int               OffMeshTransitionCount,
        int               AreaCrossingCount
    )
    {
        public long StartRef         => StartCandidate.PolyRef;
        public long LastPoly         => Corridor.Count > 0 ? Corridor[^1] : 0;
        public bool IsRequestedStart => StartCandidate.IsRequestedStart;
        public bool IsPointOverPoly  => StartCandidate.IsPointOverPoly;
        public int  SupportProbeHits => StartCandidate.SupportProbeHits;

        public float DistanceToRequestedTargetSq(Vector3 requestedTarget) => Vector3.DistanceSquared(FinalDestination, requestedTarget);
    }

    internal sealed class GroundPathDiagnosticsSnapshot
    {
        public required long GroundQueries               { get; init; }
        public required long PartialQueries              { get; init; }
        public required long SuspectedTileSeamCutoffs    { get; init; }
        public          long AnyAnglePreferred           { get; init; }
        public          long ClassicFallbacks            { get; init; }
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
    private const float TILE_BOUNDARY_MAX_BRIDGE_DISTANCE                     = 60.0f;
    private const float TILE_BOUNDARY_MAX_VERTICAL_DELTA                      = 5.0f;
    private const float OFF_MESH_DIRECTION_MIN_DISTANCE_SQ                    = 0.0001f;
    private const float OFF_MESH_LOOKAHEAD_POINT_MIN_DISTANCE_SQ              = 0.01f;
    private const float OFF_MESH_VERTEX_ALIGNMENT_IMPROVEMENT_MIN             = 0.05f;
    private const int   MAX_PATH_POLYS                                        = 4096;

    #endregion
}
