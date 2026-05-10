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
    private long endReplacementCount;

    internal NavmeshGroundQuery(NavmeshQuery query) =>
        this.query = query;

    internal GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        new()
        {
            GroundQueries               = Interlocked.Read(ref groundQueryCount),
            PartialQueries              = Interlocked.Read(ref partialGroundQueryCount),
            SuspectedTileSeamCutoffs    = Interlocked.Read(ref suspectedTileSeamCutoffCount),
            AnyAnglePreferred           = 0,
            ClassicFallbacks            = 0,
            StartReplacements           = 0,
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

        var meshTileSize = MathF.Max(query.MeshQuery.GetAttachedNavMesh().GetParams().tileWidth, query.MeshQuery.GetAttachedNavMesh().GetParams().tileHeight);
        foreach (var poly in query.FindIntersectingMeshPolys
                     (to, new Vector3(meshTileSize, MathF.Max(meshTileSize, MathF.Max(query.ConfigData.PathTolerance, float.Epsilon)), meshTileSize)))
            TryAddEndCandidate(poly, false);

        TryAddEndCandidate(requestedEndRef, true);
        endCandidates.Sort
        ((a, b) =>
            {
                if (a.IsPointOverPoly != b.IsPointOverPoly)
                    return b.IsPointOverPoly.CompareTo(a.IsPointOverPoly);

                var aAbove = a.VerticalDelta > MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
                var bAbove = b.VerticalDelta > MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
                if (aAbove != bAbove)
                    return aAbove.CompareTo(bAbove);

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
                candidate.IsPointOverPoly != currentBest.IsPointOverPoly
                    ? candidate.IsPointOverPoly
                    : candidate.VerticalDelta   > MathF.Max(query.ConfigData.PathTolerance, float.Epsilon) !=
                      currentBest.VerticalDelta > MathF.Max(query.ConfigData.PathTolerance, float.Epsilon)
                        ? !(candidate.VerticalDelta > MathF.Max(query.ConfigData.PathTolerance, float.Epsilon))
                        : !NearlyEqual(candidate.VerticalDistanceAbs, currentBest.VerticalDistanceAbs)
                            ? candidate.VerticalDistanceAbs < currentBest.VerticalDistanceAbs
                            : !NearlyEqual(candidate.HorizontalDistanceSq, currentBest.HorizontalDistanceSq)
                                ? candidate.HorizontalDistanceSq < currentBest.HorizontalDistanceSq
                                : candidate.IsRequestedEnd       != currentBest.IsRequestedEnd
                                    ? candidate.IsRequestedEnd
                                    : candidate.PolyRef < currentBest.PolyRef;

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
        var selectedAttempt = ExecuteGroundPathAttempt(from, to, requestedStartRef, endRef, requestedEndPos, query.GroundAreaFilter, useRaycast, range, cancel);

        if (selectedAttempt is not var (pathCandidate, plannerResult, startRef, lastPoly, queryStatus))
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");

        query.LastPath.Clear();
        query.LastPath.AddRange(pathCandidate.Corridor);
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}（原始起点候选 = {requestedStartRef:X}，查询模式 = {pathCandidate.QueryMode}）");
        Service.Log.Debug($"[算路] 地面终点解析：原始终点候选 = {requestedEndRef:X}，选中 = {endRef:X}，终点投影 = {resolvedDestination:f3}");
        Service.Log.Debug
            ($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {queryStatus}，路径 = {string.Join(", ", query.LastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();
        LogMeshResult(plannerResult, from, startRef, endRef, lastPoly, range, timer.Value());
        return plannerResult;

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
            var toleranceFloor       = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
            var horizontalTolerance = MathF.Max
            (
                Vector3.Distance(point, to),
                MathF.Max(query.MeshQuery.GetAttachedNavMesh().GetParams().tileWidth, query.MeshQuery.GetAttachedNavMesh().GetParams().tileHeight) * 0.5f
            );
            var verticalTolerance = MathF.Max
                                    (
                                        MathF.Max
                                        (
                                            query.MeshQuery.GetAttachedNavMesh().GetParams().tileWidth,
                                            query.MeshQuery.GetAttachedNavMesh().GetParams().tileHeight
                                        ),
                                        toleranceFloor
                                    ) *
                                    0.5f;

            if (!forceInclude)
            {
                if (horizontalDistanceSq > horizontalTolerance * horizontalTolerance)
                    return;
                if (MathF.Abs(verticalDistance) > verticalTolerance)
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

        var visited = new long[Math.Max(query.MeshQuery.GetAttachedNavMesh().GetParams().maxPolys, 1)];
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

        var movedPoint = moved.RecastToSystem();
        var movedDelta = movedPoint - partialCandidate.FinalDestination;
        var movedPoly = visited[visitedCount - 1];
        var movedDistSq = movedDelta.X * movedDelta.X + movedDelta.Z * movedDelta.Z;
        var toleranceFloor = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
        if (movedDistSq <= toleranceFloor * toleranceFloor ||
            HorizontalDistanceXZ(partialCandidate.FinalDestination, movedPoint) > HorizontalDistanceXZ(partialCandidate.FinalDestination, requestedTarget) ||
            MathF.Abs(movedPoint.Y - partialCandidate.FinalDestination.Y) > MathF.Abs(requestedTarget.Y - partialCandidate.FinalDestination.Y) + toleranceFloor)
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
                isPointOverPoly
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
        var toleranceFloor = MathF.Max(query.ConfigData.PathTolerance, float.Epsilon);
        var gapDistance = Vector3.Distance(partialEnd, requestedTarget);
        var gapVertical = MathF.Abs(requestedTarget.Y - partialEnd.Y);
        var repairCandidates = query.FindIntersectingMeshPolys(partialEnd, new Vector3(gapDistance, MathF.Max(gapVertical, toleranceFloor), gapDistance));
        MeshPathCandidate? bestResumeCandidate = null;
        List<string> candidateLogs = [];

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
            if (HorizontalDistanceXZ(partialEnd, bridgePoint) > HorizontalDistanceXZ(partialEnd, requestedTarget) ||
                MathF.Abs(bridgePoint.Y - partialEnd.Y)       > MathF.Abs(requestedTarget.Y - partialEnd.Y) + toleranceFloor)
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
                    isPointOverPoly
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
        var classicOption = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            0,
            0
        );
        var opt = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0,
            useRaycast ? 5 : 0
        );
        if (!TryClosestPointOnPolyWithFlags(from, requestedStartRef, out var startPoint, out var isPointOverPoly))
            return null;

        var status = FindPath(query.MeshQuery, requestedStartRef, endRef, startPoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);
        if (status.Failed() || corridor.Count == 0)
            return null;

        var pathCandidate = BuildMeshPathCandidate
        (
            new(requestedStartRef, startPoint, Vector3.DistanceSquared(startPoint, from), startPoint.Y - from.Y, true, isPointOverPoly),
            corridor,
            status,
            requestedEndPos,
            to,
            endRef,
            useRaycast ? GroundQueryMode.AnyAngle : GroundQueryMode.Classic,
            range
        );
        if (pathCandidate.ResultStatus == PathfindStatus.Failed)
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
        var buffer = new long[Math.Max(query.GetAttachedNavMesh().GetParams().maxPolys, 1)];
        corridor = [];

        if (opt.options != 0)
        {
            var status = query.InitSlicedFindPath(startRef, endRef, startPos, endPos, filter, opt.options);
            if (status.Failed())
                return status;

            do
            {
                status = query.UpdateSlicedFindPath(buffer.Length, out _);
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
        bool    IsPointOverPoly
    )
    {
        public float VerticalDistanceAbs => MathF.Abs(VerticalDelta);
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
}
