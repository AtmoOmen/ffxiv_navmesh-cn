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
        {
            message +=
                $"，起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}";
        }

        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Interlocked.Increment(ref _partialGroundQueryCount);
                Service.Log.Warning(message);

                if (diagnostic.IsSuspectedTileSeamCutoff)
                {
                    Interlocked.Increment(ref _suspectedTileSeamCutoffCount);
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

        if (lockedByRequested)
        {
            Service.Log.Information($"[算路] 起点锁定：原始候选 = {requestedStartRef:X}，原因 = 原始候选在 over-poly 内且算路成功。");
            return;
        }

        if (selected.StartRef == requestedStartRef)
        {
            Service.Log.Debug
            (
                $"[算路] 起点保持：原始候选 = {requestedStartRef:X}，原因 = {BuildStartKeepReason(selected, requestedSuccessful, requestedFailed)}。"
            );
            return;
        }

        Service.Log.Warning
        (
            $"[算路] 起点替换：原始候选 = {requestedStartRef:X}，选中 = {selected.StartRef:X}，原因 = {BuildStartReplacementReason(selected, requestedTarget, requestedSuccessful, requestedFailed)}。"
        );
        Interlocked.Increment(ref _startReplacementCount);
    }

    private static string BuildStartKeepReason(MeshPathCandidate selected, MeshPathCandidate? requestedSuccessful, bool requestedFailed)
    {
        if (requestedFailed)
            return "原始候选已恢复可用且综合评分最优";

        if (requestedSuccessful == null)
            return "原始候选通过筛选并最终胜出";

        if (selected.IsPointOverPoly)
            return "原始候选在 over-poly 内并保持最优";

        return "原始候选在候选竞争中综合评分最优";
    }

    private static string BuildStartReplacementReason
        (MeshPathCandidate selected, Vector3 requestedTarget, MeshPathCandidate? requestedSuccessful, bool requestedFailed)
    {
        if (requestedSuccessful == null)
            return requestedFailed ? "原始候选算路失败" : "原始候选未通过有效路径评估";

        var requested = requestedSuccessful.Value;
        if (!requested.IsPointOverPoly && selected.IsPointOverPoly)
            return "原始候选不在 over-poly，且选中候选在 over-poly";

        if (selected.SupportProbeHits > requested.SupportProbeHits)
            return "原始候选的 support 命中更低";

        var selectedRank  = ResultStatusRank(selected.ResultStatus);
        var requestedRank = ResultStatusRank(requested.ResultStatus);
        if (selectedRank < requestedRank)
            return $"选中候选路径状态更优（{selected.ResultStatus} > {requested.ResultStatus}）";

        var selectedDistance  = selected.DistanceToRequestedTargetSq(requestedTarget);
        var requestedDistance = requested.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(selectedDistance, requestedDistance) && selectedDistance < requestedDistance)
            return "选中候选更接近最终目的地";

        if (!NearlyEqual(selected.StartCandidate.VerticalDistanceAbs, requested.StartCandidate.VerticalDistanceAbs) &&
            selected.StartCandidate.VerticalDistanceAbs < requested.StartCandidate.VerticalDistanceAbs)
            return "选中候选的垂直偏移更小";

        if (!NearlyEqual(selected.StartCandidate.HorizontalDistanceSq, requested.StartCandidate.HorizontalDistanceSq) &&
            selected.StartCandidate.HorizontalDistanceSq < requested.StartCandidate.HorizontalDistanceSq)
            return "选中候选的水平偏移更小";

        return "选中候选综合评分更优";
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
        repairedResult   = default!;
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
        repairedResult   = default!;
        repairedLastPoly = partialCandidate.LastPoly;

        Span<long> visited = stackalloc long[MaxPathPolys];
        var status = MeshQuery.Raycast
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
        repairedResult   = default!;
        repairedLastPoly = partialCandidate.LastPoly;

        Span<long> visited = stackalloc long[32];
        var status = MeshQuery.MoveAlongSurface
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
        if (movedDistSq <= 0.000001f || verticalDist > ShortGapRepairMaxVerticalDelta)
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

        var resumeStatus = FindPath(MeshQuery, movedPoly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);
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
        repairedResult   = default!;
        repairedLastPoly = partialCandidate.LastPoly;

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

            if (!TryClosestPointOnPolyWithFlags(partialEnd, poly, out var bridgePoint, out var isPointOverPoly))
                continue;

            var bridgeDelta              = bridgePoint - partialEnd;
            var bridgeHorizontalDistance = new Vector2(bridgeDelta.X, bridgeDelta.Z).Length();
            var bridgeVerticalDistance   = MathF.Abs(bridgeDelta.Y);
            if (bridgeHorizontalDistance > ShortGapRepairMaxBridgeDistance || bridgeVerticalDistance > ShortGapRepairMaxVerticalDelta)
                continue;

            var status = FindPath(MeshQuery, poly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);

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
