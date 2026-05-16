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
(
    NavmeshQuery query
)
{
    private const int MAX_PATH_POLYS           = 4096;
    private const int MAX_STRAIGHT_PATH_POINTS = 256;

    private long groundQueryCount;
    private long failedGroundQueryCount;
    private long partialGroundQueryCount;
    private long reachedWithinRangeQueryCount;

    internal PlannerResult PlanMeshPathDetailed(Vector3 from, Vector3 to, float range, CancellationToken cancel)
    {
        Interlocked.Increment(ref groundQueryCount);
        query.LastPath.Clear();
        cancel.ThrowIfCancellationRequested();

        var timer    = StopWatchTimer.Create();
        var startRef = query.FindNearestMeshPoly(from, allowUnreachable: false);
        var endRef   = query.FindNearestMeshPoly(to,   allowUnreachable: false);

        if (startRef == 0 || endRef == 0)
            return LogMeshFailure(from, to, startRef, endRef, 0, range, "无法定位起点或终点的可达导航多边形");

        if (!TryClosestPointOnPolyWithFlags(from, startRef, out var projectedStart, out _))
            return LogMeshFailure(from, to, startRef, endRef, 0, range, "无法投影起点到导航多边形");

        if (!TryClosestPointOnPolyWithFlags(to, endRef, out var projectedEnd, out _))
            return LogMeshFailure(from, to, startRef, endRef, 0, range, "无法投影终点到导航多边形");

        cancel.ThrowIfCancellationRequested();

        var pathStatus = FindPath
        (
            query.MeshQuery,
            startRef,
            endRef,
            projectedStart.SystemToRecast(),
            projectedEnd.SystemToRecast(),
            query.GroundAreaFilter,
            out var corridor
        );
        if (pathStatus.Failed() || corridor.Count == 0)
            return LogMeshFailure(from, to, startRef, endRef, 0, range, $"FindPath 失败: {pathStatus}");

        cancel.ThrowIfCancellationRequested();

        if (!TryFindStraightPath
            (
                query.MeshQuery,
                projectedStart.SystemToRecast(),
                projectedEnd.SystemToRecast(),
                corridor,
                0,
                out var straightPath,
                out var straightPathCount
            ))
        {
            return LogMeshFailure(from, to, startRef, endRef, corridor[^1], range, "FindStraightPath 失败");
        }

        var lastPoly          = corridor[^1];
        var finalDestination  = straightPath[straightPathCount - 1].pos.RecastToSystem();
        var resultStatus      = ResolveStatus(lastPoly, endRef, finalDestination, to, range);
        var segment           = BuildGroundMeshCorridorSegment(from, projectedStart, finalDestination, corridor);
        var plannerResult     = BuildGroundPlannerResult(to, range, resultStatus, finalDestination, [segment]);

        query.LastPath.AddRange(corridor);
        cancel.ThrowIfCancellationRequested();

        LogMeshResult(from, plannerResult, startRef, endRef, lastPoly, timer.Value());
        return plannerResult;
    }

    internal GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        new()
        {
            GroundQueries               = Interlocked.Read(ref groundQueryCount),
            FailedQueries               = Interlocked.Read(ref failedGroundQueryCount),
            PartialQueries              = Interlocked.Read(ref partialGroundQueryCount),
            ReachedWithinRangeQueries   = Interlocked.Read(ref reachedWithinRangeQueryCount),
            GeneratedClimbLinksAccepted = query.NavmeshData.GeneratedClimbDownLinkCount,
            GeneratedJumpLinksAccepted  = query.NavmeshData.GeneratedEdgeJumpLinkCount
        };

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

    private void LogMeshResult(Vector3 from, PlannerResult result, long startRef, long endRef, long lastPoly, TimeSpan duration)
    {
        var message =
            $"地面算路完成："                                  +
            $"状态 = {result.Status}，"                    +
            $"起点 = {from:f3}，"                          +
            $"请求终点 = {result.RequestedDestination:f3}，" +
            $"实际终点 = {result.FinalDestination:f3}，"     +
            $"多边形 = {startRef:X} -> {endRef:X}，"        +
            $"最后可达 = {lastPoly:X}，"                     +
            $"容差 = {result.DestinationTolerance:f3}，"   +
            $"耗时 = {duration.TotalSeconds:f3} 秒，"       +
            $"粗路径段 = {result.Segments.Count}";

        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Interlocked.Increment(ref partialGroundQueryCount);
                Service.Log.Warning(message);
                break;
            case PathfindStatus.ReachedWithinRange:
                Interlocked.Increment(ref reachedWithinRangeQueryCount);
                Service.Log.Debug(message);
                break;
            case PathfindStatus.Failed:
                Interlocked.Increment(ref failedGroundQueryCount);
                Service.Log.Error(message);
                break;
            default:
                Service.Log.Debug(message);
                break;
        }

        Service.Log.Debug($"[算路] straight path corridor = {string.Join(", ", query.LastPath.Select(polyRef => polyRef.ToString("X")))}");
    }

    private PlannerResult LogMeshFailure(Vector3 from, Vector3 to, long startRef, long endRef, long lastPoly, float range, string reason)
    {
        Interlocked.Increment(ref failedGroundQueryCount);
        var lastPolyText = lastPoly != 0 ? lastPoly.ToString("X") : "<none>";
        Service.Log.Error($"地面算路失败："                           +
                          $"起点 = {from:f3}，"                   +
                          $"请求终点 = {to:f3}，"                   +
                          $"多边形 = {startRef:X} -> {endRef:X}，" +
                          $"最后可达 = {lastPolyText}，"            +
                          $"容差 = {range:f3}，"                  +
                          $"原因 = {reason}");
        return new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = to,
            FinalDestination     = to,
            DestinationTolerance = range
        };
    }

    private static PathfindStatus ResolveStatus(long lastPoly, long endRef, Vector3 finalDestination, Vector3 requestedTarget, float range)
    {
        if (lastPoly == endRef)
            return PathfindStatus.Complete;

        if (range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range)
            return PathfindStatus.ReachedWithinRange;

        return PathfindStatus.Partial;
    }

    private static bool TryFindStraightPath
    (
        DtNavMeshQuery       meshQuery,
        RcVec3f              startPos,
        RcVec3f              endPos,
        List<long>           corridor,
        int                  straightPathOptions,
        out DtStraightPath[] straightPath,
        out int              count
    )
    {
        straightPath = new DtStraightPath[MAX_STRAIGHT_PATH_POINTS];
        count        = 0;

        if (corridor.Count == 0)
            return false;

        var status = meshQuery.FindStraightPath(startPos, endPos, [.. corridor], corridor.Count, straightPath, out count, straightPath.Length, straightPathOptions);
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
        var option = filter is GroundAreaCostFilter groundFilter && groundFilter.RequiresZeroHeuristic
            ? DtFindPathOption.ZeroScale
            : DtFindPathOption.NoOption;
        var status = query.FindPath(startRef, endRef, startPos, endPos, filter, buffer, out var count, buffer.Length, option);
        corridor   = [.. buffer.AsSpan(0, count).ToArray()];
        return status;
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

    private static PlannerPathSegment BuildGroundMeshCorridorSegment
    (
        Vector3                  traversalStartPosition,
        Vector3                  projectedStart,
        Vector3                  finalDestination,
        IReadOnlyList<long>      corridor
    ) =>
        new()
        {
            MovementMode           = MovementMode.Ground,
            SegmentKind            = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl   = false,
            ReachabilitySource     = PathReachabilitySource.Mesh,
            GeometryKind           = PlannerSegmentGeometryKind.MeshCorridor,
            TraversalStartPosition = traversalStartPosition,
            StartPosition          = projectedStart,
            EndPosition            = finalDestination,
            Corridor               = [.. corridor]
        };

    internal sealed class GroundPathDiagnosticsSnapshot
    {
        public required long GroundQueries               { get; init; }
        public required long FailedQueries               { get; init; }
        public required long PartialQueries              { get; init; }
        public required long ReachedWithinRangeQueries   { get; init; }
        public required long GeneratedClimbLinksAccepted { get; init; }
        public required long GeneratedJumpLinksAccepted  { get; init; }
    }

    internal sealed class GroundAreaCostFilter
    (
        Navmesh navmeshData,
        bool excludeUnreachable = true
    ) : IDtQueryFilter
    {
        private readonly DtQueryDefaultFilter filter = new((int)NavmeshPolyFlags.AllTraversable, excludeUnreachable ? (int)NavmeshPolyFlags.Unreachable : 0, CreatePassCosts());

        public bool RequiresZeroHeuristic => navmeshData.HasHeuristicSensitiveOffMeshLinks;

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
        {
            var area = (NavmeshArea)curPoly.GetArea();
            if (area == NavmeshArea.Null)
                return float.MaxValue;

            if (area == NavmeshArea.Ground)
                return Vector3.Distance(pa.RecastToSystem(), pb.RecastToSystem());

            var kind = NavmeshLinkTraversalProfiles.ResolveKind(area);
            if (kind == null)
                return Vector3.Distance(pa.RecastToSystem(), pb.RecastToSystem());

            var traversalProfile = navmeshData.TryGetOffMeshLink(curRef, out var link)
                ? link.TraversalProfile
                : null;
            return NavmeshLinkTraversalProfiles.EstimateCost(pa.RecastToSystem(), pb.RecastToSystem(), kind.Value, traversalProfile);
        }

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => filter.PassFilter(refs, tile, poly);

        public bool TryGetRegisteredTraversalProfile(long polyRef, out NavmeshLinkTraversalProfile? traversalProfile)
        {
            if (navmeshData.TryGetOffMeshLink(polyRef, out var link))
            {
                traversalProfile = link.TraversalProfile;
                return true;
            }

            traversalProfile = null;
            return false;
        }

        private static float[] CreatePassCosts()
        {
            var costs = new float[DT_MAX_AREAS];
            Array.Fill(costs, 1f);
            costs[(int)NavmeshArea.Null] = float.MaxValue;
            return costs;
        }
    }
}
