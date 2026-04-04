using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Planning;

internal sealed class PathPostprocessor
{
    private const float DuplicateWaypointDistanceSq = 0.000001f;
    private const float CollinearWaypointTolerance  = 0.01f;
    private readonly DtNavMeshQuery _meshQuery;
    private readonly VoxelMap?      _volume;

    public PathPostprocessor(DtNavMeshQuery meshQuery, VoxelMap? volume = null)
    {
        _meshQuery = meshQuery;
        _volume    = volume;
    }

    public PostprocessedPath Process(PlannerResult result, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments)
        {
            var waypoints = BuildWaypoints(segment, useStringPulling, cancel);
            segments.Add
            (
                new()
                {
                    MovementMode         = segment.MovementMode,
                    SegmentKind          = segment.SegmentKind,
                    AllowVerticalControl = segment.AllowVerticalControl,
                    StartPosition        = segment.StartPosition,
                    CompletionTolerance  = 0,
                    GeometryOwnership    = PathGeometryOwnership.Postprocessor,
                    ReachabilitySource   = segment.ReachabilitySource,
                    Waypoints            = waypoints
                }
            );
        }

        return new()
        {
            Status               = result.Status,
            RequestedMode        = result.RequestedMode,
            RequestedDestination = result.RequestedDestination,
            FinalDestination     = result.FinalDestination,
            DestinationTolerance = result.DestinationTolerance,
            Segments             = segments
        };
    }

    public PostprocessedPath CreateExternalInputPath
        (List<Vector3> waypoints, MovementMode requestedMode, Vector3 requestedDestination, float destinationTolerance)
    {
        var segmentKind = requestedMode == MovementMode.Flight ? MovementSegmentKind.FlightTraverse : MovementSegmentKind.GroundTraverse;
        var normalized  = DeduplicateWaypoints(waypoints);

        return new()
        {
            Status               = PathfindStatus.Complete,
            RequestedMode        = requestedMode,
            RequestedDestination = requestedDestination,
            FinalDestination     = normalized.Count > 0 ? normalized[^1] : requestedDestination,
            DestinationTolerance = destinationTolerance,
            Segments =
            [
                new()
                {
                    MovementMode         = requestedMode,
                    SegmentKind          = segmentKind,
                    AllowVerticalControl = requestedMode == MovementMode.Flight,
                    StartPosition        = normalized.Count > 0 ? normalized[0] : requestedDestination,
                    CompletionTolerance  = 0,
                    GeometryOwnership    = PathGeometryOwnership.ExternalInput,
                    ReachabilitySource   = PathReachabilitySource.ExternalInput,
                    Waypoints            = normalized
                }
            ]
        };
    }

    private IReadOnlyList<Vector3> BuildWaypoints(PlannerPathSegment segment, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        return segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => BuildMeshWaypoints(segment, useStringPulling),
            PlannerSegmentGeometryKind.DiscretePoints => BuildDiscreteWaypoints(segment, useStringPulling, cancel),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };
    }

    private IReadOnlyList<Vector3> BuildMeshWaypoints(PlannerPathSegment segment, bool useStringPulling)
    {
        if (segment.Corridor.Count == 0)
            return [];

        if (useStringPulling)
        {
            var straightPath = new List<DtStraightPath>();
            var straightStatus = _meshQuery.FindStraightPath
                (segment.StartPosition.SystemToRecast(), segment.EndPosition.SystemToRecast(), [.. segment.Corridor], ref straightPath, 1024, 0);
            if (straightStatus.Failed())
                throw new InvalidOperationException("地面路径后处理失败：无法生成 string-pulling 路径");

            return DeduplicateWaypoints(straightPath.Select(p => p.pos.RecastToSystem()));
        }

        return DeduplicateWaypoints(segment.Corridor.Select(r => _meshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(segment.EndPosition));
    }

    private IReadOnlyList<Vector3> BuildDiscreteWaypoints(PlannerPathSegment segment, bool useStringPulling, CancellationToken cancel)
    {
        var deduplicated = DeduplicateWaypoints(segment.Points);
        if (segment.MovementMode != MovementMode.Flight)
            return deduplicated;

        var algorithmSimplified = useStringPulling && segment.ReachabilitySource == PathReachabilitySource.Volume && _volume != null
            ? SimplifyFlightWaypointsByVisibilityGraph(deduplicated, cancel)
            : [.. deduplicated];
        var simplified = SimplifyFlightWaypoints(algorithmSimplified);
        Service.Log.Debug
        (
            $"[算路] 飞行路径后处理：输入点 = {segment.Points.Count}，去重点 = {deduplicated.Count}，可见性图简化 = {algorithmSimplified.Count}，输出点 = {simplified.Count}"
        );
        return simplified;
    }

    private List<Vector3> SimplifyFlightWaypointsByVisibilityGraph(IReadOnlyList<Vector3> points, CancellationToken cancel)
    {
        if (_volume == null || points.Count <= 2)
            return [.. points];

        var anchors      = ResolveShortcutAnchors(points);
        var waypointPath = SolveMinimumLinkVisibilityPath(anchors, cancel);
        return [.. waypointPath.Select(entry => entry.point)];
    }

    private List<(Vector3 point, ulong voxel)> ResolveShortcutAnchors(IReadOnlyList<Vector3> points)
    {
        List<(Vector3 point, ulong voxel)> resolved = new(points.Count);

        foreach (var point in points)
        {
            var located = _volume!.FindLeafVoxel(point);
            if (!located.empty)
                throw new InvalidOperationException($"飞行路径后处理失败：路径点 {point:f3} 不在空体素内");

            resolved.Add((point, located.voxel));
        }

        return resolved;
    }

    private bool HasLineOfSight((Vector3 point, ulong voxel) from, (Vector3 point, ulong voxel) to) =>
        _volume != null && VoxelSearch.LineOfSight(_volume, from.voxel, to.voxel, from.point, to.point);

    private List<(Vector3 point, ulong voxel)> SolveMinimumLinkVisibilityPath(IReadOnlyList<(Vector3 point, ulong voxel)> anchors, CancellationToken cancel)
    {
        var count         = anchors.Count;
        var bestSegments  = GC.AllocateUninitializedArray<int>(count);
        var bestDistance  = GC.AllocateUninitializedArray<float>(count);
        var nextIndex     = GC.AllocateUninitializedArray<int>(count);

        Array.Fill(bestSegments, int.MaxValue);
        Array.Fill(bestDistance, float.PositiveInfinity);
        Array.Fill(nextIndex, -1);

        bestSegments[^1] = 0;
        bestDistance[^1] = 0;
        nextIndex[^1]    = count - 1;

        for (var i = count - 2; i >= 0; i--)
        {
            cancel.ThrowIfCancellationRequested();

            for (var j = count - 1; j > i; j--)
            {
                if (bestSegments[j] == int.MaxValue)
                    continue;
                if (j > i + 1 && !HasLineOfSight(anchors[i], anchors[j]))
                    continue;

                var candidateSegments = 1 + bestSegments[j];
                var candidateDistance = Vector3.Distance(anchors[i].point, anchors[j].point) + bestDistance[j];
                if (!IsBetterVisibilityCandidate(candidateSegments, candidateDistance, bestSegments[i], bestDistance[i]))
                    continue;

                bestSegments[i] = candidateSegments;
                bestDistance[i] = candidateDistance;
                nextIndex[i]    = j;
            }

            if (nextIndex[i] >= 0)
                continue;

            nextIndex[i]    = i + 1;
            bestSegments[i] = 1 + bestSegments[i + 1];
            bestDistance[i] = Vector3.Distance(anchors[i].point, anchors[i + 1].point) + bestDistance[i + 1];
        }

        List<(Vector3 point, ulong voxel)> path = [];
        for (var index = 0; index >= 0 && index < count;)
        {
            path.Add(anchors[index]);
            var next = nextIndex[index];
            if (next <= index)
                break;
            index = next;
        }

        if (path.Count == 0 || path[^1].voxel != anchors[^1].voxel || path[^1].point != anchors[^1].point)
            path.Add(anchors[^1]);

        return path;
    }

    private static bool IsBetterVisibilityCandidate(int candidateSegments, float candidateDistance, int currentSegments, float currentDistance)
    {
        if (candidateSegments != currentSegments)
            return candidateSegments < currentSegments;

        return candidateDistance + DuplicateWaypointDistanceSq < currentDistance;
    }

    private static List<Vector3> SimplifyFlightWaypoints(IReadOnlyList<Vector3> points)
    {
        if (points.Count <= 2)
            return [.. points];

        List<Vector3> simplified = [points[0]];

        for (var i = 1; i < points.Count - 1; i++)
        {
            var previous = simplified[^1];
            var current  = points[i];
            var next     = points[i + 1];
            if (IsRedundantFlightWaypoint(previous, current, next))
                continue;

            simplified.Add(current);
        }

        simplified.Add(points[^1]);
        return simplified;
    }

    private static bool IsRedundantFlightWaypoint(Vector3 previous, Vector3 current, Vector3 next)
    {
        if (Vector3.DistanceSquared(previous, current) <= DuplicateWaypointDistanceSq)
            return true;
        if (Vector3.DistanceSquared(current, next) <= DuplicateWaypointDistanceSq)
            return true;
        if (Vector3.DistanceSquared(previous, next) <= DuplicateWaypointDistanceSq)
            return true;

        return DistanceToLineSegment(current, previous, next) <= CollinearWaypointTolerance;
    }

    private static float DistanceToLineSegment(Vector3 value, Vector3 start, Vector3 end)
    {
        var segment = end - start;
        var lengthSquared = segment.LengthSquared();
        if (lengthSquared <= DuplicateWaypointDistanceSq)
            return Vector3.Distance(value, start);

        var progress = Math.Clamp(Vector3.Dot(value - start, segment) / lengthSquared, 0f, 1f);
        var projected = start + progress * segment;
        return Vector3.Distance(value, projected);
    }

    private static List<Vector3> DeduplicateWaypoints(IEnumerable<Vector3> points)
    {
        List<Vector3> result = [];

        foreach (var point in points)
        {
            if (result.Count == 0 || Vector3.DistanceSquared(result[^1], point) > DuplicateWaypointDistanceSq)
                result.Add(point);
        }

        return result;
    }
}
