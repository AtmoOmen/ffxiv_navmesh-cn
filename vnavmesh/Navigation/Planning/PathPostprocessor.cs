using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Movement.Planning;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Planning;

internal sealed class PathPostprocessor
(
    DtNavMeshQuery meshQuery
)
{
    private const float DUPLICATE_WAYPOINT_DISTANCE_SQ = 0.000001f;
    private const float COLLINEAR_WAYPOINT_TOLERANCE  = 0.01f;

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

    private List<Vector3> BuildWaypoints(PlannerPathSegment segment, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        return segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => BuildMeshWaypoints(segment, useStringPulling),
            PlannerSegmentGeometryKind.DiscretePoints => BuildDiscreteWaypoints(segment),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };
    }

    private List<Vector3> BuildMeshWaypoints(PlannerPathSegment segment, bool useStringPulling)
    {
        if (segment.Corridor.Count == 0)
            return [];

        if (useStringPulling)
        {
            var corridor = segment.Corridor.ToArray();
            var straightPath = new DtStraightPath[1024];
            var straightStatus = meshQuery.FindStraightPath
                (segment.StartPosition.SystemToRecast(), segment.EndPosition.SystemToRecast(), corridor, corridor.Length, straightPath, out var straightPathCount, straightPath.Length, 0);
            if (straightStatus.Failed())
                throw new InvalidOperationException("地面路径后处理失败：无法生成 string-pulling 路径");

            return DeduplicateWaypoints(straightPath.AsSpan(0, straightPathCount).ToArray().Select(p => p.pos.RecastToSystem()));
        }

        return DeduplicateWaypoints(segment.Corridor.Select(r => meshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(segment.EndPosition));
    }

    private static List<Vector3> BuildDiscreteWaypoints(PlannerPathSegment segment)
    {
        var deduplicated = DeduplicateWaypoints(segment.Points);
        if (segment.MovementMode != MovementMode.Flight)
            return deduplicated;

        var simplified = SimplifyFlightWaypoints(deduplicated);
        return simplified;
    }

    private static List<Vector3> SimplifyFlightWaypoints(List<Vector3> points)
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
        if (Vector3.DistanceSquared(previous, current) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;
        if (Vector3.DistanceSquared(current, next) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;
        if (Vector3.DistanceSquared(previous, next) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;

        return DistanceToLineSegment(current, previous, next) <= COLLINEAR_WAYPOINT_TOLERANCE;
    }

    private static float DistanceToLineSegment(Vector3 value, Vector3 start, Vector3 end)
    {
        var segment = end - start;
        var lengthSquared = segment.LengthSquared();
        if (lengthSquared <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return Vector3.Distance(value, start);

        var progress  = Math.Clamp(Vector3.Dot(value - start, segment) / lengthSquared, 0f, 1f);
        var projected = start + progress * segment;
        return Vector3.Distance(value, projected);
    }

    private static List<Vector3> DeduplicateWaypoints(IEnumerable<Vector3> points)
    {
        List<Vector3> result = [];

        foreach (var point in points)
        {
            if (result.Count == 0 || Vector3.DistanceSquared(result[^1], point) > DUPLICATE_WAYPOINT_DISTANCE_SQ)
                result.Add(point);
        }

        return result;
    }
}
