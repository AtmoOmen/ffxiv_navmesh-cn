using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Planning;

internal sealed class PathPostprocessor
(
    DtNavMeshQuery meshQuery
)
{
    private const float DUPLICATE_WAYPOINT_DISTANCE_SQ = 0.000001f;
    private const float COLLINEAR_WAYPOINT_TOLERANCE   = 0.01f;
    private const int   MAX_SMOOTH_PATH_POINTS         = 102400;

    public PostprocessedPath Process(PlannerResult result, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments) segments.Add(BuildSegment(segment, useStringPulling, cancel));

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

    private PostprocessedPathSegment BuildSegment(PlannerPathSegment segment, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        var groundCorridor = segment.MovementMode == MovementMode.Ground && segment.GeometryKind == PlannerSegmentGeometryKind.MeshCorridor
                                 ? BuildGroundCorridor(segment)
                                 : segment.GroundCorridor;
        var waypoints = segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => BuildMeshWaypoints(segment, groundCorridor, useStringPulling),
            PlannerSegmentGeometryKind.DiscretePoints => BuildDiscreteWaypoints(segment),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };

        return new()
        {
            MovementMode         = segment.MovementMode,
            SegmentKind          = segment.SegmentKind,
            AllowVerticalControl = segment.AllowVerticalControl,
            StartPosition        = segment.StartPosition,
            CompletionTolerance  = 0,
            GeometryOwnership    = PathGeometryOwnership.Postprocessor,
            ReachabilitySource   = segment.ReachabilitySource,
            Waypoints            = waypoints,
            GroundCorridor       = groundCorridor
        };
    }

    private List<Vector3> BuildMeshWaypoints(PlannerPathSegment segment, GroundCorridorPayload? groundCorridor, bool useStringPulling)
    {
        if (segment.Corridor.Count == 0)
            return [];

        if (groundCorridor != null)
        {
            return useStringPulling
                       ? [.. groundCorridor.Corners.Select(c => c.Position)]
                       : DeduplicateWaypoints
                           (segment.Corridor.Select(r => meshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(segment.EndPosition));
        }

        if (useStringPulling)
            return BuildStraightPathWaypoints(segment, [.. segment.Corridor]);

        return DeduplicateWaypoints(segment.Corridor.Select(r => meshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(segment.EndPosition));
    }

    private List<Vector3> BuildStraightPathWaypoints(PlannerPathSegment segment, long[] corridor)
    {
        var straightPath = new DtStraightPath[MAX_SMOOTH_PATH_POINTS];
        var straightStatus = meshQuery.FindStraightPath
        (
            segment.StartPosition.SystemToRecast(),
            segment.EndPosition.SystemToRecast(),
            corridor,
            corridor.Length,
            straightPath,
            out var straightPathCount,
            straightPath.Length,
            DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS
        );
        if (straightStatus.Failed())
            throw new InvalidOperationException("地面路径后处理失败：无法生成平滑路径");

        return DeduplicateWaypoints(straightPath.AsSpan(0, straightPathCount).ToArray().Select(p => p.pos.RecastToSystem()));
    }

    private GroundCorridorPayload? BuildGroundCorridor(PlannerPathSegment segment)
    {
        if (segment.Corridor.Count == 0)
            return null;

        var corners = BuildGroundCorners(segment, [.. segment.Corridor]);
        return new()
        {
            PolyRefs    = [.. segment.Corridor],
            Target      = segment.EndPosition,
            Corners     = corners,
            LinkMarkers = BuildGroundLinkMarkers(corners)
        };
    }

    private IReadOnlyList<GroundPathCorner> BuildGroundCorners(PlannerPathSegment segment, long[] corridor)
    {
        var straightPath = new DtStraightPath[MAX_SMOOTH_PATH_POINTS];
        var straightStatus = meshQuery.FindStraightPath
        (
            segment.StartPosition.SystemToRecast(),
            segment.EndPosition.SystemToRecast(),
            corridor,
            corridor.Length,
            straightPath,
            out var straightPathCount,
            straightPath.Length,
            DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS
        );
        if (straightStatus.Failed())
            throw new InvalidOperationException("地面路径后处理失败：无法提取 corridor 角点");

        List<GroundPathCorner> result = [];

        for (var i = 0; i < straightPathCount; i++)
        {
            var rawCorner = straightPath[i];
            var position  = rawCorner.pos.RecastToSystem();
            var area      = ResolveArea(corridor, rawCorner.refs);
            var linkKind  = ResolveLinkKind(area, rawCorner.flags);
            var corner = new GroundPathCorner
            (
                position,
                rawCorner.refs,
                rawCorner.flags,
                area,
                linkKind
            );

            if (result.Count == 0)
            {
                result.Add(corner);
                continue;
            }

            var previous = result[^1];
            if (Vector3.DistanceSquared(previous.Position, corner.Position) > DUPLICATE_WAYPOINT_DISTANCE_SQ ||
                previous.Area                                               != corner.Area                   ||
                previous.LinkKind                                           != corner.LinkKind               ||
                previous.StraightPathFlags                                  != corner.StraightPathFlags) result.Add(corner);
        }

        return result;
    }

    private static IReadOnlyList<GroundLinkMarker> BuildGroundLinkMarkers(IReadOnlyList<GroundPathCorner> corners)
    {
        List<GroundLinkMarker> markers = [];

        for (var i = 0; i < corners.Count; i++)
        {
            if (corners[i].LinkKind is not { } kind)
                continue;

            markers.Add(new(i, corners[i].Position, corners[i].PolyRef, kind));
        }

        return markers;
    }

    private NavmeshArea ResolveArea(long[] corridor, long polyRef)
    {
        var resolvedRef = polyRef != 0 ? polyRef : corridor[^1];
        meshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(resolvedRef, out _, out var poly);
        return (NavmeshArea)poly.GetArea();
    }

    private static NavmeshOffMeshKind? ResolveLinkKind(NavmeshArea area, byte straightPathFlags)
    {
        if ((straightPathFlags & DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION) == 0)
            return null;

        return area switch
        {
            NavmeshArea.GeneratedClimbDown => NavmeshOffMeshKind.GeneratedClimbDown,
            NavmeshArea.GeneratedEdgeJump  => NavmeshOffMeshKind.GeneratedEdgeJump,
            NavmeshArea.ManualOffMesh      => NavmeshOffMeshKind.ManualOffMesh,
            NavmeshArea.Teleport           => NavmeshOffMeshKind.Teleport,
            _                              => null
        };
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
        var segment       = end - start;
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
