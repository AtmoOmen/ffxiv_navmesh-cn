using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Extensions;
using vnavmesh.Movement.Drivers;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Enums;
using vnavmesh.Query.Ground;
using vnavmesh.Query.Models;

namespace vnavmesh.Query;

internal sealed class PathPostprocessor
(
    Func<DtNavMeshQuery> getMeshQuery,
    Func<IDtQueryFilter> getGroundFilter
)
{
    private DtNavMeshQuery MeshQuery    => getMeshQuery();
    private IDtQueryFilter GroundFilter => getGroundFilter();

    public PostprocessedPath Process
    (
        PlannerResult     result,
        CancellationToken cancel
    )
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments)
            segments.Add(BuildSegment(segment, cancel));

        return BuildPath(result, segments);
    }

    public PostprocessedPath ProcessStraightPath
    (
        PlannerResult     result,
        CancellationToken cancel,
        int               straightPathOptions = 0
    )
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments)
            segments.Add(BuildStraightPathSegment(segment, straightPathOptions, cancel));

        return BuildPath(result, segments);
    }

    private static PostprocessedPath BuildPath
    (
        PlannerResult                           result,
        IReadOnlyList<PostprocessedPathSegment> segments
    ) =>
        new()
        {
            Status               = result.Status,
            RequestedMode        = result.RequestedMode,
            RequestedDestination = result.RequestedDestination,
            FinalDestination     = result.FinalDestination,
            DestinationTolerance = result.DestinationTolerance,
            Segments             = segments
        };

    private PostprocessedPathSegment BuildSegment
    (
        PlannerPathSegment segment,
        CancellationToken  cancel
    )
    {
        cancel.ThrowIfCancellationRequested();

        var groundCorridor = segment is { MovementMode: MovementMode.Ground, GeometryKind: PlannerSegmentGeometryKind.MeshCorridor } ?
                                 BuildGroundCorridor(segment, cancel) :
                                 segment.GroundCorridor;
        var waypoints = segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => BuildMeshWaypoints(segment, groundCorridor),
            PlannerSegmentGeometryKind.DiscretePoints => BuildDiscreteWaypoints(segment),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };

        return BuildSegment(segment, waypoints, groundCorridor);
    }

    private PostprocessedPathSegment BuildStraightPathSegment
    (
        PlannerPathSegment segment,
        int                straightPathOptions,
        CancellationToken  cancel
    )
    {
        cancel.ThrowIfCancellationRequested();

        var groundCorridor = segment is { MovementMode: MovementMode.Ground, GeometryKind: PlannerSegmentGeometryKind.MeshCorridor } ?
                                 BuildGroundCorridor(segment, straightPathOptions, cancel) :
                                 segment.GroundCorridor;
        var waypoints = segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => BuildRawStraightPathWaypoints(segment, [.. segment.Corridor], straightPathOptions),
            PlannerSegmentGeometryKind.DiscretePoints => BuildRawDiscreteWaypoints(segment),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };

        return BuildSegment(segment, waypoints, groundCorridor);
    }

    private static PostprocessedPathSegment BuildSegment
    (
        PlannerPathSegment     segment,
        IReadOnlyList<Vector3> waypoints,
        GroundCorridorPayload? groundCorridor
    ) =>
        new()
        {
            MovementMode         = segment.MovementMode,
            SegmentKind          = segment.SegmentKind,
            AllowVerticalControl = segment.AllowVerticalControl,
            StartPosition        = segment.StartPosition,
            CompletionTolerance  = 0,
            Waypoints            = waypoints,
            GroundCorridor       = groundCorridor
        };

    private List<Vector3> BuildMeshWaypoints
    (
        PlannerPathSegment     segment,
        GroundCorridorPayload? groundCorridor
    )
    {
        if (segment.Corridor.Count == 0)
            return [];

        return groundCorridor != null ?
                   [.. groundCorridor.Corners.Select(corner => corner.Position)] :
                   BuildStraightPathWaypoints(segment, [.. segment.Corridor]);
    }

    private List<Vector3> BuildStraightPathWaypoints
    (
        PlannerPathSegment segment,
        long[]             corridor
    )
    {
        var (straightPath, straightPathCount) = QueryStraightPath(segment, corridor, 0);
        return DeduplicateWaypoints(straightPath.AsSpan(0, straightPathCount).ToArray().Select(point => point.pos.ToSystem()));
    }

    private List<Vector3> BuildRawStraightPathWaypoints
    (
        PlannerPathSegment segment,
        long[]             corridor,
        int                straightPathOptions
    )
    {
        var (straightPath, straightPathCount) = QueryStraightPath(segment, corridor, straightPathOptions);
        return [.. straightPath.AsSpan(0, straightPathCount).ToArray().Select(point => point.pos.ToSystem())];
    }

    private GroundCorridorPayload? BuildGroundCorridor
    (
        PlannerPathSegment segment,
        CancellationToken  cancel
    ) =>
        BuildGroundCorridor(segment, 0, true, cancel);

    private GroundCorridorPayload? BuildGroundCorridor
    (
        PlannerPathSegment segment,
        int                straightPathOptions,
        CancellationToken  cancel
    ) =>
        BuildGroundCorridor(segment, straightPathOptions, false, cancel);

    private GroundCorridorPayload? BuildGroundCorridor
    (
        PlannerPathSegment segment,
        int                straightPathOptions,
        bool               optimize,
        CancellationToken  cancel
    )
    {
        if (segment.Corridor.Count == 0)
            return null;

        var corridor = segment.Corridor.ToArray();
        var corners = BuildGroundCorners
            (segment, corridor, straightPathOptions, optimize, cancel, out var initialSourceIndex, out var rawCorners);
        return new()
        {
            PolyRefs           = corridor,
            Target             = segment.EndPosition,
            InitialCornerIndex = ResolveInitialCornerIndex(corners, initialSourceIndex),
            RawCorners         = rawCorners,
            Corners            = corners,
            LinkMarkers        = BuildGroundLinkMarkers(corners)
        };
    }

    private IReadOnlyList<GroundPathCorner> BuildGroundCorners
    (
        PlannerPathSegment                  segment,
        long[]                              corridor,
        int                                 straightPathOptions,
        bool                                optimize,
        CancellationToken                   cancel,
        out int                             initialSourceIndex,
        out IReadOnlyList<GroundPathCorner> rawCorners
    )
    {
        var (straightPath, straightPathCount) = QueryStraightPath(segment, corridor, straightPathOptions);
        initialSourceIndex                    = ResolveInitialSourceIndex(segment, straightPath, straightPathCount);

        List<GroundPathCorner> result = new(straightPathCount);

        for (var i = 0; i < straightPathCount; ++i)
        {
            cancel.ThrowIfCancellationRequested();

            var rawCorner = straightPath[i];
            var area      = ResolveArea(corridor, rawCorner.refs);
            var corner = new GroundPathCorner
            (
                rawCorner.pos.ToSystem(),
                rawCorner.refs,
                rawCorner.flags,
                area,
                ResolveLinkKind(area, rawCorner.flags),
                i
            );

            if (result.Count                                                  == 0                             ||
                Vector3.DistanceSquared(result[^1].Position, corner.Position) > DUPLICATE_WAYPOINT_DISTANCE_SQ ||
                result[^1].Area                                               != corner.Area                   ||
                result[^1].LinkKind                                           != corner.LinkKind               ||
                result[^1].StraightPathFlags                                  != corner.StraightPathFlags) result.Add(corner);
        }

        rawCorners = result;
        return optimize ?
                   new GroundPathOptimizer(MeshQuery, GroundFilter).Optimize(result, initialSourceIndex, cancel) :
                   result;
    }

    private (DtStraightPath[] StraightPath, int Count) QueryStraightPath
    (
        PlannerPathSegment segment,
        long[]             corridor,
        int                straightPathOptions
    )
    {
        var straightPath = new DtStraightPath[MAX_STRAIGHT_PATH_POINTS];
        var status = MeshQuery.FindStraightPath
        (
            segment.StartPosition.ToRecast(),
            segment.EndPosition.ToRecast(),
            corridor,
            corridor.Length,
            straightPath,
            out var straightPathCount,
            straightPath.Length,
            straightPathOptions
        );
        if (status.Failed())
            throw new InvalidOperationException("地面路径后处理失败：无法生成 straight path");

        return (straightPath, straightPathCount);
    }

    private static int ResolveInitialSourceIndex
    (
        PlannerPathSegment segment,
        DtStraightPath[]   straightPath,
        int                straightPathCount
    )
    {
        if (segment.MovementMode != MovementMode.Ground || segment.SegmentKind != MovementSegmentKind.GroundTraverse || straightPathCount <= 0)
            return 0;

        List<Vector3> rawWaypoints = new(straightPathCount);
        for (var i = 0; i < straightPathCount; ++i)
            rawWaypoints.Add(straightPath[i].pos.ToSystem());

        return Math.Clamp
        (
            WaypointProgression.ConsumeGroundWaypoints
            (
                segment.TraversalStartPosition,
                segment.TraversalStartPosition,
                segment.StartPosition,
                rawWaypoints
            ),
            0,
            straightPathCount
        );
    }

    private static int ResolveInitialCornerIndex
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex
    )
    {
        if (corners.Count == 0)
            return 0;

        for (var i = 0; i < corners.Count; ++i)
            if (corners[i].SourceIndex >= initialSourceIndex)
                return i;

        return corners.Count - 1;
    }

    private IReadOnlyList<GroundLinkMarker> BuildGroundLinkMarkers
    (
        IReadOnlyList<GroundPathCorner> corners
    )
    {
        List<GroundLinkMarker> markers = [];

        for (var i = 0; i < corners.Count; ++i)
        {
            if (corners[i].LinkKind is not { } kind)
                continue;

            var traversalProfile = ResolveTraversalProfile(corners[i].PolyRef, kind);
            markers.Add
            (
                new
                (
                    i,
                    corners[i].Position,
                    corners[i].PolyRef,
                    kind,
                    traversalProfile,
                    NavmeshLinkTraversalProfiles.EstimateCost(corners[i].Position, ResolveLinkEndPosition(corners, i), kind, traversalProfile)
                )
            );
        }

        return markers;
    }

    private NavmeshArea ResolveArea
    (
        long[] corridor,
        long   polyRef
    )
    {
        var resolvedRef = polyRef != 0 ?
                              polyRef :
                              corridor[^1];
        MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(resolvedRef, out _, out var poly);
        return (NavmeshArea)poly.GetArea();
    }

    private static NavmeshOffMeshKind? ResolveLinkKind
    (
        NavmeshArea area,
        byte        straightPathFlags
    )
    {
        if ((straightPathFlags & DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION) == 0)
            return null;

        return area switch
        {
            NavmeshArea.GeneratedClimbDown => NavmeshOffMeshKind.GeneratedClimbDown,
            NavmeshArea.GeneratedEdgeJump  => NavmeshOffMeshKind.GeneratedEdgeJump,
            NavmeshArea.ManualOffMesh      => NavmeshOffMeshKind.ManualOffMesh,
            NavmeshArea.Shortcut           => NavmeshOffMeshKind.Shortcut,
            NavmeshArea.Teleport           => NavmeshOffMeshKind.Teleport,
            NavmeshArea.ClientPath         => NavmeshOffMeshKind.ClientPath,
            _                              => null
        };
    }

    private NavmeshLinkTraversalProfile? ResolveTraversalProfile
    (
        long                polyRef,
        NavmeshOffMeshKind? kind
    )
    {
        if (kind == null)
            return null;

        return GroundFilter is NavmeshGroundQuery.GroundAreaCostFilter groundFilter &&
               groundFilter.TryGetRegisteredTraversalProfile(polyRef, out var traversalProfile) ?
                   traversalProfile :
                   null;
    }

    private static Vector3 ResolveLinkEndPosition
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             index
    ) =>
        index + 1 < corners.Count ?
            corners[index + 1].Position :
            corners[index].Position;

    private static List<Vector3> BuildRawDiscreteWaypoints
    (
        PlannerPathSegment segment
    ) => [.. segment.Points];

    private static List<Vector3> BuildDiscreteWaypoints
    (
        PlannerPathSegment segment
    )
    {
        if (segment.MovementMode != MovementMode.Flight)
            return DeduplicateWaypoints(segment.Points);

        List<Vector3> deduplicated = [];

        foreach (var point in segment.Points)
        {
            if (deduplicated.Count == 0 || Vector3.DistanceSquared(deduplicated[^1], point) > DUPLICATE_WAYPOINT_DISTANCE_SQ)
                deduplicated.Add(point);
        }

        return SimplifyFlightWaypoints(deduplicated);
    }

    private static List<Vector3> SimplifyFlightWaypoints
    (
        List<Vector3> points
    )
    {
        if (points.Count <= 2)
            return [.. points];

        List<Vector3> simplified = [points[0]];

        for (var i = 1; i < points.Count - 1; ++i)
        {
            var previous = simplified[^1];
            var current  = points[i];
            var next     = points[i + 1];
            if (!IsRedundantFlightWaypoint(previous, current, next))
                simplified.Add(current);
        }

        simplified.Add(points[^1]);
        return simplified;
    }

    private static bool IsRedundantFlightWaypoint
    (
        Vector3 previous,
        Vector3 current,
        Vector3 next
    )
    {
        if (Vector3.DistanceSquared(previous, current) <= DUPLICATE_WAYPOINT_DISTANCE_SQ ||
            Vector3.DistanceSquared(current,  next)    <= DUPLICATE_WAYPOINT_DISTANCE_SQ ||
            Vector3.DistanceSquared(previous, next)    <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;
        if (NeedsFlightVerticalPreservation(previous, next))
            return false;

        return DistanceToLineSegment(current, previous, next) <= COLLINEAR_WAYPOINT_TOLERANCE;
    }

    private static bool NeedsFlightVerticalPreservation
    (
        Vector3 previous,
        Vector3 next
    )
    {
        var verticalDelta = MathF.Abs(next.Y - previous.Y);
        if (verticalDelta <= FLIGHT_DESCENT_PRESERVE_MIN_DROP)
            return false;

        var horizontalDistance = HorizontalDistanceXZ(previous, next);
        if (horizontalDistance <= FLIGHT_DESCENT_PRESERVE_NEAR_VERTICAL_HORIZONTAL)
            return true;

        return verticalDelta / horizontalDistance >= FLIGHT_DESCENT_PRESERVE_MAX_SLOPE;
    }

    private static float DistanceToLineSegment
    (
        Vector3 value,
        Vector3 start,
        Vector3 end
    )
    {
        var segment       = end - start;
        var lengthSquared = segment.LengthSquared();
        if (lengthSquared <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return Vector3.Distance(value, start);

        var progress  = Math.Clamp(Vector3.Dot(value - start, segment) / lengthSquared, 0f, 1f);
        var projected = start + (progress * segment);
        return Vector3.Distance(value, projected);
    }

    private static float HorizontalDistanceXZ
    (
        Vector3 left,
        Vector3 right
    )
    {
        var dx = left.X             - right.X;
        var dz = left.Z             - right.Z;
        return MathF.Sqrt((dx * dx) + (dz * dz));
    }

    private static List<Vector3> DeduplicateWaypoints
    (
        IEnumerable<Vector3> points
    )
    {
        List<Vector3> result = [];

        foreach (var point in points)
        {
            if (result.Count == 0 || Vector3.DistanceSquared(result[^1], point) > DUPLICATE_WAYPOINT_DISTANCE_SQ)
                result.Add(point);
        }

        return result;
    }

    private const int   MAX_STRAIGHT_PATH_POINTS                         = 4097;
    private const float DUPLICATE_WAYPOINT_DISTANCE_SQ                   = 0.000001f;
    private const float COLLINEAR_WAYPOINT_TOLERANCE                     = 0.01f;
    private const float FLIGHT_DESCENT_PRESERVE_MIN_DROP                 = 1.00f;
    private const float FLIGHT_DESCENT_PRESERVE_NEAR_VERTICAL_HORIZONTAL = 1.20f;
    private const float FLIGHT_DESCENT_PRESERVE_MAX_SLOPE                = 0.90f;
}
