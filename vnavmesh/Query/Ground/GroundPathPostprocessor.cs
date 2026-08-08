using System.Buffers;
using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Extensions;
using vnavmesh.Movement.Drivers;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Enums;
using vnavmesh.Query.Ground.Models;
using vnavmesh.Query.Models;

namespace vnavmesh.Query.Ground;

internal sealed class GroundPathPostprocessor
(
    Func<DtNavMeshQuery> getMeshQuery,
    Func<IDtQueryFilter> getGroundFilter
)
{
    private DtNavMeshQuery MeshQuery    => getMeshQuery();
    private IDtQueryFilter GroundFilter => getGroundFilter();

    internal PostprocessedPathSegment ProcessSegment
    (
        PlannerPathSegment segment,
        CancellationToken  cancel
    )
    {
        cancel.ThrowIfCancellationRequested();

        var groundCorridor = segment is { MovementMode: MovementMode.Ground, GeometryKind: PlannerSegmentGeometryKind.MeshCorridor } ?
                                 BuildGroundCorridor(segment, cancel) :
                                 segment.GroundCorridor;
        var waypoints = BuildMeshWaypoints(segment, groundCorridor);
        return BuildSegment(segment, waypoints, groundCorridor);
    }

    internal PostprocessedPathSegment ProcessStraightPathSegment
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
        var waypoints = BuildRawStraightPathWaypoints(segment, [.. segment.Corridor], straightPathOptions);
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
        rawCorners                            = BuildGroundCornersFromStraightPath(corridor, straightPath, straightPathCount, cancel);

        if (!optimize)
            return rawCorners;

        var centerline = BuildPortalCenterline(segment, corridor, cancel);
        return centerline != null ?
                   Optimize(centerline, initialSourceIndex, cancel) :
                   Optimize(rawCorners, initialSourceIndex, cancel);
    }

    private List<GroundPathCorner> BuildGroundCornersFromStraightPath
    (
        long[]             corridor,
        DtStraightPath[]   straightPath,
        int                straightPathCount,
        CancellationToken  cancel
    )
    {
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
            AppendDistinct(result, corner);
        }

        return result;
    }

    private List<GroundPathCorner>? BuildPortalCenterline
    (
        PlannerPathSegment segment,
        long[]             corridor,
        CancellationToken  cancel
    )
    {
        List<GroundPathCorner> result =
        [
            new
            (
                segment.StartPosition,
                corridor[0],
                DtStraightPathFlags.DT_STRAIGHTPATH_START,
                ResolveArea(corridor, corridor[0]),
                null,
                0
            )
        ];

        for (var i = 0; i < corridor.Length - 1; ++i)
        {
            cancel.ThrowIfCancellationRequested();

            if (!MeshQuery.GetPortalPoints
                (
                    corridor[i],
                    corridor[i + 1],
                    out var left,
                    out var right,
                    out _,
                    out var toType
                ).Succeeded())
                return null;

            var polyRef = corridor[i + 1];
            var area    = ResolveArea(corridor, polyRef);
            var flags  = toType == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION ?
                             DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION :
                             (byte)0;
            var position = new Vector3
            (
                (left.X + right.X) * 0.5f,
                (left.Y + right.Y) * 0.5f,
                (left.Z + right.Z) * 0.5f
            );
            if (toType == DtPolyTypes.DT_POLYTYPE_GROUND && TryResolveStartPoly(position, polyRef, out polyRef, out position))
                area = ResolveArea(corridor, polyRef);

            AppendDistinct
            (
                result,
                new
                (
                    position,
                    polyRef,
                    flags,
                    area,
                    ResolveLinkKind(area, flags),
                    i + 1
                )
            );
        }

        AppendDistinct
        (
            result,
            new
            (
                segment.EndPosition,
                0,
                DtStraightPathFlags.DT_STRAIGHTPATH_END,
                ResolveArea(corridor, 0),
                null,
                corridor.Length
            )
        );
        return result;
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

    private IReadOnlyList<GroundPathCorner> Optimize
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex,
        CancellationToken               cancel
    )
    {
        if (corners.Count <= 2)
            return corners;

        var simplified = SimplifyCenterline(corners, initialSourceIndex, cancel);
        var prepared   = RelaxAwayFromWalls(simplified, initialSourceIndex, cancel);
        return RoundCorners(prepared, initialSourceIndex, cancel);
    }

    private IReadOnlyList<GroundPathCorner> RelaxAwayFromWalls
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex,
        CancellationToken               cancel
    )
    {
        List<GroundPathCorner> result = [.. corners];

        for (var iteration = 0; iteration < CLEARANCE_RELAX_ITERATIONS; ++iteration)
        {
            var changed = false;

            for (var i = 1; i < result.Count - 1; ++i)
            {
                cancel.ThrowIfCancellationRequested();

                if (IsProtected(result, i, initialSourceIndex) ||
                    IsSemanticAnchor(result[i])                 ||
                    IsSemanticAnchor(result[i - 1])             ||
                    IsSemanticAnchor(result[i + 1]))
                    continue;

                if (!TryRelaxCorner(result, i, out var relaxed))
                    continue;

                result[i] = relaxed;
                changed   = true;
            }

            if (!changed)
                break;
        }

        return result;
    }

    private bool TryRelaxCorner
    (
        List<GroundPathCorner> corners,
        int                    index,
        out GroundPathCorner   relaxed
    )
    {
        relaxed = corners[index];
        var corner = corners[index];
        var prev   = corners[index - 1];
        var next   = corners[index + 1];

        var push = Vector3.Zero;

        if (TryGetWallClearance(corner, out var ownDist, out var ownNormal) && ownDist < PREFERRED_PATH_CLEARANCE)
            push += new Vector3(ownNormal.X, 0, ownNormal.Z) * MathF.Min(PREFERRED_PATH_CLEARANCE - ownDist, MAX_RELAX_PUSH);

        push += ComputeSegmentPush(corner, prev);
        push += ComputeSegmentPush(corner, next);

        if (push.LengthSquared() < 1e-6f)
            return false;

        if (push.Length() > MAX_RELAX_PUSH)
            push *= MAX_RELAX_PUSH / push.Length();

        var candidate = corner.Position + push;
        if (!TryResolveStartPoly(candidate, corner.PolyRef, out var polyRef, out var projected))
            return false;
        if (HorizontalDistance(candidate, projected) > MAX_RELAX_PROJECTION)
            return false;

        var candidateCorner = corner with
        {
            Position = projected,
            PolyRef = polyRef
        };
        if (!HasLineOfSight(prev, candidateCorner) || !HasLineOfSight(candidateCorner, next))
            return false;
        if (!TryGetWallClearance(candidateCorner, out var newDist, out _) || newDist <= ownDist + 0.05f)
            return false;

        relaxed = candidateCorner;
        return true;
    }

    private Vector3 ComputeSegmentPush
    (
        GroundPathCorner corner,
        GroundPathCorner other
    )
    {
        var push     = Vector3.Zero;
        var distance = HorizontalDistance(corner.Position, other.Position);
        if (distance < 1e-4f)
            return push;

        for (var sampleIndex = 1; sampleIndex <= RELAX_SEGMENT_SAMPLES; ++sampleIndex)
        {
            var t = sampleIndex / (float)(RELAX_SEGMENT_SAMPLES + 1);
            var position = Vector3.Lerp(corner.Position, other.Position, t);
            if (!TryResolveStartPoly(position, corner.PolyRef, out var polyRef, out var projected))
                continue;
            if (!TryGetWallClearance(projected, polyRef, out var dist, out var normal))
                continue;
            if (dist >= PREFERRED_PATH_CLEARANCE)
                continue;

            var amount = MathF.Min(PREFERRED_PATH_CLEARANCE - dist, MAX_RELAX_PUSH) * 0.5f;
            push += new Vector3(normal.X, 0, normal.Z) * amount;
        }

        return push;
    }

    private bool TryGetWallClearance
    (
        GroundPathCorner corner,
        out float        distance,
        out Vector3      normal
    ) =>
        TryGetWallClearance(corner.Position, corner.PolyRef, out distance, out normal);

    private bool TryGetWallClearance
    (
        Vector3   position,
        long      polyRef,
        out float distance,
        out Vector3 normal
    )
    {
        distance = 0;
        normal   = default;

        var status = MeshQuery.FindDistanceToWall
        (
            polyRef,
            position.ToRecast(),
            PREFERRED_PATH_CLEARANCE + 2f,
            GroundFilter,
            out distance,
            out _,
            out var hitNormal
        );
        normal = new(hitNormal.X, hitNormal.Y, hitNormal.Z);
        return status.Succeeded();
    }

    private List<GroundPathCorner> SimplifyCenterline
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex,
        CancellationToken               cancel
    )
    {
        List<GroundPathCorner> result = new(corners.Count) { corners[0] };
        var current = 0;

        while (current < corners.Count - 1)
        {
            cancel.ThrowIfCancellationRequested();

            var furthest = Math.Min(corners.Count - 1, current + MAX_CENTERLINE_LOOKAHEAD);
            while (furthest > current + 1 &&
                   (!CanSkipRange(corners, current, furthest, initialSourceIndex) ||
                    !CanUseCenterlineShortcut(corners[current], corners[furthest]))) --furthest;

            current = furthest;
            AppendDistinct(result, corners[current]);
        }

        return result;
    }

    private bool CanUseCenterlineShortcut
    (
        GroundPathCorner start,
        GroundPathCorner end
    ) =>
        HorizontalDistance(start.Position, end.Position) <= MAX_CENTERLINE_SHORTCUT_DISTANCE &&
        HasLineOfSight(start, end)                                                            &&
        HasPreferredClearance(start, end);

    private bool HasPreferredClearance
    (
        GroundPathCorner start,
        GroundPathCorner end
    )
    {
        var distance = HorizontalDistance(start.Position, end.Position);
        var sampleCount = Math.Clamp
        (
            (int)MathF.Ceiling(distance / CLEARANCE_SAMPLE_STEP),
            1,
            MAX_CLEARANCE_SAMPLES
        );

        for (var sampleIndex = 1; sampleIndex < sampleCount; ++sampleIndex)
        {
            var position = Vector3.Lerp(start.Position, end.Position, sampleIndex / (float)sampleCount);
            if (!TryResolveStartPoly(position, start.PolyRef, out var polyRef, out var projectedPosition) ||
                !HasPreferredClearanceAtPoint(projectedPosition, polyRef))
                return false;
        }

        return true;
    }

    private bool HasPreferredClearanceAtPoint
    (
        Vector3 position,
        long    polyRef
    )
    {
        var status = MeshQuery.FindDistanceToWall
        (
            polyRef,
            position.ToRecast(),
            PREFERRED_PATH_CLEARANCE,
            GroundFilter,
            out var distance,
            out _,
            out _
        );
        return status.Succeeded() && distance >= PREFERRED_PATH_CLEARANCE;
    }

    private List<GroundPathCorner> RoundCorners
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex,
        CancellationToken               cancel
    )
    {
        if (corners.Count <= 2)
            return [.. corners];

        List<GroundPathCorner> result = new(corners.Count * 2) { corners[0] };

        for (var i = 1; i < corners.Count - 1; ++i)
        {
            cancel.ThrowIfCancellationRequested();

            var previous = result[^1];
            var current  = corners[i];
            var next     = corners[i + 1];

            if (IsProtected(corners, i, initialSourceIndex) ||
                IsSemanticAnchor(previous)                  ||
                IsSemanticAnchor(next)                      ||
                !TryBuildRoundedCorner(previous, current, next, out var first, out var second))
            {
                AppendDistinct(result, current);
                continue;
            }

            if (!HasLineOfSight(previous, first)  ||
                !HasLineOfSight(first,    second) ||
                !HasLineOfSight(second,   next)   ||
                !HasArcPreferredClearance(previous, first, second, next))
            {
                AppendDistinct(result, current);
                continue;
            }

            AppendDistinct(result, first);
            AppendDistinct(result, second);
        }

        AppendDistinct(result, corners[^1]);
        return result;
    }

    private bool HasArcPreferredClearance
    (
        GroundPathCorner previous,
        GroundPathCorner first,
        GroundPathCorner second,
        GroundPathCorner next
    ) =>
        HasSegmentPreferredClearance(previous, first) &&
        HasSegmentPreferredClearance(first,    second) &&
        HasSegmentPreferredClearance(second,   next);

    private bool HasSegmentPreferredClearance
    (
        GroundPathCorner start,
        GroundPathCorner end
    )
    {
        var distance    = HorizontalDistance(start.Position, end.Position);
        var sampleCount = Math.Clamp
        (
            (int)MathF.Ceiling(distance / CLEARANCE_SAMPLE_STEP),
            1,
            MAX_ARC_CLEARANCE_SAMPLES
        );

        for (var sampleIndex = 1; sampleIndex < sampleCount; ++sampleIndex)
        {
            var position = Vector3.Lerp(start.Position, end.Position, sampleIndex / (float)sampleCount);
            if (!TryResolveStartPoly(position, start.PolyRef, out var polyRef, out var projectedPosition) ||
                !HasPreferredClearanceAtPoint(projectedPosition, polyRef))
                return false;
        }

        return true;
    }

    private bool TryBuildRoundedCorner
    (
        GroundPathCorner     previous,
        GroundPathCorner     current,
        GroundPathCorner     next,
        out GroundPathCorner first,
        out GroundPathCorner second
    )
    {
        first  = default;
        second = default;

        var incoming       = HorizontalDelta(previous.Position, current.Position);
        var outgoing       = HorizontalDelta(current.Position,  next.Position);
        var incomingLength = incoming.Length();
        var outgoingLength = outgoing.Length();
        if (incomingLength < MIN_ROUNDING_SEGMENT_LENGTH || outgoingLength < MIN_ROUNDING_SEGMENT_LENGTH)
            return false;

        incoming /= incomingLength;
        outgoing /= outgoingLength;
        var directionDot = Vector2.Dot(incoming, outgoing);
        if (directionDot > ROUNDING_MAX_DIRECTION_DOT || directionDot < ROUNDING_MIN_DIRECTION_DOT)
            return false;

        var turnStrength = Math.Clamp((1f - directionDot) * 0.5f, 0f, 1f);
        var tangentDistance = MathF.Min
        (
            TURN_RADIUS                               * (0.45f + (turnStrength * 0.55f)),
            MathF.Min(incomingLength, outgoingLength) * MAX_TANGENT_SEGMENT_FRACTION
        );
        if (tangentDistance < MIN_TANGENT_DISTANCE)
            return false;

        var entry           = current.Position - new Vector3(incoming.X * tangentDistance, 0, incoming.Y * tangentDistance);
        var exit            = current.Position + new Vector3(outgoing.X * tangentDistance, 0, outgoing.Y * tangentDistance);
        var firstCandidate  = QuadraticBezier(entry, current.Position, exit, 1f / 3f);
        var secondCandidate = QuadraticBezier(entry, current.Position, exit, 2f / 3f);

        if (!TryProjectToMesh(firstCandidate,  current.PolyRef, out firstCandidate,  out var firstRef) ||
            !TryProjectToMesh(secondCandidate, current.PolyRef, out secondCandidate, out var secondRef)) return false;

        first = current with
        {
            Position = firstCandidate,
            PolyRef = firstRef,
            StraightPathFlags = 0,
            LinkKind = null
        };
        second = current with
        {
            Position = secondCandidate,
            PolyRef = secondRef,
            StraightPathFlags = 0,
            LinkKind = null
        };
        return true;
    }

    private bool HasLineOfSight
    (
        GroundPathCorner start,
        GroundPathCorner end
    )
    {
        if (start.PolyRef != 0 && TryRaycastFromPoly(start.PolyRef, start.Position, end.Position))
            return true;

        var candidateRefs = ArrayPool<long>.Shared.Rent(MAX_START_POLY_CANDIDATES);

        try
        {
            var status = MeshQuery.QueryPolygons
            (
                start.Position.ToRecast(),
                new(POINT_QUERY_HALF_EXTENT, POINT_QUERY_HEIGHT, POINT_QUERY_HALF_EXTENT),
                GroundFilter,
                candidateRefs,
                out var candidateCount,
                MAX_START_POLY_CANDIDATES
            );
            if (status.Failed())
                return false;

            for (var i = 0; i < candidateCount; ++i)
                if (candidateRefs[i] != start.PolyRef && TryRaycastFromPoly(candidateRefs[i], start.Position, end.Position))
                    return true;

            return false;
        }
        finally
        {
            ArrayPool<long>.Shared.Return(candidateRefs);
        }
    }

    private bool TryRaycastFromPoly
    (
        long    startRef,
        Vector3 start,
        Vector3 end
    )
    {
        var horizontalDistance = HorizontalDistance(start, end);
        var insetFactor = horizontalDistance > RAYCAST_START_INSET ?
                              RAYCAST_START_INSET / horizontalDistance :
                              0f;
        var startProbe = Vector3.Lerp(start, end,   insetFactor);
        var endProbe   = Vector3.Lerp(end,   start, insetFactor);
        if (!MeshQuery.GetPolyHeight(startRef, startProbe.ToRecast(), out var startHeight).Succeeded())
            return false;

        var        projectedStart = startProbe with { Y = startHeight };
        Span<long> visited        = stackalloc long[MAX_RAYCAST_POLYS];
        var status = MeshQuery.Raycast
        (
            startRef,
            projectedStart.ToRecast(),
            endProbe.ToRecast(),
            GroundFilter,
            out var hitTime,
            out _,
            visited,
            out var visitedCount,
            visited.Length
        );
        if (status.Failed() || hitTime < 1f || visitedCount == 0 || visitedCount >= visited.Length)
            return false;

        var endRef = visited[visitedCount - 1];
        if (!MeshQuery.GetPolyHeight(endRef, endProbe.ToRecast(), out var height).Succeeded())
            return false;

        return MathF.Abs(height - end.Y) <= MAX_SHORTCUT_VERTICAL_ERROR;
    }

    private bool TryResolveStartPoly
    (
        Vector3     position,
        long        preferredRef,
        out long    polyRef,
        out Vector3 projectedPosition
    )
    {
        if (preferredRef != 0 && MeshQuery.GetPolyHeight(preferredRef, position.ToRecast(), out var preferredHeight).Succeeded())
        {
            polyRef           = preferredRef;
            projectedPosition = position with { Y = preferredHeight };
            return true;
        }

        var status = MeshQuery.FindNearestPoly
        (
            position.ToRecast(),
            new(POINT_QUERY_HALF_EXTENT, POINT_QUERY_HEIGHT, POINT_QUERY_HALF_EXTENT),
            GroundFilter,
            out polyRef,
            out var nearest,
            out var isOverPoly
        );
        projectedPosition = nearest.ToSystem();
        return status.Succeeded() && polyRef != 0 && isOverPoly;
    }

    private bool TryProjectToMesh
    (
        Vector3     candidate,
        long        preferredRef,
        out Vector3 projected,
        out long    polyRef
    )
    {
        if (TryResolveStartPoly(candidate, preferredRef, out polyRef, out projected) &&
            HorizontalDistance(candidate, projected) <= MAX_PROJECTION_DISTANCE) return true;

        projected = default;
        polyRef   = 0;
        return false;
    }

    private static bool CanSkipRange
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             start,
        int                             end,
        int                             initialSourceIndex
    )
    {
        if (IsSemanticAnchor(corners[start]) || IsSemanticAnchor(corners[end]))
            return false;

        for (var i = start + 1; i < end; ++i)
            if (IsProtected(corners, i, initialSourceIndex))
                return false;

        return true;
    }

    private static bool IsProtected
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             index,
        int                             initialSourceIndex
    ) =>
        corners[index].SourceIndex < initialSourceIndex ||
        IsSemanticAnchor(corners[index])                ||
        (index > 0 && IsSemanticAnchor(corners[index - 1]));

    private static bool IsSemanticAnchor
    (
        GroundPathCorner corner
    ) =>
        corner.LinkKind                                                                     != null ||
        (corner.StraightPathFlags & DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0;

    private static Vector2 HorizontalDelta
    (
        Vector3 from,
        Vector3 to
    ) => new(to.X - from.X, to.Z - from.Z);

    private static float HorizontalDistance
    (
        Vector3 left,
        Vector3 right
    ) => HorizontalDelta(left, right).Length();

    private static Vector3 QuadraticBezier
    (
        Vector3 start,
        Vector3 control,
        Vector3 end,
        float   progress
    )
    {
        var inverse = 1f - progress;
        return (inverse * inverse * start) + (2f * inverse * progress * control) + (progress * progress * end);
    }

    private static void AppendDistinct
    (
        List<GroundPathCorner> result,
        GroundPathCorner       corner
    )
    {
        if (result.Count                                                  == 0                          ||
            Vector3.DistanceSquared(result[^1].Position, corner.Position) > DUPLICATE_POINT_DISTANCE_SQ ||
            result[^1].Area                                               != corner.Area               ||
            result[^1].LinkKind                                           != corner.LinkKind           ||
            result[^1].StraightPathFlags                                  != corner.StraightPathFlags) result.Add(corner);
    }

    private const int   MAX_STRAIGHT_PATH_POINTS       = 4097;
    private const float DUPLICATE_WAYPOINT_DISTANCE_SQ = 0.000001f;
    private const int   MAX_CENTERLINE_LOOKAHEAD       = 64;
    private const int   MAX_CLEARANCE_SAMPLES          = 256;
    private const int   MAX_RAYCAST_POLYS              = 512;
    private const int   MAX_START_POLY_CANDIDATES      = 32;
    private const float MAX_SHORTCUT_VERTICAL_ERROR    = 0.75f;
    private const float MAX_CENTERLINE_SHORTCUT_DISTANCE = 500f;
    private const float CLEARANCE_SAMPLE_STEP          = 0.50f;
    private const float POINT_QUERY_HALF_EXTENT        = 0.10f;
    private const float POINT_QUERY_HEIGHT             = 2f;
    private const float MAX_PROJECTION_DISTANCE        = 0.15f;
    private const float RAYCAST_START_INSET            = 0.02f;
    private const float MIN_ROUNDING_SEGMENT_LENGTH    = 0.75f;
    private const float MIN_TANGENT_DISTANCE           = 0.50f;
    private const float MAX_TANGENT_SEGMENT_FRACTION   = 0.32f;
    private const float PREFERRED_PATH_CLEARANCE       = 0.70f;
    private const int   MAX_ARC_CLEARANCE_SAMPLES      = 32;
    private const int   CLEARANCE_RELAX_ITERATIONS     = 4;
    private const float MAX_RELAX_PUSH                 = 0.50f;
    private const float MAX_RELAX_PROJECTION           = 0.50f;
    private const int   RELAX_SEGMENT_SAMPLES          = 3;
    private const float TURN_RADIUS                    = 5.00f;
    private const float ROUNDING_MAX_DIRECTION_DOT     = 0.92f;
    private const float ROUNDING_MIN_DIRECTION_DOT     = -0.70f;
    private const float DUPLICATE_POINT_DISTANCE_SQ    = 0.000001f;
}
