using System.Buffers;
using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Build;
using vnavmesh.Common.Extensions;

namespace vnavmesh.Query.Ground;

internal sealed class GroundPathOptimizer
(
    DtNavMeshQuery meshQuery,
    IDtQueryFilter groundFilter
)
{
    public IReadOnlyList<GroundPathCorner> Optimize
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex,
        CancellationToken               cancel
    )
    {
        if (corners.Count <= 2)
            return corners;

        var simplified = Simplify(corners, initialSourceIndex, cancel);
        return RoundCorners(simplified, initialSourceIndex, cancel);
    }

    private List<GroundPathCorner> Simplify
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             initialSourceIndex,
        CancellationToken               cancel
    )
    {
        List<GroundPathCorner> result  = new(corners.Count) { corners[0] };
        var                    current = 0;

        while (current < corners.Count - 1)
        {
            cancel.ThrowIfCancellationRequested();

            var furthest = Math.Min(corners.Count - 1, current + MAX_SHORTCUT_LOOKAHEAD);
            while (furthest > current + 1 &&
                   (!CanSkipRange(corners, current, furthest, initialSourceIndex) ||
                    !MaintainsTurnQuality(result, corners, current, furthest)     ||
                    !CanUseShortcut(corners, current, furthest))) --furthest;

            current = furthest;
            AppendDistinct(result, corners[current]);
        }

        return result;
    }

    private static bool MaintainsTurnQuality
    (
        IReadOnlyList<GroundPathCorner> result,
        IReadOnlyList<GroundPathCorner> corners,
        int                             startIndex,
        int                             endIndex
    )
    {
        if (result.Count > 1 && DirectionDot(result[^2].Position, corners[startIndex].Position, corners[endIndex].Position) < MIN_SHORTCUT_DIRECTION_DOT)
            return false;

        return endIndex + 1                                                                                           >= corners.Count ||
               DirectionDot(corners[startIndex].Position, corners[endIndex].Position, corners[endIndex + 1].Position) >= MIN_SHORTCUT_DIRECTION_DOT;
    }

    private bool CanUseShortcut
    (
        IReadOnlyList<GroundPathCorner> corners,
        int                             startIndex,
        int                             endIndex
    )
    {
        var distance = HorizontalDistance(corners[startIndex].Position, corners[endIndex].Position);
        return distance <= MAX_SHORTCUT_DISTANCE && HasLineOfSight(corners[startIndex], corners[endIndex]);
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
                !HasLineOfSight(second,   next))
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
            var status = meshQuery.QueryPolygons
            (
                start.Position.ToRecast(),
                new(POINT_QUERY_HALF_EXTENT, POINT_QUERY_HEIGHT, POINT_QUERY_HALF_EXTENT),
                groundFilter,
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
        if (!meshQuery.GetPolyHeight(startRef, startProbe.ToRecast(), out var startHeight).Succeeded())
            return false;

        var        projectedStart = startProbe with { Y = startHeight };
        Span<long> visited        = stackalloc long[MAX_RAYCAST_POLYS];
        var status = meshQuery.Raycast
        (
            startRef,
            projectedStart.ToRecast(),
            endProbe.ToRecast(),
            groundFilter,
            out var hitTime,
            out _,
            visited,
            out var visitedCount,
            visited.Length
        );
        if (status.Failed() || hitTime < 1f || visitedCount == 0 || visitedCount >= visited.Length)
            return false;

        var endRef = visited[visitedCount - 1];
        if (!meshQuery.GetPolyHeight(endRef, endProbe.ToRecast(), out var height).Succeeded())
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
        if (preferredRef != 0 && meshQuery.GetPolyHeight(preferredRef, position.ToRecast(), out var preferredHeight).Succeeded())
        {
            polyRef           = preferredRef;
            projectedPosition = position with { Y = preferredHeight };
            return true;
        }

        var status = meshQuery.FindNearestPoly
        (
            position.ToRecast(),
            new(POINT_QUERY_HALF_EXTENT, POINT_QUERY_HEIGHT, POINT_QUERY_HALF_EXTENT),
            groundFilter,
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

    private static float DirectionDot
    (
        Vector3 previous,
        Vector3 current,
        Vector3 next
    )
    {
        var incoming    = HorizontalDelta(previous, current);
        var outgoing    = HorizontalDelta(current,  next);
        var denominator = MathF.Sqrt(incoming.LengthSquared() * outgoing.LengthSquared());
        return denominator > DUPLICATE_POINT_DISTANCE_SQ ?
                   Vector2.Dot(incoming, outgoing) / denominator :
                   1f;
    }

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
            IsSemanticAnchor(corner)) result.Add(corner);
    }

    private const int   MAX_SHORTCUT_LOOKAHEAD       = 8;
    private const int   MAX_RAYCAST_POLYS            = 512;
    private const int   MAX_START_POLY_CANDIDATES    = 32;
    private const float MAX_SHORTCUT_DISTANCE        = 40f;
    private const float MIN_SHORTCUT_DIRECTION_DOT   = 0.70f;
    private const float MAX_SHORTCUT_VERTICAL_ERROR  = 0.75f;
    private const float POINT_QUERY_HALF_EXTENT      = 0.10f;
    private const float POINT_QUERY_HEIGHT           = 2f;
    private const float MAX_PROJECTION_DISTANCE      = 0.15f;
    private const float RAYCAST_START_INSET          = 0.02f;
    private const float MIN_ROUNDING_SEGMENT_LENGTH  = 0.75f;
    private const float MIN_TANGENT_DISTANCE         = 0.18f;
    private const float MAX_TANGENT_SEGMENT_FRACTION = 0.32f;
    private const float TURN_RADIUS                  = 0.70f;
    private const float ROUNDING_MAX_DIRECTION_DOT   = 0.92f;
    private const float ROUNDING_MIN_DIRECTION_DOT   = -0.70f;
    private const float DUPLICATE_POINT_DISTANCE_SQ  = 0.000001f;
}
