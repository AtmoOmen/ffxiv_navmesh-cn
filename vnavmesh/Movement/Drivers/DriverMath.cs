using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Movement.Execution;
using vnavmesh.Navigation.Mesh.Runtime;

namespace vnavmesh.Movement.Drivers;

internal static class DriverMath
{
    private const float MinSegmentLengthSq = 0.000001f;
    private const float SegmentEndCaptureRadius = 0.05f;

    public static int ConsumeGroundWaypoints(Vector3 currentPosition, Vector3? previousPosition, Vector3 startPosition, IReadOnlyList<Vector3> waypoints) =>
        ConsumeTraverseWaypoints(currentPosition, previousPosition, startPosition, waypoints, flatten: true);

    public static int ConsumeFlightWaypoints(Vector3 currentPosition, Vector3? previousPosition, Vector3 startPosition, IReadOnlyList<Vector3> waypoints) =>
        ConsumeTraverseWaypoints(currentPosition, previousPosition, startPosition, waypoints, flatten: false);

    public static int ConsumeGroundWaypoints(MovementExecutionContext context) =>
        ConsumeTraverseSegments(context, flatten: true);

    public static int ConsumeFlightWaypoints(MovementExecutionContext context) =>
        ConsumeTraverseSegments(context, flatten: false);

    public static float DistanceToLineSegment(Vector3 v, Vector3 a, Vector3 b)
    {
        var ab = b - a;
        var av = v - a;

        if (ab.Length() == 0 || Vector3.Dot(av, ab) <= 0)
            return av.Length();

        var bv = v - b;
        if (Vector3.Dot(bv, ab) >= 0)
            return bv.Length();

        return Vector3.Cross(ab, av).Length() / ab.Length();
    }

    private static int ConsumeTraverseSegments(MovementExecutionContext context, bool flatten)
    {
        var currentTraverseSegmentIndex = context.CurrentTraverseSegmentIndex;
        var current                     = Project(context.Player.Position, flatten);
        var previous                    = Project(context.PreviousPosition ?? context.Player.Position, flatten);

        if (context.Plan.DestinationTolerance > 0 && Vector3.Distance(current, Project(context.Plan.FinalDestination, flatten)) <= context.Plan.DestinationTolerance)
            return context.TraverseSegmentCount;

        while (context.TryGetCurrentTraverseSegment(currentTraverseSegmentIndex, out var segmentStart, out var segmentEnd))
        {
            if (flatten && ShouldHoldForClientPath(context, currentTraverseSegmentIndex, out var proceed))
            {
                if (proceed)
                {
                    currentTraverseSegmentIndex++;
                    continue;
                }

                break;
            }

            if (ShouldAdvanceTraverseSegment(current, previous, segmentStart, segmentEnd, flatten))
            {
                currentTraverseSegmentIndex++;
                continue;
            }

            break;
        }

        return currentTraverseSegmentIndex;
    }

    private static int ConsumeTraverseWaypoints
    (
        Vector3                currentPosition,
        Vector3?               previousPosition,
        Vector3                startPosition,
        IReadOnlyList<Vector3> waypoints,
        bool                   flatten
    )
    {
        var currentTraverseSegmentIndex = 0;
        var current                     = Project(currentPosition, flatten);
        var previous                    = Project(previousPosition ?? currentPosition, flatten);

        while (TryGetTraverseSegment(startPosition, waypoints, currentTraverseSegmentIndex, out var segmentStart, out var segmentEnd))
        {
            if (ShouldAdvanceTraverseSegment(current, previous, segmentStart, segmentEnd, flatten))
            {
                currentTraverseSegmentIndex++;
                continue;
            }

            break;
        }

        return currentTraverseSegmentIndex;
    }

    private static bool TryGetTraverseSegment(Vector3 startPosition, IReadOnlyList<Vector3> waypoints, int traverseSegmentIndex, out Vector3 start, out Vector3 end)
    {
        if (traverseSegmentIndex < 0 || traverseSegmentIndex >= waypoints.Count)
        {
            start = default;
            end   = default;
            return false;
        }

        start = traverseSegmentIndex == 0 ? startPosition : waypoints[traverseSegmentIndex - 1];
        end   = waypoints[traverseSegmentIndex];
        return true;
    }

    private static bool ShouldAdvanceTraverseSegment(Vector3 current, Vector3 previous, Vector3 segmentStart, Vector3 segmentEnd, bool flatten)
    {
        var projectedStart = Project(segmentStart, flatten);
        var projectedEnd   = Project(segmentEnd, flatten);
        var progress       = ComputeProjectionParameter(current, projectedStart, projectedEnd);

        if (progress >= 1f)
            return true;

        if (Vector3.Distance(current, projectedEnd) <= SegmentEndCaptureRadius)
            return true;

        return DistanceToLineSegment(projectedEnd, previous, current) <= SegmentEndCaptureRadius;
    }

    private static float ComputeProjectionParameter(Vector3 position, Vector3 start, Vector3 end)
    {
        var segment      = end - start;
        var segmentLenSq = segment.LengthSquared();
        return segmentLenSq > MinSegmentLengthSq
            ? Vector3.Dot(position - start, segment) / segmentLenSq
            : 1f;
    }

    private static Vector3 Project(Vector3 value, bool flatten) => flatten
        ? new(value.X, 0, value.Z)
        : value;

    private static bool ShouldHoldForClientPath(MovementExecutionContext context, int waypointIndex, out bool proceed)
    {
        proceed = false;

        if (context.Segment.GroundCorridor is not { } corridor)
            return false;

        foreach (var marker in corridor.LinkMarkers)
        {
            if (marker.Kind != NavmeshOffMeshKind.ClientPath)
                continue;

            if (marker.CornerIndex == waypointIndex)
            {
                proceed = IsClientPathActive();
                return true;
            }

            if (marker.CornerIndex + 1 == waypointIndex)
            {
                proceed = !IsClientPathActive();
                return true;
            }
        }

        return false;
    }

    private static bool IsClientPathActive() =>
        Service.Condition.Any(ConditionFlag.Jumping61, ConditionFlag.Unknown101);
}
