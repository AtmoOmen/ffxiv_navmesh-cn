using System.Numerics;
using vnavmesh.Movement.Execution;

namespace vnavmesh.Movement.Drivers;

internal static class DriverMath
{
    private const float MinSegmentLengthSq = 0.000001f;

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

        if (context.Plan.DestinationTolerance > 0 && Vector3.Distance(current, Project(context.Plan.RequestedDestination, flatten)) <= context.Plan.DestinationTolerance)
            return context.TraverseSegmentCount;

        while (context.TryGetCurrentTraverseSegment(currentTraverseSegmentIndex, out var segmentStart, out var segmentEnd))
        {
            var projectedStart = Project(segmentStart, flatten);
            var projectedEnd   = Project(segmentEnd, flatten);
            var progress       = ComputeProjectionParameter(current, projectedStart, projectedEnd);

            if (progress >= 1f)
            {
                currentTraverseSegmentIndex++;
                continue;
            }

            if (Vector3.Distance(current, projectedEnd) <= context.PathTolerance)
            {
                currentTraverseSegmentIndex++;
                continue;
            }

            if (DistanceToLineSegment(projectedEnd, previous, current) <= context.PathTolerance)
            {
                currentTraverseSegmentIndex++;
                continue;
            }

            break;
        }

        return currentTraverseSegmentIndex;
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
}
