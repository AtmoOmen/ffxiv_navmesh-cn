using System.Numerics;
using vnavmesh.Movement.Execution;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Drivers;

internal sealed class FlightTraverseDriver : IMovementSegmentDriver
{
    public void Enter(MovementExecutionContext context)
    {
    }

    public SegmentDriverUpdate Update(MovementExecutionContext context)
    {
        var nextWaypointIndex = ConsumeReachedWaypoints(context);
        if (nextWaypointIndex >= context.WaypointCount)
            return new(CreateIdleCommand(context.Player.Position), nextWaypointIndex);

        var desired = context.Segment.Waypoints[nextWaypointIndex];
        var delta = new Vector3
        (
            desired.X - context.Player.Position.X,
            desired.Y - context.Player.Position.Y,
            desired.Z - context.Player.Position.Z
        );
        return new
        (
            new
            (
                desired,
                context.MovementAllowed,
                true,
                context.Config.AlignCameraToMovement,
                Angle.FromDirectionXZ(delta) + 180.Degrees(),
                context.Config.AlignCameraHeight.Degrees(),
                false
            ),
            nextWaypointIndex
        );
    }

    public bool ShouldAdvance(MovementExecutionContext context) => context.ActiveWaypointIndex >= context.WaypointCount;

    public void Exit(MovementExecutionContext context)
    {
    }

    private static int ConsumeReachedWaypoints(MovementExecutionContext context)
    {
        var nextWaypointIndex = context.ActiveWaypointIndex;

        while (nextWaypointIndex < context.WaypointCount)
        {
            var current  = context.Player.Position;
            var previous = context.PreviousPosition ?? current;

            if (context.Plan.DestinationTolerance > 0 && Vector3.Distance(current, context.Plan.RequestedDestination) <= context.Plan.DestinationTolerance)
                return context.WaypointCount;

            if (DriverMath.DistanceToLineSegment(context.Segment.Waypoints[nextWaypointIndex], current, previous) > context.PathTolerance)
                return nextWaypointIndex;

            nextWaypointIndex++;
        }

        return nextWaypointIndex;
    }

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false);
}
