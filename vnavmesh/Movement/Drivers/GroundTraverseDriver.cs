using System.Numerics;
using vnavmesh.Models;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Drivers;

internal sealed class GroundTraverseDriver : IMovementSegmentDriver
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
        return new(BuildCommand(context, desired, false, false), nextWaypointIndex);
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
            var target   = Flatten(context.Segment.Waypoints[nextWaypointIndex]);
            var current  = Flatten(context.Player.Position);
            var previous = Flatten(context.PreviousPosition ?? context.Player.Position);

            if (context.Plan.DestinationTolerance > 0 && Vector3.Distance(current, Flatten(context.Plan.RequestedDestination)) <= context.Plan.DestinationTolerance)
                return context.WaypointCount;

            if (DriverMath.DistanceToLineSegment(target, current, previous) > context.PathTolerance)
                return nextWaypointIndex;

            nextWaypointIndex++;
        }

        return nextWaypointIndex;
    }

    private static Vector3 Flatten(Vector3 value) => new(value.X, 0, value.Z);

    private static MovementFrameCommand BuildCommand(MovementExecutionContext context, Vector3 desired, bool allowVerticalControl, bool requestJump)
    {
        var delta = new Vector3
        (
            desired.X - context.Player.Position.X,
            desired.Y - context.Player.Position.Y,
            desired.Z - context.Player.Position.Z
        );
        return new
        (
            desired,
            context.MovementAllowed,
            allowVerticalControl,
            context.Config.AlignCameraToMovement,
            Angle.FromDirectionXZ(delta) + 180.Degrees(),
            context.Config.AlignCameraHeight.Degrees(),
            requestJump
        );
    }

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false);
}
