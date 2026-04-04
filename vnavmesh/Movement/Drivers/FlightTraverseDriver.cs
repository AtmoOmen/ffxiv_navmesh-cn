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
        var nextWaypointIndex = DriverMath.ConsumeFlightWaypoints(context);
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

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false);
}
