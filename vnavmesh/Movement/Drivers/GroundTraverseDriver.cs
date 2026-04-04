using System.Numerics;
using vnavmesh.Movement.Execution;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Drivers;

internal sealed class GroundTraverseDriver : IMovementSegmentDriver
{
    public void Enter(MovementExecutionContext context)
    {
    }

    public SegmentDriverUpdate Update(MovementExecutionContext context)
    {
        var nextWaypointIndex = DriverMath.ConsumeGroundWaypoints(context);
        if (nextWaypointIndex >= context.WaypointCount)
            return new(CreateIdleCommand(context.Player.Position), nextWaypointIndex);

        var desired = context.Segment.Waypoints[nextWaypointIndex];
        return new(BuildCommand(context, desired, false, false), nextWaypointIndex);
    }

    public bool ShouldAdvance(MovementExecutionContext context) => context.ActiveWaypointIndex >= context.WaypointCount;

    public void Exit(MovementExecutionContext context)
    {
    }

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
            requestJump,
            false,
            default
        );
    }

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false, false, default);
}
