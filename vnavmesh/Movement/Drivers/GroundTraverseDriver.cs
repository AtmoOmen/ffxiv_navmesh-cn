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
        var segment = (GroundTraverseSegment)context.Segment;
        ConsumeReachedWaypoints(segment, context);
        if(segment.Waypoints.Count == 0)
            return new(CreateIdleCommand(context.Player.Position));

        var desired = segment.Waypoints[0];
        return new(BuildCommand(context, desired, false, false));
    }

    public bool ShouldAdvance(MovementExecutionContext context) => context.Segment.Waypoints.Count == 0;

    public void Exit(MovementExecutionContext context)
    {
    }

    private static void ConsumeReachedWaypoints(GroundTraverseSegment segment, MovementExecutionContext context)
    {
        while (segment.Waypoints.Count > 0)
        {
            var target   = Flatten(segment.Waypoints[0]);
            var current  = Flatten(context.Player.Position);
            var previous = Flatten(context.PreviousPosition ?? context.Player.Position);

            if(context.Plan.DestinationTolerance > 0 && Vector3.Distance(current, Flatten(context.Plan.RequestedDestination)) <= context.Plan.DestinationTolerance)
            {
                segment.Waypoints.Clear();
                break;
            }

            if(DriverMath.DistanceToLineSegment(target, current, previous) > context.PathTolerance)
                break;

            segment.Waypoints.RemoveAt(0);
        }
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
            Service.Config.AlignCameraToMovement,
            Angle.FromDirectionXZ(delta) + 180.Degrees(),
            Service.Config.AlignCameraHeight.Degrees(),
            requestJump
        );
    }

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false);
}
