using System.Numerics;
using vnavmesh.Models;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Drivers;

internal sealed class FlightTraverseDriver : IMovementSegmentDriver
{
    public void Enter(MovementExecutionContext context)
    {
    }

    public SegmentDriverUpdate Update(MovementExecutionContext context)
    {
        var segment = (FlightTraverseSegment)context.Segment;
        ConsumeReachedWaypoints(segment, context);
        if(segment.Waypoints.Count == 0)
            return new(CreateIdleCommand(context.Player.Position));

        var desired = segment.Waypoints[0];
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
                Service.Config.AlignCameraToMovement,
                Angle.FromDirectionXZ(delta) + 180.Degrees(),
                Service.Config.AlignCameraHeight.Degrees(),
                false
            )
        );
    }

    public bool ShouldAdvance(MovementExecutionContext context) => context.Segment.Waypoints.Count == 0;

    public void Exit(MovementExecutionContext context)
    {
    }

    private static void ConsumeReachedWaypoints(FlightTraverseSegment segment, MovementExecutionContext context)
    {
        while (segment.Waypoints.Count > 0)
        {
            var current  = context.Player.Position;
            var previous = context.PreviousPosition ?? current;

            if(context.Plan.DestinationTolerance > 0 && Vector3.Distance(current, context.Plan.RequestedDestination) <= context.Plan.DestinationTolerance)
            {
                segment.Waypoints.Clear();
                break;
            }

            if(DriverMath.DistanceToLineSegment(segment.Waypoints[0], current, previous) > context.PathTolerance)
                break;

            segment.Waypoints.RemoveAt(0);
        }
    }

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false);
}
