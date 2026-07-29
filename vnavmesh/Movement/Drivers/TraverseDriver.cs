using System.Numerics;
using vnavmesh.Common.Extensions;
using vnavmesh.Common.Models;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Drivers;

internal sealed class TraverseDriver : IMovementSegmentDriver
{
    public void Enter
    (
        MovementExecutionContext context
    )
    {
    }

    public SegmentDriverUpdate Update
    (
        MovementExecutionContext context
    )
    {
        var nextWaypointIndex = ConsumeWaypoints(context);
        if (nextWaypointIndex >= context.WaypointCount)
            return new(CreateIdleCommand(context.Player.Position), nextWaypointIndex);

        var desired = context.Segment.Waypoints[nextWaypointIndex];
        return new(BuildCommand(context, desired), nextWaypointIndex);
    }

    public bool ShouldAdvance
    (
        MovementExecutionContext context
    ) => context.ActiveWaypointIndex >= context.WaypointCount;

    public void Exit
    (
        MovementExecutionContext context
    )
    {
    }

    private static int ConsumeWaypoints
    (
        MovementExecutionContext context
    ) => context.Segment.Kind switch
    {
        MovementSegmentKind.GroundTraverse => WaypointProgression.ConsumeGroundWaypoints(context),
        MovementSegmentKind.FlightTraverse => WaypointProgression.ConsumeFlightWaypoints(context),
        _                                  => -1
    };

    private static MovementFrameCommand BuildCommand
    (
        MovementExecutionContext context,
        Vector3                  desired
    )
    {
        var delta = desired - context.Player.Position;
        return new
        (
            desired,
            context.MovementAllowed,
            context.Segment.Kind == MovementSegmentKind.FlightTraverse,
            context.Config.AlignCameraToMovement,
            Angle.FromDirectionXZ(delta) + 180.Degrees(),
            context.Config.AlignCameraHeight.Degrees(),
            false,
            false,
            default
        );
    }

    private static MovementFrameCommand CreateIdleCommand
    (
        Vector3 current
    ) => new(current, false, false, false, default, default, false, false, default);
}
