using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using vnavmesh.Models;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Drivers;

internal sealed class TakeoffDriver : IMovementSegmentDriver
{
    public void Enter(MovementExecutionContext context)
    {
    }

    public SegmentDriverUpdate Update(MovementExecutionContext context)
    {
        if(IsAirborne)
            return new(CreateIdleCommand(context.Player.Position));

        if(!Service.Condition[ConditionFlag.Mounted])
            return new
            (
                CreateIdleCommand(context.Player.Position),
                new
                (
                    MovementFailureReason.TakeoffUnavailable,
                    context.Plan.RequestedMode,
                    context.Segment.Kind,
                    context.Plan.RequestedDestination,
                    context.Plan.DestinationTolerance,
                    ResolveFallbackWaypoint(context)
                )
            );

        var desired = ResolveFallbackWaypoint(context);
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
                false,
                Service.Config.AlignCameraToMovement,
                Angle.FromDirectionXZ(delta) + 180.Degrees(),
                Service.Config.AlignCameraHeight.Degrees(),
                true
            )
        );
    }

    public bool ShouldAdvance(MovementExecutionContext context) => IsAirborne;

    public void Exit(MovementExecutionContext context)
    {
    }

    private static bool IsAirborne => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];

    private static Vector3 ResolveFallbackWaypoint(MovementExecutionContext context)
    {
        for (var i = context.SegmentIndex + 1; i < context.Plan.Segments.Count; i++)
            if(context.Plan.Segments[i].Waypoints.Count > 0)
                return context.Plan.Segments[i].Waypoints[0];

        return context.Plan.FinalDestination;
    }

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) => new(current, false, false, false, default, default, false);
}
