using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using vnavmesh.Common.Models;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Planning;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Drivers;

internal sealed class TakeoffDriver : IMovementSegmentDriver
{
    private const float FacingToleranceRad     = 5 * Angle.DEG_TO_RAD;
    private const float ElevatedWaypointDeltaY = 0.1f;
    private static readonly TimeSpan AlignTimeout       = TimeSpan.FromMilliseconds(300);
    private static readonly TimeSpan RetryJumpInterval  = TimeSpan.FromMilliseconds(150);

    private TakeoffState _state;
    private DateTime     _stateEnteredAtUtc;
    private int          _alignedFrameCount;

    public void Enter(MovementExecutionContext context)
    {
        TransitionTo(TakeoffState.Align);
    }

    public SegmentDriverUpdate Update(MovementExecutionContext context)
    {
        if (IsAirborne)
            return new(CreateIdleCommand(context.Player.Position));

        if (!Service.Condition[ConditionFlag.Mounted])
        {
            return new
            (
                CreateIdleCommand(context.Player.Position),
                Failure: new
                (
                    MovementFailureReason.TakeoffUnavailable,
                    context.Plan.RequestedMode,
                    context.Segment.Kind,
                    context.Plan.RequestedDestination,
                    context.Plan.DestinationTolerance,
                    ResolveTakeoffWaypoint(context)
                )
            );
        }

        var target        = ResolveTakeoffWaypoint(context);
        var desiredFacing = ResolveDesiredFacing(context.Player.Position, context.Player.Rotation.Radians(), target);
        return _state switch
        {
            TakeoffState.Align       => UpdateAlignState(context, target, desiredFacing),
            TakeoffState.Jump        => UpdateJumpState(target, desiredFacing),
            TakeoffState.WaitAirborne => UpdateWaitAirborneState(target, desiredFacing),
            _                        => new(CreateIdleCommand(context.Player.Position))
        };
    }

    public bool ShouldAdvance(MovementExecutionContext context) => IsAirborne;

    public void Exit(MovementExecutionContext context)
    {
        TransitionTo(TakeoffState.Align);
        _alignedFrameCount = 0;
    }

    private static bool IsAirborne => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];

    private SegmentDriverUpdate UpdateAlignState(MovementExecutionContext context, Vector3 target, Angle desiredFacing)
    {
        var currentFacing = context.Player.Rotation.Radians();
        var aligned       = (desiredFacing - currentFacing).Normalized().Abs().Rad <= FacingToleranceRad;

        _alignedFrameCount = aligned ? _alignedFrameCount + 1 : 0;
        if (_alignedFrameCount >= 2 || ElapsedSinceStateEntered() >= AlignTimeout)
        {
            TransitionTo(TakeoffState.Jump);
            return UpdateJumpState(target, desiredFacing);
        }

        return new(CreateFacingCommand(target, desiredFacing, requestJump: false));
    }

    private SegmentDriverUpdate UpdateWaitAirborneState(Vector3 target, Angle desiredFacing)
    {
        if (ElapsedSinceStateEntered() >= RetryJumpInterval)
        {
            TransitionTo(TakeoffState.Jump);
            return UpdateJumpState(target, desiredFacing);
        }

        return new(CreateFacingCommand(target, desiredFacing, requestJump: false));
    }

    private SegmentDriverUpdate UpdateJumpState(Vector3 target, Angle desiredFacing)
    {
        TransitionTo(TakeoffState.WaitAirborne);
        return new(CreateFacingCommand(target, desiredFacing, requestJump: true));
    }

    private static Vector3 ResolveTakeoffWaypoint(MovementExecutionContext context)
    {
        if (context.TryGetFirstElevatedRemainingWaypoint(ElevatedWaypointDeltaY, out var elevatedWaypoint))
            return elevatedWaypoint;

        if (context.TryGetFirstRemainingWaypoint(out var waypoint))
            return waypoint;

        return context.Plan.FinalDestination;
    }

    private static Angle ResolveDesiredFacing(Vector3 playerPosition, Angle currentFacing, Vector3 target)
    {
        var delta = new Vector3(target.X - playerPosition.X, 0, target.Z - playerPosition.Z);
        return delta.LengthSquared() > 0.000001f ? Angle.FromDirectionXZ(delta) : currentFacing;
    }

    private static MovementFrameCommand CreateFacingCommand(Vector3 desired, Angle desiredFacing, bool requestJump) =>
        new(desired, false, false, false, default, default, requestJump, true, desiredFacing);

    private static MovementFrameCommand CreateIdleCommand(Vector3 current) =>
        new(current, false, false, false, default, default, false, false, default);

    private TimeSpan ElapsedSinceStateEntered() => DateTime.UtcNow - _stateEnteredAtUtc;

    private void TransitionTo(TakeoffState state)
    {
        _state             = state;
        _stateEnteredAtUtc = DateTime.UtcNow;
        if (state == TakeoffState.Align)
            _alignedFrameCount = 0;
    }

    private enum TakeoffState
    {
        Align,
        Jump,
        WaitAirborne
    }
}
