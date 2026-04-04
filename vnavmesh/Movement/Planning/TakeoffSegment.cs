namespace vnavmesh.Movement.Planning;

internal sealed class TakeoffSegment : MovementSegment
{
    public override MovementSegmentKind Kind                 => MovementSegmentKind.Takeoff;
    public override bool                AllowVerticalControl => false;
}
