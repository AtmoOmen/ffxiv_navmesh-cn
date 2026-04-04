namespace vnavmesh.Movement.Planning;

internal sealed class FlightTraverseSegment : MovementSegment
{
    public override MovementSegmentKind Kind                 => MovementSegmentKind.FlightTraverse;
    public override bool                AllowVerticalControl => true;
}
