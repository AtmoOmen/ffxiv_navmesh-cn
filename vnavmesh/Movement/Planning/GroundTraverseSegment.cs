namespace vnavmesh.Movement.Planning;

internal sealed class GroundTraverseSegment : MovementSegment
{
    public override MovementSegmentKind Kind                 => MovementSegmentKind.GroundTraverse;
    public override bool                AllowVerticalControl => false;
}
