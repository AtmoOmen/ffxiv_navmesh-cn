using vnavmesh.PathPostprocess;

namespace vnavmesh.Movement.Planning;

internal sealed class GroundTraverseSegment : MovementSegment
{
    public GroundTraverseSegment()
    {
        MovementMode = MovementMode.Ground;
    }

    public override MovementSegmentKind Kind                 => MovementSegmentKind.GroundTraverse;
    public override bool                AllowVerticalControl => false;
}
