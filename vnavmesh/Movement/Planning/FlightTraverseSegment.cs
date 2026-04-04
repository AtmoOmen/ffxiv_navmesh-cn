using vnavmesh.PathPostprocess;

namespace vnavmesh.Movement.Planning;

internal sealed class FlightTraverseSegment : MovementSegment
{
    public FlightTraverseSegment()
    {
        MovementMode = MovementMode.Flight;
    }

    public override MovementSegmentKind Kind                 => MovementSegmentKind.FlightTraverse;
    public override bool                AllowVerticalControl => true;
}
