using vnavmesh.PathPostprocess;

namespace vnavmesh.Movement.Planning;

internal sealed class TakeoffSegment : MovementSegment
{
    public TakeoffSegment()
    {
        MovementMode      = MovementMode.Flight;
        GeometryOwnership = PathGeometryOwnership.None;
        ReachabilitySource = PathReachabilitySource.Volume;
    }

    public override MovementSegmentKind Kind                 => MovementSegmentKind.Takeoff;
    public override bool                AllowVerticalControl => false;
}
