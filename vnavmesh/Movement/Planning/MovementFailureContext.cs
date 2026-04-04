using System.Numerics;

namespace vnavmesh.Movement.Planning;

internal readonly record struct MovementFailureContext
(
    MovementFailureReason Reason,
    MovementMode          RequestedMode,
    MovementSegmentKind   CurrentSegmentKind,
    Vector3               RequestedDestination,
    float                 DestinationTolerance,
    Vector3               FinalActiveWaypoint
);