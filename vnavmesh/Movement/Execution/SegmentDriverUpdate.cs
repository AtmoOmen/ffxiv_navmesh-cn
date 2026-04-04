using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Execution;

internal readonly record struct SegmentDriverUpdate
(
    MovementFrameCommand    Command,
    MovementFailureContext? Failure = null
);