using System.Collections.Generic;
using System.Numerics;

namespace vnavmesh.Movement.Planning;

internal sealed class MovementPlan
{
    public required MovementMode          RequestedMode        { get; init; }
    public required Vector3               RequestedDestination { get; init; }
    public required Vector3               FinalDestination     { get; init; }
    public required float                 DestinationTolerance { get; init; }
    public          List<MovementSegment> Segments             { get; init; } = [];
}