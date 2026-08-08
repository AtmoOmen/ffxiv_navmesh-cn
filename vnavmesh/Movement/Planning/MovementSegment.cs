using System.Numerics;
using vnavmesh.Query.Ground;
using vnavmesh.Query.Ground.Models;

namespace vnavmesh.Movement.Planning;

internal sealed class MovementSegment
{
    public required MovementSegmentKind    Kind                { get; init; }
    public required MovementMode           MovementMode        { get; init; }
    public          Vector3                StartPosition       { get; init; }
    public          float                  CompletionTolerance { get; init; }
    public          IReadOnlyList<Vector3> Waypoints           { get; init; } = [];
    public          GroundCorridorPayload? GroundCorridor      { get; init; }
}
