using System.Numerics;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Ground;

namespace vnavmesh.Query.Models;

internal sealed class PostprocessedPathSegment
{
    public required MovementMode           MovementMode         { get; init; }
    public required MovementSegmentKind    SegmentKind          { get; init; }
    public required bool                   AllowVerticalControl { get; init; }
    public required Vector3                StartPosition        { get; init; }
    public required float                  CompletionTolerance  { get; init; }
    public          IReadOnlyList<Vector3> Waypoints            { get; init; } = [];
    public          GroundCorridorPayload? GroundCorridor       { get; init; }
}
