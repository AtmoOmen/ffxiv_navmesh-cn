using System.Numerics;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Enums;
using vnavmesh.Query.Ground.Models;

namespace vnavmesh.Query.Models;

internal sealed class PlannerPathSegment
{
    public required MovementMode               MovementMode           { get; init; }
    public required MovementSegmentKind        SegmentKind            { get; init; }
    public required bool                       AllowVerticalControl   { get; init; }
    public required PlannerSegmentGeometryKind GeometryKind           { get; init; }
    public required Vector3                    TraversalStartPosition { get; init; }
    public required Vector3                    StartPosition          { get; init; }
    public required Vector3                    EndPosition            { get; init; }

    public IReadOnlyList<long>    Corridor       { get; init; } = [];
    public IReadOnlyList<Vector3> Points         { get; init; } = [];
    public GroundCorridorPayload? GroundCorridor { get; init; }
}
