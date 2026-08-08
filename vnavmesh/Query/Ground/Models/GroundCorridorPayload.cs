using System.Numerics;

namespace vnavmesh.Query.Ground.Models;

internal sealed class GroundCorridorPayload
{
    public required IReadOnlyList<long>             PolyRefs           { get; init; }
    public required Vector3                         Target             { get; init; }
    public required int                             InitialCornerIndex { get; init; }
    public required IReadOnlyList<GroundPathCorner> RawCorners         { get; init; }
    public required IReadOnlyList<GroundPathCorner> Corners            { get; init; }
    public required IReadOnlyList<GroundLinkMarker> LinkMarkers        { get; init; }
}
