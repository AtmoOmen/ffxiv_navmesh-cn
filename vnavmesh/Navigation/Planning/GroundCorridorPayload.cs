using System.Numerics;
using vnavmesh.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Planning;

internal readonly record struct GroundPathCorner
(
    Vector3             Position,
    long                PolyRef,
    byte                StraightPathFlags,
    NavmeshArea         Area,
    NavmeshOffMeshKind? LinkKind
);

internal readonly record struct GroundLinkMarker
(
    int                CornerIndex,
    Vector3            Position,
    long               PolyRef,
    NavmeshOffMeshKind Kind
);

internal sealed class GroundCorridorPayload
{
    public required IReadOnlyList<long>             PolyRefs    { get; init; }
    public required Vector3                         Target      { get; init; }
    public required IReadOnlyList<GroundPathCorner> Corners     { get; init; }
    public required IReadOnlyList<GroundLinkMarker> LinkMarkers { get; init; }
}
