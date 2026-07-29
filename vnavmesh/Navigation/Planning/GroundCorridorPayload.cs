using System.Numerics;
using vnavmesh.Common.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Planning;

internal readonly record struct GroundPathCorner
(
    Vector3                Position,
    long                   PolyRef,
    byte                   StraightPathFlags,
    NavmeshArea            Area,
    NavmeshOffMeshKind?    LinkKind,
    int                    SourceIndex
);

internal readonly record struct GroundLinkMarker
(
    int                          CornerIndex,
    Vector3                      Position,
    long                         PolyRef,
    NavmeshOffMeshKind           Kind,
    NavmeshLinkTraversalProfile? TraversalProfile       = null,
    float                        EstimatedTraversalCost = 0f
);

internal sealed class GroundCorridorPayload
{
    public required IReadOnlyList<long>             PolyRefs             { get; init; }
    public required Vector3                         Target               { get; init; }
    public required int                             InitialCornerIndex   { get; init; }
    public required IReadOnlyList<GroundPathCorner> RawCorners           { get; init; }
    public required IReadOnlyList<GroundPathCorner> Corners              { get; init; }
    public required IReadOnlyList<GroundLinkMarker> LinkMarkers          { get; init; }
}
