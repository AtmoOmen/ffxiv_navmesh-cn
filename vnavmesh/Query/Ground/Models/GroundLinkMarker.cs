using System.Numerics;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Query.Ground;

internal readonly record struct GroundLinkMarker
(
    int                          CornerIndex,
    Vector3                      Position,
    long                         PolyRef,
    NavmeshOffMeshKind           Kind,
    NavmeshLinkTraversalProfile? TraversalProfile       = null,
    float                        EstimatedTraversalCost = 0f
);
