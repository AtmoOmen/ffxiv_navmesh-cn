using System.Numerics;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Build.Ground.Enums;
using vnavmesh.Common.Build.Ground.Models;

namespace vnavmesh.Query.Ground.Models;

internal readonly record struct GroundLinkMarker
(
    int                          CornerIndex,
    Vector3                      Position,
    long                         PolyRef,
    NavmeshOffMeshKind           Kind,
    NavmeshLinkTraversalProfile? TraversalProfile       = null,
    float                        EstimatedTraversalCost = 0f
);
