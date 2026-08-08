using System.Numerics;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Common.Build.Models;

public readonly record struct OffMeshConnection
(
    Vector3                      Start,
    Vector3                      End,
    float                        Radius,
    bool                         Bidirectional,
    int                          UserId,
    int                          Area,
    int                          Flags,
    int                          Kind,
    NavmeshLinkTraversalProfile? TraversalProfile = null
);
