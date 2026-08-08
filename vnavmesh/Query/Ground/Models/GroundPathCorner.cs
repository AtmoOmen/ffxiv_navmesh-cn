using System.Numerics;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Query.Ground.Models;

internal readonly record struct GroundPathCorner
(
    Vector3             Position,
    long                PolyRef,
    byte                StraightPathFlags,
    NavmeshArea         Area,
    NavmeshOffMeshKind? LinkKind,
    int                 SourceIndex
);
