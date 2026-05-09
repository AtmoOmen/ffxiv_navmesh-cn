using System.Numerics;

namespace vnavmesh.Common.Navigation.Mesh.Runtime;

[Flags]
public enum NavmeshPolyFlags
{
    None               = 0,
    Ground             = 1 << 0,
    GeneratedClimbDown = 1 << 1,
    GeneratedEdgeJump  = 1 << 2,
    ManualOffMesh      = 1 << 3,
    Teleport           = 1 << 4,
    ClientPath         = 1 << 5,
    Unreachable        = 1 << 8,
    AllTraversable     = Ground | GeneratedClimbDown | GeneratedEdgeJump | ManualOffMesh | Teleport | ClientPath
}

public enum NavmeshArea
{
    Null               = 0,
    Ground             = 1,
    GeneratedClimbDown = 2,
    GeneratedEdgeJump  = 3,
    ManualOffMesh      = 4,
    Teleport           = 5,
    ClientPath         = 6
}

public enum NavmeshOffMeshKind
{
    GeneratedClimbDown,
    GeneratedEdgeJump,
    ManualOffMesh,
    Teleport,
    ClientPath
}

public readonly record struct NavmeshLink
(
    Vector3            Start,
    Vector3            End,
    NavmeshOffMeshKind Kind,
    bool               Bidirectional,
    int                UserId
);
