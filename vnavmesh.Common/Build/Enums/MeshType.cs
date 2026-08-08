namespace vnavmesh.Common.Build.Enums;

[Flags]
public enum MeshType
{
    None          = 0,
    Terrain       = 1 << 0,
    FileMesh      = 1 << 1,
    CylinderMesh  = 1 << 2,
    AnalyticShape = 1 << 3,
    AnalyticPlane = 1 << 4,
    All           = (1 << 5) - 1
}
