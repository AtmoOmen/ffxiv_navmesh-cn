namespace vnavmesh.UI.Editor.Types;

internal enum PickKind
{
    None,
    SelectCollider,
    SelectTriangle,
    Aabb,
    OrientedBox,
    Cylinder,
    OrientedCylinder,
    Sphere,
    Wall,
    Ramp,
    RemoveInstancesVolume,
    SetInstanceFlagsVolume,
    LinkPoints,
    LinkShortcut,
    LinkClientPath,
    OffMesh
}
