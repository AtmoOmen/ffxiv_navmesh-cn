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
    WalkableFloor,
    Wall,
    Ramp,
    RemoveInstancesVolume,
    SetInstanceFlagsVolume,
    LinkPoints,
    LinkShortcut,
    LinkClientPath,
    OffMesh
}
