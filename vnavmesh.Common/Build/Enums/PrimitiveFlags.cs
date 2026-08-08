namespace vnavmesh.Common.Build.Enums;

[Flags]
public enum PrimitiveFlags
{
    None            = 0,
    ForceUnwalkable = 1 << 0,
    FlyThrough      = 1 << 1,
    Unlandable      = 1 << 2,
    ForceWalkable   = 1 << 3,
    Fishable        = 1 << 4
}
