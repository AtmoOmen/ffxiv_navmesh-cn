namespace vnavmesh.Common.Build.Enums;

[Flags]
public enum CollisionMaterialFlags : ulong
{
    None                   = 0,
    WaterSurface           = 0x800,
    CharacterCollision     = 0x2000,
    FlightBoundary         = 0x100000,
    Unlandable             = 0x200000,
    DiveEntry              = 0x800000,
    DiveExit               = 0x1000000,
    MapRange0              = 0x2000000,
    MapRange1              = 0x4000000,
    MapRange2              = 0x100000000,
    MapRange3              = 0x200000000,
    ConditionalPenetration = 0x800000000
}
