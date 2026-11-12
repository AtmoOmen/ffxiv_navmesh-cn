using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Common.Build;

public static class CollisionMaterial
{
    public static PrimitiveFlags GetFlags
    (
        ulong material
    )
    {
        if (!MatchesFilter(material, CollisionMaterialFlags.CharacterCollision, CollisionMaterialFlags.CharacterCollision))
            return PrimitiveFlags.Ignore;

        var flags = PrimitiveFlags.None;

        if (!MatchesFilter
            (
                material,
                CollisionMaterialFlags.CharacterCollision |
                CollisionMaterialFlags.WaterSurface       |
                CollisionMaterialFlags.FlightBoundary     |
                CollisionMaterialFlags.DiveEntry          |
                CollisionMaterialFlags.DiveExit,
                CollisionMaterialFlags.CharacterCollision
            ))
            flags |= PrimitiveFlags.FlyThrough;

        if (((CollisionMaterialFlags)material & CollisionMaterialFlags.Unlandable) != 0)
            flags |= PrimitiveFlags.Unlandable;

        return flags;
    }

    public static bool MatchesFilter
    (
        ulong                  material,
        CollisionMaterialFlags mask,
        CollisionMaterialFlags expected
    )
    {
        var selected = (CollisionMaterialFlags)material & mask;

        return expected     != CollisionMaterialFlags.None ?
                   selected == expected :
                   selected != CollisionMaterialFlags.None;
    }
}
