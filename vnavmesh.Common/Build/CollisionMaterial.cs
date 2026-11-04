using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Common.Build;

public static class CollisionMaterial
{
    private static readonly ulong[] MaterialsFlyThrough =
    [
        0x100000,  // generally set on the invisible walls surrounding walkable areas that can be flown from
        0x1000000, // if this bit is set, flying upwards into the surface will trigger dive -> fly (or swim) transition
        0x800000,  // not really sure what this is, appears on invisible roof of divable zones

        // 0xBC00 can be dived into, but 0xB800 cannot. both allow fishing
        // 0x400 is some kind of generic "this collider is conditionally active" flag, but it's set on zone walls, so we can't skip it
        0xB400
    ];

    public static PrimitiveFlags GetFlags
    (
        ulong mat
    )
    {
        var res = PrimitiveFlags.None;

        if ((mat & 0x410) == 0x400)
            // 这类表面不参与地面导航，但仍会阻挡飞行。
            res |= PrimitiveFlags.VolumeOnly;

        foreach (var fly in MaterialsFlyThrough)
        {
            if ((mat & fly) == fly)
                res |= PrimitiveFlags.FlyThrough;
        }

        if ((mat & 0x200000) != 0)
            res |= PrimitiveFlags.Unlandable;

        if ((mat & 0x2000000) != 0)
            res |= PrimitiveFlags.ForceUnwalkable;

        // 0x11 is set on all the invisible walls surrounding every zone; some are not marked as unlandable so we can't just use that
        // some regular terrain materials have 0x10 set as well (see flowers in il mheg) which is why we check for both bits here
        if ((mat & 0x1F) == 0x11)
            res |= PrimitiveFlags.Unlandable | PrimitiveFlags.ForceUnwalkable;

        if ((mat & 0x8000) != 0)
            res |= PrimitiveFlags.Fishable;

        return res;
    }
}
