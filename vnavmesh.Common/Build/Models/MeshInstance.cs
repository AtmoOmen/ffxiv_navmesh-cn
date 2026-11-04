using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Models;

namespace vnavmesh.Common.Build.Models;

public sealed class MeshInstance
(
    ulong          id,
    Matrix4x3      worldTransform,
    AABB           worldBounds,
    ulong          material,
    PrimitiveFlags forceSetPrimFlags,
    PrimitiveFlags forceClearPrimFlags,
    ulong          materialMask = 0
)
{
    public ulong          ID                  = id;
    public ulong          Material            = material;
    public ulong          MaterialMask        = materialMask;
    public Matrix4x3      WorldTransform      = worldTransform;
    public AABB           WorldBounds         = worldBounds;
    public PrimitiveFlags ForceSetPrimFlags   = forceSetPrimFlags;
    public PrimitiveFlags ForceClearPrimFlags = forceClearPrimFlags;

    public PrimitiveFlags GetPrimitiveFlags
    (
        Primitive primitive
    )
    {
        var flags = primitive.Flags;

        if (MaterialMask != 0)
        {
            var originalFlags     = CollisionMaterial.GetFlags(primitive.Material);
            var effectiveMaterial = (primitive.Material & ~MaterialMask) | (Material & MaterialMask);
            flags = (flags & ~originalFlags) | CollisionMaterial.GetFlags(effectiveMaterial);
        }

        return (flags & ~ForceClearPrimFlags) | ForceSetPrimFlags;
    }
}
