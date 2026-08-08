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
    PrimitiveFlags forceClearPrimFlags
)
{
    public ulong          ID                  = id;
    public ulong          Material            = material;
    public Matrix4x3      WorldTransform      = worldTransform;
    public AABB           WorldBounds         = worldBounds;
    public PrimitiveFlags ForceSetPrimFlags   = forceSetPrimFlags;
    public PrimitiveFlags ForceClearPrimFlags = forceClearPrimFlags;
}
