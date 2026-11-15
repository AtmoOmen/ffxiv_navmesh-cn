using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Extensions;

namespace vnavmesh.Query.Utils;

internal sealed class MeshFloorPointQuery
(
    DtNavMesh mesh,
    RcVec3f   center
) : IDtPolyQuery
{
    internal Vector3? Point { get; private set; }

    public void Process(DtMeshTile tile, ReadOnlySpan<int> polys, ReadOnlySpan<long> refs, int count)
    {
        for (var index = 0; index < count; ++index)
        {
            mesh.ClosestPointOnPoly(refs[index], center, out var closest, out _);
            if (closest.Y <= center.Y && (Point == null || closest.Y > Point.Value.Y))
                Point = closest.ToSystem();
        }
    }
}
