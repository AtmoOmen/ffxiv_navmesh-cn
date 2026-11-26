using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight;

namespace vnavmesh.Query.Utils;

internal sealed class MeshFloorPointQuery
(
    DtNavMesh           mesh,
    RcVec3f             center,
    VoxelPathfind?      flight         = null,
    ulong               sourceVoxel    = VoxelMap.INVALID_VOXEL,
    IReadOnlySet<long>? reachablePolys = null
) : IDtPolyQuery
{
    internal Vector3? Point { get; private set; }

    public void Process(DtMeshTile tile, ReadOnlySpan<int> polys, ReadOnlySpan<long> refs, int count)
    {
        for (var index = 0; index < count; ++index)
        {
            if (reachablePolys != null &&
                !reachablePolys.Contains(refs[index]))
                continue;

            var poly = tile.data.polys[polys[index]];
            if (!mesh.GetPolyHeight(tile, poly, center, out var height) ||
                height > center.Y                                       ||
                !float.IsFinite(height))
                continue;

            if (Point is { } current &&
                height <= current.Y)
                continue;

            if (flight != null)
            {
                var clearance = MathF.Max(tile.data.header.walkableHeight * 0.5f, flight.Volume.Levels[^1].CellSize.Y);
                var leaf      = flight.Volume.FindLeafVoxel(new(center.X, height + clearance, center.Z));
                if (!leaf.empty                          ||
                    leaf.voxel == VoxelMap.INVALID_VOXEL ||
                    !flight.IsConnected(sourceVoxel, leaf.voxel))
                    continue;
            }

            Point = new(center.X, height, center.Z);
        }
    }
}
