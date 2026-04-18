using System.Numerics;
using vnavmesh.Navigation.Volume.Map;

namespace vnavmesh.Navigation.Volume.Search;

public static class VoxelSearch
{
    public static Vector3 FindClosestVoxelPoint(VoxelMap volume, ulong index, Vector3 p, float eps = 0.1f) =>
        volume.ClampPointToVoxel(index, p, eps);

    public static ulong FindNearestEmptyVoxel(VoxelMap volume, Vector3 center, Vector3 halfExtent)
    {
        var centerLeaf = volume.FindLeafVoxel(center);
        if (centerLeaf.empty)
            return centerLeaf.voxel;

        var minDist = float.MaxValue;
        var nearestVoxel = VoxelMap.INVALID_VOXEL;

        foreach (var v in volume.RootTile.EnumerateLeafVoxels(center - halfExtent, center + halfExtent))
        {
            if (!v.empty)
                continue;

            var p    = FindClosestVoxelPoint(volume, v.index, center, 0);
            var d    = p - center;
            var dist = d.LengthSquared();
            if (d.X != 0 || d.Z != 0)
                dist += 100;
            if (d.Y < 0)
                dist += 400;

            if (dist < minDist)
            {
                minDist = dist;
                nearestVoxel = v.index;
            }
        }

        return nearestVoxel;
    }

    public static IEnumerable<(ulong voxel, float t, bool empty)> EnumerateVoxelsInLine
    (
        VoxelMap volume,
        ulong    fromVoxel,
        ulong    toVoxel,
        Vector3  fromPos,
        Vector3  toPos
    )
    {
        if (fromVoxel == toVoxel || Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            yield break;

        var line = CreateLineState(fromVoxel, toPos - fromPos);

        while (fromVoxel != toVoxel)
        {
            StepToNextVoxel(volume, line.originVoxel, toVoxel, fromPos, line.delta, line.epsilon, fromVoxel, out var nextVoxel, out var t, out var nextEmpty);
            yield return (nextVoxel, t, nextEmpty);
            fromVoxel = nextVoxel;
        }
    }

    public static bool LineOfSight
    (
        VoxelMap volume,
        ulong    fromVoxel,
        ulong    toVoxel,
        Vector3  fromPos,
        Vector3  toPos
    )
    {
        if (fromVoxel == toVoxel)
            return true;
        if (Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            return false;

        var line = CreateLineState(fromVoxel, toPos - fromPos);

        while (fromVoxel != toVoxel)
        {
            StepToNextVoxel(volume, line.originVoxel, toVoxel, fromPos, line.delta, line.epsilon, fromVoxel, out var nextVoxel, out _, out var nextEmpty);
            if (!nextEmpty)
                return false;

            fromVoxel = nextVoxel;
        }

        return true;
    }

    private static (ulong originVoxel, Vector3 delta, float epsilon) CreateLineState(ulong fromVoxel, Vector3 delta)
        => (fromVoxel, delta, 0.1f / delta.Length());

    private static void StepToNextVoxel
    (
        VoxelMap  volume,
        ulong     origFrom,
        ulong     toVoxel,
        Vector3   fromPos,
        Vector3   ab,
        float     eps,
        ulong     fromVoxel,
        out ulong nextVoxel,
        out float t,
        out bool  nextEmpty
    )
    {
        var (vMin, vMax) = volume.VoxelBounds(fromVoxel, 0);
        
        var tx = ab.X == 0 ? float.MaxValue : ((ab.X > 0 ? vMax.X : vMin.X) - fromPos.X) / ab.X;
        var ty = ab.Y == 0 ? float.MaxValue : ((ab.Y > 0 ? vMax.Y : vMin.Y) - fromPos.Y) / ab.Y;
        var tz = ab.Z == 0 ? float.MaxValue : ((ab.Z > 0 ? vMax.Z : vMin.Z) - fromPos.Z) / ab.Z;
        
        t = MathF.Min(MathF.Min(tx, ty), MathF.Min(tz, 1));
        
        var tAdj = MathF.Min(t + eps, 1);
        var proj = fromPos + tAdj * ab;
        
        (nextVoxel, nextEmpty) = volume.FindLeafVoxel(proj);
        if (nextVoxel == fromVoxel)
            throw new PathfindLoopException(origFrom, toVoxel, fromPos, fromPos + ab);
    }
}
