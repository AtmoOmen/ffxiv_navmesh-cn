using System.Numerics;

namespace vnavmesh.Navigation.Volume;

public static class VoxelSearch
{
    public static IEnumerable<(ulong index, bool empty)> EnumerateLeafVoxels(VoxelMap volume, Vector3 center, Vector3 halfExtent)
        => volume.RootTile.EnumerateLeafVoxels(center - halfExtent, center + halfExtent);

    public static Vector3 FindClosestVoxelPoint(VoxelMap volume, ulong index, Vector3 p, float eps = 0.1f)
        => volume.ClampPointToVoxel(index, p, eps);

    public static ulong FindNearestEmptyVoxel(VoxelMap volume, Vector3 center, Vector3 halfExtent)
    {
        var cv = volume.FindLeafVoxel(center);
        //Service.Log.Debug($"Searching {cv}");
        if (cv.empty)
            return cv.voxel; // fast path: the cell is empty already

        var minDist = float.MaxValue;
        var res     = VoxelMap.InvalidVoxel;

        foreach (var v in volume.RootTile.EnumerateLeafVoxels(center - halfExtent, center + halfExtent))
        {
            if (!v.empty)
                continue;

            var p    = FindClosestVoxelPoint(volume, v.index, center, 0);
            var d    = p - center;
            var dist = d.LengthSquared();
            if (d.X != 0 || d.Z != 0)
                dist += 100; // penalty for moving sideways vs up - TODO reconsider...
            if (d.Y < 0)
                dist += 400; // penalty for lower voxels to reduce chance of it being underground - TODO reconsider...
            // Service.Log.Debug($"Considering {v.index:X} @ {p.X}x{p.Y}x{p.Z}: {dist}, min so far {minDist}");

            if (dist < minDist)
            {
                minDist = dist;
                res     = v.index;
            }
        }

        return res;
    }

    // enumerate entered voxels along line; starting voxel is not returned, ending voxel is
    public static IEnumerable<(ulong voxel, float t, bool empty)> EnumerateVoxelsInLine
        (VoxelMap volume, ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        if (fromVoxel == toVoxel || Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            yield break;

        var origFrom = fromVoxel;
        var ab       = toPos - fromPos;
        var eps      = 0.1f / ab.Length();

        while (fromVoxel != toVoxel)
        {
            StepLine(volume, origFrom, toVoxel, fromPos, ab, eps, fromVoxel, out var nextVoxel, out var t, out var nextEmpty);
            yield return (nextVoxel, t, nextEmpty);
            fromVoxel = nextVoxel;
        }
    }

    public static bool LineOfSight(VoxelMap volume, ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        if (fromVoxel == toVoxel)
            return true;
        if (Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            return false;

        var origFrom = fromVoxel;
        var ab       = toPos - fromPos;
        var eps      = 0.1f / ab.Length();

        while (fromVoxel != toVoxel)
        {
            StepLine(volume, origFrom, toVoxel, fromPos, ab, eps, fromVoxel, out var nextVoxel, out _, out var nextEmpty);
            if (!nextEmpty)
                return false;

            fromVoxel = nextVoxel;
        }

        return true;
    }

    private static void StepLine
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
        var (vmin, vmax) = volume.VoxelBounds(fromVoxel, 0);
        var tx = ab.X == 0 ? float.MaxValue : ((ab.X > 0 ? vmax.X : vmin.X) - fromPos.X) / ab.X;
        var ty = ab.Y == 0 ? float.MaxValue : ((ab.Y > 0 ? vmax.Y : vmin.Y) - fromPos.Y) / ab.Y;
        var tz = ab.Z == 0 ? float.MaxValue : ((ab.Z > 0 ? vmax.Z : vmin.Z) - fromPos.Z) / ab.Z;
        t = Math.Min(Math.Min(tx, ty), Math.Min(tz, 1));
        var tAdj = Math.Min(t + eps, 1);
        var proj = fromPos + tAdj * ab;
        (nextVoxel, nextEmpty) = volume.FindLeafVoxel(proj);
        if (nextVoxel == fromVoxel)
            throw new PathfindLoopException(origFrom, toVoxel, fromPos, fromPos + ab);
    }
}
