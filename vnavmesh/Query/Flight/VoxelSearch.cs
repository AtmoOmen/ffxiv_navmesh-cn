using System.Numerics;
using vnavmesh.Common.Build.Flight;

namespace vnavmesh.Query.Flight;

public static class VoxelSearch
{
    private const float UpwardVoxelPreferencePenaltyScale  = 12f;
    private const float UpwardVoxelPreferencePenaltyLinear = 4f;
    private const float DownwardVoxelPreferencePenalty     = 0.5f;
    private const float BelowFloorVoxelBottomPenaltyScale  = 96f;
    private const float BelowFloorVoxelBottomPenaltyLinear = 12f;
    private const int   MAX_LINE_STEP_ITERATIONS           = 8192;
    private const int   MAX_NEAREST_BFS_CELLS              = 100_000;

    public static Vector3 FindClosestVoxelPoint
    (
        SparseVoxelOctree volume,
        ulong             index,
        Vector3           p,
        float             eps = 0.1f
    ) =>
        volume.ClampPointToVoxel(index, p, eps);

    public static ulong FindNearestEmptyVoxel
    (
        SparseVoxelOctree volume,
        Vector3           center,
        Vector3           halfExtent,
        bool              preferNonBelow = false,
        float?            minCandidateY  = null
    )
    {
        var centerLeaf = volume.FindLeafVoxel(center);

        if (centerLeaf.empty && minCandidateY == null)
            return centerLeaf.voxel;

        var leafSize  = volume.LeafSize;
        var cellCount = 1 << volume.MaxDepth;
        var searchMinX = Math.Max((int)((center.X - halfExtent.X - volume.BoundsMin.X) / leafSize), 0);
        var searchMinY = Math.Max((int)((center.Y - halfExtent.Y - volume.BoundsMin.Y) / leafSize), 0);
        var searchMinZ = Math.Max((int)((center.Z - halfExtent.Z - volume.BoundsMin.Z) / leafSize), 0);
        var maxX = Math.Min((int)((center.X + halfExtent.X - volume.BoundsMin.X) / leafSize), cellCount - 1);
        var maxY = Math.Min((int)((center.Y + halfExtent.Y - volume.BoundsMin.Y) / leafSize), cellCount - 1);
        var maxZ = Math.Min((int)((center.Z + halfExtent.Z - volume.BoundsMin.Z) / leafSize), cellCount - 1);

        if (searchMinX > maxX || searchMinY > maxY || searchMinZ > maxZ)
            return SparseVoxelOctree.INVALID_VOXEL;

        var startX = Math.Clamp((int)((center.X - volume.BoundsMin.X) / leafSize), 0, cellCount - 1);
        var startY = Math.Clamp((int)((center.Y - volume.BoundsMin.Y) / leafSize), 0, cellCount - 1);
        var startZ = Math.Clamp((int)((center.Z - volume.BoundsMin.Z) / leafSize), 0, cellCount - 1);

        var queue   = new Queue<(int X, int Y, int Z)>();
        var visited = new HashSet<int>();
        queue.Enqueue((startX, startY, startZ));
        visited.Add(((startX * cellCount) + startY) * cellCount + startZ);

        var minDist          = float.MaxValue;
        var nearestVoxel     = SparseVoxelOctree.INVALID_VOXEL;
        var preferredMinDist = float.MaxValue;
        var preferredVoxel   = SparseVoxelOctree.INVALID_VOXEL;
        var belowFloorSlack  = MathF.Max(leafSize * 0.5f, 0.25f);
        var popped           = 0;

        while (queue.Count > 0 && popped < MAX_NEAREST_BFS_CELLS)
        {
            var (x, y, z) = queue.Dequeue();
            ++popped;

            var leafMin = volume.BoundsMin + new Vector3(x * leafSize, y * leafSize, z * leafSize);
            var leafVoxel = SparseVoxelOctree.EncodeCoord(volume.MaxDepth, x, y, z);
            var located = volume.FindLeafVoxel(leafMin + new Vector3(leafSize * 0.5f));

            if (located.empty)
            {
                var p = volume.ClampPointToVoxel(leafVoxel, center, 0);

                if (minCandidateY is { } minY && p.Y + float.Epsilon < minY)
                    continue;

                var d        = p - center;
                var dist     = d.LengthSquared();
                var upward   = MathF.Max(d.Y, 0f);
                var downward = MathF.Max(-d.Y, 0f);
                dist += upward   * upward * UpwardVoxelPreferencePenaltyScale;
                dist += upward   * UpwardVoxelPreferencePenaltyLinear;
                dist += downward * DownwardVoxelPreferencePenalty;

                if (minCandidateY is { } floorMinY)
                {
                    var bottomPenetration = MathF.Max(floorMinY - (leafMin.Y + belowFloorSlack), 0f);
                    dist += bottomPenetration * bottomPenetration * BelowFloorVoxelBottomPenaltyScale;
                    dist += bottomPenetration * BelowFloorVoxelBottomPenaltyLinear;
                }

                if (p.Y + float.Epsilon >= center.Y && dist < preferredMinDist)
                {
                    preferredMinDist = dist;
                    preferredVoxel   = leafVoxel;
                }

                if (dist < minDist)
                {
                    minDist      = dist;
                    nearestVoxel = leafVoxel;
                }
            }

            for (var dx = -1; dx <= 1; ++dx)
            for (var dy = -1; dy <= 1; ++dy)
            for (var dz = -1; dz <= 1; ++dz)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;

                var nx = x + dx;
                var ny = y + dy;
                var nz = z + dz;

                if (nx < searchMinX || nx > maxX || ny < searchMinY || ny > maxY || nz < searchMinZ || nz > maxZ)
                    continue;

                var key = ((nx * cellCount) + ny) * cellCount + nz;

                if (!visited.Add(key))
                    continue;

                queue.Enqueue((nx, ny, nz));
            }
        }

        return preferNonBelow && preferredVoxel != SparseVoxelOctree.INVALID_VOXEL ?
                   preferredVoxel :
                   nearestVoxel;
    }

    public static IEnumerable<(ulong voxel, float t, bool empty)> EnumerateVoxelsInLine
    (
        SparseVoxelOctree volume,
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos
    )
    {
        if (fromVoxel == toVoxel || Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            yield break;

        var line       = CreateLineState(toPos - fromPos);
        var iterations = 0;
        var prevVoxel  = fromVoxel;

        while (fromVoxel != toVoxel)
        {
            if (++iterations > MAX_LINE_STEP_ITERATIONS)
                yield break;

            if (!StepToNextVoxel(volume, fromPos, line.delta, line.epsilon, fromVoxel, out var nextVoxel, out var t, out var nextEmpty))
                yield break;

            if (nextVoxel == prevVoxel)
                yield break;

            yield return (nextVoxel, t, nextEmpty);
            prevVoxel = fromVoxel;
            fromVoxel = nextVoxel;
        }
    }

    public static bool LineOfSight
    (
        SparseVoxelOctree volume,
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos
    )
    {
        if (fromVoxel == toVoxel)
            return true;
        if (Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            return false;

        if (volume.TryLineOfSightDDA(fromPos, toPos, out var ddaResult))
            return ddaResult;

        var line       = CreateLineState(toPos - fromPos);
        var iterations = 0;
        var prevVoxel  = fromVoxel;

        while (fromVoxel != toVoxel)
        {
            if (++iterations > MAX_LINE_STEP_ITERATIONS)
                return false;

            if (!StepToNextVoxel(volume, fromPos, line.delta, line.epsilon, fromVoxel, out var nextVoxel, out _, out var nextEmpty))
                return false;

            if (!nextEmpty)
                return false;
            if (nextVoxel == prevVoxel)
                return false;

            prevVoxel = fromVoxel;
            fromVoxel = nextVoxel;
        }

        return true;
    }

    private static (Vector3 delta, float epsilon) CreateLineState
    (
        Vector3 delta
    ) => (delta, MathF.Max(0.1f / delta.Length(), 1e-6f));

    private static bool StepToNextVoxel
    (
        SparseVoxelOctree volume,
        Vector3           fromPos,
        Vector3           ab,
        float             eps,
        ulong             fromVoxel,
        out ulong         nextVoxel,
        out float         t,
        out bool          nextEmpty
    )
    {
        var (vMin, vMax) = volume.TryGetLeafVoxelBounds(fromVoxel, out var leafBounds) ?
                               leafBounds :
                               volume.VoxelBounds(fromVoxel, 0);

        var tx = ab.X == 0 ?
                     float.MaxValue :
                     ((ab.X > 0 ? vMax.X : vMin.X) - fromPos.X) / ab.X;
        var ty = ab.Y == 0 ?
                     float.MaxValue :
                     ((ab.Y > 0 ? vMax.Y : vMin.Y) - fromPos.Y) / ab.Y;
        var tz = ab.Z == 0 ?
                     float.MaxValue :
                     ((ab.Z > 0 ? vMax.Z : vMin.Z) - fromPos.Z) / ab.Z;

        t = MathF.Min(MathF.Min(tx, ty), MathF.Min(tz, 1));
        var tAdj = MathF.Min(t + eps, 1);
        var proj = fromPos + (tAdj * ab);

        if (volume.TryFindLeafVoxelFast(proj, out var fastVoxel, out var fastEmpty))
        {
            nextVoxel  = fastVoxel;
            nextEmpty  = fastEmpty;
            return nextVoxel != fromVoxel;
        }

        (nextVoxel, nextEmpty) = volume.FindLeafVoxel(proj);
        return nextVoxel != fromVoxel;
    }
}
