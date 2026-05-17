using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Map;

namespace vnavmesh.Common.Navigation.Volume.Search;

public static class VoxelSearch
{
    private const float UpwardVoxelPreferencePenaltyScale  = 12f;
    private const float UpwardVoxelPreferencePenaltyLinear = 4f;
    private const float DownwardVoxelPreferencePenalty     = 0.5f;
    private const float BelowFloorVoxelBottomPenaltyScale  = 96f;
    private const float BelowFloorVoxelBottomPenaltyLinear = 12f;

    public static Vector3 FindClosestVoxelPoint(VoxelMap volume, ulong index, Vector3 p, float eps = 0.1f) =>
        volume.ClampPointToVoxel(index, p, eps);

    public static ulong FindNearestEmptyVoxel
    (
        VoxelMap volume,
        Vector3  center,
        Vector3  halfExtent,
        bool     preferNonBelow = false,
        float?   minCandidateY  = null
    )
    {
        var centerLeaf = volume.FindLeafVoxel(center);
        if (centerLeaf.empty && minCandidateY == null)
        {
            return centerLeaf.voxel;
        }

        var minDist            = float.MaxValue;
        var nearestVoxel       = VoxelMap.INVALID_VOXEL;
        var preferredMinDist   = float.MaxValue;
        var preferredVoxel     = VoxelMap.INVALID_VOXEL;
        var belowFloorSlack    = MathF.Max(volume.Levels[^1].CellSize.Y * 0.5f, 0.25f);

        foreach (var v in volume.RootTile.EnumerateLeafVoxels(center - halfExtent, center + halfExtent))
        {
            if (!v.empty)
                continue;

            var p          = FindClosestVoxelPoint(volume, v.index, center, 0);
            if (minCandidateY is { } minY && p.Y + float.Epsilon < minY)
                continue;

            var d          = p - center;
            var dist       = d.LengthSquared();
            var upward     = MathF.Max(d.Y, 0f);
            var downward   = MathF.Max(-d.Y, 0f);
            dist          += upward * upward * UpwardVoxelPreferencePenaltyScale;
            dist          += upward * UpwardVoxelPreferencePenaltyLinear;
            dist          += downward * DownwardVoxelPreferencePenalty;

            if (minCandidateY is { } floorMinY)
            {
                var bounds            = volume.VoxelBounds(v.index, 0);
                var bottomPenetration = MathF.Max(floorMinY - (bounds.min.Y + belowFloorSlack), 0f);
                dist                 += bottomPenetration * bottomPenetration * BelowFloorVoxelBottomPenaltyScale;
                dist                 += bottomPenetration * BelowFloorVoxelBottomPenaltyLinear;
            }

            if (p.Y + float.Epsilon >= center.Y && dist < preferredMinDist)
            {
                preferredMinDist = dist;
                preferredVoxel   = v.index;
            }

            if (dist < minDist)
            {
                minDist      = dist;
                nearestVoxel = v.index;
            }
        }

        return preferNonBelow && preferredVoxel != VoxelMap.INVALID_VOXEL ? preferredVoxel : nearestVoxel;
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
