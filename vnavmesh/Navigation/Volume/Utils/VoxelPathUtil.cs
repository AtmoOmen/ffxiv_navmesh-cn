using System.Numerics;

namespace vnavmesh.Navigation.Volume.Utils;

public static class VoxelPathUtil
{
    public static int DetermineEffectiveLookBackDepth(float currentBestScore, float currentNodeGScore, float epsilon)
    {
        const int MAX_ANCESTOR_LOOK_BACK = 6;

        if (currentBestScore == float.MaxValue)
            return MAX_ANCESTOR_LOOK_BACK;

        var improvementRatio = (currentNodeGScore - currentBestScore) / MathF.Max(currentNodeGScore, epsilon);
        if (improvementRatio < 0.05f)
            return 2;
        if (improvementRatio < 0.15f)
            return 4;

        return MAX_ANCESTOR_LOOK_BACK;
    }

    public static List<(ulong voxel, Vector3 p)> MergePathSegments(List<(ulong voxel, Vector3 p)> head, List<(ulong voxel, Vector3 p)> tail, float epsilon)
    {
        if (head.Count == 0)
            return tail;
        if (tail.Count == 0)
            return head;

        List<(ulong voxel, Vector3 p)> merged = new(head.Count + tail.Count);
        merged.AddRange(head);

        var tailStartIndex = head[^1].voxel == tail[0].voxel && Vector3.DistanceSquared(head[^1].p, tail[0].p) <= epsilon * epsilon ? 1 : 0;
        for (var i = tailStartIndex; i < tail.Count; ++i)
            merged.Add(tail[i]);

        return merged;
    }

    public static void AppendPathPoint(List<(ulong voxel, Vector3 p)> output, (ulong voxel, Vector3 p) point, float epsilon)
    {
        if (output.Count > 0 && Vector3.DistanceSquared(output[^1].p, point.p) <= epsilon * epsilon)
            return;

        output.Add(point);
    }

    public static int FindPathPointIndex(IReadOnlyList<(ulong voxel, Vector3 p)> path, (ulong voxel, Vector3 p) point, int startIndex, float epsilon)
    {
        for (var i = Math.Max(0, startIndex); i < path.Count; ++i)
            if (Vector3.DistanceSquared(path[i].p, point.p) <= epsilon * epsilon)
                return i;

        return -1;
    }

    public static bool IsConstrainedTunnelDescent
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize,
        float                    upClearance,
        float                    preferredHeightTarget,
        float                    minDistance,
        float                    trendToleranceLeafScale,
        float                    trendToleranceMin,
        float                    catchupHeadroomLeafScale,
        float                    catchupHeadroomMin
    )
    {
        var descentTolerance = MathF.Max(leafVerticalSize * trendToleranceLeafScale, trendToleranceMin);
        if (next.p.Y + descentTolerance >= current.p.Y &&
            next.p.Y + descentTolerance >= previous.p.Y)
            return false;

        var targetLift = MathF.Max(0f, preferredHeightTarget - current.p.Y);
        if (targetLift < minDistance)
            return false;

        var requiredHeadroom = targetLift + MathF.Max(leafVerticalSize * catchupHeadroomLeafScale, catchupHeadroomMin);
        return upClearance < requiredHeadroom;
    }

    public static bool IsTunnelDescentTrend
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize,
        float                    trendToleranceLeafScale,
        float                    trendToleranceMin
    )
    {
        var descentTolerance = MathF.Max(leafVerticalSize * trendToleranceLeafScale, trendToleranceMin);
        return next.p.Y    + descentTolerance < current.p.Y  ||
               current.p.Y + descentTolerance < previous.p.Y ||
               next.p.Y    + descentTolerance < previous.p.Y;
    }

    public static bool TryResolveConstrainedTunnelDescentOffset
    (
        bool                     constrainedTunnelDescent,
        bool                     downhillTunnelTrend,
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize,
        float                    voxelVertical,
        float                    verticalScanDistance,
        float                    upClearance,
        float                    downClearance,
        float                    minDistance,
        float                    softHeadroomVoxelScale,
        float                    softHeadroomLeafScale,
        float                    downwardClearanceVoxelScale,
        float                    downwardClearanceLeafScale,
        float                    clearanceLeadLeafScale,
        float                    clearanceLeadMin,
        float                    followNextScale,
        float                    previousFollowScale,
        float                    extraFollowLeafScale,
        float                    clearanceAdvantageScale,
        float                    scanPushFraction,
        float                    maxClearanceFraction,
        out Vector3              verticalOffset
    )
    {
        verticalOffset = default;
        if (!constrainedTunnelDescent && !downhillTunnelTrend)
            return false;

        var clearanceAdvantage = MathF.Max(0f,       downClearance - upClearance);
        var nextDrop           = MathF.Max(0f,       current.p.Y   - next.p.Y);
        var previousDrop       = MathF.Max(0f,       previous.p.Y  - current.p.Y);
        var trendDrop          = MathF.Max(nextDrop, previousDrop * previousFollowScale);
        var minimumLead        = MathF.Max(leafVerticalSize       * clearanceLeadLeafScale, clearanceLeadMin);
        if (trendDrop          < minDistance &&
            clearanceAdvantage < minimumLead)
            return false;

        var downwardClearanceFloor = MathF.Max
            (voxelVertical * downwardClearanceVoxelScale, leafVerticalSize * downwardClearanceLeafScale);
        if (downClearance < downwardClearanceFloor)
            return false;

        if (!constrainedTunnelDescent)
        {
            var softHeadroomLimit = MathF.Max
                (voxelVertical * softHeadroomVoxelScale, leafVerticalSize * softHeadroomLeafScale);
            if (upClearance > softHeadroomLimit || clearanceAdvantage < minimumLead)
                return false;
        }

        var desiredVerticalPush = MathF.Max
        (
            trendDrop          * followNextScale,
            clearanceAdvantage * clearanceAdvantageScale
        );
        var maxVerticalPush = MathF.Min(verticalScanDistance * scanPushFraction, downClearance * maxClearanceFraction);
        var pushCap = constrainedTunnelDescent
                          ? Math.Max(nextDrop, trendDrop)
                          : Math.Max(nextDrop, trendDrop + (leafVerticalSize * extraFollowLeafScale));
        var verticalPushDistance = MathF.Min(pushCap, MathF.Min(maxVerticalPush, desiredVerticalPush));
        if (verticalPushDistance < minDistance)
            return false;

        verticalOffset = -Vector3.UnitY * verticalPushDistance;
        return true;
    }
}
