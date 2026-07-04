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

        var tailStartIndex = head[^1].voxel == tail[0].voxel && Vector3.DistanceSquared(head[^1].p, tail[0].p) <= epsilon * epsilon ?
                                 1 :
                                 0;
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
}
