using System.Runtime.CompilerServices;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float TotalScore(float gScore, float hScore) => gScore + (hScore * heuristicWeight);

    private void UpdateBestNode(int nodeIndex)
    {
        if (nodes.Count == 0 || IsBetterBestNode(nodeIndex, bestNodeIndex))
            bestNodeIndex = nodeIndex;
    }

    private bool IsBetterBestNode(int candidateIndex, int currentIndex)
    {
        var     nodeSpan  = NodeSpan;
        ref var candidate = ref nodeSpan[candidateIndex];
        ref var current   = ref nodeSpan[currentIndex];

        if (candidate.HScore + SCORE_EPSILON < current.HScore)
            return true;
        if (current.HScore + SCORE_EPSILON < candidate.HScore)
            return false;
        return candidate.GScore + SCORE_EPSILON < current.GScore;
    }

    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref NodeSpan[nodeIndex];

        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = openList.Count;
            openList.Add(nodeIndex);
            if (openList.Count > peakOpenListSize)
                peakOpenListSize = openList.Count;
        }

        PercolateUp(node.OpenHeapIndex);
    }

    private int PopMinOpen()
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = openList[0];
        openList[0] = openList[^1];
        openList.RemoveAt(openList.Count - 1);
        nodeSpan[nodeIndex].OpenHeapIndex = -1;

        if (openList.Count > 0)
        {
            nodeSpan[openList[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }

        return nodeIndex;
    }

    private void PercolateUp(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = openList[heapIndex];
        var parent    = (heapIndex - 1) >> 2;

        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[openList[parent]]))
        {
            openList[heapIndex]                         = openList[parent];
            nodeSpan[openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex                                   = parent;
            parent                                      = (heapIndex - 1) >> 2;
        }

        openList[heapIndex]               = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private void PercolateDown(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = openList[heapIndex];
        var maxSize   = openList.Count;

        while (true)
        {
            var firstChild = (heapIndex << 2) + 1;
            if (firstChild >= maxSize)
                break;

            var bestChild = firstChild;
            var lastChild = Math.Min(firstChild + 4, maxSize);
            for (var c = firstChild + 1; c < lastChild; ++c)
                if (HeapLess(ref nodeSpan[openList[c]], ref nodeSpan[openList[bestChild]]))
                    bestChild = c;

            if (!HeapLess(ref nodeSpan[openList[bestChild]], ref nodeSpan[nodeIndex]))
                break;

            openList[heapIndex]                         = openList[bestChild];
            nodeSpan[openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex                                   = bestChild;
        }

        openList[heapIndex]               = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HeapLess(ref VolumePathfindNode left, ref VolumePathfindNode right)
    {
        var leftF  = TotalScore(left.GScore,  left.HScore);
        var rightF = TotalScore(right.GScore, right.HScore);
        if (leftF + SCORE_EPSILON < rightF)
            return true;
        if (rightF + SCORE_EPSILON < leftF)
            return false;
        if (left.HScore + SCORE_EPSILON < right.HScore)
            return true;
        if (right.HScore + SCORE_EPSILON < left.HScore)
            return false;
        return left.GScore > right.GScore;
    }
}
