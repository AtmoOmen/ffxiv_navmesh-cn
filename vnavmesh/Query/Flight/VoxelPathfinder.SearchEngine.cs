using System.Numerics;
using System.Runtime.CompilerServices;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight.Models;

namespace vnavmesh.Query.Flight;

public sealed partial class VoxelPathfinder
{
    private VolumeSearchTermination Execute
    (
        CancellationToken cancel,
        int               maxSteps
    )
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (nodes.Count >= VoxelPathfinderConstants.MAX_NODE_COUNT)
                return VolumeSearchTermination.StepBudgetReached;
            if (openList.Count == 0)
                return VolumeSearchTermination.SearchExhausted;

            var currentIndex = PopMinOpen();
            var current = nodes[currentIndex];
            current.Closed = true;
            nodes[currentIndex] = current;
            ++visitedNodes;
            --globalRemainingBudget;

            if (layerDepth == layerDepths[0])
                ++coarseExpandedNodes;

            if (currentIndex == goalNodeIndex)
                return VolumeSearchTermination.ReachedGoal;

            if (current.HScore <= layerCellSize && TryConnectGoal(currentIndex))
                return VolumeSearchTermination.ReachedGoal;

            if (current.ParentIndex != currentIndex && current.ParentIndex != current.DiscoveredByIndex)
            {
                var parent = nodes[current.ParentIndex];

                if (!TryLineOfSight(parent.Voxel, parent.Position, current.Voxel, current.Position))
                {
                    var discoveredBy = nodes[current.DiscoveredByIndex];
                    current.ParentIndex = current.DiscoveredByIndex;
                    current.GScore      = discoveredBy.GScore + Vector3.Distance(discoveredBy.Position, current.Position);
                    current.Closed      = false;
                    nodes[currentIndex] = current;
                    AddToOpen(currentIndex);
                    continue;
                }
            }

            ExpandNode(currentIndex);

            if ((i & 0xff) == 0)
                cancel.ThrowIfCancellationRequested();
        }

        return VolumeSearchTermination.StepBudgetReached;
    }

    private void ExpandNode
    (
        int currentIndex
    )
    {
        var current       = nodes[currentIndex];
        var (x, y, z)     = SparseVoxelOctree.CoordAtDepth(current.Voxel, layerDepth);
        var cellCount     = 1 << layerDepth;
        var parentIndex   = current.ParentIndex;
        var parentPosition = nodes[parentIndex].Position;
        var currentPosition = current.Position;

        foreach (var (dx, dy, dz) in VoxelNeighbourOffsets.All)
        {
            var nx = x + dx;
            var ny = y + dy;
            var nz = z + dz;

            if ((uint)nx >= (uint)cellCount || (uint)ny >= (uint)cellCount || (uint)nz >= (uint)cellCount)
                continue;
            if (!IsWithinCorridor(nx, ny, nz))
                continue;
            if (!Volume.IsCellTraversable(layerDepth, nx, ny, nz))
                continue;

            var neighbourVoxel    = SparseVoxelOctree.EncodeCoord(layerDepth, nx, ny, nz);
            var neighbourPosition = LayerCellCenter(nx, ny, nz);
            var bestParent        = currentIndex;
            var bestG             = current.GScore + Vector3.Distance(currentPosition, neighbourPosition);

            if (parentIndex != currentIndex)
            {
                var viaParent = nodes[parentIndex].GScore + Vector3.Distance(parentPosition, neighbourPosition);

                if (viaParent < bestG)
                {
                    bestG       = viaParent;
                    bestParent  = parentIndex;
                }
            }

            var neighbourIndex = GetOrCreateNode(neighbourVoxel, currentIndex);
            var neighbour = nodes[neighbourIndex];

            if (neighbour.Closed || bestG + VoxelPathfinderConstants.SCORE_EPSILON >= neighbour.GScore)
                continue;

            neighbour.GScore            = bestG;
            neighbour.HScore            = Vector3.Distance(neighbourPosition, layerGoalPos);
            neighbour.ParentIndex       = bestParent;
            neighbour.DiscoveredByIndex = currentIndex;
            neighbour.Position          = neighbourPosition;
            nodes[neighbourIndex]       = neighbour;
            AddToOpen(neighbourIndex);
            UpdateBestNode(neighbourIndex);
        }
    }

    private bool TryConnectGoal
    (
        int currentIndex
    )
    {
        var current = nodes[currentIndex];

        if (!TryLineOfSight(current.Voxel, current.Position, layerGoalVoxel, layerGoalPos))
            return false;

        var goalIndex = GetOrCreateNode(layerGoalVoxel, currentIndex);
        var goal      = nodes[goalIndex];
        var score     = current.GScore + Vector3.Distance(current.Position, layerGoalPos);

        if (goal.Closed || score + VoxelPathfinderConstants.SCORE_EPSILON >= goal.GScore)
            return false;

        goal.GScore            = score;
        goal.HScore            = 0;
        goal.ParentIndex       = currentIndex;
        goal.DiscoveredByIndex = currentIndex;
        goal.Position          = layerGoalPos;
        nodes[goalIndex]       = goal;
        goalNodeIndex          = goalIndex;
        AddToOpen(goalIndex);
        return true;
    }

    private int GetOrCreateNode
    (
        ulong voxel,
        int   fallbackParentIndex
    )
    {
        if (nodeLookup.TryGetValue(voxel, out var nodeIndex))
            return nodeIndex;

        nodeIndex = nodes.Count;
        nodes.Add
        (
            new()
            {
                GScore            = float.MaxValue,
                HScore            = float.MaxValue,
                Voxel             = voxel,
                ParentIndex       = fallbackParentIndex,
                DiscoveredByIndex = fallbackParentIndex,
                OpenHeapIndex     = -1,
                Closed            = false
            }
        );
        nodeLookup.Set(voxel, nodeIndex);
        ++generatedNodes;
        return nodeIndex;
    }

    private void UpdateBestNode
    (
        int nodeIndex
    )
    {
        if (bestNodeIndex < 0)
        {
            bestNodeIndex = nodeIndex;
            return;
        }

        var candidate = nodes[nodeIndex];
        var current   = nodes[bestNodeIndex];

        if (candidate.HScore + VoxelPathfinderConstants.SCORE_EPSILON < current.HScore)
            bestNodeIndex = nodeIndex;
        else if (current.HScore + VoxelPathfinderConstants.SCORE_EPSILON < candidate.HScore)
            return;
        else if (candidate.GScore + VoxelPathfinderConstants.SCORE_EPSILON < current.GScore)
            bestNodeIndex = nodeIndex;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathTo
    (
        int nodeIndex
    )
    {
        var result = new List<(ulong voxel, Vector3 p)>();
        var span   = NodeSpan;
        var current = nodeIndex;

        while (true)
        {
            result.Add((span[current].Voxel, span[current].Position));

            if (span[current].ParentIndex == current)
                break;

            current = span[current].ParentIndex;
        }

        result.Reverse();
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float TotalScore
    (
        float gScore,
        float hScore
    ) => gScore + hScore;

    private void AddToOpen
    (
        int nodeIndex
    )
    {
        var node = nodes[nodeIndex];

        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = openList.Count;
            nodes[nodeIndex]   = node;
            openList.Add(nodeIndex);

            if (openList.Count > peakOpenListSize)
                peakOpenListSize = openList.Count;
        }

        PercolateUp(node.OpenHeapIndex);
    }

    private int PopMinOpen()
    {
        var nodeIndex = openList[0];
        openList[0]   = openList[^1];
        openList.RemoveAt(openList.Count - 1);
        var node = nodes[nodeIndex];
        node.OpenHeapIndex = -1;
        nodes[nodeIndex]   = node;

        if (openList.Count > 0)
        {
            var first = nodes[openList[0]];
            first.OpenHeapIndex = 0;
            nodes[openList[0]]  = first;
            PercolateDown(0);
        }

        return nodeIndex;
    }

    private void PercolateUp
    (
        int heapIndex
    )
    {
        var nodeIndex = openList[heapIndex];
        var parent    = (heapIndex - 1) >> 2;

        while (heapIndex > 0 && HeapLess(nodes[nodeIndex], nodes[openList[parent]]))
        {
            openList[heapIndex]              = openList[parent];
            var moved = nodes[openList[heapIndex]];
            moved.OpenHeapIndex              = heapIndex;
            nodes[openList[heapIndex]]       = moved;
            heapIndex                        = parent;
            parent                           = (heapIndex - 1) >> 2;
        }

        openList[heapIndex]         = nodeIndex;
        var updated = nodes[nodeIndex];
        updated.OpenHeapIndex       = heapIndex;
        nodes[nodeIndex]            = updated;
    }

    private void PercolateDown
    (
        int heapIndex
    )
    {
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
                if (HeapLess(nodes[openList[c]], nodes[openList[bestChild]]))
                    bestChild = c;

            if (!HeapLess(nodes[openList[bestChild]], nodes[nodeIndex]))
                break;

            openList[heapIndex]         = openList[bestChild];
            var moved = nodes[openList[heapIndex]];
            moved.OpenHeapIndex         = heapIndex;
            nodes[openList[heapIndex]]  = moved;
            heapIndex                   = bestChild;
        }

        openList[heapIndex]         = nodeIndex;
        var updated = nodes[nodeIndex];
        updated.OpenHeapIndex       = heapIndex;
        nodes[nodeIndex]            = updated;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HeapLess
    (
        VolumePathfindNode left,
        VolumePathfindNode right
    )
    {
        var leftF  = TotalScore(left.GScore,  left.HScore);
        var rightF = TotalScore(right.GScore, right.HScore);

        if (leftF + VoxelPathfinderConstants.SCORE_EPSILON < rightF)
            return true;
        if (rightF + VoxelPathfinderConstants.SCORE_EPSILON < leftF)
            return false;
        if (left.HScore + VoxelPathfinderConstants.SCORE_EPSILON < right.HScore)
            return true;
        if (right.HScore + VoxelPathfinderConstants.SCORE_EPSILON < left.HScore)
            return false;
        return left.GScore > right.GScore;
    }
}
