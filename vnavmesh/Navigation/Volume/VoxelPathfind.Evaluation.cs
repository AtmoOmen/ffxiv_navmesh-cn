using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Navigation.Volume.Models;
using vnavmesh.Navigation.Volume.Utils;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private void ApplyNeighbourEvaluation(VolumeNeighbourEvaluation evaluation)
    {
        var     nodeIndex = GetOrCreateNode(evaluation.Voxel, evaluation.BestParentIndex);
        var     nodeSpan  = NodeSpan;
        ref var node      = ref nodeSpan[nodeIndex];
        if (node.Closed || evaluation.BestScore + SCORE_EPSILON >= node.GScore)
            return;

        node.GScore      = evaluation.BestScore;
        node.HScore      = HeuristicDistance(evaluation.BestPosition, evaluation.Voxel);
        node.ParentIndex = evaluation.BestParentIndex;
        node.Position    = evaluation.BestPosition;

        unchecked
        {
            ++node.Revision;
        }

        AddToOpen(nodeIndex);
        UpdateBestNode(nodeIndex);
    }

    private int GetOrCreateNode(ulong voxel, int fallbackParentIndex)
    {
        if (nodeLookup.TryGetValue(voxel, out var nodeIndex))
            return nodeIndex;

        nodeIndex = nodes.Count;
        nodes.Add
        (
            new()
            {
                GScore        = float.MaxValue,
                HScore        = float.MaxValue,
                Voxel         = voxel,
                ParentIndex   = fallbackParentIndex,
                OpenHeapIndex = -1,
                Closed        = false
            }
        );
        nodeLookup.Set(voxel, nodeIndex);
        ++generatedNodes;
        return nodeIndex;
    }

    private bool TryGetBestCandidate(int currentIndex, ulong neighbourVoxel, out int bestParentIndex, out Vector3 bestPosition, out float bestScore)
    {
        bestParentIndex = -1;
        bestPosition    = default;
        bestScore       = float.MaxValue;

        var requireDirectVisibility = VoxelIndexUtil.IsCoarseNeighbour(neighbourVoxel);
        if (!TryEvaluateCandidate(currentIndex, neighbourVoxel, requireDirectVisibility, ref bestParentIndex, ref bestPosition, ref bestScore))
            return false;

        var nodeSpan           = NodeSpan;
        var currentGScore      = nodeSpan[currentIndex].GScore;
        var earlyStopThreshold = currentGScore + CalculateMinimumEdgeCost(nodeSpan[currentIndex].Position, neighbourVoxel);
        if (bestScore <= earlyStopThreshold)
            return true;

        var ancestorIndex     = nodeSpan[currentIndex].ParentIndex;
        var effectiveLookBack = VoxelPathUtil.DetermineEffectiveLookBackDepth(bestScore, currentGScore, SCORE_EPSILON);
        var lookBackCount     = 0;
        var lastEvaluated     = -1;

        while (ancestorIndex >= 0 && lookBackCount < effectiveLookBack)
        {
            if (!TryEvaluateCandidate(ancestorIndex, neighbourVoxel, true, ref bestParentIndex, ref bestPosition, ref bestScore))
            {
                ancestorIndex = nodeSpan[ancestorIndex].ParentIndex;
                ++lookBackCount;
                continue;
            }

            lastEvaluated = ancestorIndex;

            ref var ancestor = ref nodeSpan[ancestorIndex];
            if (ancestor.ParentIndex == ancestorIndex)
                return bestParentIndex >= 0;

            if (bestScore <= earlyStopThreshold)
                return true;

            ancestorIndex = ancestor.ParentIndex;
            ++lookBackCount;
        }

        if (ancestorIndex >= 0 && effectiveLookBack >= MAX_ANCESTOR_LOOK_BACK)
        {
            var rootIndex = ancestorIndex;

            while (true)
            {
                ref var ancestor = ref nodeSpan[rootIndex];
                if (ancestor.ParentIndex == rootIndex)
                    break;

                rootIndex = ancestor.ParentIndex;
            }

            if (rootIndex != lastEvaluated)
                TryEvaluateCandidate(rootIndex, neighbourVoxel, true, ref bestParentIndex, ref bestPosition, ref bestScore);
        }

        return bestParentIndex >= 0;
    }

    private bool TryEvaluateCandidate
    (
        int         parentIndex,
        ulong       voxel,
        bool        requireVisibility,
        ref int     bestParentIndex,
        ref Vector3 bestPosition,
        ref float   bestScore
    )
    {
        var position = voxel == goalVoxel ? goalPos : ResolveVoxelCenter(voxel);

        if (requireVisibility && !TryLineOfSight(parentIndex, voxel, position))
            return false;

        var corridorPenalty = 0f;

        if (useGuidedCorridor && voxel != goalVoxel)
        {
            corridorPenalty = CalculateCorridorOverflowPenalty(position);
            if (corridorPenalty < 0f)
                return false;
        }

        var score = CalculateNodeScore(parentIndex, voxel, position) + corridorPenalty;
        if (!IsBetterCandidate(parentIndex, score, bestParentIndex, bestScore))
            return false;

        bestParentIndex = parentIndex;
        bestPosition    = position;
        bestScore       = score;
        return true;
    }

    private float CalculateMinimumEdgeCost(Vector3 fromPos, ulong toVoxel)
    {
        var (voxelMin, voxelMax) = Volume.VoxelBounds(toVoxel, 0);
        var voxelCenter = (voxelMin + voxelMax) * 0.5f;
        return Vector3.Distance(fromPos, voxelCenter) * 0.8f;
    }

    private static bool IsBetterCandidate(int candidateParentIndex, float candidateScore, int currentBestParentIndex, float currentBestScore)
    {
        if (candidateScore + SCORE_EPSILON < currentBestScore)
            return true;
        if (currentBestScore + SCORE_EPSILON < candidateScore)
            return false;
        return candidateParentIndex < currentBestParentIndex;
    }

    private float CalculateNodeScore(int parentIndex, ulong voxel, Vector3 destination)
    {
        var nodeSpan = NodeSpan;
        var edgeCost = Vector3.Distance(nodeSpan[parentIndex].Position, destination);
        var score = nodeSpan[parentIndex].GScore                                                               +
                    edgeCost                                                                                   +
                    CalculateLongRangeLateralTraversalPenalty(nodeSpan[parentIndex].Voxel, nodeSpan[parentIndex].Position, voxel, destination);

        if (previouslyVisitedVoxels is { } visited && visited.Contains(voxel))
            score += edgeCost * REVISIT_PENALTY_SCALE;

        return score;
    }

    private bool TryLineOfSight(int fromNodeIndex, ulong toVoxel, Vector3 toPosition)
    {
        var     nodeSpan = NodeSpan;
        ref var fromNode = ref nodeSpan[fromNodeIndex];
        var     cacheKey = new VolumeVisibilityKey(fromNodeIndex, fromNode.Revision, toVoxel);
        if (visibilityCache.TryGetValue(cacheKey, out var cached))
            return cached;

        Interlocked.Increment(ref lineOfSightChecks);
        var visible = VoxelSearch.LineOfSight(Volume, fromNode.Voxel, toVoxel, fromNode.Position, toPosition);
        if (!visibilityCache.TryAdd(cacheKey, visible))
            return visibilityCache[cacheKey];

        if (visible)
            Interlocked.Increment(ref lineOfSightHits);
        return visible;
    }
}
