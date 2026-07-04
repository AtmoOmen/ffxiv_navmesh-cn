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

        // 预计算邻居位置，避免祖先链中重复调用 ResolveVoxelCenter
        var neighbourPosition = neighbourVoxel == goalVoxel ? goalPos : ResolveVoxelCenter(neighbourVoxel);

        var nodeSpan      = NodeSpan;
        ref var current   = ref nodeSpan[currentIndex];
        var currentGScore = current.GScore;
        var edgeCost      = Vector3.Distance(current.Position, neighbourPosition);

        // 预检：邻居已关闭则直接跳过（ApplyNeighbourEvaluation 也会跳过，提前省掉 LoS）
        if (nodeLookup.TryGetValue(neighbourVoxel, out var existingIndex))
        {
            ref var existing = ref nodeSpan[existingIndex];
            if (existing.Closed)
                return false;
        }

        var requireDirectVisibility = VoxelIndexUtil.IsCoarseNeighbour(neighbourVoxel);
        if (!TryEvaluateCandidate(currentIndex, neighbourVoxel, neighbourPosition, requireDirectVisibility, ref bestParentIndex, ref bestPosition, ref bestScore))
            return false;

        var earlyStopThreshold = currentGScore + edgeCost * 0.8f;
        if (bestScore <= earlyStopThreshold)
            return true;

        // 预取祖先索引链，避免评估循环中反复 nodeSpan[...].ParentIndex 指针追逐
        Span<int> ancestorChain = stackalloc int[MAX_ANCESTOR_LOOK_BACK];
        var       ancestorCount = 0;
        var       walkIndex     = nodeSpan[currentIndex].ParentIndex;

        while (ancestorCount < MAX_ANCESTOR_LOOK_BACK && walkIndex >= 0)
        {
            ancestorChain[ancestorCount++] = walkIndex;
            ref var node = ref nodeSpan[walkIndex];
            if (node.ParentIndex == walkIndex)
                break;
            walkIndex = node.ParentIndex;
        }

        var effectiveLookBack = VoxelPathUtil.DetermineEffectiveLookBackDepth(bestScore, currentGScore, SCORE_EPSILON);
        var lookBackCount     = 0;
        var lastEvaluated     = -1;

        for (var i = 0; i < ancestorCount && lookBackCount < effectiveLookBack; ++i)
        {
            var ancestorIndex = ancestorChain[i];

            if (!TryEvaluateCandidate(ancestorIndex, neighbourVoxel, neighbourPosition, true, ref bestParentIndex, ref bestPosition, ref bestScore))
            {
                ++lookBackCount;
                continue;
            }

            lastEvaluated = ancestorIndex;

            if (i == ancestorCount - 1)
            {
                ref var ancestor = ref nodeSpan[ancestorIndex];
                if (ancestor.ParentIndex == ancestorIndex)
                    return bestParentIndex >= 0;
            }

            if (bestScore <= earlyStopThreshold)
                return true;

            ++lookBackCount;
        }

        if (ancestorCount > 0 && effectiveLookBack >= MAX_ANCESTOR_LOOK_BACK)
        {
            var rootIndex = ancestorChain[ancestorCount - 1];

            ref var rootNode = ref nodeSpan[rootIndex];
            if (rootNode.ParentIndex != rootIndex)
            {
                while (true)
                {
                    ref var node = ref nodeSpan[rootIndex];
                    if (node.ParentIndex == rootIndex)
                        break;
                    rootIndex = node.ParentIndex;
                }
            }

            if (rootIndex != lastEvaluated)
                TryEvaluateCandidate(rootIndex, neighbourVoxel, neighbourPosition, true, ref bestParentIndex, ref bestPosition, ref bestScore);
        }

        return bestParentIndex >= 0;
    }

    private bool TryEvaluateCandidate
    (
        int         parentIndex,
        ulong       voxel,
        Vector3     position,
        bool        requireVisibility,
        ref int     bestParentIndex,
        ref Vector3 bestPosition,
        ref float   bestScore
    )
    {
        var     nodeSpan = NodeSpan;
        ref var parent   = ref nodeSpan[parentIndex];

        // 预 LoS 分数下界过滤：parentGScore + 直线距离是分数下界（所有惩罚项 ≥ 0）
        // 如果下界已不优于当前最优，跳过昂贵的 LoS 检查
        var edgeCost = Vector3.Distance(parent.Position, position);
        if (parent.GScore + edgeCost >= bestScore)
            return false;

        if (requireVisibility && !TryLineOfSight(parentIndex, voxel, position))
            return false;

        var corridorPenalty = 0f;

        if (useGuidedCorridor && voxel != goalVoxel)
            corridorPenalty = CalculateCorridorOverflowPenalty(position);

        var score = parent.GScore +
                    edgeCost +
                    CalculateLongRangeLateralTraversalPenalty(parent.Voxel, parent.Position, voxel, position) +
                    corridorPenalty;

        if (previouslyVisitedVoxels is { } visited && visited.Contains(voxel))
            score += edgeCost * REVISIT_PENALTY_SCALE;

        if (!IsBetterCandidate(parentIndex, score, bestParentIndex, bestScore))
            return false;

        bestParentIndex = parentIndex;
        bestPosition    = position;
        bestScore       = score;
        return true;
    }

    private static bool IsBetterCandidate(int candidateParentIndex, float candidateScore, int currentBestParentIndex, float currentBestScore)
    {
        if (candidateScore + SCORE_EPSILON < currentBestScore)
            return true;
        if (currentBestScore + SCORE_EPSILON < candidateScore)
            return false;
        return candidateParentIndex < currentBestParentIndex;
    }

    private bool TryLineOfSight(int fromNodeIndex, ulong toVoxel, Vector3 toPosition)
    {
        var     nodeSpan = NodeSpan;
        ref var fromNode = ref nodeSpan[fromNodeIndex];
        var     cacheKey = new VolumeVisibilityKey(fromNode.Voxel, toVoxel);
        var     stripe   = (int)((uint)cacheKey.GetHashCode() & (VISIBILITY_CACHE_STRIPES - 1));

        lock (visibilityLocks[stripe])
        {
            if (visibilityCaches[stripe].TryGetValue(cacheKey, out var cached))
                return cached;
        }

        Interlocked.Increment(ref lineOfSightChecks);
        var visible = VoxelSearch.LineOfSight(Volume, fromNode.Voxel, toVoxel, fromNode.Position, toPosition);

        lock (visibilityLocks[stripe])
        {
            visibilityCaches[stripe].TryAdd(cacheKey, visible);
        }

        if (visible)
            Interlocked.Increment(ref lineOfSightHits);
        return visible;
    }
}
