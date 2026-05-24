using System.Numerics;
using System.Runtime.CompilerServices;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;

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

        var requireDirectVisibility = IsCoarseNeighbour(neighbourVoxel);
        if (!TryEvaluateCandidate(currentIndex, neighbourVoxel, requireDirectVisibility, ref bestParentIndex, ref bestPosition, ref bestScore))
            return false;

        var nodeSpan           = NodeSpan;
        var currentGScore      = nodeSpan[currentIndex].GScore;
        var earlyStopThreshold = currentGScore + CalculateMinimumEdgeCost(nodeSpan[currentIndex].Position, neighbourVoxel);
        if (bestScore <= earlyStopThreshold)
            return true;

        var ancestorIndex      = nodeSpan[currentIndex].ParentIndex;
        var effectiveLookBack  = DetermineEffectiveLookBackDepth(bestScore, currentGScore);
        var lookBackCount      = 0;
        var lastEvaluated      = -1;

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
        Span<Vector3>                 candidatePositions = stackalloc Vector3[3];
        Span<VolumePathCandidateKind> candidateKinds     = stackalloc VolumePathCandidateKind[3];
        candidatePositions[0] = ResolveProjectedPosition(voxel, parentIndex);
        candidateKinds[0]     = VolumePathCandidateKind.Projected;
        var projectedScore = CalculateNodeScore(parentIndex, voxel, candidatePositions[0]);
        if (bestParentIndex >= 0 && projectedScore > bestScore + SCORE_EPSILON)
            return false;

        candidatePositions[1] = ResolveGoalAlignedPosition(voxel);
        candidateKinds[1]     = VolumePathCandidateKind.GoalAligned;
        var goalAlignedScore = CalculateNodeScore(parentIndex, voxel, candidatePositions[1]);
        if (bestParentIndex >= 0 && goalAlignedScore > bestScore + SCORE_EPSILON)
        {
            candidatePositions[2] = ResolveCenterBiasedPosition(voxel, parentIndex);
            candidateKinds[2]     = VolumePathCandidateKind.CenterBiased;
            var centerBiasedScore = CalculateNodeScore(parentIndex, voxel, candidatePositions[2]);
            if (centerBiasedScore > bestScore + SCORE_EPSILON)
                return false;
        }
        else
        {
            candidatePositions[2] = ResolveCenterBiasedPosition(voxel, parentIndex);
            candidateKinds[2]     = VolumePathCandidateKind.CenterBiased;
        }

        var found = false;

        for (var i = 0; i < candidatePositions.Length; ++i)
        {
            var candidatePosition = candidatePositions[i];

            if (i > 0)
            {
                var duplicated = false;

                foreach (var existingPosition in candidatePositions[..i])
                {
                    if (Vector3.DistanceSquared(existingPosition, candidatePosition) > SCORE_EPSILON * SCORE_EPSILON)
                        continue;

                    duplicated = true;
                    break;
                }

                if (duplicated)
                    continue;
            }

            if (requireVisibility && !TryLineOfSight(parentIndex, voxel, candidatePosition, candidateKinds[i]))
                continue;

            var corridorPenalty = 0f;

            if (useGuidedCorridor && voxel != goalVoxel)
            {
                corridorPenalty = CalculateCorridorOverflowPenalty(candidatePosition);
                if (corridorPenalty < 0f)
                    continue;
            }

            var candidateScore = (i == 0 ? projectedScore : i == 1 ? goalAlignedScore : CalculateNodeScore(parentIndex, voxel, candidatePosition)) + corridorPenalty;
            if (!IsBetterCandidate(parentIndex, candidatePosition, candidateScore, bestParentIndex, bestPosition, bestScore, voxel))
                continue;

            bestParentIndex = parentIndex;
            bestPosition    = candidatePosition;
            bestScore       = candidateScore;
            found           = true;
        }

        return found;
    }

    private Vector3 ResolveProjectedPosition(ulong voxel, int parentIndex)
    {
        if (voxel == goalVoxel)
            return goalPos;

        var nodeSpan = NodeSpan;
        return ResolveSearchCandidatePosition(voxel, nodeSpan[parentIndex].Position);
    }

    private Vector3 ResolveGoalAlignedPosition(ulong voxel)
    {
        if (voxel == goalVoxel)
            return goalPos;

        return ResolveSearchCandidatePosition(voxel, goalPos);
    }

    private Vector3 ResolveCenterBiasedPosition(ulong voxel, int parentIndex)
    {
        if (voxel == goalVoxel)
            return goalPos;

        var projected     = ResolveProjectedPosition(voxel, parentIndex);
        var goalAligned   = ResolveGoalAlignedPosition(voxel);
        var voxelCenter   = ResolveVoxelCenter(voxel);
        var blendedTarget = Vector3.Lerp(projected, goalAligned, SEARCH_PATH_GOAL_BLEND);
        blendedTarget = Vector3.Lerp(blendedTarget, voxelCenter, SEARCH_PATH_CENTER_BIAS);
        return ResolveSearchCandidatePosition(voxel, blendedTarget);
    }

    private static int DetermineEffectiveLookBackDepth(float currentBestScore, float currentNodeGScore)
    {
        if (currentBestScore == float.MaxValue)
            return MAX_ANCESTOR_LOOK_BACK;

        var improvementRatio = (currentNodeGScore - currentBestScore) / MathF.Max(currentNodeGScore, SCORE_EPSILON);
        if (improvementRatio < 0.05f)
            return 2;
        if (improvementRatio < 0.15f)
            return 4;

        return MAX_ANCESTOR_LOOK_BACK;
    }

    private float CalculateMinimumEdgeCost(Vector3 fromPos, ulong toVoxel)
    {
        var (voxelMin, voxelMax) = Volume.VoxelBounds(toVoxel, 0);
        var voxelCenter          = (voxelMin + voxelMax) * 0.5f;
        return Vector3.Distance(fromPos, voxelCenter) * 0.8f;
    }

    private bool IsBetterCandidate
    (
        int     candidateParentIndex,
        Vector3 candidatePosition,
        float   candidateScore,
        int     currentBestParentIndex,
        Vector3 currentBestPosition,
        float   currentBestScore,
        ulong   voxel
    )
    {
        if (candidateScore + SCORE_EPSILON < currentBestScore)
            return true;
        if (currentBestScore + SCORE_EPSILON < candidateScore)
            return false;

        var candidateF = TotalScore(candidateScore,   HeuristicDistance(candidatePosition,   voxel));
        var currentF   = TotalScore(currentBestScore, HeuristicDistance(currentBestPosition, voxel));
        if (candidateF + SCORE_EPSILON < currentF)
            return true;
        if (currentF + SCORE_EPSILON < candidateF)
            return false;

        return candidateParentIndex < currentBestParentIndex;
    }

    private float CalculateNodeScore(int parentIndex, ulong voxel, Vector3 destination)
    {
        var nodeSpan = NodeSpan;
        var edgeCost = CalculateEdgeCost(nodeSpan[parentIndex].Position, destination);
        var score = nodeSpan[parentIndex].GScore                                                                                               +
                    edgeCost                                                                                                                   +
                    CalculateLongRangeLateralTraversalPenalty(nodeSpan[parentIndex].Voxel, nodeSpan[parentIndex].Position, voxel, destination) +
                    CalculateWallProximityPenalty(voxel, destination);

        if (previouslyVisitedVoxels is { } visited && visited.Contains(voxel))
            score += edgeCost * REVISIT_PENALTY_SCALE;

        return score;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float CalculateEdgeCost(Vector3 from, Vector3 to) => Vector3.Distance(from, to);

    private bool TryLineOfSight(int fromNodeIndex, ulong toVoxel, Vector3 toPosition, VolumePathCandidateKind candidateKind)
    {
        var     nodeSpan = NodeSpan;
        ref var fromNode = ref nodeSpan[fromNodeIndex];
        var     cacheKey = new VolumeVisibilityKey(fromNodeIndex, fromNode.Revision, toVoxel, candidateKind);
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

    private static ushort ExtractL0Index(ulong voxel)
    {
        var temp = voxel;
        return VoxelMap.DecodeIndex(ref temp);
    }
}
