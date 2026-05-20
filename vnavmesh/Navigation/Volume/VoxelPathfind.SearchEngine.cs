using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, LongRangeLateralBias lateralBias = default)
    {
        ResetSearchState();
        longRangeLateralBias = lateralBias;

        if (fromVoxel == VoxelMap.INVALID_VOXEL || toVoxel == VoxelMap.INVALID_VOXEL)
        {
            Service.Log.Error($"输入体素非法：{fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        goalVoxel = toVoxel;
        goalPos   = toPos;

        nodes.Add
        (
            new()
            {
                GScore        = 0,
                HScore        = HeuristicDistance(fromPos, fromVoxel),
                Voxel         = fromVoxel,
                ParentIndex   = 0,
                OpenHeapIndex = -1,
                Closed        = false,
                Position      = fromPos
            }
        );
        nodeLookup.SetOrUpdate(fromVoxel, 0);
        generatedNodes                      = 1;
        guidedCorridorInitialGoalDistance   = Vector3.Distance(fromPos, toPos);
        guidedCorridorInitialAboveGoal      = MathF.Max(fromPos.Y - toPos.Y, 0f);
        guidedCorridorLastProgressDistance  = guidedCorridorInitialGoalDistance;
        guidedCorridorLastProgressAboveGoal = guidedCorridorInitialAboveGoal;
        guidedCorridorLastProgressVisited   = 0;
        AddToOpen(0);
    }

    private VolumeSearchTermination Execute(CancellationToken cancel, int maxSteps = DEFAULT_MAX_SEARCH_STEPS)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (nodes.Count >= MAX_NODE_COUNT)
                return VolumeSearchTermination.StepBudgetReached;
            if (!ExecuteStep())
                return goalReached ? VolumeSearchTermination.ReachedGoal : VolumeSearchTermination.SearchExhausted;
            if (ShouldAbortGuidedCorridorEarly())
                return VolumeSearchTermination.SearchExhausted;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }

        return VolumeSearchTermination.StepBudgetReached;
    }

    private bool ShouldAbortGuidedCorridorEarly()
    {
        if (!useGuidedCorridor                                      ||
            goalReached                                             ||
            visitedNodes  < GUIDED_CORRIDOR_EARLY_ABORT_MIN_VISITED ||
            bestNodeIndex < 0                                       ||
            bestNodeIndex >= nodes.Count)
            return false;

        var bestNode      = NodeSpan[bestNodeIndex];
        var bestDistance  = bestNode.HScore;
        var bestAboveGoal = MathF.Max(bestNode.Position.Y - goalPos.Y, 0f);
        var descendingCaseMinDrop = MathF.Max
        (
            l2Desc.CellSize.Y * GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_MIN_DROP_LEAF_CELLS,
            GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_MIN_DROP_DISTANCE
        );

        if (guidedCorridorInitialAboveGoal >= descendingCaseMinDrop)
        {
            var verticalProgressThreshold = MathF.Max
            (
                l2Desc.CellSize.Y * GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_PROGRESS_LEAF_CELLS,
                GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_PROGRESS_MIN_DISTANCE
            );

            if (guidedCorridorLastProgressAboveGoal - bestAboveGoal >= verticalProgressThreshold)
            {
                guidedCorridorLastProgressAboveGoal = bestAboveGoal;
                guidedCorridorLastProgressDistance  = bestDistance;
                guidedCorridorLastProgressVisited   = visitedNodes;
                return false;
            }

            if (bestAboveGoal <= guidedCorridorInitialAboveGoal * GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_SUFFICIENT_PROGRESS_RATIO)
                return false;

            if (visitedNodes < GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_HARD_VISITED_THRESHOLD)
                return false;

            if (visitedNodes - guidedCorridorLastProgressVisited < GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_STALL_WINDOW)
                return false;

            guidedCorridorEarlyAbortTriggered = true;
            return true;
        }

        var progressThreshold = MathF.Max
        (
            GUIDED_CORRIDOR_EARLY_ABORT_PROGRESS_MIN_DISTANCE,
            guidedCorridorInitialGoalDistance * GUIDED_CORRIDOR_EARLY_ABORT_PROGRESS_RATIO
        );

        if (guidedCorridorLastProgressDistance - bestDistance >= progressThreshold)
        {
            guidedCorridorLastProgressDistance = bestDistance;
            guidedCorridorLastProgressVisited  = visitedNodes;
            return false;
        }

        if (bestDistance <= guidedCorridorInitialGoalDistance * GUIDED_CORRIDOR_EARLY_ABORT_SUFFICIENT_PROGRESS_RATIO)
            return false;

        if (visitedNodes < GUIDED_CORRIDOR_EARLY_ABORT_HARD_VISITED_THRESHOLD)
            return false;

        if (visitedNodes - guidedCorridorLastProgressVisited < GUIDED_CORRIDOR_EARLY_ABORT_STALL_WINDOW)
            return false;

        guidedCorridorEarlyAbortTriggered = true;
        return true;
    }

    private List<(ulong voxel, Vector3 p)> RunSearchAttempt
    (
        ulong                 fromVoxel,
        ulong                 toVoxel,
        Vector3               fromPos,
        Vector3               toPos,
        bool                  returnIntermediatePoints,
        CancellationToken     cancel,
        int                   maxSteps,
        int                   attempts,
        GuidedSearchCorridor? corridor                = null,
        float?                heuristicWeightOverride = null,
        LongRangeLateralBias  lateralBias             = default
    )
    {
        if (attempts > 1 && nodes.Count > 0)
            RetainClosedSetKnowledge();
        Start(fromVoxel, toVoxel, fromPos, toPos, lateralBias);
        allowCoarseL1Stepping = maxSteps > RAYCAST_SEARCH_STEP_BUDGET;
        useGuidedCorridor     = corridor.HasValue;
        guidedCorridor        = corridor.GetValueOrDefault();
        heuristicWeight       = heuristicWeightOverride ?? SHORT_RANGE_HEURISTIC_WEIGHT;
        lastSearchAttempts    = attempts;
        lastTermination       = Execute(cancel, maxSteps);
        return BuildPathToVisitedNode(bestNodeIndex, returnIntermediatePoints);
    }

    private bool TryConnectGoal(int currentIndex)
    {
        var     nodeSpan = NodeSpan;
        ref var current  = ref nodeSpan[currentIndex];
        if (current.HScore > goalVisibilityProbeDistance)
            return false;
        if (!TryLineOfSight(currentIndex, goalVoxel, goalPos, VolumePathCandidateKind.GoalAligned))
            return false;

        var goalScore = current.GScore + CalculateEdgeCost(current.Position, goalPos);
        var goalIndex = GetOrCreateNode(goalVoxel, currentIndex);
        nodeSpan = NodeSpan;
        ref var goal = ref nodeSpan[goalIndex];
        if (goal.Closed || goalScore + SCORE_EPSILON >= goal.GScore)
            return false;

        goal.GScore      = goalScore;
        goal.HScore      = 0;
        goal.ParentIndex = currentIndex;
        goal.Position    = goalPos;

        unchecked
        {
            ++goal.Revision;
        }

        bestNodeIndex = goalIndex;
        goalReached   = true;
        return true;
    }

    public bool ExecuteStep()
    {
        if (openList.Count == 0)
            return false;

        var     currentIndex = PopMinOpen();
        var     nodeSpan     = NodeSpan;
        ref var current      = ref nodeSpan[currentIndex];
        current.Closed = true;
        ++visitedNodes;
        UpdateBestNode(currentIndex);

        if (current.Voxel == goalVoxel)
        {
            bestNodeIndex = currentIndex;
            goalReached   = true;
            return false;
        }

        if (TryConnectGoal(currentIndex))
            return false;

        var savedCoarseStepping = allowCoarseL1Stepping;
        if (allowCoarseL1Stepping && current.HScore <= goalVisibilityProbeDistance)
            allowCoarseL1Stepping = false;

        var neighbours = CollectNeighbours(current.Voxel);

        allowCoarseL1Stepping = savedCoarseStepping;

        if (neighbours.Count >= RAYCAST_PARALLEL_NEIGHBOUR_THRESHOLD && Environment.ProcessorCount > 1)
        {
            var evaluations = new VolumeNeighbourEvaluation?[neighbours.Count];
            Parallel.For
            (
                0,
                neighbours.Count,
                new ParallelOptions { MaxDegreeOfParallelism = Math.Min(Environment.ProcessorCount, neighbours.Count) },
                i =>
                {
                    if (TryGetBestCandidate(currentIndex, neighbours[i], out var bestParentIndex, out var bestPosition, out var bestScore))
                        evaluations[i] = new(neighbours[i], bestParentIndex, bestPosition, bestScore);
                }
            );

            foreach (var evaluation in evaluations)
            {
                if (evaluation is { } resolved)
                    ApplyNeighbourEvaluation(resolved);
            }
        }
        else
        {
            foreach (var neighbourVoxel in neighbours)
            {
                if (!TryGetBestCandidate(currentIndex, neighbourVoxel, out var bestParentIndex, out var bestPosition, out var bestScore))
                    continue;

                ApplyNeighbourEvaluation(new(neighbourVoxel, bestParentIndex, bestPosition, bestScore));
            }
        }

        return true;
    }

    private void ResetSearchState()
    {
        nodes.Clear();
        nodeLookup.Clear();
        openList.Clear();
        visibilityCache.Clear();
        TrimCachesIfNeeded();
        bestNodeIndex                        = 0;
        goalReached                          = false;
        visitedNodes                         = 0;
        generatedNodes                       = 0;
        lineOfSightChecks                    = 0;
        lineOfSightHits                      = 0;
        peakOpenListSize                     = 0;
        heuristicWeight                      = 1;
        useGuidedCorridor                    = false;
        guidedCorridor                       = default;
        longRangeLateralBias                 = default;
        debugReturnedLongRangeBestEffortPath = false;
        guidedCorridorEarlyAbortTriggered    = false;
        currentL1CorridorRadius              = 0;
        lastTermination                      = VolumeSearchTermination.SearchExhausted;
        lastSearchAttempts                   = 0;
        LastPathDebug                        = null;
        pendingLongRangeProxyDebug           = null;
        guidedCorridorInitialAboveGoal       = 0;
        guidedCorridorInitialGoalDistance    = 0;
        guidedCorridorLastProgressAboveGoal  = 0;
        guidedCorridorLastProgressDistance   = 0;
        guidedCorridorLastProgressVisited    = 0;
    }

    private void TrimCachesIfNeeded()
    {
        if (voxelWallMaskCache.Count > WALL_MASK_CACHE_MAX_SIZE)
            voxelWallMaskCache.Clear();
        if (verifiedDownwardOpeningCache.Count > VERTICAL_ACCESS_CACHE_MAX_SIZE)
            verifiedDownwardOpeningCache.Clear();
        if (verifiedTopEntryCache.Count > VERTICAL_ACCESS_CACHE_MAX_SIZE)
            verifiedTopEntryCache.Clear();
        if (l1FaceConnectivityCache.Count > L1_FACE_CACHE_MAX_SIZE)
            l1FaceConnectivityCache.Clear();
    }

    private void RetainClosedSetKnowledge()
    {
        previouslyVisitedVoxels ??= new(Math.Min(nodeLookup.Count, MAX_PREVIOUSLY_VISITED));
        if (previouslyVisitedVoxels.Count >= MAX_PREVIOUSLY_VISITED)
            return;

        var nodeSpan   = NodeSpan;
        var enumerator = nodeLookup.GetClosedEnumerator(nodeSpan);

        while (enumerator.MoveNext())
        {
            previouslyVisitedVoxels.Add(enumerator.Current);
            if (previouslyVisitedVoxels.Count >= MAX_PREVIOUSLY_VISITED)
                break;
        }
    }

    internal void ReleaseRetainedState()
    {
        ResetSearchState();
        neighbourScratch.Clear();
        neighbourScratch.TrimExcess();
        nodes.TrimExcess();
        nodeLookup.TrimExcess();
        openList.TrimExcess();
        l1PathSet          = null;
        l0PathSet          = null;
        l1CorridorDistance = null;
        l0CorridorDistance = null;
        l1DistanceField    = null;
        l0DistanceField    = null;
        verifiedDownwardOpeningCache.Clear();
        verifiedTopEntryCache.Clear();
        l1FaceConnectivityCache.Clear();
    }

    private bool TryBuildDirectPath(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, out List<(ulong voxel, Vector3 p)> path)
    {
        ++lineOfSightChecks;

        if (!VoxelSearch.LineOfSight(Volume, fromVoxel, toVoxel, fromPos, toPos))
        {
            path = [];
            return false;
        }

        ++lineOfSightHits;
        goalVoxel          = toVoxel;
        goalPos            = toPos;
        bestNodeIndex      = 0;
        goalReached        = true;
        visitedNodes       = 1;
        generatedNodes     = 1;
        lastTermination    = VolumeSearchTermination.ReachedGoal;
        lastSearchAttempts = 1;
        path               = [(toVoxel, toPos)];
        return true;
    }

    private bool ShouldUseShortRangeExploratoryHeuristic(Vector3 fromPos, Vector3 toPos)
    {
        var leafHorizontalSize = MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z);
        var leafVerticalSize   = l2Desc.CellSize.Y;
        var horizontalDistance = HorizontalDistanceXZ(fromPos, toPos);
        var verticalDrop       = fromPos.Y - toPos.Y;
        var minHorizontal      = MathF.Max(leafHorizontalSize * SHORT_RANGE_EXPLORATION_MIN_HORIZONTAL_LEAF_CELLS, SHORT_RANGE_EXPLORATION_MIN_HORIZONTAL_DISTANCE);
        var minDrop            = MathF.Max(leafVerticalSize   * SHORT_RANGE_EXPLORATION_MIN_DROP_LEAF_CELLS,       SHORT_RANGE_EXPLORATION_MIN_DROP_DISTANCE);
        return horizontalDistance >= minHorizontal && verticalDrop >= minDrop;
    }
}
