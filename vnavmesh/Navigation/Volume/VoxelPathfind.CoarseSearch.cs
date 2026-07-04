using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Navigation.Volume.Utils;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private readonly record struct L1Neighbour
    (
        ulong Voxel,
        int   Dir,
        int   Dx,
        int   Dy,
        int   Dz,
        int   L0X,
        int   L0Y,
        int   L0Z,
        int   L1X,
        int   L1Y,
        int   L1Z
    );

    private int EnumerateL1Neighbours(ulong voxel, Span<L1Neighbour> output)
    {
        var t     = voxel;
        var l0Idx = VoxelMap.DecodeIndex(ref t);
        var l1Idx = VoxelMap.DecodeIndex(ref t);
        var l0C   = l0Desc.IndexToVoxel(l0Idx);
        var l1C   = l1Desc.IndexToVoxel(l1Idx);
        var count = 0;

        for (var dir = 0; dir < 6; dir++)
        {
            var dx = dir switch { 2 => -1, 3 => 1, _ => 0 };
            var dy = dir switch { 0 => -1, 1 => 1, _ => 0 };
            var dz = dir switch { 4 => -1, 5 => 1, _ => 0 };

            var nl1X = l1C.x + dx;
            var nl1Y = l1C.y + dy;
            var nl1Z = l1C.z + dz;
            var nl0X = l0C.x;
            var nl0Y = l0C.y;
            var nl0Z = l0C.z;

            if (nl1X < 0)
            {
                nl0X--;
                nl1X = l1Desc.NumCellsX - 1;
            }
            else if (nl1X >= l1Desc.NumCellsX)
            {
                nl0X++;
                nl1X = 0;
            }

            if (nl1Y < 0)
            {
                nl0Y--;
                nl1Y = l1Desc.NumCellsY - 1;
            }
            else if (nl1Y >= l1Desc.NumCellsY)
            {
                nl0Y++;
                nl1Y = 0;
            }

            if (nl1Z < 0)
            {
                nl0Z--;
                nl1Z = l1Desc.NumCellsZ - 1;
            }
            else if (nl1Z >= l1Desc.NumCellsZ)
            {
                nl0Z++;
                nl1Z = 0;
            }

            if (!l0Desc.InBounds(nl0X, nl0Y, nl0Z))
                continue;

            var neighbour = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(nl1X, nl1Y, nl1Z));
            neighbour       = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(nl0X, nl0Y, nl0Z), neighbour);
            output[count++] = new(neighbour, dir, dx, dy, dz, nl0X, nl0Y, nl0Z, nl1X, nl1Y, nl1Z);
        }

        return count;
    }

    private void VisitL1Neighbours(ulong voxel, Action<ulong> visitor)
    {
        Span<L1Neighbour> buffer = stackalloc L1Neighbour[6];
        var               count  = EnumerateL1Neighbours(voxel, buffer);
        for (var i = 0; i < count; i++)
            visitor(buffer[i].Voxel);
    }

    private void ComputeL1DistanceField(ulong goalVoxel, Vector3 goalPoint, int budget = L1_DISTANCE_FIELD_BUDGET)
    {
        var goalL1 = ResolveRepresentativeL1Voxel(goalVoxel, goalPoint);
        l1DistanceField = new() { [goalL1] = 0 };
        var frontier = new Queue<ulong>();
        frontier.Enqueue(goalL1);
        var               expanded = 0;
        Span<L1Neighbour> buffer   = stackalloc L1Neighbour[6];

        while (frontier.TryDequeue(out var current) && expanded < budget)
        {
            expanded++;
            var currentDist = l1DistanceField[current];

            var count = EnumerateL1Neighbours(current, buffer);

            for (var i = 0; i < count; i++)
            {
                ref readonly var n = ref buffer[i];
                if (!Volume.IsEmpty(n.Voxel) || l1DistanceField.ContainsKey(n.Voxel))
                    continue;
                if (!HasTraversableL1FaceTransition(current, n.Voxel, n.Dx, n.Dy, n.Dz))
                    continue;

                var edgeCost = n.Dx != 0 ? l1Desc.CellSize.X : n.Dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                l1DistanceField[n.Voxel] = currentDist + edgeCost;
                frontier.Enqueue(n.Voxel);
            }
        }

        l0DistanceField = new();

        foreach (var (l1Voxel, l1Dist) in l1DistanceField)
        {
            var l0Index = VoxelIndexUtil.ExtractL0Index(l1Voxel);
            if (!l0DistanceField.TryGetValue(l0Index, out var existing) || l1Dist < existing)
                l0DistanceField[l0Index] = l1Dist;
        }
    }

    private void ComputeL1DistanceFieldFromCoarsePath(IReadOnlyList<ulong> orderedPath, float terminalDistance)
    {
        l1DistanceField = new();

        if (orderedPath.Count == 0)
        {
            l0DistanceField = null;
            return;
        }

        var remainingDistance = MathF.Max(terminalDistance, 0f);
        var lastIndex         = orderedPath.Count - 1;
        l1DistanceField[orderedPath[lastIndex]] = remainingDistance;

        for (var i = lastIndex - 1; i >= 0; --i)
        {
            remainingDistance += CalculateL1TransitionCost(orderedPath[i], orderedPath[i + 1]);
            var voxel = orderedPath[i];
            if (!l1DistanceField.TryGetValue(voxel, out var existing) || remainingDistance < existing)
                l1DistanceField[voxel] = remainingDistance;
        }

        var frontier = new PriorityQueue<ulong, float>();
        foreach (var (seedVoxel, seedDistance) in l1DistanceField)
            frontier.Enqueue(seedVoxel, seedDistance);

        var               expanded = 0;
        Span<L1Neighbour> buffer   = stackalloc L1Neighbour[6];

        while (frontier.TryDequeue(out var current, out var queuedDistance) && expanded < L1_DISTANCE_FIELD_BUDGET)
        {
            if (!l1DistanceField.TryGetValue(current, out var currentDistance) || queuedDistance > currentDistance + SCORE_EPSILON)
                continue;
            expanded++;

            var count = EnumerateL1Neighbours(current, buffer);

            for (var i = 0; i < count; i++)
            {
                ref readonly var n = ref buffer[i];
                if (!Volume.IsEmpty(n.Voxel))
                    continue;
                if (!HasTraversableL1FaceTransition(current, n.Voxel, n.Dx, n.Dy, n.Dz))
                    continue;

                var edgeCost          = n.Dx != 0 ? l1Desc.CellSize.X : n.Dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                var neighbourDistance = currentDistance + edgeCost;
                if (l1DistanceField.TryGetValue(n.Voxel, out var existingDistance) && existingDistance <= neighbourDistance + SCORE_EPSILON)
                    continue;

                l1DistanceField[n.Voxel] = neighbourDistance;
                frontier.Enqueue(n.Voxel, neighbourDistance);
            }
        }

        l0DistanceField = new();

        foreach (var (l1Voxel, l1Dist) in l1DistanceField)
        {
            var l0Index = VoxelIndexUtil.ExtractL0Index(l1Voxel);
            if (!l0DistanceField.TryGetValue(l0Index, out var existing) || l1Dist < existing)
                l0DistanceField[l0Index] = l1Dist;
        }
    }

    private float CalculateL1TransitionCost(ulong fromL1, ulong toL1)
    {
        var fromTemp = fromL1;
        var fromL0   = l0Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref fromTemp));
        var fromL1C  = l1Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref fromTemp));
        var toTemp   = toL1;
        var toL0     = l0Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref toTemp));
        var toL1C    = l1Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref toTemp));

        var globalFromX = (fromL0.x * l1Desc.NumCellsX) + fromL1C.x;
        var globalFromY = (fromL0.y * l1Desc.NumCellsY) + fromL1C.y;
        var globalFromZ = (fromL0.z * l1Desc.NumCellsZ) + fromL1C.z;
        var globalToX   = (toL0.x   * l1Desc.NumCellsX) + toL1C.x;
        var globalToY   = (toL0.y   * l1Desc.NumCellsY) + toL1C.y;
        var globalToZ   = (toL0.z   * l1Desc.NumCellsZ) + toL1C.z;

        return (MathF.Abs(globalFromX - globalToX) * l1Desc.CellSize.X) +
               (MathF.Abs(globalFromY - globalToY) * l1Desc.CellSize.Y) +
               (MathF.Abs(globalFromZ - globalToZ) * l1Desc.CellSize.Z);
    }

    private HashSet<ulong>? SearchL1CoarsePath(ulong fromVoxel, Vector3 fromPoint, ulong toVoxel, Vector3 toPoint)
    {
        var fromL1 = ResolveRepresentativeL1Voxel(fromVoxel, fromPoint);
        var toL1   = ResolveRepresentativeL1Voxel(toVoxel,   toPoint);
        if (fromL1 == toL1)
            return [fromL1];

        var result = TrySearchL1Path(fromL1, toL1, false);
        if (result is { Count: > 0 })
            return result;

        if (Volume.IsEmpty(toL1))
            return result;

        return TrySearchL1Path(fromL1, toL1, true);
    }

    private L1BestEffortSearchResult SearchL1BestEffortPath(ulong fromVoxel, Vector3 fromPoint, ulong toVoxel, Vector3 toPoint)
    {
        var fromL1       = ResolveRepresentativeL1Voxel(fromVoxel, fromPoint);
        var toL1         = ResolveRepresentativeL1Voxel(toVoxel,   toPoint);
        var strictResult = TrySearchL1BestEffortPath(fromL1, toL1, fromVoxel, fromPoint, toVoxel, toPoint, false, false);
        if (strictResult.ReachedGoal)
            return strictResult;

        if (Volume.IsEmpty(toL1))
            return strictResult;

        var relaxedResult = TrySearchL1BestEffortPath(fromL1, toL1, fromVoxel, fromPoint, toVoxel, toPoint, true, true);
        return IsBetterL1BestEffortResult(relaxedResult, strictResult) ?
                   relaxedResult :
                   strictResult;
    }

    private void BuildL1Corridor(HashSet<ulong> pathSet, int corridorRadius)
    {
        currentL1CorridorRadius = corridorRadius;
        l0PathSet               = [];
        foreach (var l1Voxel in pathSet)
            l0PathSet.Add(VoxelIndexUtil.ExtractL0Index(l1Voxel));

        if (corridorRadius <= 0)
        {
            l1PathSet          = pathSet;
            l1CorridorDistance = null;
            l0CorridorDistance = null;
            return;
        }

        l1PathSet = null;
        var distances = new Dictionary<ulong, int>(pathSet.Count * 2);
        var frontier  = new Queue<ulong>();

        foreach (var pathVoxel in pathSet)
        {
            if (!distances.TryAdd(pathVoxel, 0))
                continue;

            frontier.Enqueue(pathVoxel);
        }

        while (frontier.TryDequeue(out var current))
        {
            var currentDistance = distances[current];
            if (currentDistance >= corridorRadius)
                continue;

            VisitL1Neighbours
            (
                current,
                neighbour =>
                {
                    if (distances.ContainsKey(neighbour))
                        return;

                    distances[neighbour] = currentDistance + 1;
                    frontier.Enqueue(neighbour);
                }
            );
        }

        l1CorridorDistance = distances;
        l0CorridorDistance = new();

        foreach (var (l1Voxel, distance) in distances)
        {
            var l0Index = VoxelIndexUtil.ExtractL0Index(l1Voxel);
            if (!l0CorridorDistance.TryGetValue(l0Index, out var existing) || distance < existing)
                l0CorridorDistance[l0Index] = distance;
        }
    }

    private void ClearL1AreaConstraint()
    {
        currentL1CorridorRadius = 0;
        l1PathSet               = null;
        l0PathSet               = null;
        l1CorridorDistance      = null;
        l0CorridorDistance      = null;
    }

    private LongRangeLateralBias BuildLongRangeLateralBias(Vector3 fromPos, Vector3 toPos, int attempt)
    {
        var horizontalDelta    = new Vector2(toPos.X - fromPos.X, toPos.Z - fromPos.Z);
        var horizontalDistance = horizontalDelta.Length();
        var forward = VoxelMathUtil.TryNormalize(horizontalDelta, out var normalizedForward) ?
                          normalizedForward :
                          Vector2.UnitX;
        var right            = new Vector2(-forward.Y, forward.X);
        var verticalDrop     = MathF.Max(fromPos.Y - toPos.Y, 0f);
        var leafVertical     = MathF.Max(l2Desc.CellSize.Y,   SCORE_EPSILON);
        var preferDescending = verticalDrop >= leafVertical * LONG_RANGE_LATERAL_DESCENT_ENABLE_MIN_DROP_LEAF_CELLS;
        var heightPriority =
            1f + Math.Clamp(verticalDrop / (leafVertical * LONG_RANGE_LATERAL_DESCENT_PRIORITY_DROP_LEAF_CELLS), 0f, LONG_RANGE_LATERAL_DESCENT_PRIORITY_MAX_BONUS);
        var directionalPenalty = MathF.Max(LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_MIN, 1f - (attempt * LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_ATTEMPT_STEP));
        return new(true, fromPos, forward, right, horizontalDistance, preferDescending, heightPriority, directionalPenalty);
    }

    private int ResolveLongRangeL1BestEffortStepBudget(Vector3 fromPos, Vector3 toPos, bool includeNonEmpty)
    {
        var horizontalL1Cells = VoxelMathUtil.HorizontalDistanceXZ(fromPos, toPos) / MathF.Max(MathF.Max(l1Desc.CellSize.X, l1Desc.CellSize.Z), SCORE_EPSILON);
        var verticalL1Cells   = MathF.Abs(fromPos.Y - toPos.Y)                     / MathF.Max(l1Desc.CellSize.Y,                               SCORE_EPSILON);
        var estimatedCells    = horizontalL1Cells + (verticalL1Cells * LONG_RANGE_L1_BEST_EFFORT_VERTICAL_DISTANCE_BUDGET_SCALE);
        var distanceBudget    = (int)(estimatedCells                 * LONG_RANGE_L1_BEST_EFFORT_DISTANCE_BUDGET_PER_CELL);
        var budget            = LONG_RANGE_L1_BEST_EFFORT_STEP_BUDGET + distanceBudget;

        if (includeNonEmpty)
            budget = (int)(budget * LONG_RANGE_L1_BEST_EFFORT_RELAXED_BUDGET_SCALE);

        return Math.Clamp(budget, LONG_RANGE_L1_BEST_EFFORT_STEP_BUDGET, LONG_RANGE_L1_BEST_EFFORT_MAX_STEP_BUDGET);
    }

    private float ResolveLongRangeL1GoalCaptureDistanceThreshold(Vector3 fromPos, Vector3 toPos)
    {
        var maxL1Extent    = MathF.Max(l1Desc.CellSize.X, MathF.Max(l1Desc.CellSize.Y, l1Desc.CellSize.Z));
        var directDistance = Vector3.Distance(fromPos, toPos);
        return MathF.Max(maxL1Extent * LONG_RANGE_L1_GOAL_CAPTURE_DISTANCE_THRESHOLD_L1_CELLS, directDistance * LONG_RANGE_L1_GOAL_CAPTURE_DIRECT_DISTANCE_RATIO);
    }

    private int ResolveLongRangeL1GoalCaptureStepBudget(float bestDistance)
    {
        var maxL1Extent    = MathF.Max(l1Desc.CellSize.X, MathF.Max(l1Desc.CellSize.Y, l1Desc.CellSize.Z));
        var estimatedCells = bestDistance / MathF.Max(maxL1Extent, SCORE_EPSILON);
        var budget         = LONG_RANGE_L1_GOAL_CAPTURE_BASE_STEP_BUDGET + (int)(estimatedCells * LONG_RANGE_L1_GOAL_CAPTURE_BUDGET_PER_CELL);
        return Math.Clamp(budget, LONG_RANGE_L1_GOAL_CAPTURE_BASE_STEP_BUDGET, LONG_RANGE_L1_GOAL_CAPTURE_MAX_STEP_BUDGET);
    }

    private int ResolveLongRangeGuidedFullSearchCorridorRadius(float bestDistance)
    {
        var maxL1Extent   = MathF.Max(l1Desc.CellSize.X, MathF.Max(l1Desc.CellSize.Y, l1Desc.CellSize.Z));
        var gapCells      = bestDistance / MathF.Max(maxL1Extent, SCORE_EPSILON);
        var dynamicRadius = LONG_RANGE_L1_GUIDED_FULL_SEARCH_BASE_CORRIDOR_RADIUS + (int)MathF.Ceiling(gapCells * LONG_RANGE_L1_GUIDED_FULL_SEARCH_GAP_RADIUS_SCALE);
        return Math.Clamp(dynamicRadius, LONG_RANGE_L1_GUIDED_FULL_SEARCH_BASE_CORRIDOR_RADIUS, LONG_RANGE_L1_GUIDED_FULL_SEARCH_MAX_CORRIDOR_RADIUS);
    }

    private L1BestEffortSearchResult TrySearchL1BestEffortPath
    (
        ulong   fromL1,
        ulong   toL1,
        ulong   fromVoxel,
        Vector3 fromPoint,
        ulong   toVoxel,
        Vector3 toPoint,
        bool    includeNonEmpty,
        bool    allowGoalCaptureExtension
    )
    {
        var tGoal               = toL1;
        var gL0                 = VoxelMap.DecodeIndex(ref tGoal);
        var gL1                 = VoxelMap.DecodeIndex(ref tGoal);
        var gL0c                = l0Desc.IndexToVoxel(gL0);
        var gL1c                = l1Desc.IndexToVoxel(gL1);
        var rootMin             = Volume.RootTile.BoundsMin;
        var coarseBias          = BuildLongRangeLateralBias(fromPoint, toPoint, 0);
        var stepBudget          = ResolveLongRangeL1BestEffortStepBudget(fromPoint, toPoint, includeNonEmpty);
        var totalBudget         = stepBudget;
        var startReachableFaces = GetReachableL1FacesFromPoint(fromL1, fromVoxel, fromPoint);
        var goalReachableFaces  = GetReachableL1FacesFromPoint(toL1,   toVoxel,   toPoint);
        var directGoalConnected = fromL1 == toL1 && ArePointsConnectedWithinL1(fromL1, fromVoxel, fromPoint, toVoxel, toPoint);
        var goalFacePenalty     = MathF.Min(l1Desc.CellSize.X, MathF.Min(l1Desc.CellSize.Y, l1Desc.CellSize.Z));

        Vector3 CoarseCellCenter((int x, int y, int z) l0c, (int x, int y, int z) l1c)
        {
            return rootMin                                                                                                                 +
                   new Vector3(l0c.x          * l0Desc.CellSize.X, l0c.y          * l0Desc.CellSize.Y, l0c.z          * l0Desc.CellSize.Z) +
                   new Vector3((l1c.x + 0.5f) * l1Desc.CellSize.X, (l1c.y + 0.5f) * l1Desc.CellSize.Y, (l1c.z + 0.5f) * l1Desc.CellSize.Z);
        }

        float H((int x, int y, int z) l0c, (int x, int y, int z) l1c)
        {
            var dx = ((l0c.x - gL0c.x) * l0Desc.CellSize.X) + ((l1c.x - gL1c.x) * l1Desc.CellSize.X);
            var dy = ((l0c.y - gL0c.y) * l0Desc.CellSize.Y) + ((l1c.y - gL1c.y) * l1Desc.CellSize.Y);
            var dz = ((l0c.z - gL0c.z) * l0Desc.CellSize.Z) + ((l1c.z - gL1c.z) * l1Desc.CellSize.Z);
            return MathF.Sqrt((dx      * dx)                + (dy               * dy) + (dz * dz));
        }

        float StateHeuristic(L1TraversalState state)
        {
            var temp  = state.Voxel;
            var l0Idx = VoxelMap.DecodeIndex(ref temp);
            var l1Idx = VoxelMap.DecodeIndex(ref temp);
            var l0c   = l0Desc.IndexToVoxel(l0Idx);
            var l1c   = l1Desc.IndexToVoxel(l1Idx);
            var position = state.Voxel   == fromL1 ? fromPoint
                           : state.Voxel == toL1   ? toPoint
                                                     : CoarseCellCenter(l0c, l1c);
            var baseH = coarseBias.Enabled ?
                            ComputeLongRangeLateralHeuristic(position, state.Voxel, coarseBias, toPoint) :
                            H(l0c, l1c);
            if (state.Voxel != toL1)
                return baseH;
            if (state.EntryFace == L1_FACE_INSIDE)
                return directGoalConnected ?
                           0f :
                           goalFacePenalty;

            return VoxelIndexUtil.HasL1Face(goalReachableFaces, state.EntryFace) ?
                       0f :
                       goalFacePenalty;
        }

        var startState = new L1TraversalState(fromL1, L1_FACE_INSIDE);
        var gScore     = new Dictionary<L1TraversalState, float> { [startState] = 0 };
        var cameFrom   = new Dictionary<L1TraversalState, L1TraversalState>();
        var closed     = new HashSet<L1TraversalState>();
        var openQ      = new PriorityQueue<L1TraversalState, float>();
        openQ.Enqueue(startState, StateHeuristic(startState));

        var bestState    = startState;
        var bestDistance = StateHeuristic(startState);
        var expanded     = 0;

        bool TryRunSearchPhase(int phaseBudget, out L1BestEffortSearchResult result)
        {
            Span<L1Neighbour> nbBuffer      = stackalloc L1Neighbour[6];
            var               phaseExpanded = 0;

            while (openQ.TryDequeue(out var current, out _) && phaseExpanded < phaseBudget)
            {
                if (!closed.Add(current))
                    continue;
                expanded++;
                phaseExpanded++;

                var t     = current.Voxel;
                var l0Idx = VoxelMap.DecodeIndex(ref t);
                var l1Idx = VoxelMap.DecodeIndex(ref t);
                var l0c   = l0Desc.IndexToVoxel(l0Idx);
                var l1c   = l1Desc.IndexToVoxel(l1Idx);
                var cg    = gScore[current];
                var h     = StateHeuristic(current);
                var currentPosition = current.Voxel   == fromL1 ? fromPoint
                                      : current.Voxel == toL1   ? toPoint
                                                                  : CoarseCellCenter(l0c, l1c);

                if (h + SCORE_EPSILON < bestDistance || (MathF.Abs(h - bestDistance) <= SCORE_EPSILON && cg < gScore.GetValueOrDefault(bestState, float.MaxValue)))
                {
                    bestState    = current;
                    bestDistance = h;
                }

                var reachedGoal = current.Voxel == toL1 &&
                                  (current.EntryFace == L1_FACE_INSIDE ?
                                       directGoalConnected :
                                       VoxelIndexUtil.HasL1Face(goalReachableFaces, current.EntryFace));

                if (reachedGoal)
                {
                    var orderedPath = ReconstructL1OrderedPath(cameFrom, current);
                    result = new([..orderedPath], orderedPath, true, expanded, 0f, totalBudget);
                    return true;
                }

                var exitFaceMask = current.EntryFace == L1_FACE_INSIDE ?
                                       startReachableFaces :
                                       GetReachableL1FacesFromEntry(current.Voxel, current.EntryFace);
                if (exitFaceMask == 0)
                    continue;

                var nbCount = EnumerateL1Neighbours(current.Voxel, nbBuffer);

                for (var i = 0; i < nbCount; i++)
                {
                    ref readonly var n = ref nbBuffer[i];
                    if (!VoxelIndexUtil.HasL1Face(exitFaceMask, n.Dir))
                        continue;

                    var nextState = new L1TraversalState(n.Voxel, (byte)(n.Dir ^ 1));
                    if (closed.Contains(nextState))
                        continue;
                    if (!HasTraversableL1FaceTransition(current.Voxel, n.Voxel, n.Dx, n.Dy, n.Dz))
                        continue;

                    var isEmpty = Volume.IsEmpty(n.Voxel);
                    if (!isEmpty && !VoxelIndexUtil.CanTraverseMixedL1Cell(includeNonEmpty, n.Voxel, toL1))
                        continue;

                    if (!isEmpty)
                    {
                        var entryReachableFaces = GetReachableL1FacesFromEntry(n.Voxel, nextState.EntryFace);
                        var canReachGoal        = n.Voxel == toL1 && VoxelIndexUtil.HasL1Face(goalReachableFaces, nextState.EntryFace);
                        if (entryReachableFaces == 0 && !canReachGoal)
                            continue;
                    }

                    var neighbourPosition = n.Voxel == toL1 ?
                                                toPoint :
                                                CoarseCellCenter((n.L0X, n.L0Y, n.L0Z), (n.L1X, n.L1Y, n.L1Z));
                    var edgeCost = n.Dx != 0 ? l1Desc.CellSize.X : n.Dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                    var mixedPenalty = isEmpty ?
                                           0f :
                                           edgeCost * LONG_RANGE_L1_BEST_EFFORT_MIXED_CELL_PENALTY_SCALE;
                    var traversalPenalty = coarseBias.Enabled ?
                                               CalculateLongRangeLateralTraversalPenalty
                                                   (current.Voxel, currentPosition, n.Voxel, neighbourPosition, coarseBias, toPoint) :
                                               0f;
                    var tentativeG = cg + edgeCost + mixedPenalty + traversalPenalty;

                    if (gScore.TryGetValue(nextState, out var existingG) && tentativeG >= existingG)
                        continue;

                    gScore[nextState]   = tentativeG;
                    cameFrom[nextState] = current;
                    openQ.Enqueue(nextState, tentativeG + StateHeuristic(nextState));
                }
            }

            result = default;
            return false;
        }

        if (TryRunSearchPhase(stepBudget, out var phaseResult))
            return phaseResult;

        var goalCaptureThreshold = ResolveLongRangeL1GoalCaptureDistanceThreshold(fromPoint, toPoint);

        if (allowGoalCaptureExtension && expanded >= stepBudget && openQ.Count > 0 && bestDistance <= goalCaptureThreshold)
        {
            var extraBudget = ResolveLongRangeL1GoalCaptureStepBudget(bestDistance);
            totalBudget += extraBudget;
            Service.Log.Debug($"[算路] 飞行体素粗层 best-effort 触发近终点续搜：最佳粗距离 = {bestDistance:f3}，附加预算 = {extraBudget}");

            if (TryRunSearchPhase(extraBudget, out phaseResult))
            {
                Service.Log.Debug($"[算路] 飞行体素粗层 best-effort 近终点续搜命中终点：总扩展节点 = {expanded}/{totalBudget}");
                return phaseResult;
            }

            Service.Log.Debug($"[算路] 飞行体素粗层 best-effort 近终点续搜仍未达终点：总扩展节点 = {expanded}/{totalBudget}，最佳粗距离 = {bestDistance:f3}");
        }

        var bestOrderedPath = ReconstructL1OrderedPath(cameFrom, bestState);
        return new([..bestOrderedPath], bestOrderedPath, false, expanded, bestDistance, totalBudget);
    }

    private static List<ulong> ReconstructL1OrderedPath(Dictionary<L1TraversalState, L1TraversalState> cameFrom, L1TraversalState endState)
    {
        List<ulong> path  = [];
        var         state = endState;

        while (true)
        {
            if (path.Count == 0 || path[^1] != state.Voxel)
                path.Add(state.Voxel);
            if (!cameFrom.TryGetValue(state, out var parent))
                break;
            state = parent;
        }

        path.Reverse();
        return path;
    }

    private static bool IsBetterL1BestEffortResult(L1BestEffortSearchResult candidate, L1BestEffortSearchResult current)
    {
        if (candidate.ReachedGoal != current.ReachedGoal)
            return candidate.ReachedGoal;

        if (candidate.BestDistance + SCORE_EPSILON < current.BestDistance)
            return true;
        if (current.BestDistance + SCORE_EPSILON < candidate.BestDistance)
            return false;

        if (candidate.PathSet.Count != current.PathSet.Count)
            return candidate.PathSet.Count > current.PathSet.Count;

        return candidate.ExpandedNodes < current.ExpandedNodes;
    }

    private HashSet<ulong>? TrySearchL1Path(ulong fromL1, ulong toL1, bool includeNonEmpty)
    {
        var tGoal = toL1;
        var gL0   = VoxelMap.DecodeIndex(ref tGoal);
        var gL1   = VoxelMap.DecodeIndex(ref tGoal);
        var gL0C  = l0Desc.IndexToVoxel(gL0);
        var gL1C  = l1Desc.IndexToVoxel(gL1);

        var gScore   = new Dictionary<ulong, float> { [fromL1] = 0 };
        var cameFrom = new Dictionary<ulong, ulong>();
        var closed   = new HashSet<ulong>();
        var openQ    = new PriorityQueue<ulong, float>();
        openQ.Enqueue(fromL1, 0);
        var               expanded = 0;
        Span<L1Neighbour> buffer   = stackalloc L1Neighbour[6];

        while (openQ.TryDequeue(out var current, out _) && expanded < L1_A_STAR_MAX_EXPANSIONS)
        {
            if (!closed.Add(current))
                continue;
            expanded++;

            if (current == toL1)
            {
                var pathSet = new HashSet<ulong>();
                var node    = toL1;

                while (true)
                {
                    pathSet.Add(node);
                    if (!cameFrom.TryGetValue(node, out var parent))
                        break;
                    node = parent;
                }

                return pathSet;
            }

            var cg = gScore[current];

            var count = EnumerateL1Neighbours(current, buffer);

            for (var i = 0; i < count; i++)
            {
                ref readonly var n = ref buffer[i];
                if (closed.Contains(n.Voxel))
                    continue;
                if (!HasTraversableL1FaceTransition(current, n.Voxel, n.Dx, n.Dy, n.Dz))
                    continue;
                var isEmpty = Volume.IsEmpty(n.Voxel);
                if (!isEmpty && !VoxelIndexUtil.CanTraverseMixedL1Cell(includeNonEmpty, n.Voxel, toL1))
                    continue;

                var edgeCost   = n.Dx != 0 ? l1Desc.CellSize.X : n.Dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                var tentativeG = cg + edgeCost;

                if (gScore.TryGetValue(n.Voxel, out var existingG) && tentativeG >= existingG)
                    continue;

                gScore[n.Voxel]   = tentativeG;
                cameFrom[n.Voxel] = current;
                openQ.Enqueue(n.Voxel, tentativeG + H((n.L0X, n.L0Y, n.L0Z), (n.L1X, n.L1Y, n.L1Z)));
            }
        }

        return null;

        float H((int x, int y, int z) l0C, (int x, int y, int z) l1C)
        {
            var dx = ((l0C.x - gL0C.x) * l0Desc.CellSize.X) + ((l1C.x - gL1C.x) * l1Desc.CellSize.X);
            var dy = ((l0C.y - gL0C.y) * l0Desc.CellSize.Y) + ((l1C.y - gL1C.y) * l1Desc.CellSize.Y);
            var dz = ((l0C.z - gL0C.z) * l0Desc.CellSize.Z) + ((l1C.z - gL1C.z) * l1Desc.CellSize.Z);
            return MathF.Sqrt((dx      * dx)                + (dy               * dy) + (dz * dz));
        }
    }
}
