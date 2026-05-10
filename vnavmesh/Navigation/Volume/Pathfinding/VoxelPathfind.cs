using System.Collections.Concurrent;
using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Configuration;

namespace vnavmesh.Navigation.Volume.Pathfinding;

public class VoxelPathfind
{
    private readonly VolumeLevel l0Desc;
    private readonly VolumeLevel l1Desc;
    private readonly VolumeLevel l2Desc;
    
    private readonly float       maxSearchRaycastDistance;
    private readonly float       goalVisibilityProbeDistance;
    private readonly List<ulong> neighbourScratch = new(64);

    private readonly List<VolumePathfindNode>                        nodes           = new(1024);
    private readonly Dictionary<ulong, int>                          nodeLookup      = new(1024);
    private readonly List<int>                                       openList        = new(256);
    private readonly ConcurrentDictionary<VolumeVisibilityKey, bool> visibilityCache = new(Environment.ProcessorCount, 4096);

    private int                       bestNodeIndex;
    private ulong                     goalVoxel;
    private Vector3                   goalPos;
    private bool                      goalReached;
    private bool                      useSearchRaycast;
    private float                     heuristicWeight;
    private int                       visitedNodes;
    private int                       generatedNodes;
    private int                       lineOfSightChecks;
    private int                       lineOfSightHits;
    private int                       peakOpenListSize;
    private VolumeSearchTermination   lastTermination;
    private bool                      lastSearchRaycastEnabled;
    private int                       lastSearchAttempts;
    private HashSet<ulong>?           l1PathSet;
    private Dictionary<ulong, float>? l1DistanceField;

    public VoxelMap Volume { get; }

    public Span<VolumePathfindNode> NodeSpan => CollectionsMarshal.AsSpan(nodes);

    internal VolumeSearchTelemetry LastTelemetry => new
    (
        visitedNodes,
        generatedNodes,
        lineOfSightChecks,
        lineOfSightHits,
        peakOpenListSize,
        lastTermination,
        lastSearchRaycastEnabled,
        lastSearchAttempts,
        heuristicWeight
    );

    public VoxelPathfind(VoxelMap volume, Config _)
    {
        Volume                      = volume;
        l0Desc                      = volume.Levels[0];
        l1Desc                      = volume.Levels[1];
        l2Desc                      = volume.Levels[2];
        maxSearchRaycastDistance    = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS;
        goalVisibilityProbeDistance = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS;
    }

    public List<(ulong voxel, Vector3 p)> FindPath
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              useRaycast,
        bool              returnIntermediatePoints,
        CancellationToken cancel
    )
    {
        l1PathSet       = null;
        l1DistanceField = null;

        if (fromVoxel == toVoxel)
        {
            ResetSearchState();
            goalVoxel          = toVoxel;
            goalPos            = toPos;
            bestNodeIndex      = 0;
            goalReached        = true;
            visitedNodes       = 1;
            generatedNodes     = 1;
            lastTermination    = VolumeSearchTermination.ReachedGoal;
            lastSearchAttempts = 1;
            nodes.Add
            (
                new()
                {
                    GScore        = 0,
                    HScore        = 0,
                    Voxel         = toVoxel,
                    ParentIndex   = 0,
                    OpenHeapIndex = -1,
                    Closed        = true,
                    Position      = toPos
                }
            );
            nodeLookup[toVoxel] = 0;
            return [(toVoxel, toPos)];
        }

        if (useRaycast && TryBuildDirectPath(fromVoxel, toVoxel, fromPos, toPos, out var directPath))
            return directPath;

        var searchRaycast = useRaycast && Vector3.Distance(fromPos, toPos) <= maxSearchRaycastDistance;

        if (searchRaycast)
        {
            var path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, true, returnIntermediatePoints, cancel, RAYCAST_SEARCH_STEP_BUDGET, 1);

            if (lastTermination == VolumeSearchTermination.StepBudgetReached)
            {
                Service.Log.Debug
                (
                    $"[算路] 飞行体素搜索触发降级重试：直线距离 = {Vector3.Distance(fromPos, toPos):f3}，首轮访问节点 = {visitedNodes}，LoS 检查 = {lineOfSightChecks}"
                );
                path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, false, returnIntermediatePoints, cancel, DEFAULT_MAX_SEARCH_STEPS, 2);
            }

            return useRaycast ? RefineSimplifiedPath(path, cancel) : path;
        }

        var l1Path = SearchL1CoarsePath(fromVoxel, toVoxel);

        if (l1Path is { Count: > 0 } pathSet)
        {
            l1PathSet = pathSet;
            var constrainedPath = RunSearchAttempt
                (fromVoxel, toVoxel, fromPos, toPos, false, returnIntermediatePoints, cancel, DEFAULT_MAX_SEARCH_STEPS, 1);

            if (lastTermination == VolumeSearchTermination.ReachedGoal)
            {
                Service.Log.Debug
                (
                    $"[算路] 飞行体素 L1 约束搜索完成：访问节点 = {visitedNodes}，L1 路径单元 = {pathSet.Count}"
                );
                return useRaycast ? RefineSimplifiedPath(constrainedPath, cancel) : constrainedPath;
            }

            Service.Log.Debug
            (
                $"[算路] 飞行体素 L1 约束搜索未达终点（{lastTermination}），回退全搜索"
            );
        }
        else Service.Log.Debug("[算路] 飞行体素 L1 粗搜索未找到路径，回退到全搜索");

        l1PathSet = null;
        if (l1DistanceField is not { Count: > 0 })
            ComputeL1DistanceField(toVoxel);
        var fallbackPath = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, false, returnIntermediatePoints, cancel, DEFAULT_MAX_SEARCH_STEPS, 1);
        return useRaycast ? RefineSimplifiedPath(fallbackPath, cancel) : fallbackPath;
    }

    public void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        ResetSearchState();

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
        nodeLookup[fromVoxel] = 0;
        generatedNodes        = 1;
        AddToOpen(0);
    }

    private VolumeSearchTermination Execute(CancellationToken cancel, int maxSteps = DEFAULT_MAX_SEARCH_STEPS)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return goalReached ? VolumeSearchTermination.ReachedGoal : VolumeSearchTermination.SearchExhausted;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }

        return VolumeSearchTermination.StepBudgetReached;
    }

    private List<(ulong voxel, Vector3 p)> RunSearchAttempt
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              useRaycast,
        bool              returnIntermediatePoints,
        CancellationToken cancel,
        int               maxSteps,
        int               attempts
    )
    {
        Start(fromVoxel, toVoxel, fromPos, toPos);
        useSearchRaycast = useRaycast;
        heuristicWeight = useRaycast
                              ? SHORT_RANGE_HEURISTIC_WEIGHT
                              : SHORT_RANGE_HEURISTIC_WEIGHT +
                                (LONG_RANGE_HEURISTIC_WEIGHT - SHORT_RANGE_HEURISTIC_WEIGHT) *
                                Math.Clamp(Vector3.Distance(fromPos, toPos) / (maxSearchRaycastDistance * LONG_RANGE_HEURISTIC_BLEND_DISTANCE), 0f, 1f);
        lastSearchRaycastEnabled = useRaycast;
        lastSearchAttempts       = attempts;
        lastTermination          = Execute(cancel, maxSteps);
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

        var neighbours = CollectNeighbours(current.Voxel);

        if (useSearchRaycast && neighbours.Count >= RAYCAST_PARALLEL_NEIGHBOUR_THRESHOLD && Environment.ProcessorCount > 1)
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
        bestNodeIndex            = 0;
        goalReached              = false;
        visitedNodes             = 0;
        generatedNodes           = 0;
        lineOfSightChecks        = 0;
        lineOfSightHits          = 0;
        peakOpenListSize         = 0;
        heuristicWeight          = 1;
        lastTermination          = VolumeSearchTermination.SearchExhausted;
        lastSearchRaycastEnabled = false;
        lastSearchAttempts       = 0;
    }

    internal void ReleaseRetainedState()
    {
        ResetSearchState();
        neighbourScratch.Clear();
        neighbourScratch.TrimExcess();
        nodes.TrimExcess();
        nodeLookup.TrimExcess();
        openList.TrimExcess();
        l1PathSet       = null;
        l1DistanceField = null;
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

    private List<(ulong voxel, Vector3 p)> SimplifyPath(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return path;

        List<(ulong voxel, Vector3 p)> simplified  = [path[0]];
        var                            anchorIndex = 0;

        while (anchorIndex < path.Count - 1)
        {
            var furthestVisibleIndex = FindFurthestVisibleIndex(path, anchorIndex, cancel);

            simplified.Add(path[furthestVisibleIndex]);
            anchorIndex = furthestVisibleIndex;
        }

        return simplified;
    }

    private List<(ulong voxel, Vector3 p)> RefineSimplifiedPath(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return path;

        var refined = SimplifyPath(path, cancel);
        refined.Reverse();
        refined = SimplifyPath(refined, cancel);
        refined.Reverse();
        RelaxInteriorWaypoints(refined, cancel);
        return SimplifyPath(refined, cancel);
    }

    private void RelaxInteriorWaypoints(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return;

        for (var i = 1; i < path.Count - 1; ++i)
        {
            if ((i & 0x3f) == 0)
                cancel.ThrowIfCancellationRequested();

            var     previous = path[i - 1];
            var     current  = path[i];
            var     next     = path[i + 1];
            Vector3 relaxed;

            if (current.voxel == goalVoxel) relaxed = goalPos;
            else
            {
                var segment       = next.p - previous.p;
                var lengthSquared = segment.LengthSquared();
                if (lengthSquared <= SCORE_EPSILON * SCORE_EPSILON)
                    continue;

                var progress  = Math.Clamp(Vector3.Dot(current.p - previous.p, segment) / lengthSquared, 0f, 1f);
                var projected = previous.p + progress * segment;
                relaxed = Volume.ClampPointToVoxel(current.voxel, projected);
            }

            if (Vector3.DistanceSquared(relaxed, current.p) <= SCORE_EPSILON * SCORE_EPSILON)
                continue;
            if (!HasLineOfSight(previous, current.voxel, relaxed))
                continue;
            if (!HasLineOfSight((current.voxel, relaxed), next.voxel, next.p))
                continue;

            path[i] = (current.voxel, relaxed);
        }
    }

    private int FindFurthestVisibleIndex(List<(ulong voxel, Vector3 p)> path, int anchorIndex, CancellationToken cancel)
    {
        var pathLastIndex = path.Count  - 1;
        var nextIndex     = anchorIndex + 1;
        if (nextIndex >= pathLastIndex || !HasLineOfSight(path, anchorIndex, nextIndex, cancel))
            return nextIndex;

        var furthestVisibleIndex = nextIndex;
        var step                 = 1;

        while (furthestVisibleIndex < pathLastIndex)
        {
            var probeIndex = Math.Min(furthestVisibleIndex + step, pathLastIndex);
            if (!HasLineOfSight(path, anchorIndex, probeIndex, cancel))
                return FindVisibleBoundary(path, anchorIndex, furthestVisibleIndex, probeIndex - 1, cancel);

            furthestVisibleIndex =   probeIndex;
            step                 <<= 1;
        }

        return furthestVisibleIndex;
    }

    private int FindVisibleBoundary
    (
        List<(ulong voxel, Vector3 p)> path,
        int                            anchorIndex,
        int                            visibleIndex,
        int                            blockedIndex,
        CancellationToken              cancel
    )
    {
        while (visibleIndex < blockedIndex)
        {
            var mid = visibleIndex + (blockedIndex - visibleIndex + 1 >> 1);
            if (HasLineOfSight(path, anchorIndex, mid, cancel)) visibleIndex = mid;
            else blockedIndex                                                = mid - 1;
        }

        return visibleIndex;
    }

    private bool HasLineOfSight(List<(ulong voxel, Vector3 p)> path, int anchorIndex, int probeIndex, CancellationToken cancel)
    {
        if ((probeIndex & 0x3f) == 0)
            cancel.ThrowIfCancellationRequested();

        var anchor = path[anchorIndex];
        var probe  = path[probeIndex];
        ++lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, anchor.voxel, probe.voxel, anchor.p, probe.p))
            return false;

        ++lineOfSightHits;
        return true;
    }

    private bool HasLineOfSight((ulong voxel, Vector3 p) from, ulong toVoxel, Vector3 toPosition)
    {
        ++lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, from.voxel, toVoxel, from.p, toPosition))
            return false;

        ++lineOfSightHits;
        return true;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var result = new List<(ulong voxel, Vector3 p)>();

        if ((uint)nodeIndex >= (uint)nodes.Count)
            return result;

        var nodeSpan = NodeSpan;
        result.Add((nodeSpan[nodeIndex].Voxel, nodeSpan[nodeIndex].Position));

        while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
        {
            ref var child       = ref nodeSpan[nodeIndex];
            var     parentIndex = child.ParentIndex;
            ref var parent      = ref nodeSpan[parentIndex];

            if (returnIntermediatePoints)
            {
                var delta = parent.Position - child.Position;

                foreach (var step in VoxelSearch.EnumerateVoxelsInLine(Volume, child.Voxel, parent.Voxel, child.Position, parent.Position))
                {
                    if (!step.empty)
                        continue;

                    result.Add((step.voxel, child.Position + step.t * delta));
                }
            }
            else result.Add((parent.Voxel, parent.Position));

            nodeIndex = parentIndex;
        }

        result.Reverse();
        return result;
    }

    private List<ulong> CollectNeighbours(ulong voxel)
    {
        neighbourScratch.Clear();

        var encodedVoxel = voxel;
        var l0Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l1Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l2Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l0Coords     = l0Desc.IndexToVoxel(l0Index);
        var l1Coords     = l1Index != VoxelMap.INDEX_LEVEL_MASK ? l1Desc.IndexToVoxel(l1Index) : default;
        var l2Coords     = l2Index != VoxelMap.INDEX_LEVEL_MASK ? l2Desc.IndexToVoxel(l2Index) : default;

        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  -1, 0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  +1, 0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, -1, 0,  0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, +1, 0,  0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  0,  -1);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  0,  +1);

        return neighbourScratch;
    }

    private void CollectDirection
    (
        ushort                l0Index,
        ushort                l1Index,
        ushort                l2Index,
        (int x, int y, int z) l0Coords,
        (int x, int y, int z) l1Coords,
        (int x, int y, int z) l2Coords,
        int                   dx,
        int                   dy,
        int                   dz
    )
    {
        if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);

            if (l2Desc.InBounds(l2Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                AddNeighbourIfEmpty(neighbourVoxel);
                return;
            }
        }

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);

            if (l1Desc.InBounds(l1Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);

                if (Volume.IsEmpty(neighbourVoxel)) AddNeighbourIfEmpty(neighbourVoxel);
                else if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    AddNeighbourIfEmpty(l2NeighbourVoxel);
                }
                else CollectBorder(neighbourVoxel, l2Desc, 2, dx, dy, dz);

                return;
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!l0Desc.InBounds(l0Neighbour))
            return;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));

        if (Volume.IsEmpty(l0NeighbourVoxel))
        {
            AddNeighbourIfEmpty(l0NeighbourVoxel);
            return;
        }

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1X              = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
            var l1Y              = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
            var l1Z              = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
            var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(l0NeighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);

            if (Volume.IsEmpty(l1NeighbourVoxel)) AddNeighbourIfEmpty(l1NeighbourVoxel);
            else if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                AddNeighbourIfEmpty(l2NeighbourVoxel);
            }
            else CollectBorder(l1NeighbourVoxel, l2Desc, 2, dx, dy, dz);

            return;
        }

        CollectBorderWithSubdivisions(l0NeighbourVoxel, dx, dy, dz);
    }

    private void CollectBorder(ulong voxel, VolumeLevel levelDesc, int level, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, levelDesc.NumCellsX - 1) : dx > 0 ? (0, 0) : (levelDesc.NumCellsX - 1, levelDesc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, levelDesc.NumCellsY - 1) : dy > 0 ? (0, 0) : (levelDesc.NumCellsY - 1, levelDesc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, levelDesc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (levelDesc.NumCellsZ - 1, levelDesc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
            AddNeighbourIfEmpty(VoxelMap.EncodeSubIndex(voxel, levelDesc.VoxelToIndex(x, y, z), level));
    }

    private void CollectBorderWithSubdivisions(ulong voxel, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (l1Desc.NumCellsX - 1, l1Desc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (l1Desc.NumCellsY - 1, l1Desc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (l1Desc.NumCellsZ - 1, l1Desc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel)) AddNeighbourIfEmpty(l1Voxel);
            else CollectBorder(l1Voxel, l2Desc, 2, dx, dy, dz);
        }
    }

    private void AddNeighbourIfEmpty(ulong voxel)
    {
        if (l1PathSet is { } pathSet && !pathSet.Contains(ExtractL1Parent(voxel)))
            return;
        if (Volume.IsEmpty(voxel))
            neighbourScratch.Add(voxel);
    }

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
        nodeLookup[voxel] = nodeIndex;
        ++generatedNodes;
        return nodeIndex;
    }

    private bool TryGetBestCandidate(int currentIndex, ulong neighbourVoxel, out int bestParentIndex, out Vector3 bestPosition, out float bestScore)
    {
        bestParentIndex = -1;
        bestPosition    = default;
        bestScore       = float.MaxValue;

        if (!TryEvaluateCandidate(currentIndex, neighbourVoxel, false, ref bestParentIndex, ref bestPosition, ref bestScore))
            return false;

        if (!useSearchRaycast)
            return true;

        var nodeSpan      = NodeSpan;
        var ancestorIndex = nodeSpan[currentIndex].ParentIndex;
        var lookBackCount = 0;
        var lastEvaluated = -1;

        while (ancestorIndex >= 0 && lookBackCount < MAX_ANCESTOR_LOOK_BACK)
        {
            TryEvaluateCandidate(ancestorIndex, neighbourVoxel, true, ref bestParentIndex, ref bestPosition, ref bestScore);
            lastEvaluated = ancestorIndex;

            ref var ancestor = ref nodeSpan[ancestorIndex];
            if (ancestor.ParentIndex == ancestorIndex)
                return bestParentIndex >= 0;

            ancestorIndex = ancestor.ParentIndex;
            ++lookBackCount;
        }

        if (ancestorIndex >= 0)
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
        var projectedScore = CalculateNodeScore(parentIndex, candidatePositions[0]);
        if (bestParentIndex >= 0 && projectedScore > bestScore + SCORE_EPSILON)
            return false;

        candidatePositions[1] = ResolveGoalAlignedPosition(voxel);
        candidateKinds[1]     = VolumePathCandidateKind.GoalAligned;
        candidatePositions[2] = ResolveCenterBiasedPosition(voxel, parentIndex);
        candidateKinds[2]     = VolumePathCandidateKind.CenterBiased;

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

            var candidateScore = i == 0 ? projectedScore : CalculateNodeScore(parentIndex, candidatePosition);
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
        return Volume.ClampPointToVoxel(voxel, nodeSpan[parentIndex].Position);
    }

    private Vector3 ResolveGoalAlignedPosition(ulong voxel)
    {
        if (voxel == goalVoxel)
            return goalPos;

        return Volume.ClampPointToVoxel(voxel, goalPos);
    }

    private Vector3 ResolveCenterBiasedPosition(ulong voxel, int parentIndex)
    {
        if (voxel == goalVoxel)
            return goalPos;

        var projected     = ResolveProjectedPosition(voxel, parentIndex);
        var goalAligned   = ResolveGoalAlignedPosition(voxel);
        var blendedTarget = Vector3.Lerp(projected, goalAligned, 0.5f);
        return Volume.ClampPointToVoxel(voxel, blendedTarget);
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

    private float CalculateNodeScore(int parentIndex, Vector3 destination)
    {
        var nodeSpan = NodeSpan;
        return nodeSpan[parentIndex].GScore + CalculateEdgeCost(nodeSpan[parentIndex].Position, destination);
    }

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

    private static ulong ExtractL1Parent(ulong voxel)
    {
        var temp    = voxel;
        var l0      = VoxelMap.DecodeIndex(ref temp);
        var l1      = VoxelMap.DecodeIndex(ref temp);
        var l1Voxel = VoxelMap.EncodeIndex(l1);
        l1Voxel = VoxelMap.EncodeIndex(l0, l1Voxel);
        return l1Voxel;
    }

    private void ComputeL1DistanceField(ulong goalVoxel)
    {
        var goalL1 = ExtractL1Parent(goalVoxel);
        l1DistanceField = new() { [goalL1] = 0 };
        var frontier = new Queue<ulong>();
        frontier.Enqueue(goalL1);
        var expanded = 0;

        while (frontier.TryDequeue(out var current) && expanded < L1_DISTANCE_FIELD_BUDGET)
        {
            expanded++;
            var currentDist = l1DistanceField[current];

            var t     = current;
            var l0Idx = VoxelMap.DecodeIndex(ref t);
            var l1Idx = VoxelMap.DecodeIndex(ref t);
            var l0c   = l0Desc.IndexToVoxel(l0Idx);
            var l1c   = l1Desc.IndexToVoxel(l1Idx);

            for (var dir = 0; dir < 6; dir++)
            {
                var dx = dir == 2 ? -1 : dir == 3 ? 1 : 0;
                var dy = dir == 0 ? -1 : dir == 1 ? 1 : 0;
                var dz = dir == 4 ? -1 : dir == 5 ? 1 : 0;

                var nl1x = l1c.x + dx;
                var nl1y = l1c.y + dy;
                var nl1z = l1c.z + dz;
                var nl0x = l0c.x;
                var nl0y = l0c.y;
                var nl0z = l0c.z;

                if (nl1x < 0)
                {
                    nl0x--;
                    nl1x = l1Desc.NumCellsX - 1;
                }
                else if (nl1x >= l1Desc.NumCellsX)
                {
                    nl0x++;
                    nl1x = 0;
                }

                if (nl1y < 0)
                {
                    nl0y--;
                    nl1y = l1Desc.NumCellsY - 1;
                }
                else if (nl1y >= l1Desc.NumCellsY)
                {
                    nl0y++;
                    nl1y = 0;
                }

                if (nl1z < 0)
                {
                    nl0z--;
                    nl1z = l1Desc.NumCellsZ - 1;
                }
                else if (nl1z >= l1Desc.NumCellsZ)
                {
                    nl0z++;
                    nl1z = 0;
                }

                if (!l0Desc.InBounds(nl0x, nl0y, nl0z))
                    continue;

                var neighbour = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(nl1x, nl1y, nl1z));
                neighbour = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(nl0x, nl0y, nl0z), neighbour);

                if (!Volume.IsEmpty(neighbour) || l1DistanceField.ContainsKey(neighbour))
                    continue;

                var edgeCost = dx != 0 ? l1Desc.CellSize.X : dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                l1DistanceField[neighbour] = currentDist + edgeCost;
                frontier.Enqueue(neighbour);
            }
        }
    }

    private HashSet<ulong>? SearchL1CoarsePath(ulong fromVoxel, ulong toVoxel)
    {
        var fromL1 = ExtractL1Parent(fromVoxel);
        var toL1   = ExtractL1Parent(toVoxel);
        if (fromL1 == toL1)
            return [fromL1];

        var result = TrySearchL1Path(fromL1, toL1, false);
        if (result is { Count: > 0 })
            return result;

        return TrySearchL1Path(fromL1, toL1, true);
    }

    private HashSet<ulong>? TrySearchL1Path(ulong fromL1, ulong toL1, bool includeNonEmpty)
    {
        var tGoal = toL1;
        var gL0   = VoxelMap.DecodeIndex(ref tGoal);
        var gL1   = VoxelMap.DecodeIndex(ref tGoal);
        var gL0c  = l0Desc.IndexToVoxel(gL0);
        var gL1c  = l1Desc.IndexToVoxel(gL1);

        float H((int x, int y, int z) l0c, (int x, int y, int z) l1c)
        {
            var dx = (l0c.x - gL0c.x) * l0Desc.CellSize.X + (l1c.x - gL1c.x) * l1Desc.CellSize.X;
            var dy = (l0c.y - gL0c.y) * l0Desc.CellSize.Y + (l1c.y - gL1c.y) * l1Desc.CellSize.Y;
            var dz = (l0c.z - gL0c.z) * l0Desc.CellSize.Z + (l1c.z - gL1c.z) * l1Desc.CellSize.Z;
            return MathF.Sqrt(dx      * dx                + dy               * dy + dz * dz);
        }

        var gScore   = new Dictionary<ulong, float> { [fromL1] = 0 };
        var cameFrom = new Dictionary<ulong, ulong>();
        var closed   = new HashSet<ulong>();
        var openQ    = new PriorityQueue<ulong, float>();
        openQ.Enqueue(fromL1, 0);
        var expanded = 0;

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

            var t     = current;
            var l0Idx = VoxelMap.DecodeIndex(ref t);
            var l1Idx = VoxelMap.DecodeIndex(ref t);
            var l0c   = l0Desc.IndexToVoxel(l0Idx);
            var l1c   = l1Desc.IndexToVoxel(l1Idx);
            var cg    = gScore[current];

            for (var dir = 0; dir < 6; dir++)
            {
                var dx = dir == 2 ? -1 : dir == 3 ? 1 : 0;
                var dy = dir == 0 ? -1 : dir == 1 ? 1 : 0;
                var dz = dir == 4 ? -1 : dir == 5 ? 1 : 0;

                var nl1x = l1c.x + dx;
                var nl1y = l1c.y + dy;
                var nl1z = l1c.z + dz;
                var nl0x = l0c.x;
                var nl0y = l0c.y;
                var nl0z = l0c.z;

                if (nl1x < 0)
                {
                    nl0x--;
                    nl1x = l1Desc.NumCellsX - 1;
                }
                else if (nl1x >= l1Desc.NumCellsX)
                {
                    nl0x++;
                    nl1x = 0;
                }

                if (nl1y < 0)
                {
                    nl0y--;
                    nl1y = l1Desc.NumCellsY - 1;
                }
                else if (nl1y >= l1Desc.NumCellsY)
                {
                    nl0y++;
                    nl1y = 0;
                }

                if (nl1z < 0)
                {
                    nl0z--;
                    nl1z = l1Desc.NumCellsZ - 1;
                }
                else if (nl1z >= l1Desc.NumCellsZ)
                {
                    nl0z++;
                    nl1z = 0;
                }

                if (!l0Desc.InBounds(nl0x, nl0y, nl0z))
                    continue;

                var neighbour = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(nl1x, nl1y, nl1z));
                neighbour = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(nl0x, nl0y, nl0z), neighbour);

                if (closed.Contains(neighbour))
                    continue;
                if (!includeNonEmpty && !Volume.IsEmpty(neighbour))
                    continue;

                var edgeCost   = dx != 0 ? l1Desc.CellSize.X : dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                var tentativeG = cg + edgeCost;

                if (gScore.TryGetValue(neighbour, out var existingG) && tentativeG >= existingG)
                    continue;

                gScore[neighbour]   = tentativeG;
                cameFrom[neighbour] = current;
                openQ.Enqueue(neighbour, tentativeG + H((nl0x, nl0y, nl0z), (nl1x, nl1y, nl1z)));
            }
        }

        return null;
    }

    private float HeuristicDistance(Vector3 position, ulong voxel)
    {
        var h = Vector3.Distance(position, goalPos);
        if (l1DistanceField is { Count: > 0 } df && df.TryGetValue(ExtractL1Parent(voxel), out var l1Dist))
            h = MathF.Max(h, l1Dist);
        return h;
    }

    private float TotalScore(float gScore, float hScore) => gScore + hScore * heuristicWeight;

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
        var parent    = heapIndex - 1 >> 1;

        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[openList[parent]]))
        {
            openList[heapIndex]                         = openList[parent];
            nodeSpan[openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex                                   = parent;
            parent                                      = heapIndex - 1 >> 1;
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
            var child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            var child2 = child1 + 1;

            if (child2 == maxSize || HeapLess(ref nodeSpan[openList[child1]], ref nodeSpan[openList[child2]]))
            {
                if (HeapLess(ref nodeSpan[openList[child1]], ref nodeSpan[nodeIndex]))
                {
                    openList[heapIndex]                         = openList[child1];
                    nodeSpan[openList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex                                   = child1;
                }
                else break;
            }
            else if (HeapLess(ref nodeSpan[openList[child2]], ref nodeSpan[nodeIndex]))
            {
                openList[heapIndex]                         = openList[child2];
                nodeSpan[openList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex                                   = child2;
            }
            else break;
        }

        openList[heapIndex]               = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

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

    #region 常量

    private const float SCORE_EPSILON                                = 0.00001f;
    private const int   DEFAULT_MAX_SEARCH_STEPS                     = 1_0000_0000;
    private const int   RAYCAST_SEARCH_STEP_BUDGET                   = 200000;
    private const int   MAX_ANCESTOR_LOOK_BACK                       = 6;
    private const int   RAYCAST_PARALLEL_NEIGHBOUR_THRESHOLD         = 12;
    private const float MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS    = 96f;
    private const float SHORT_RANGE_HEURISTIC_WEIGHT                 = 1.05f;
    private const float LONG_RANGE_HEURISTIC_WEIGHT                  = 1.45f;
    private const float LONG_RANGE_HEURISTIC_BLEND_DISTANCE          = 4f;
    private const float GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS = 48f;
    private const int   L1_A_STAR_MAX_EXPANSIONS                     = 200_000;
    private const int   L1_DISTANCE_FIELD_BUDGET                     = 500_000;

    #endregion
}
