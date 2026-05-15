using System.Collections.Concurrent;
using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Planning;

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
    private readonly ConcurrentDictionary<ulong, byte>               voxelWallMaskCache = new(Environment.ProcessorCount, 4096);
    private readonly List<int>                                       openList        = new(256);
    private readonly ConcurrentDictionary<VolumeVisibilityKey, bool> visibilityCache = new(Environment.ProcessorCount, 4096);

    private int                       bestNodeIndex;
    private ulong                     goalVoxel;
    private Vector3                   goalPos;
    private bool                      goalReached;
    private bool                      useSearchRaycast;
    private bool                      useGuidedCorridor;
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
    private FlightPathDebugPayload?   lastPathDebug;
    private GuidedSearchCorridor      guidedCorridor;

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

    internal FlightPathDebugPayload? LastPathDebug => lastPathDebug;

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
        lastPathDebug   = null;

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

        if (TryCreateGuidedCorridor(fromPos, toPos, out var corridor))
        {
            var corridorPath = RunSearchAttempt
            (
                fromVoxel,
                toVoxel,
                fromPos,
                toPos,
                false,
                returnIntermediatePoints,
                cancel,
                GUIDED_CORRIDOR_SEARCH_STEP_BUDGET,
                1,
                corridor,
                GUIDED_CORRIDOR_HEURISTIC_WEIGHT
            );

            if (lastTermination == VolumeSearchTermination.ReachedGoal)
            {
                Service.Log.Debug
                (
                    $"[算路] 飞行体素定向走廊搜索完成：访问节点 = {visitedNodes}，走廊半径 = {corridor.HorizontalRadius:f3}，上抬余量 = {corridor.UpwardAllowance:f3}"
                );
                return useRaycast ? RefineSimplifiedPath(corridorPath, cancel) : corridorPath;
            }

            Service.Log.Debug
            (
                $"[算路] 飞行体素定向走廊搜索未达终点（{lastTermination}），回退 L1/全搜索"
            );
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
        int               attempts,
        GuidedSearchCorridor? corridor = null,
        float?            heuristicWeightOverride = null
    )
    {
        Start(fromVoxel, toVoxel, fromPos, toPos);
        useSearchRaycast = useRaycast;
        useGuidedCorridor = corridor.HasValue;
        guidedCorridor    = corridor.GetValueOrDefault();
        heuristicWeight = heuristicWeightOverride ??
                          (useRaycast
                              ? SHORT_RANGE_HEURISTIC_WEIGHT
                              : SHORT_RANGE_HEURISTIC_WEIGHT +
                                (LONG_RANGE_HEURISTIC_WEIGHT - SHORT_RANGE_HEURISTIC_WEIGHT) *
                                Math.Clamp(Vector3.Distance(fromPos, toPos) / (maxSearchRaycastDistance * LONG_RANGE_HEURISTIC_BLEND_DISTANCE), 0f, 1f));
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
        useGuidedCorridor        = false;
        guidedCorridor           = default;
        lastTermination          = VolumeSearchTermination.SearchExhausted;
        lastSearchRaycastEnabled = false;
        lastSearchAttempts       = 0;
        lastPathDebug            = null;
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
        {
            lastPathDebug = null;
            return path;
        }

        var refined = SimplifyPath(path, cancel);
        refined.Reverse();
        refined = SimplifyPath(refined, cancel);
        refined.Reverse();
        RelaxInteriorWaypoints(refined, cancel);
        FlightPathWaypointDebug?[] debugInfos = new FlightPathWaypointDebug?[refined.Count];
        PushInteriorWaypoints(refined, cancel, debugInfos);
        var finalPath = SimplifyPath(refined, cancel);
        finalPath = RestoreSteepDescentWaypoints(refined, finalPath);
        lastPathDebug = BuildFlightPathDebugPayload(refined, debugInfos, finalPath);
        return finalPath;
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

    private void PushInteriorWaypoints(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel, FlightPathWaypointDebug?[] debugInfos)
    {
        if (path.Count <= 2)
            return;

        for (var i = 1; i < path.Count - 1; ++i)
        {
            if ((i & 0x1f) == 0)
                cancel.ThrowIfCancellationRequested();

            var previous = path[i - 1];
            var current  = path[i];
            var next     = path[i + 1];

            var pushed = TryPushInteriorWaypoint(previous, current, next, i, out var adjusted, out var debugInfo);
            debugInfos[i] = debugInfo;
            if (!pushed)
                continue;

            path[i] = adjusted;
        }
    }

    private bool TryPushInteriorWaypoint
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        int                      pathIndex,
        out (ulong voxel, Vector3 p) adjusted,
        out FlightPathWaypointDebug? debugInfo
    )
    {
        adjusted = current;
        debugInfo = null;

        var overallDirection = next.p - previous.p;
        if (overallDirection.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
            return false;

        var horizontalForward = new Vector2(overallDirection.X, overallDirection.Z);
        if (horizontalForward.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
        {
            horizontalForward = new Vector2(next.p.X - current.p.X, next.p.Z - current.p.Z);
            if (horizontalForward.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
                horizontalForward = new Vector2(current.p.X - previous.p.X, current.p.Z - previous.p.Z);
        }

        if (!TryNormalize(horizontalForward, out var normalizedHorizontalForward))
            horizontalForward = Vector2.UnitX;
        else
            horizontalForward = normalizedHorizontalForward;

        var horizontalRight = new Vector2(-horizontalForward.Y, horizontalForward.X);
        var forward3        = new Vector3(horizontalForward.X, 0, horizontalForward.Y);
        var right3          = new Vector3(horizontalRight.X,   0, horizontalRight.Y);
        var forwardRight3   = Vector3.Normalize(forward3 + right3);
        var forwardLeft3    = Vector3.Normalize(forward3 - right3);
        var backwardRight3  = Vector3.Normalize(-forward3 + right3);
        var backwardLeft3   = Vector3.Normalize(-forward3 - right3);

        var voxelSize          = GetVoxelSize(current.voxel);
        var leafHorizontalSize = MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z);
        var leafVerticalSize   = l2Desc.CellSize.Y;
        var voxelHorizontal    = MathF.Max(voxelSize.X, voxelSize.Z);
        var voxelVertical      = voxelSize.Y;

        var horizontalSampleStep = MathF.Max
        (
            FLIGHT_PUSH_MIN_DISTANCE,
            MathF.Max(leafHorizontalSize * FLIGHT_PUSH_SAMPLE_STEP_SCALE, MathF.Min(voxelHorizontal * 0.5f, leafHorizontalSize * FLIGHT_PUSH_SAMPLE_STEP_MAX_SCALE))
        );
        var verticalSampleStep = MathF.Max
        (
            FLIGHT_PUSH_MIN_DISTANCE,
            MathF.Max(leafVerticalSize * FLIGHT_PUSH_SAMPLE_STEP_SCALE, MathF.Min(voxelVertical * 0.5f, leafVerticalSize * FLIGHT_PUSH_SAMPLE_STEP_MAX_SCALE))
        );

        var horizontalScanDistance = MathF.Max(voxelHorizontal, leafHorizontalSize * FLIGHT_PUSH_SCAN_DISTANCE_SCALE);
        horizontalScanDistance = MathF.Min(horizontalScanDistance, leafHorizontalSize * FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS);
        var verticalScanDistance = MathF.Max(voxelVertical, leafVerticalSize * FLIGHT_PUSH_SCAN_DISTANCE_SCALE);
        verticalScanDistance = MathF.Min(verticalScanDistance, leafVerticalSize * FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS);

        var forwardSample       = MeasureDirectionalClearance(current, forward3,       horizontalScanDistance, horizontalSampleStep);
        var backwardSample      = MeasureDirectionalClearance(current, -forward3,      horizontalScanDistance, horizontalSampleStep);
        var rightSample         = MeasureDirectionalClearance(current, right3,         horizontalScanDistance, horizontalSampleStep);
        var leftSample          = MeasureDirectionalClearance(current, -right3,        horizontalScanDistance, horizontalSampleStep);
        var forwardRightSample  = MeasureDirectionalClearance(current, forwardRight3,  horizontalScanDistance, horizontalSampleStep);
        var forwardLeftSample   = MeasureDirectionalClearance(current, forwardLeft3,   horizontalScanDistance, horizontalSampleStep);
        var backwardRightSample = MeasureDirectionalClearance(current, backwardRight3, horizontalScanDistance, horizontalSampleStep);
        var backwardLeftSample  = MeasureDirectionalClearance(current, backwardLeft3,  horizontalScanDistance, horizontalSampleStep);
        var upSample            = MeasureDirectionalClearance(current, Vector3.UnitY,  verticalScanDistance,   verticalSampleStep);
        var downSample          = MeasureDirectionalClearance(current, -Vector3.UnitY, verticalScanDistance,   verticalSampleStep);

        var forwardClearance       = forwardSample.Clearance;
        var backwardClearance      = backwardSample.Clearance;
        var rightClearance         = rightSample.Clearance;
        var leftClearance          = leftSample.Clearance;
        var forwardRightClearance  = forwardRightSample.Clearance;
        var forwardLeftClearance   = forwardLeftSample.Clearance;
        var backwardRightClearance = backwardRightSample.Clearance;
        var backwardLeftClearance  = backwardLeftSample.Clearance;
        var upClearance            = upSample.Clearance;
        var downClearance          = downSample.Clearance;

        var horizontalBias = Vector3.Zero;
        var horizontalTotalClearance = 0f;
        var maxHorizontalClearance = 0f;
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forward3,        forwardClearance,       FLIGHT_PUSH_FORWARD_WEIGHT);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -forward3,       backwardClearance,      FLIGHT_PUSH_BACKWARD_WEIGHT);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, right3,          rightClearance,         1f);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, rightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -right3,         leftClearance,          1f);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, leftClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardRight3,   forwardRightClearance,  FLIGHT_PUSH_DIAGONAL_WEIGHT);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardRightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardLeft3,    forwardLeftClearance,   FLIGHT_PUSH_DIAGONAL_WEIGHT);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardLeftClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardRight3,  backwardRightClearance, FLIGHT_PUSH_DIAGONAL_WEIGHT);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardRightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardLeft3,   backwardLeftClearance,  FLIGHT_PUSH_DIAGONAL_WEIGHT);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardLeftClearance);
        var sweepSamples = BuildHorizontalSweepSamples
        (
            current,
            forward3,
            right3,
            horizontalScanDistance,
            horizontalSampleStep,
            ref horizontalBias,
            ref horizontalTotalClearance,
            ref maxHorizontalClearance
        );
        var directionalHorizontalCandidates = BuildDirectionalHorizontalCandidates
        (
            current.p,
            forward3,
            forwardSample,
            backwardSample,
            rightSample,
            leftSample,
            forwardRightSample,
            forwardLeftSample,
            backwardRightSample,
            backwardLeftSample,
            sweepSamples
        );

        Vector3 horizontalOffset = default;
        var maxHorizontalPush = MathF.Min(horizontalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxHorizontalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
        var horizontalBiasFlat = new Vector2(horizontalBias.X, horizontalBias.Z);
        var horizontalBiasMagnitude = horizontalBiasFlat.Length();
        var horizontalImbalance = horizontalTotalClearance > SCORE_EPSILON ? horizontalBiasMagnitude / horizontalTotalClearance : 0f;
        if (horizontalBiasMagnitude >= FLIGHT_PUSH_MIN_DISTANCE &&
            horizontalImbalance >= FLIGHT_PUSH_MIN_HORIZONTAL_IMBALANCE &&
            TryNormalize(horizontalBiasFlat, out var horizontalPushDirection))
        {
            var desiredHorizontalPush = horizontalBiasMagnitude * FLIGHT_PUSH_HORIZONTAL_BIAS_SCALE;
            var horizontalPushDistance = MathF.Min(maxHorizontalPush, desiredHorizontalPush);
            horizontalOffset = new Vector3(horizontalPushDirection.X * horizontalPushDistance, 0, horizontalPushDirection.Y * horizontalPushDistance);
        }

        Vector3 verticalOffset = default;
        var verticalBias      = upClearance - downClearance;
        var verticalMagnitude = MathF.Abs(verticalBias);
        var verticalTotal     = upClearance + downClearance;
        var verticalImbalance = verticalTotal > SCORE_EPSILON ? verticalMagnitude / verticalTotal : 0f;
        var goalDescentApproach = IsGoalDescentApproach(previous, current, next, leafVerticalSize);
        var downhillTunnelTrend = IsTunnelDescentTrend(previous, current, next, leafVerticalSize);
        var heightMatchTarget = current.p.Y;
        if (previous.p.Y >= current.p.Y - FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE)
            heightMatchTarget = MathF.Max(heightMatchTarget, previous.p.Y + FLIGHT_PUSH_HEIGHT_MATCH_BIAS);
        var constrainedTunnelDescent = IsConstrainedTunnelDescent(previous, current, next, leafVerticalSize, upClearance, heightMatchTarget);
        var preferredMinHeight       = !goalDescentApproach && !constrainedTunnelDescent ? heightMatchTarget : current.p.Y;
        var catchupHeight          = MathF.Max(0f, preferredMinHeight - current.p.Y);
        var catchupHeadroomRequired = catchupHeight + ResolveFlightHeightCatchupHeadroom(leafVerticalSize);
        var shouldCatchUpHeight    = !constrainedTunnelDescent &&
                                     catchupHeight >= FLIGHT_PUSH_MIN_DISTANCE &&
                                     upClearance >= catchupHeadroomRequired;
        if (downhillTunnelTrend && next.p.Y + FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE < current.p.Y)
        {
            preferredMinHeight = current.p.Y;
            catchupHeight = 0f;
            catchupHeadroomRequired = ResolveFlightHeightCatchupHeadroom(leafVerticalSize);
            shouldCatchUpHeight = false;
        }
        var preferredFloorClearance = MathF.Max(voxelVertical * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_VOXEL_SCALE, leafVerticalSize * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_LEAF_SCALE);
        var downwardHeadroomLimit   = MathF.Max(voxelVertical * FLIGHT_PUSH_DOWNWARD_UPWARD_BLOCKED_VOXEL_SCALE, leafVerticalSize * FLIGHT_PUSH_DOWNWARD_UPWARD_BLOCKED_LEAF_SCALE);
        var downwardClearanceFloor  = MathF.Max(voxelVertical * FLIGHT_PUSH_DOWNWARD_MIN_CLEARANCE_VOXEL_SCALE, leafVerticalSize * FLIGHT_PUSH_DOWNWARD_MIN_CLEARANCE_LEAF_SCALE);
        var downwardLeadRequired    = MathF.Max(leafVerticalSize * FLIGHT_PUSH_DOWNWARD_MIN_LEAD_LEAF_SCALE, FLIGHT_PUSH_MIN_DISTANCE * 2f);
        var allowDownwardPush       = verticalBias < 0 &&
                                      upClearance < downwardHeadroomLimit &&
                                      downClearance >= downwardClearanceFloor &&
                                      downClearance - upClearance >= downwardLeadRequired;
        var verticalMode = FlightPathVerticalMode.None;
        var tunnelDescentAssist = false;
        if (TryResolveConstrainedTunnelDescentOffset(constrainedTunnelDescent, downhillTunnelTrend, previous, current, next, leafVerticalSize, voxelVertical, verticalScanDistance, upClearance, downClearance, out var tunnelDescentOffset))
        {
            verticalOffset = tunnelDescentOffset;
            tunnelDescentAssist = true;
            verticalMode = FlightPathVerticalMode.TunnelDescentAssist;
        }
        else if ((verticalBias > 0 &&
             verticalMagnitude >= FLIGHT_PUSH_MIN_DISTANCE &&
             verticalImbalance >= FLIGHT_PUSH_MIN_VERTICAL_IMBALANCE) ||
            shouldCatchUpHeight)
        {
            var maxVerticalClearance = MathF.Max(upClearance, downClearance);
            var desiredVerticalPush  = verticalBias > 0
                                           ? verticalMagnitude * FLIGHT_PUSH_VERTICAL_BIAS_SCALE
                                           : 0f;
            if (shouldCatchUpHeight)
                desiredVerticalPush = MathF.Max(desiredVerticalPush, catchupHeight * FLIGHT_PUSH_HEIGHT_CATCHUP_SCALE);
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxVerticalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush, desiredVerticalPush);
            verticalOffset = Vector3.UnitY * verticalPushDistance;
            verticalMode = shouldCatchUpHeight && verticalBias <= 0
                               ? FlightPathVerticalMode.HeightCatchUp
                               : FlightPathVerticalMode.UpwardBias;
        }
        else if (allowDownwardPush &&
                 verticalMagnitude >= FLIGHT_PUSH_MIN_DISTANCE &&
                 verticalImbalance >= FLIGHT_PUSH_MIN_VERTICAL_IMBALANCE)
        {
            var maxVerticalClearance = downClearance;
            var desiredVerticalPush  = verticalMagnitude * FLIGHT_PUSH_VERTICAL_BIAS_SCALE * FLIGHT_PUSH_DOWNWARD_SCALE;
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxVerticalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush, desiredVerticalPush);
            verticalOffset = -Vector3.UnitY * verticalPushDistance;
            verticalMode = FlightPathVerticalMode.DownwardBias;
        }
        else if (downClearance < preferredFloorClearance && upClearance > FLIGHT_PUSH_MIN_DISTANCE)
        {
            var floorPressure        = preferredFloorClearance - downClearance;
            var desiredVerticalPush  = floorPressure * FLIGHT_PUSH_FLOOR_AVOIDANCE_SCALE;
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, upClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush, desiredVerticalPush);
            if (verticalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
            {
                verticalOffset = Vector3.UnitY * verticalPushDistance;
                verticalMode = FlightPathVerticalMode.FloorAvoidance;
            }
        }

        var combinedOffset = horizontalOffset + verticalOffset;
        var adjustedResolved = current;
        var baseAdjustedResolved = current;
        var upwardPreferred = verticalOffset.Y > FLIGHT_PUSH_MIN_DISTANCE;
        var downwardPreferred = verticalOffset.Y < -FLIGHT_PUSH_MIN_DISTANCE;
        var selectedAdjustmentKind = FlightPathAdjustmentKind.None;
        if (upwardPreferred)
        {
            if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardVerticalOnly;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + horizontalOffset * FLIGHT_PUSH_VERTICAL_FIRST_HORIZONTAL_BLEND, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardVerticalBlend;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardHorizontalOnly;
            }
            else adjusted = current;
        }
        else if (tunnelDescentAssist)
        {
            var tunnelCombinedStrong = verticalOffset + horizontalOffset * FLIGHT_TUNNEL_DESCENT_HORIZONTAL_BLEND_STRONG;
            var tunnelCombinedMedium = verticalOffset + horizontalOffset * FLIGHT_TUNNEL_DESCENT_HORIZONTAL_BLEND_MEDIUM;

            if (TryAcceptAdjustedWaypoint(previous, current, next, tunnelCombinedStrong, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelCombinedStrong;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelVerticalOnly;
            }
            else if (TryAcceptDirectionalHorizontalCandidatesWithFixedVertical(previous, current, next, directionalHorizontalCandidates, maxHorizontalPush, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelDirectionalFixedVertical;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, tunnelCombinedMedium, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelCombinedMedium;
            }
            else if (TryAcceptAdjustedWaypointAfterVerticalDrop(previous, current, next, horizontalOffset, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelAfterVerticalDrop;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelHorizontalOnly;
            }
            else adjusted = current;
        }
        else if (downwardPreferred)
        {
            var downwardStrongOffset = verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG;
            var downwardMediumOffset = verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM;

            if (TryAcceptAdjustedWaypointWithFixedVertical(previous, current, next, horizontalOffset, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardFixedVertical;
            }
            else if (TryAcceptDirectionalHorizontalCandidatesWithFixedVertical(previous, current, next, directionalHorizontalCandidates, maxHorizontalPush, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardDirectionalFixedVertical;
            }
            else if (TryAcceptAdjustedWaypointAfterVerticalDrop(previous, current, next, horizontalOffset, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardAfterVerticalDrop;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, downwardStrongOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardStrongBlend;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, downwardMediumOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardMediumBlend;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardHorizontalOnly;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardVerticalOnly;
            }
            else adjusted = current;
        }
        else
        {
            if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.NeutralCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.NeutralHorizontalOnly;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.NeutralVerticalOnly;
            }
            else adjusted = current;
        }

        baseAdjustedResolved = adjustedResolved;
        var finalRaiseApplied = false;
        if (!goalDescentApproach &&
            !constrainedTunnelDescent &&
            !(downhillTunnelTrend && next.p.Y + FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE < adjustedResolved.p.Y) &&
            preferredMinHeight > adjustedResolved.p.Y + FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE &&
            TryRaiseWaypointToPreferredHeight(previous, adjustedResolved, next, preferredMinHeight, out var lifted))
        {
            adjusted = lifted;
            adjustedResolved = lifted;
            finalRaiseApplied = true;
        }

        var maxClearance = MathF.Max
        (
            MathF.Max(horizontalScanDistance, verticalScanDistance),
            MathF.Max
            (
                MathF.Max(forwardClearance, backwardClearance),
                MathF.Max
                (
                    MathF.Max(leftClearance, rightClearance),
                    MathF.Max
                    (
                        MathF.Max(forwardLeftClearance, forwardRightClearance),
                        MathF.Max(MathF.Max(backwardLeftClearance, backwardRightClearance), MathF.Max(upClearance, downClearance))
                    )
                )
            )
        );

        List<FlightPathDebugSample> samples =
        [
            BuildFlightDebugSample(FlightPathDebugSampleKind.Forward,       current.p, forwardSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Backward,      current.p, backwardSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Right,         current.p, rightSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Left,          current.p, leftSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.ForwardRight,  current.p, forwardRightSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.ForwardLeft,   current.p, forwardLeftSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.BackwardRight, current.p, backwardRightSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.BackwardLeft,  current.p, backwardLeftSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Up,            current.p, upSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Down,          current.p, downSample)
        ];
        samples.AddRange(sweepSamples);

        debugInfo = new FlightPathWaypointDebug
        (
            pathIndex,
            current.voxel,
            adjustedResolved.voxel,
            current.p,
            adjustedResolved.p,
            current.p + horizontalOffset,
            current.p + verticalOffset,
            current.p + combinedOffset,
            horizontalOffset.Length(),
            MathF.Abs(verticalOffset.Y),
            Vector3.Distance(current.p, adjustedResolved.p),
            horizontalImbalance,
            verticalImbalance,
            forwardClearance,
            backwardClearance,
            leftClearance,
            rightClearance,
            forwardLeftClearance,
            forwardRightClearance,
            backwardLeftClearance,
            backwardRightClearance,
            upClearance,
            downClearance,
            maxClearance,
            verticalMode,
            selectedAdjustmentKind,
            goalDescentApproach,
            downhillTunnelTrend,
            constrainedTunnelDescent,
            tunnelDescentAssist,
            shouldCatchUpHeight,
            allowDownwardPush,
            finalRaiseApplied,
            heightMatchTarget,
            preferredMinHeight,
            baseAdjustedResolved.p,
            samples
        );

        return debugInfo.Value.PushApplied;
    }

    private FlightPushProbeResult MeasureDirectionalClearance
    (
        (ulong voxel, Vector3 p) origin,
        Vector3                  direction,
        float                    maxDistance,
        float                    stepDistance
    )
    {
        if (direction.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON ||
            maxDistance <= FLIGHT_PUSH_MIN_DISTANCE ||
            stepDistance <= FLIGHT_PUSH_MIN_DISTANCE)
            return new(0, origin.p);

        direction = Vector3.Normalize(direction);
        var clearance = 0f;
        var endpoint  = origin.p;

        for (var distance = stepDistance; distance <= maxDistance + SCORE_EPSILON; distance += stepDistance)
        {
            var probePoint = origin.p + direction * distance;
            if (!TryResolveEmptyWaypoint(probePoint, direction, out var probe))
                break;
            if (!HasLineOfSight(origin, probe.voxel, probe.p))
                break;

            clearance = distance;
            endpoint  = probe.p;
        }

        return new(clearance, endpoint);
    }

    private bool TryAcceptAdjustedWaypoint
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        Vector3                  offset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (offset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        const float scale100 = 1.00f;
        const float scale085 = 0.85f;
        const float scale070 = 0.70f;
        const float scale055 = 0.55f;
        const float scale040 = 0.40f;
        const float scale025 = 0.25f;

        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, scale100, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, scale085, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, scale070, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, scale055, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, scale040, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, scale025, out adjusted))
            return true;

        adjusted = current;
        return false;
    }

    private bool TryAcceptAdjustedWaypointAfterVerticalDrop
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        Vector3                  horizontalOffset,
        Vector3                  verticalOffset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (verticalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE ||
            horizontalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        if (!TryResolveEmptyWaypoint(current.p + verticalOffset, verticalOffset, out var lowered))
            return false;
        if (Vector3.DistanceSquared(lowered.p, current.p) <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_HIGH, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_LIGHT, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MIN, out adjusted))
            return true;

        return false;
    }

    private bool TryAcceptAdjustedWaypointWithFixedVertical
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        Vector3                  horizontalOffset,
        Vector3                  verticalOffset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (verticalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE ||
            horizontalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG, 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_HIGH, 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM, 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_LIGHT, 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MIN, 1.00f, out adjusted))
            return true;

        return false;
    }

    private bool TryAcceptDirectionalHorizontalCandidatesWithFixedVertical
    (
        (ulong voxel, Vector3 p)                 previous,
        (ulong voxel, Vector3 p)                 current,
        (ulong voxel, Vector3 p)                 next,
        IReadOnlyList<FlightPushHorizontalCandidate> candidates,
        float                                    maxHorizontalPush,
        Vector3                                  verticalOffset,
        out (ulong voxel, Vector3 p)             adjusted
    )
    {
        adjusted = current;
        if (verticalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE ||
            maxHorizontalPush <= FLIGHT_PUSH_MIN_DISTANCE ||
            candidates.Count == 0)
            return false;

        foreach (var candidate in candidates)
        {
            var horizontalDistance = MathF.Min(maxHorizontalPush, candidate.Clearance * FLIGHT_PUSH_DIRECTIONAL_CLEARANCE_FRACTION);
            if (horizontalDistance <= FLIGHT_PUSH_MIN_DISTANCE)
                continue;

            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + candidate.Direction * horizontalDistance, 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + candidate.Direction * horizontalDistance * 0.85f, 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + candidate.Direction * horizontalDistance * 0.70f, 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + candidate.Direction * horizontalDistance * 0.55f, 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + candidate.Direction * horizontalDistance * 0.40f, 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + candidate.Direction * horizontalDistance * 0.25f, 1.00f, out adjusted))
                return true;
        }

        return false;
    }

    private bool TryAcceptAdjustedWaypointFromBase
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        Vector3                  offset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (offset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        const float scale100 = 1.00f;
        const float scale085 = 0.85f;
        const float scale070 = 0.70f;
        const float scale055 = 0.55f;
        const float scale040 = 0.40f;
        const float scale025 = 0.25f;

        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, scale100, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, scale085, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, scale070, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, scale055, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, scale040, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, scale025, out adjusted))
            return true;

        adjusted = current;
        return false;
    }

    private bool TryAcceptAdjustedWaypointFromBase
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        Vector3                  offset,
        float                    scale,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (scale <= 0)
            return false;

        var candidatePoint = current.p + offset * scale;
        if (!TryResolveEmptyWaypoint(candidatePoint, offset, out var candidate))
            return false;
        if (Vector3.DistanceSquared(candidate.p, current.p) <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;
        if (!HasLineOfSight(previous, candidate.voxel, candidate.p))
            return false;
        if (!HasLineOfSight(candidate, next.voxel, next.p))
            return false;

        adjusted = candidate;
        return true;
    }

    private bool TryAcceptAdjustedWaypoint
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        Vector3                  offset,
        float                    scale,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (scale <= 0)
            return false;

        var candidatePoint = current.p + offset * scale;
        if (!TryResolveEmptyWaypoint(candidatePoint, offset, out var candidate))
            return false;
        if (Vector3.DistanceSquared(candidate.p, current.p) <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;
        if (!HasLineOfSight(previous, candidate.voxel, candidate.p))
            return false;
        if (!HasLineOfSight(candidate, next.voxel, next.p))
            return false;

        adjusted = candidate;
        return true;
    }

    private bool TryResolveEmptyWaypoint(Vector3 point, Vector3 directionHint, out (ulong voxel, Vector3 p) waypoint)
    {
        var resolved = Volume.FindLeafVoxel(point);
        if (!resolved.empty || resolved.voxel == VoxelMap.INVALID_VOXEL)
        {
            if (!TryNormalize(directionHint, out var normalizedHint))
            {
                waypoint = default;
                return false;
            }

            var nudgeDistance = MathF.Max(ResolveLeafInset(), FLIGHT_PUSH_MIN_DISTANCE);
            point -= normalizedHint * nudgeDistance;
            resolved = Volume.FindLeafVoxel(point);
            if (!resolved.empty || resolved.voxel == VoxelMap.INVALID_VOXEL)
            {
                waypoint = default;
                return false;
            }
        }

        point = Volume.ClampPointToVoxel(resolved.voxel, point, ResolveVoxelInset(resolved.voxel));
        waypoint = (resolved.voxel, point);
        return true;
    }

    private bool TryRaiseWaypointToPreferredHeight
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    preferredMinHeight,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        var requiredLift = preferredMinHeight - current.p.Y;
        if (requiredLift <= FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE)
            return false;

        var horizontal = new Vector3
        (
            current.p.X - previous.p.X,
            0,
            current.p.Z - previous.p.Z
        );
        var directionHint = horizontal.LengthSquared() > SCORE_EPSILON * SCORE_EPSILON
                                ? Vector3.Normalize(horizontal) * FLIGHT_PUSH_HEIGHT_RAISE_HORIZONTAL_BLEND + Vector3.UnitY
                                : Vector3.UnitY;
        if (!TryNormalize(directionHint, out var normalizedDirectionHint))
            directionHint = Vector3.UnitY;
        else
            directionHint = normalizedDirectionHint;

        var attemptLift = requiredLift + FLIGHT_PUSH_HEIGHT_STRICT_BIAS;
        Span<float> scales = stackalloc float[4] { 1.0f, 0.85f, 0.70f, 0.55f };
        for (var i = 0; i < 4; ++i)
        {
            var candidatePoint = current.p + Vector3.UnitY * (attemptLift * scales[i]);
            if (!TryResolveEmptyWaypoint(candidatePoint, directionHint, out var candidate))
                continue;
            if (candidate.p.Y + FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE < preferredMinHeight)
                continue;
            if (!HasLineOfSight(previous, candidate.voxel, candidate.p))
                continue;
            if (!HasLineOfSight(candidate, next.voxel, next.p))
                continue;

            adjusted = candidate;
            return true;
        }

        return false;
    }

    private Vector3 GetVoxelSize(ulong voxel)
    {
        var (min, max) = Volume.VoxelBounds(voxel, 0);
        return max - min;
    }

    private float ResolveVoxelInset(ulong voxel)
    {
        var voxelSize  = GetVoxelSize(voxel);
        var minExtent  = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
        var scaledInset = minExtent * FLIGHT_PUSH_VOXEL_INSET_RATIO;
        return Math.Clamp(scaledInset, FLIGHT_PUSH_VOXEL_INSET_MIN, FLIGHT_PUSH_VOXEL_INSET_MAX);
    }

    private float ResolveLeafInset()
    {
        var minLeafExtent = MathF.Min(l2Desc.CellSize.X, MathF.Min(l2Desc.CellSize.Y, l2Desc.CellSize.Z));
        var scaledInset   = minLeafExtent * FLIGHT_PUSH_VOXEL_INSET_RATIO;
        return Math.Clamp(scaledInset, FLIGHT_PUSH_VOXEL_INSET_MIN, FLIGHT_PUSH_VOXEL_INSET_MAX);
    }

    private static void AccumulateDirectionalBias
    (
        ref Vector3 bias,
        ref float   totalClearance,
        Vector3     direction,
        float       clearance,
        float       weight
    )
    {
        if (clearance <= SCORE_EPSILON || weight <= SCORE_EPSILON)
            return;

        var contribution = clearance * weight;
        bias           += direction * contribution;
        totalClearance += contribution;
    }

    private static bool TryNormalize(Vector2 value, out Vector2 normalized)
    {
        if (value.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
        {
            normalized = default;
            return false;
        }

        normalized = Vector2.Normalize(value);
        return true;
    }

    private static bool TryNormalize(Vector3 value, out Vector3 normalized)
    {
        if (value.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
        {
            normalized = default;
            return false;
        }

        normalized = Vector3.Normalize(value);
        return true;
    }

    private static FlightPathDebugSample BuildFlightDebugSample(FlightPathDebugSampleKind kind, Vector3 start, FlightPushProbeResult sample)
        => new(kind, start, sample.Endpoint, sample.Clearance);

    private List<FlightPushHorizontalCandidate> BuildDirectionalHorizontalCandidates
    (
        Vector3                            origin,
        Vector3                            forward,
        FlightPushProbeResult              forwardSample,
        FlightPushProbeResult              backwardSample,
        FlightPushProbeResult              rightSample,
        FlightPushProbeResult              leftSample,
        FlightPushProbeResult              forwardRightSample,
        FlightPushProbeResult              forwardLeftSample,
        FlightPushProbeResult              backwardRightSample,
        FlightPushProbeResult              backwardLeftSample,
        IReadOnlyList<FlightPathDebugSample> sweepSamples
    )
    {
        List<FlightPushHorizontalCandidate> candidates = new(FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT);

        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, rightSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, leftSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardRightSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardLeftSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardRightSample);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardLeftSample);

        foreach (var sample in sweepSamples)
            TryAddDirectionalHorizontalCandidate(candidates, origin, forward, new(sample.Clearance, sample.Endpoint));

        candidates.Sort(static (left, right) => right.Score.CompareTo(left.Score));
        if (candidates.Count > FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT)
            candidates.RemoveRange(FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT, candidates.Count - FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT);

        return candidates;
    }

    private void TryAddDirectionalHorizontalCandidate
    (
        List<FlightPushHorizontalCandidate> candidates,
        Vector3                             origin,
        Vector3                             forward,
        FlightPushProbeResult               sample
    )
    {
        if (sample.Clearance <= FLIGHT_PUSH_MIN_DISTANCE)
            return;

        var delta = sample.Endpoint - origin;
        delta.Y = 0;
        if (!TryNormalize(delta, out var direction))
            return;

        var forwardAlignment = MathF.Max(0f, Vector3.Dot(direction, forward));
        var score = sample.Clearance * (1f + forwardAlignment * FLIGHT_PUSH_DIRECTIONAL_FORWARD_BONUS);

        for (var i = 0; i < candidates.Count; ++i)
        {
            if (Vector3.Dot(candidates[i].Direction, direction) < FLIGHT_PUSH_DIRECTIONAL_DUPLICATE_DOT)
                continue;

            if (score > candidates[i].Score)
                candidates[i] = new(direction, sample.Clearance, score);
            return;
        }

        candidates.Add(new(direction, sample.Clearance, score));
    }

    private List<FlightPathDebugSample> BuildHorizontalSweepSamples
    (
        (ulong voxel, Vector3 p) origin,
        Vector3                  forward,
        Vector3                  right,
        float                    maxDistance,
        float                    stepDistance,
        ref Vector3              horizontalBias,
        ref float                horizontalTotalClearance,
        ref float                maxHorizontalClearance
    )
    {
        List<FlightPathDebugSample> samples = new(FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT);
        var stepAngle      = 2f * MathF.PI / FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT;
        var primaryDivisor = FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT / 8;

        for (var i = 0; i < FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT; ++i)
        {
            if (primaryDivisor > 0 && i % primaryDivisor == 0)
                continue;

            var angle = i * stepAngle;
            var direction = forward * MathF.Cos(angle) + right * MathF.Sin(angle);
            if (!TryNormalize(direction, out direction))
                continue;

            var sample = MeasureDirectionalClearance(origin, direction, maxDistance, stepDistance);
            maxHorizontalClearance = MathF.Max(maxHorizontalClearance, sample.Clearance);

            var forwardDot = Vector3.Dot(direction, forward);
            var weight = FLIGHT_PUSH_HORIZONTAL_SWEEP_WEIGHT;
            if (forwardDot > 0f)
                weight += forwardDot * FLIGHT_PUSH_HORIZONTAL_SWEEP_FORWARD_BONUS;

            AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, direction, sample.Clearance, weight);
            samples.Add(BuildFlightDebugSample(FlightPathDebugSampleKind.Sweep, origin.p, sample));
        }

        return samples;
    }

    private static FlightPathDebugPayload? BuildFlightPathDebugPayload
    (
        IReadOnlyList<(ulong voxel, Vector3 p)> refinedPath,
        IReadOnlyList<FlightPathWaypointDebug?> debugInfos,
        IReadOnlyList<(ulong voxel, Vector3 p)> finalPath
    )
    {
        if (refinedPath.Count == 0 || debugInfos.Count == 0 || finalPath.Count == 0)
            return null;

        List<FlightPathWaypointDebug> remapped = [];
        var searchStart = 0;

        for (var finalIndex = 0; finalIndex < finalPath.Count; ++finalIndex)
        {
            var finalPoint = finalPath[finalIndex];
            for (var refinedIndex = searchStart; refinedIndex < refinedPath.Count; ++refinedIndex)
            {
                var refinedPoint = refinedPath[refinedIndex];
                if (Vector3.DistanceSquared(refinedPoint.p, finalPoint.p) > SCORE_EPSILON * SCORE_EPSILON)
                    continue;

                if (refinedIndex < debugInfos.Count && debugInfos[refinedIndex] is { } debug)
                    remapped.Add(debug with { PathIndex = finalIndex });

                searchStart = refinedIndex + 1;
                break;
            }
        }

        if (remapped.Count == 0)
            return null;

        return new()
        {
            Waypoints = remapped
        };
    }

    private List<(ulong voxel, Vector3 p)> RestoreSteepDescentWaypoints
    (
        IReadOnlyList<(ulong voxel, Vector3 p)> refined,
        IReadOnlyList<(ulong voxel, Vector3 p)> simplified
    )
    {
        if (refined.Count == 0 || simplified.Count <= 1)
            return [.. simplified];

        List<(ulong voxel, Vector3 p)> restored = [simplified[0]];
        var refinedSearchStart = 0;

        for (var simplifiedIndex = 1; simplifiedIndex < simplified.Count; ++simplifiedIndex)
        {
            var restoredStart = restored[^1];
            var segmentEnd    = simplified[simplifiedIndex];
            var startIndex    = FindPathPointIndex(refined, restoredStart, refinedSearchStart);
            if (startIndex < 0)
            {
                AppendPathPoint(restored, segmentEnd);
                continue;
            }

            var endIndex = FindPathPointIndex(refined, segmentEnd, startIndex + 1);
            if (endIndex < 0)
            {
                AppendPathPoint(restored, segmentEnd);
                continue;
            }

            AppendSteepDescentAwareSegment(restored, refined, startIndex, endIndex);
            refinedSearchStart = endIndex;
        }

        return restored;
    }

    private void AppendSteepDescentAwareSegment
    (
        List<(ulong voxel, Vector3 p)>          output,
        IReadOnlyList<(ulong voxel, Vector3 p)> refined,
        int                                     startIndex,
        int                                     endIndex
    )
    {
        var currentIndex = startIndex;
        while (currentIndex < endIndex)
        {
            var nextIndex = endIndex;
            if (NeedsFlightDescentSmoothing(refined[currentIndex].p, refined[endIndex].p))
            {
                nextIndex = currentIndex + 1;
                for (var probeIndex = endIndex - 1; probeIndex > currentIndex; --probeIndex)
                {
                    if (NeedsFlightDescentSmoothing(refined[currentIndex].p, refined[probeIndex].p))
                        continue;

                    nextIndex = probeIndex;
                    break;
                }
            }

            AppendPathPoint(output, refined[nextIndex]);
            currentIndex = nextIndex;
        }
    }

    private static void AppendPathPoint(List<(ulong voxel, Vector3 p)> output, (ulong voxel, Vector3 p) point)
    {
        if (output.Count > 0 && Vector3.DistanceSquared(output[^1].p, point.p) <= SCORE_EPSILON * SCORE_EPSILON)
            return;

        output.Add(point);
    }

    private static int FindPathPointIndex(IReadOnlyList<(ulong voxel, Vector3 p)> path, (ulong voxel, Vector3 p) point, int startIndex)
    {
        for (var i = Math.Max(0, startIndex); i < path.Count; ++i)
        {
            if (Vector3.DistanceSquared(path[i].p, point.p) <= SCORE_EPSILON * SCORE_EPSILON)
                return i;
        }

        return -1;
    }

    private bool IsGoalDescentApproach
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize
    )
    {
        if (next.voxel != goalVoxel)
            return false;

        var tolerance = MathF.Max(leafVerticalSize * FLIGHT_GOAL_DESCENT_HEIGHT_TOLERANCE_LEAF_SCALE, FLIGHT_GOAL_DESCENT_HEIGHT_TOLERANCE_MIN);
        return next.p.Y + tolerance < current.p.Y &&
               next.p.Y + tolerance < previous.p.Y;
    }

    private bool IsConstrainedTunnelDescent
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize,
        float                    upClearance,
        float                    preferredHeightTarget
    )
    {
        var descentTolerance = MathF.Max(leafVerticalSize * FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_LEAF_SCALE, FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_MIN);
        if (next.p.Y + descentTolerance >= current.p.Y &&
            next.p.Y + descentTolerance >= previous.p.Y)
            return false;

        var targetLift = MathF.Max(0f, preferredHeightTarget - current.p.Y);
        if (targetLift < FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        var requiredHeadroom = targetLift + ResolveFlightHeightCatchupHeadroom(leafVerticalSize);
        return upClearance < requiredHeadroom;
    }

    private static float ResolveFlightHeightCatchupHeadroom(float leafVerticalSize)
        => MathF.Max(leafVerticalSize * FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_LEAF_SCALE, FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_MIN);

    private bool TryResolveConstrainedTunnelDescentOffset
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
        out Vector3              verticalOffset
    )
    {
        verticalOffset = default;
        if (!constrainedTunnelDescent && !downhillTunnelTrend)
            return false;

        var clearanceAdvantage = MathF.Max(0f, downClearance - upClearance);
        var nextDrop = MathF.Max(0f, current.p.Y - next.p.Y);
        var previousDrop = MathF.Max(0f, previous.p.Y - current.p.Y);
        var trendDrop = MathF.Max(nextDrop, previousDrop * FLIGHT_TUNNEL_DESCENT_PREVIOUS_FOLLOW_SCALE);
        var minimumLead = MathF.Max(leafVerticalSize * FLIGHT_TUNNEL_DESCENT_CLEARANCE_LEAD_LEAF_SCALE, FLIGHT_TUNNEL_DESCENT_CLEARANCE_LEAD_MIN);
        if (trendDrop < FLIGHT_PUSH_MIN_DISTANCE &&
            clearanceAdvantage < minimumLead)
            return false;

        var downwardClearanceFloor = MathF.Max(voxelVertical * FLIGHT_TUNNEL_DESCENT_DOWNWARD_CLEARANCE_VOXEL_SCALE, leafVerticalSize * FLIGHT_TUNNEL_DESCENT_DOWNWARD_CLEARANCE_LEAF_SCALE);
        if (downClearance < downwardClearanceFloor)
            return false;

        if (!constrainedTunnelDescent)
        {
            var softHeadroomLimit = MathF.Max(voxelVertical * FLIGHT_TUNNEL_DESCENT_SOFT_HEADROOM_VOXEL_SCALE, leafVerticalSize * FLIGHT_TUNNEL_DESCENT_SOFT_HEADROOM_LEAF_SCALE);
            if (upClearance > softHeadroomLimit || clearanceAdvantage < minimumLead)
                return false;
        }

        var desiredVerticalPush = MathF.Max
        (
            trendDrop * FLIGHT_TUNNEL_DESCENT_FOLLOW_NEXT_SCALE,
            clearanceAdvantage * FLIGHT_TUNNEL_DESCENT_CLEARANCE_ADVANTAGE_SCALE
        );
        var maxVerticalPush = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, downClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
        var pushCap = constrainedTunnelDescent
                          ? Math.Max(nextDrop, trendDrop)
                          : Math.Max(nextDrop, trendDrop + leafVerticalSize * FLIGHT_TUNNEL_DESCENT_EXTRA_FOLLOW_LEAF_SCALE);
        var verticalPushDistance = MathF.Min(pushCap, MathF.Min(maxVerticalPush, desiredVerticalPush));
        if (verticalPushDistance < FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        verticalOffset = -Vector3.UnitY * verticalPushDistance;
        return true;
    }

    private bool IsTunnelDescentTrend
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize
    )
    {
        var descentTolerance = MathF.Max(leafVerticalSize * FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_LEAF_SCALE, FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_MIN);
        return next.p.Y + descentTolerance < current.p.Y ||
               current.p.Y + descentTolerance < previous.p.Y ||
               next.p.Y + descentTolerance < previous.p.Y;
    }

    private bool NeedsFlightDescentSmoothing(Vector3 from, Vector3 to)
    {
        var verticalDrop = from.Y - to.Y;
        if (verticalDrop <= ResolveFlightDescentSmoothingMinDrop())
            return false;

        var horizontalDistance = HorizontalDistanceXZ(from, to);
        if (horizontalDistance <= ResolveFlightDescentNearVerticalDistance())
            return true;

        return verticalDrop / horizontalDistance >= FLIGHT_DESCENT_SMOOTHING_MAX_SLOPE;
    }

    private float ResolveFlightDescentSmoothingMinDrop()
        => MathF.Max(l2Desc.CellSize.Y * FLIGHT_DESCENT_SMOOTHING_MIN_DROP_LEAF_SCALE, FLIGHT_DESCENT_SMOOTHING_MIN_DROP_MIN);

    private float ResolveFlightDescentNearVerticalDistance()
        => MathF.Max(MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z) * FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_LEAF_SCALE, FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_MIN);

    private static float HorizontalDistanceXZ(Vector3 left, Vector3 right)
    {
        var dx = left.X - right.X;
        var dz = left.Z - right.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
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
        var projectedScore = CalculateNodeScore(parentIndex, voxel, candidatePositions[0]);
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
            if (useGuidedCorridor && voxel != goalVoxel && !IsInsideGuidedCorridor(candidatePosition))
                continue;

            var candidateScore = i == 0 ? projectedScore : CalculateNodeScore(parentIndex, voxel, candidatePosition);
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
        blendedTarget     = Vector3.Lerp(blendedTarget, voxelCenter, SEARCH_PATH_CENTER_BIAS);
        return ResolveSearchCandidatePosition(voxel, blendedTarget);
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
        return nodeSpan[parentIndex].GScore +
               CalculateEdgeCost(nodeSpan[parentIndex].Position, destination) +
               CalculateWallProximityPenalty(voxel, destination);
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

    private bool TryCreateGuidedCorridor(Vector3 fromPos, Vector3 toPos, out GuidedSearchCorridor corridor)
    {
        corridor = default;

        var horizontalDelta = new Vector2(toPos.X - fromPos.X, toPos.Z - fromPos.Z);
        var horizontalLength = horizontalDelta.Length();
        if (horizontalLength < GUIDED_CORRIDOR_MIN_HORIZONTAL_DISTANCE)
            return false;

        var horizontalDirection = horizontalDelta / horizontalLength;
        var leafHorizontalSize  = MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z);
        var leafVerticalSize    = l2Desc.CellSize.Y;
        var verticalDelta       = toPos.Y - fromPos.Y;
        var horizontalRadius    = Math.Clamp
        (
            horizontalLength * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_SCALE,
            leafHorizontalSize * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MIN_LEAF_CELLS,
            MathF.Max(leafHorizontalSize * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_LEAF_CELLS, horizontalLength * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_DISTANCE_SCALE)
        );
        var upwardAllowance     = Math.Max
        (
            MathF.Abs(verticalDelta) + leafVerticalSize * GUIDED_CORRIDOR_UPWARD_ALLOWANCE_MIN_LEAF_CELLS,
            horizontalLength * GUIDED_CORRIDOR_UPWARD_ALLOWANCE_DISTANCE_SCALE
        );
        var downwardAllowance   = Math.Max
        (
            MathF.Abs(verticalDelta) + leafVerticalSize * GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_MIN_LEAF_CELLS,
            horizontalLength * GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_DISTANCE_SCALE
        );
        var endpointSlack       = Math.Max(horizontalRadius * GUIDED_CORRIDOR_ENDPOINT_SLACK_RADIUS_SCALE, leafHorizontalSize * GUIDED_CORRIDOR_ENDPOINT_SLACK_MIN_LEAF_CELLS);

        corridor = new
        (
            fromPos,
            horizontalDirection,
            horizontalLength,
            verticalDelta,
            horizontalRadius,
            upwardAllowance,
            downwardAllowance,
            endpointSlack
        );
        return true;
    }

    private bool IsInsideGuidedCorridor(Vector3 point)
    {
        var relative = point - guidedCorridor.Start;
        var relativeHorizontal = new Vector2(relative.X, relative.Z);
        var advance = Vector2.Dot(relativeHorizontal, guidedCorridor.HorizontalDirection);
        if (advance < -guidedCorridor.EndpointSlack || advance > guidedCorridor.HorizontalLength + guidedCorridor.EndpointSlack)
            return false;

        var lateral = relativeHorizontal - guidedCorridor.HorizontalDirection * advance;
        if (lateral.LengthSquared() > guidedCorridor.HorizontalRadius * guidedCorridor.HorizontalRadius)
            return false;

        var t = guidedCorridor.HorizontalLength > SCORE_EPSILON ? Math.Clamp(advance / guidedCorridor.HorizontalLength, 0f, 1f) : 0f;
        var baselineY = guidedCorridor.Start.Y + guidedCorridor.VerticalDelta * t;
        var verticalOffset = point.Y - baselineY;
        if (verticalOffset >= 0f)
            return verticalOffset <= guidedCorridor.UpwardAllowance;

        return -verticalOffset <= guidedCorridor.DownwardAllowance;
    }

    private Vector3 ResolveSearchCandidatePosition(ulong voxel, Vector3 point)
    {
        point = Volume.ClampPointToVoxel(voxel, point);
        return ApplyWallAwareInset(voxel, point);
    }

    private Vector3 ResolveVoxelCenter(ulong voxel)
    {
        var (min, max) = Volume.VoxelBounds(voxel, 0);
        return (min + max) * 0.5f;
    }

    private Vector3 ApplyWallAwareInset(ulong voxel, Vector3 point)
    {
        var wallMask = GetVoxelWallMask(voxel);
        if (wallMask == 0)
            return point;

        var (min, max) = Volume.VoxelBounds(voxel, 0);
        var inset = ResolveSearchVoxelInset(voxel);

        if ((wallMask & SearchWallNegX) != 0)
            point.X = MathF.Max(point.X, min.X + inset);
        if ((wallMask & SearchWallPosX) != 0)
            point.X = MathF.Min(point.X, max.X - inset);
        if ((wallMask & SearchWallNegY) != 0)
            point.Y = MathF.Max(point.Y, min.Y + inset);
        if ((wallMask & SearchWallPosY) != 0)
            point.Y = MathF.Min(point.Y, max.Y - inset);
        if ((wallMask & SearchWallNegZ) != 0)
            point.Z = MathF.Max(point.Z, min.Z + inset);
        if ((wallMask & SearchWallPosZ) != 0)
            point.Z = MathF.Min(point.Z, max.Z - inset);

        return point;
    }

    private float CalculateWallProximityPenalty(ulong voxel, Vector3 position)
    {
        if (voxel == goalVoxel)
            return 0f;

        var wallMask = GetVoxelWallMask(voxel);
        if (wallMask == 0)
            return 0f;

        var (min, max) = Volume.VoxelBounds(voxel, 0);
        var voxelSize = max - min;
        var minExtent = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
        var preferredClearance = Math.Clamp(minExtent * SEARCH_PATH_WALL_PREFERRED_CLEARANCE_RATIO, SEARCH_PATH_WALL_PREFERRED_CLEARANCE_MIN, minExtent * SEARCH_PATH_WALL_PREFERRED_CLEARANCE_MAX_FRACTION);
        if (preferredClearance <= SCORE_EPSILON)
            return 0f;

        var pressure = 0f;
        if ((wallMask & SearchWallNegX) != 0)
            pressure += WallPressure(position.X - min.X, preferredClearance);
        if ((wallMask & SearchWallPosX) != 0)
            pressure += WallPressure(max.X - position.X, preferredClearance);
        if ((wallMask & SearchWallNegY) != 0)
            pressure += WallPressure(position.Y - min.Y, preferredClearance);
        if ((wallMask & SearchWallPosY) != 0)
            pressure += WallPressure(max.Y - position.Y, preferredClearance);
        if ((wallMask & SearchWallNegZ) != 0)
            pressure += WallPressure(position.Z - min.Z, preferredClearance);
        if ((wallMask & SearchWallPosZ) != 0)
            pressure += WallPressure(max.Z - position.Z, preferredClearance);

        return pressure * minExtent * SEARCH_PATH_WALL_PENALTY_SCALE;
    }

    private static float WallPressure(float clearance, float preferredClearance)
    {
        if (preferredClearance <= SCORE_EPSILON)
            return 0f;

        return Math.Clamp((preferredClearance - clearance) / preferredClearance, 0f, 1f);
    }

    private float ResolveSearchVoxelInset(ulong voxel)
    {
        var voxelSize   = GetVoxelSize(voxel);
        var minExtent   = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
        var scaledInset = minExtent * SEARCH_PATH_WALL_INSET_RATIO;
        var maxInset    = minExtent * SEARCH_PATH_WALL_INSET_MAX_FRACTION;
        var minInset    = MathF.Min(SEARCH_PATH_WALL_INSET_MIN, maxInset);
        return Math.Clamp(scaledInset, minInset, maxInset);
    }

    private byte GetVoxelWallMask(ulong voxel)
    {
        if (voxelWallMaskCache.TryGetValue(voxel, out var cached))
            return cached;

        byte wallMask = 0;
        if (!HasEmptyNeighbourInDirection(voxel, -1, 0, 0))
            wallMask |= SearchWallNegX;
        if (!HasEmptyNeighbourInDirection(voxel, +1, 0, 0))
            wallMask |= SearchWallPosX;
        if (!HasEmptyNeighbourInDirection(voxel, 0, -1, 0))
            wallMask |= SearchWallNegY;
        if (!HasEmptyNeighbourInDirection(voxel, 0, +1, 0))
            wallMask |= SearchWallPosY;
        if (!HasEmptyNeighbourInDirection(voxel, 0, 0, -1))
            wallMask |= SearchWallNegZ;
        if (!HasEmptyNeighbourInDirection(voxel, 0, 0, +1))
            wallMask |= SearchWallPosZ;

        if (!voxelWallMaskCache.TryAdd(voxel, wallMask))
            return voxelWallMaskCache[voxel];

        return wallMask;
    }

    private bool HasEmptyNeighbourInDirection(ulong voxel, int dx, int dy, int dz)
    {
        var encodedVoxel = voxel;
        var l0Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l1Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l2Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l0Coords     = l0Desc.IndexToVoxel(l0Index);
        var l1Coords     = l1Index != VoxelMap.INDEX_LEVEL_MASK ? l1Desc.IndexToVoxel(l1Index) : default;
        var l2Coords     = l2Index != VoxelMap.INDEX_LEVEL_MASK ? l2Desc.IndexToVoxel(l2Index) : default;

        if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);
            if (l2Desc.InBounds(l2Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                return Volume.IsEmpty(neighbourVoxel);
            }
        }

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);
            if (l1Desc.InBounds(l1Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                if (Volume.IsEmpty(neighbourVoxel))
                    return true;

                if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    return Volume.IsEmpty(l2NeighbourVoxel);
                }

                return HasEmptyOnBorder(neighbourVoxel, l2Desc, 2, dx, dy, dz);
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!l0Desc.InBounds(l0Neighbour))
            return false;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));
        if (Volume.IsEmpty(l0NeighbourVoxel))
            return true;

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1X              = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
            var l1Y              = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
            var l1Z              = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
            var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(l0NeighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);

            if (Volume.IsEmpty(l1NeighbourVoxel))
                return true;

            if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                return Volume.IsEmpty(l2NeighbourVoxel);
            }

            return HasEmptyOnBorder(l1NeighbourVoxel, l2Desc, 2, dx, dy, dz);
        }

        return HasEmptyBorderWithSubdivisions(l0NeighbourVoxel, dx, dy, dz);
    }

    private bool HasEmptyOnBorder(ulong voxel, VolumeLevel levelDesc, int level, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, levelDesc.NumCellsX - 1) : dx > 0 ? (0, 0) : (levelDesc.NumCellsX - 1, levelDesc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, levelDesc.NumCellsY - 1) : dy > 0 ? (0, 0) : (levelDesc.NumCellsY - 1, levelDesc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, levelDesc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (levelDesc.NumCellsZ - 1, levelDesc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            if (Volume.IsEmpty(VoxelMap.EncodeSubIndex(voxel, levelDesc.VoxelToIndex(x, y, z), level)))
                return true;
        }

        return false;
    }

    private bool HasEmptyBorderWithSubdivisions(ulong voxel, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (l1Desc.NumCellsX - 1, l1Desc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (l1Desc.NumCellsY - 1, l1Desc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (l1Desc.NumCellsZ - 1, l1Desc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel) || HasEmptyOnBorder(l1Voxel, l2Desc, 2, dx, dy, dz))
                return true;
        }

        return false;
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

    private readonly record struct GuidedSearchCorridor
    (
        Vector3 Start,
        Vector2 HorizontalDirection,
        float   HorizontalLength,
        float   VerticalDelta,
        float   HorizontalRadius,
        float   UpwardAllowance,
        float   DownwardAllowance,
        float   EndpointSlack
    );

    private readonly record struct FlightPushProbeResult(float Clearance, Vector3 Endpoint);
    private readonly record struct FlightPushHorizontalCandidate(Vector3 Direction, float Clearance, float Score);

    private const float SCORE_EPSILON                                = 0.00001f;
    private const int   DEFAULT_MAX_SEARCH_STEPS                     = 1_0000_0000;
    private const int   RAYCAST_SEARCH_STEP_BUDGET                   = 200000;
    private const int   GUIDED_CORRIDOR_SEARCH_STEP_BUDGET           = 2_000_000;
    private const int   MAX_ANCESTOR_LOOK_BACK                       = 6;
    private const int   RAYCAST_PARALLEL_NEIGHBOUR_THRESHOLD         = 12;
    private const float MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS    = 96f;
    private const float SHORT_RANGE_HEURISTIC_WEIGHT                 = 1.05f;
    private const float LONG_RANGE_HEURISTIC_WEIGHT                  = 1.45f;
    private const float GUIDED_CORRIDOR_HEURISTIC_WEIGHT             = 1.90f;
    private const float LONG_RANGE_HEURISTIC_BLEND_DISTANCE          = 4f;
    private const float GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS = 48f;
    private const int   L1_A_STAR_MAX_EXPANSIONS                     = 200_000;
    private const int   L1_DISTANCE_FIELD_BUDGET                     = 500_000;
    private const float GUIDED_CORRIDOR_MIN_HORIZONTAL_DISTANCE           = 4.00f;
    private const float GUIDED_CORRIDOR_HORIZONTAL_RADIUS_SCALE           = 0.18f;
    private const float GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_DISTANCE_SCALE = 0.35f;
    private const float GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MIN_LEAF_CELLS  = 6.00f;
    private const float GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_LEAF_CELLS  = 20.00f;
    private const float GUIDED_CORRIDOR_UPWARD_ALLOWANCE_DISTANCE_SCALE    = 0.35f;
    private const float GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_DISTANCE_SCALE  = 0.15f;
    private const float GUIDED_CORRIDOR_UPWARD_ALLOWANCE_MIN_LEAF_CELLS    = 8.00f;
    private const float GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_MIN_LEAF_CELLS  = 4.00f;
    private const float GUIDED_CORRIDOR_ENDPOINT_SLACK_RADIUS_SCALE        = 0.65f;
    private const float GUIDED_CORRIDOR_ENDPOINT_SLACK_MIN_LEAF_CELLS      = 3.00f;
    private const byte  SearchWallNegX                               = 1 << 0;
    private const byte  SearchWallPosX                               = 1 << 1;
    private const byte  SearchWallNegY                               = 1 << 2;
    private const byte  SearchWallPosY                               = 1 << 3;
    private const byte  SearchWallNegZ                               = 1 << 4;
    private const byte  SearchWallPosZ                               = 1 << 5;
    private const float SEARCH_PATH_GOAL_BLEND                       = 0.50f;
    private const float SEARCH_PATH_CENTER_BIAS                      = 0.35f;
    private const float SEARCH_PATH_WALL_INSET_RATIO                 = 0.18f;
    private const float SEARCH_PATH_WALL_INSET_MIN                   = 0.03f;
    private const float SEARCH_PATH_WALL_INSET_MAX_FRACTION          = 0.40f;
    private const float SEARCH_PATH_WALL_PREFERRED_CLEARANCE_RATIO   = 0.28f;
    private const float SEARCH_PATH_WALL_PREFERRED_CLEARANCE_MIN     = 0.05f;
    private const float SEARCH_PATH_WALL_PREFERRED_CLEARANCE_MAX_FRACTION = 0.45f;
    private const float SEARCH_PATH_WALL_PENALTY_SCALE               = 0.22f;
    private const float FLIGHT_PUSH_SAMPLE_STEP_SCALE                = 0.75f;
    private const float FLIGHT_PUSH_SAMPLE_STEP_MAX_SCALE            = 1.50f;
    private const float FLIGHT_PUSH_SCAN_DISTANCE_SCALE              = 6.00f;
    private const float FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS  = 18.00f;
    private const float FLIGHT_PUSH_HORIZONTAL_BIAS_SCALE            = 0.95f;
    private const float FLIGHT_PUSH_VERTICAL_BIAS_SCALE              = 0.40f;
    private const float FLIGHT_PUSH_SCAN_PUSH_FRACTION               = 0.62f;
    private const float FLIGHT_PUSH_MAX_CLEARANCE_FRACTION           = 0.72f;
    private const float FLIGHT_PUSH_MIN_HORIZONTAL_IMBALANCE         = 0.05f;
    private const float FLIGHT_PUSH_MIN_VERTICAL_IMBALANCE           = 0.10f;
    private const float FLIGHT_PUSH_FORWARD_WEIGHT                   = 0.85f;
    private const float FLIGHT_PUSH_BACKWARD_WEIGHT                  = 0.55f;
    private const float FLIGHT_PUSH_DIAGONAL_WEIGHT                  = 1.05f;
    private const float FLIGHT_PUSH_MIN_DISTANCE                     = 0.02f;
    private const float FLIGHT_PUSH_VOXEL_INSET_RATIO                = 0.10f;
    private const float FLIGHT_PUSH_VOXEL_INSET_MIN                  = 0.01f;
    private const float FLIGHT_PUSH_VOXEL_INSET_MAX                  = 0.08f;
    private const int   FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT    = 24;
    private const float FLIGHT_PUSH_HORIZONTAL_SWEEP_WEIGHT          = 0.70f;
    private const float FLIGHT_PUSH_HORIZONTAL_SWEEP_FORWARD_BONUS   = 0.20f;
    private const float FLIGHT_PUSH_DIRECTIONAL_FORWARD_BONUS        = 0.85f;
    private const float FLIGHT_PUSH_DIRECTIONAL_CLEARANCE_FRACTION   = 0.85f;
    private const float FLIGHT_PUSH_DIRECTIONAL_DUPLICATE_DOT        = 0.96f;
    private const int   FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT      = 10;
    private const float FLIGHT_DESCENT_SMOOTHING_MAX_SLOPE           = 0.90f;
    private const float FLIGHT_DESCENT_SMOOTHING_MIN_DROP_LEAF_SCALE = 2.50f;
    private const float FLIGHT_DESCENT_SMOOTHING_MIN_DROP_MIN        = 1.00f;
    private const float FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_LEAF_SCALE = 2.00f;
    private const float FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_MIN   = 1.20f;
    private const float FLIGHT_GOAL_DESCENT_HEIGHT_TOLERANCE_LEAF_SCALE = 1.00f;
    private const float FLIGHT_GOAL_DESCENT_HEIGHT_TOLERANCE_MIN     = 0.35f;
    private const float FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_LEAF_SCALE = 0.75f;
    private const float FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_MIN    = 0.20f;
    private const float FLIGHT_TUNNEL_DESCENT_SOFT_HEADROOM_VOXEL_SCALE = 1.20f;
    private const float FLIGHT_TUNNEL_DESCENT_SOFT_HEADROOM_LEAF_SCALE  = 3.00f;
    private const float FLIGHT_TUNNEL_DESCENT_DOWNWARD_CLEARANCE_VOXEL_SCALE = 1.00f;
    private const float FLIGHT_TUNNEL_DESCENT_DOWNWARD_CLEARANCE_LEAF_SCALE  = 1.50f;
    private const float FLIGHT_TUNNEL_DESCENT_CLEARANCE_LEAD_LEAF_SCALE = 1.20f;
    private const float FLIGHT_TUNNEL_DESCENT_CLEARANCE_LEAD_MIN      = 0.60f;
    private const float FLIGHT_TUNNEL_DESCENT_FOLLOW_NEXT_SCALE        = 0.65f;
    private const float FLIGHT_TUNNEL_DESCENT_PREVIOUS_FOLLOW_SCALE    = 0.75f;
    private const float FLIGHT_TUNNEL_DESCENT_EXTRA_FOLLOW_LEAF_SCALE  = 0.75f;
    private const float FLIGHT_TUNNEL_DESCENT_CLEARANCE_ADVANTAGE_SCALE = 0.35f;
    private const float FLIGHT_TUNNEL_DESCENT_HORIZONTAL_BLEND_STRONG = 0.85f;
    private const float FLIGHT_TUNNEL_DESCENT_HORIZONTAL_BLEND_MEDIUM = 0.50f;
    private const float FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_VOXEL_SCALE = 0.85f;
    private const float FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_LEAF_SCALE  = 1.60f;
    private const float FLIGHT_PUSH_DOWNWARD_UPWARD_BLOCKED_VOXEL_SCALE    = 0.45f;
    private const float FLIGHT_PUSH_DOWNWARD_UPWARD_BLOCKED_LEAF_SCALE     = 0.90f;
    private const float FLIGHT_PUSH_DOWNWARD_MIN_CLEARANCE_VOXEL_SCALE     = 1.20f;
    private const float FLIGHT_PUSH_DOWNWARD_MIN_CLEARANCE_LEAF_SCALE      = 2.20f;
    private const float FLIGHT_PUSH_DOWNWARD_MIN_LEAD_LEAF_SCALE           = 0.90f;
    private const float FLIGHT_PUSH_DOWNWARD_SCALE                         = 0.65f;
    private const float FLIGHT_PUSH_FLOOR_AVOIDANCE_SCALE                  = 0.85f;
    private const float FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE                 = 0.05f;
    private const float FLIGHT_PUSH_HEIGHT_MATCH_BIAS                      = 0.08f;
    private const float FLIGHT_PUSH_HEIGHT_CATCHUP_SCALE                   = 1.20f;
    private const float FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_LEAF_SCALE     = 0.90f;
    private const float FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_MIN            = 0.25f;
    private const float FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE                = 0.02f;
    private const float FLIGHT_PUSH_HEIGHT_STRICT_BIAS                     = 0.06f;
    private const float FLIGHT_PUSH_HEIGHT_RAISE_HORIZONTAL_BLEND          = 0.20f;
    private const float FLIGHT_PUSH_VERTICAL_FIRST_HORIZONTAL_BLEND        = 0.40f;
    private const float FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG       = 1.60f;
    private const float FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_HIGH         = 1.35f;
    private const float FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM       = 1.15f;
    private const float FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_LIGHT        = 0.90f;
    private const float FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MIN          = 0.65f;

    #endregion
}
