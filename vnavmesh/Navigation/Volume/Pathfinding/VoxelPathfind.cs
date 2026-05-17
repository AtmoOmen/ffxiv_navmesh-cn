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
    private readonly ConcurrentDictionary<ulong, byte>               verifiedDownwardOpeningCache = new(Environment.ProcessorCount, 2048);
    private readonly ConcurrentDictionary<ulong, byte>               verifiedTopEntryCache = new(Environment.ProcessorCount, 2048);
    private readonly ConcurrentDictionary<ulong, ulong>              l1FaceConnectivityCache = new(Environment.ProcessorCount, 2048);
    private readonly List<int>                                       openList        = new(256);
    private readonly ConcurrentDictionary<VolumeVisibilityKey, bool> visibilityCache = new(Environment.ProcessorCount, 4096);

    private int                       bestNodeIndex;
    private ulong                     goalVoxel;
    private Vector3                   goalPos;
    private bool                      goalReached;
    private bool                      useGuidedCorridor;
    private float                     heuristicWeight;
    private int                       visitedNodes;
    private int                       generatedNodes;
    private int                       lineOfSightChecks;
    private int                       lineOfSightHits;
    private int                       peakOpenListSize;
    private VolumeSearchTermination   lastTermination;
    private int                       lastSearchAttempts;
    private bool                      debugReturnedLongRangeBestEffortPath;
    private bool                      guidedCorridorEarlyAbortTriggered;
    private int                       currentL1CorridorRadius;
    private HashSet<ulong>?           l1PathSet;
    private HashSet<ushort>?          l0PathSet;
    private Dictionary<ulong, int>?   l1CorridorDistance;
    private Dictionary<ushort, int>?  l0CorridorDistance;
    private Dictionary<ulong, float>? l1DistanceField;
    private Dictionary<ushort, float>? l0DistanceField;
    private FlightPathDebugPayload?   lastPathDebug;
    private FlightLongRangeProxyDebug? pendingLongRangeProxyDebug;
    private GuidedSearchCorridor      guidedCorridor;
    private LongRangeLateralBias      longRangeLateralBias;
    private float                     guidedCorridorInitialGoalDistance;
    private float                     guidedCorridorInitialAboveGoal;
    private float                     guidedCorridorLastProgressDistance;
    private float                     guidedCorridorLastProgressAboveGoal;
    private int                       guidedCorridorLastProgressVisited;

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
        bool              returnIntermediatePoints,
        CancellationToken cancel
    )
    {
        l1PathSet          = null;
        l0PathSet          = null;
        l1CorridorDistance = null;
        l0CorridorDistance = null;
        l1DistanceField    = null;
        l0DistanceField    = null;
        lastPathDebug      = null;
        pendingLongRangeProxyDebug = null;

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

        if (TryBuildDirectPath(fromVoxel, toVoxel, fromPos, toPos, out var directPath))
            return directPath;

        var searchRaycast = Vector3.Distance(fromPos, toPos) <= maxSearchRaycastDistance;

        if (searchRaycast)
        {
            var path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel, RAYCAST_SEARCH_STEP_BUDGET, 1);

            if (lastTermination != VolumeSearchTermination.ReachedGoal)
                path = RunShortRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel);

            return RefineSimplifiedPath(path, cancel);
        }

        if (TryCreateGuidedCorridor(fromPos, toPos, out var corridor))
        {
            var corridorPath = RunSearchAttempt
            (
                fromVoxel,
                toVoxel,
                fromPos,
                toPos,
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
                return RefineSimplifiedPath(corridorPath, cancel);
            }

            Service.Log.Debug
            (
                guidedCorridorEarlyAbortTriggered
                    ? $"[算路] 飞行体素定向走廊搜索提前回退（进展停滞），访问节点 = {visitedNodes}，最佳距离 = {NodeSpan[bestNodeIndex].HScore:f3}，当前高差余量 = {MathF.Max(NodeSpan[bestNodeIndex].Position.Y - goalPos.Y, 0f):f3}"
                    : $"[算路] 飞行体素定向走廊搜索未达终点（{lastTermination}），回退侧向探测/全搜索"
            );
        }

        var longRangePath = RunLongRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel);
        if (debugReturnedLongRangeBestEffortPath)
        {
            Service.Log.Debug("[算路] 飞行体素调试：best-effort 后直接停止，并返回粗路径调试叠加");
            return [];
        }

        if (lastTermination == VolumeSearchTermination.ReachedGoal)
            return RefineSimplifiedPath(longRangePath, cancel);

        if (!EnableLongRangeGlobalFallback)
        {
            Service.Log.Debug
            (
                longRangePath.Count > 0
                    ? $"[算路] 飞行体素已禁用长距离全局回退，返回当前长距离代理/接力结果用于调试：路径点 = {longRangePath.Count}，终止 = {lastTermination}"
                    : "[算路] 飞行体素已禁用长距离全局回退，且当前长距离代理/接力未产出有效路径"
            );
            return longRangePath.Count > 0 ? RefineSimplifiedPath(longRangePath, cancel) : [];
        }

        if (l1DistanceField is not { Count: > 0 })
            ComputeL1DistanceField(toVoxel, toPos);
        var fallbackPath = RunSearchAttempt
        (
            fromVoxel,
            toVoxel,
            fromPos,
            toPos,
            returnIntermediatePoints,
            cancel,
            LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET,
            LONG_RANGE_LATERAL_EXPLORATION_ATTEMPTS + 2,
            heuristicWeightOverride: LONG_RANGE_GLOBAL_FALLBACK_HEURISTIC_WEIGHT
        );
        return RefineSimplifiedPath(fallbackPath, cancel);
    }

    private List<(ulong voxel, Vector3 p)> RunLongRangeFallback
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel,
        int               recursionDepth = 0
    )
    {
        if (recursionDepth == 0)
            pendingLongRangeProxyDebug = null;
        var coarsePath = SearchL1BestEffortPath(fromVoxel, fromPos, toVoxel, toPos);
        if (coarsePath.PathSet.Count == 0)
        {
            ClearL1AreaConstraint();
            Service.Log.Debug("[算路] 飞行体素粗层 best-effort 搜索未产出有效路径");
            return [];
        }

        Service.Log.Debug
        (
            $"[算路] 飞行体素粗层 best-effort 完成：到达终点 = {(coarsePath.ReachedGoal ? 1 : 0)}，粗路径单元 = {coarsePath.PathSet.Count}，扩展节点 = {coarsePath.ExpandedNodes}/{coarsePath.StepBudget}，最佳粗距离 = {coarsePath.BestDistance:f3}"
        );

        if (!EnableLongRangeGlobalFallback && DebugReturnLongRangeBestEffortPath)
        {
            ClearL1AreaConstraint();
            debugReturnedLongRangeBestEffortPath = true;
            lastTermination    = VolumeSearchTermination.SearchExhausted;
            lastSearchAttempts = 1;
            visitedNodes       = coarsePath.ExpandedNodes;
            generatedNodes     = coarsePath.ExpandedNodes;
            lineOfSightChecks  = 0;
            lineOfSightHits    = 0;
            peakOpenListSize   = 0;
            heuristicWeight    = LONG_RANGE_HEURISTIC_WEIGHT;
            lastPathDebug      = BuildCoarsePathDebugPayload(coarsePath.OrderedPath, fromPos, toPos, coarsePath.ReachedGoal);
            return [];
        }

        var guidedCorridorRadius = ResolveLongRangeGuidedFullSearchCorridorRadius(coarsePath.BestDistance);
        var proxyGoalVoxel       = coarsePath.OrderedPath[^1];
        var proxyGoalPos         = ResolveSearchCandidatePosition(proxyGoalVoxel, ResolveVoxelCenter(proxyGoalVoxel));

        BuildL1Corridor(coarsePath.PathSet, guidedCorridorRadius);
        ComputeL1DistanceFieldFromCoarsePath(coarsePath.OrderedPath, 0f);

        var proxyPath = RunSearchAttempt
        (
            fromVoxel,
            proxyGoalVoxel,
            fromPos,
            proxyGoalPos,
            returnIntermediatePoints,
            cancel,
            LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET,
            1,
            heuristicWeightOverride: LONG_RANGE_GLOBAL_FALLBACK_HEURISTIC_WEIGHT
        );

        if (lastTermination == VolumeSearchTermination.ReachedGoal)
        {
            pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyPath[^1].p, toPos, FlightLongRangeTailKind.None);
            Service.Log.Debug
            (
                $"[算路] 飞行体素粗层代理搜索完成：访问节点 = {visitedNodes}，粗路径单元 = {coarsePath.PathSet.Count}，引导半径 = {guidedCorridorRadius}，启发式权重 = {heuristicWeight:f2}"
            );
            ClearL1AreaConstraint();

            if (proxyPath.Count == 0)
                return proxyPath;

            var proxyEndpoint = proxyPath[^1];
            if (TryBuildDirectPath(proxyEndpoint.voxel, toVoxel, proxyEndpoint.p, toPos, out var directTail))
            {
                pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyEndpoint.p, toPos, FlightLongRangeTailKind.DirectToGoal);
                Service.Log.Debug("[算路] 飞行体素粗层代理搜索后直连终点成功");
                return MergePathSegments(proxyPath, directTail);
            }

            var tailPath = RunTailSearchFromProxy(proxyEndpoint.voxel, toVoxel, proxyEndpoint.p, toPos, returnIntermediatePoints, cancel, coarsePath.ReachedGoal, recursionDepth, out var usedLongRangeReentry);
            if (lastTermination == VolumeSearchTermination.ReachedGoal)
            {
                if (!(usedLongRangeReentry && pendingLongRangeProxyDebug is not null))
                    pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyEndpoint.p, toPos, FlightLongRangeTailKind.ShortRangeRelay);
                Service.Log.Debug
                (
                    usedLongRangeReentry
                        ? $"[算路] 飞行体素粗层代理搜索后二次粗搜接力完成：访问节点 = {visitedNodes}，路径点 = {tailPath.Count}"
                        : $"[算路] 飞行体素粗层代理搜索后短程接力完成：访问节点 = {visitedNodes}，路径点 = {tailPath.Count}"
                );
                return MergePathSegments(proxyPath, tailPath);
            }

            if (!(usedLongRangeReentry && pendingLongRangeProxyDebug is not null))
                pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyEndpoint.p, toPos, FlightLongRangeTailKind.ShortRangeRelayPartial);
            Service.Log.Debug
            (
                usedLongRangeReentry
                    ? $"[算路] 飞行体素粗层代理搜索后二次粗搜接力未达终点（{lastTermination}），访问节点 = {visitedNodes}"
                    : $"[算路] 飞行体素粗层代理搜索后短程接力未达终点（{lastTermination}），访问节点 = {visitedNodes}"
            );
            return MergePathSegments(proxyPath, tailPath);
        }

        pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyPath.Count > 0 ? proxyPath[^1].p : fromPos, toPos, FlightLongRangeTailKind.None);
        Service.Log.Debug
        (
            $"[算路] 飞行体素粗层代理搜索未达终点（{lastTermination}），粗路径单元 = {coarsePath.PathSet.Count}，引导半径 = {guidedCorridorRadius}，访问节点 = {visitedNodes}"
        );
        ClearL1AreaConstraint();
        return proxyPath;
    }

    private List<(ulong voxel, Vector3 p)> RunTailSearchFromProxy
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel,
        bool              coarseReachedGoal,
        int               recursionDepth,
        out bool          usedLongRangeReentry
    )
    {
        usedLongRangeReentry = false;
        ClearL1AreaConstraint();
        l1DistanceField = null;
        l0DistanceField = null;

        if (TryBuildDirectPath(fromVoxel, toVoxel, fromPos, toPos, out var directPath))
            return directPath;

        var remainingDistance         = Vector3.Distance(fromPos, toPos);
        var searchRaycast             = remainingDistance <= maxSearchRaycastDistance;
        var allowLongRangeCoarseReentry = recursionDepth < LONG_RANGE_PROXY_COARSE_REENTRY_MAX_DEPTH && !searchRaycast;
        if (searchRaycast)
        {
            var path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel, RAYCAST_SEARCH_STEP_BUDGET, 1);
            if (lastTermination != VolumeSearchTermination.ReachedGoal)
                path = RunShortRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel);
            return path;
        }

        var allowProxyGuidedCorridor = coarseReachedGoal;
        if (!allowProxyGuidedCorridor)
        {
            Service.Log.Debug
            (
                $"[算路] 飞行体素粗层未触达终点，代理尾段跳过定向走廊：剩余直线距离 = {remainingDistance:f3}，直接回退{(allowLongRangeCoarseReentry ? "二次粗搜" : "真实终点距离场全搜索")}"
            );
        }
        else if (TryCreateGuidedCorridor(fromPos, toPos, out var corridor))
        {
            var corridorPath = RunSearchAttempt
            (
                fromVoxel,
                toVoxel,
                fromPos,
                toPos,
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
                    $"[算路] 飞行体素粗层代理后定向走廊搜索完成：访问节点 = {visitedNodes}，走廊半径 = {corridor.HorizontalRadius:f3}，上抬余量 = {corridor.UpwardAllowance:f3}"
                );
                return corridorPath;
            }

            Service.Log.Debug
            (
                guidedCorridorEarlyAbortTriggered
                    ? $"[算路] 飞行体素粗层代理后定向走廊提前回退（进展停滞），访问节点 = {visitedNodes}，最佳距离 = {NodeSpan[bestNodeIndex].HScore:f3}，当前高差余量 = {MathF.Max(NodeSpan[bestNodeIndex].Position.Y - goalPos.Y, 0f):f3}"
                    : $"[算路] 飞行体素粗层代理后定向走廊未达终点（{lastTermination}），回退{(allowLongRangeCoarseReentry ? "二次粗搜" : "真实终点距离场全搜索")}"
            );
        }

        if (allowLongRangeCoarseReentry)
        {
            usedLongRangeReentry = true;
            Service.Log.Debug
            (
                $"[算路] 飞行体素粗层代理后触发二次粗搜：剩余直线距离 = {remainingDistance:f3}，递归深度 = {recursionDepth + 1}/{LONG_RANGE_PROXY_COARSE_REENTRY_MAX_DEPTH}"
            );

            var reentryPath = RunLongRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel, recursionDepth + 1);
            if (lastTermination == VolumeSearchTermination.ReachedGoal)
                return reentryPath;

            if (reentryPath.Count > 0)
            {
                Service.Log.Debug($"[算路] 飞行体素粗层代理后二次粗搜未达终点（{lastTermination}），返回其最佳结果：路径点 = {reentryPath.Count}");
                return reentryPath;
            }

            Service.Log.Debug("[算路] 飞行体素粗层代理后二次粗搜未产出有效路径，回退真实终点距离场全搜索");
            usedLongRangeReentry = false;
        }

        ComputeL1DistanceField(toVoxel, toPos);
        return RunSearchAttempt
        (
            fromVoxel,
            toVoxel,
            fromPos,
            toPos,
            returnIntermediatePoints,
            cancel,
            LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET,
            2,
            heuristicWeightOverride: LONG_RANGE_GLOBAL_FALLBACK_HEURISTIC_WEIGHT
        );
    }

    private List<(ulong voxel, Vector3 p)> RunShortRangeFallback
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel
    )
    {
        var attempts = 2;
        var useExploratoryHeuristic = ShouldUseShortRangeExploratoryHeuristic(fromPos, toPos);
        var fallbackHeuristicWeight = useExploratoryHeuristic ? SHORT_RANGE_EXPLORATORY_HEURISTIC_WEIGHT : SHORT_RANGE_HEURISTIC_WEIGHT;

        Service.Log.Debug
        (
            $"[算路] 飞行体素短距搜索进入拓扑回退：直线距离 = {Vector3.Distance(fromPos, toPos):f3}，首轮终止 = {lastTermination}，首轮访问节点 = {visitedNodes}，LoS 检查 = {lineOfSightChecks}，探索式启发 = {(useExploratoryHeuristic ? "是" : "否")}"
        );

        var l1Path = SearchL1CoarsePath(fromVoxel, fromPos, toVoxel, toPos);
        if (l1Path is { Count: > 1 } pathSet)
        {
            BuildL1Corridor(pathSet, 0);
            var constrainedPath = RunSearchAttempt
            (
                fromVoxel,
                toVoxel,
                fromPos,
                toPos,
                returnIntermediatePoints,
                cancel,
                DEFAULT_MAX_SEARCH_STEPS,
                attempts++,
                heuristicWeightOverride: fallbackHeuristicWeight
            );

            if (lastTermination == VolumeSearchTermination.ReachedGoal)
            {
                Service.Log.Debug
                (
                    $"[算路] 飞行体素短距 L1 约束搜索完成：访问节点 = {visitedNodes}，L1 路径单元 = {pathSet.Count}，启发式权重 = {heuristicWeight:f2}"
                );
                return constrainedPath;
            }

            Service.Log.Debug
            (
                $"[算路] 飞行体素短距 L1 约束搜索未达终点（{lastTermination}），回退 L1 距离场全搜索"
            );
            l1PathSet = null;
            l0PathSet = null;
        }
        else if (l1Path is { Count: 1 })
        {
            Service.Log.Debug("[算路] 飞行体素短距起终点位于同一 L1 单元，跳过 L1 约束，直接使用距离场回退");
        }
        else Service.Log.Debug("[算路] 飞行体素短距 L1 粗搜索未找到路径，直接使用距离场回退");

        if (l1DistanceField is not { Count: > 0 })
            ComputeL1DistanceField(toVoxel, toPos);

        return RunSearchAttempt
        (
            fromVoxel,
            toVoxel,
            fromPos,
            toPos,
            returnIntermediatePoints,
            cancel,
            DEFAULT_MAX_SEARCH_STEPS,
            attempts,
            heuristicWeightOverride: fallbackHeuristicWeight
        );
    }

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
        nodeLookup[fromVoxel] = 0;
        generatedNodes        = 1;
        guidedCorridorInitialGoalDistance = Vector3.Distance(fromPos, toPos);
        guidedCorridorInitialAboveGoal    = MathF.Max(fromPos.Y - toPos.Y, 0f);
        guidedCorridorLastProgressDistance = guidedCorridorInitialGoalDistance;
        guidedCorridorLastProgressAboveGoal = guidedCorridorInitialAboveGoal;
        guidedCorridorLastProgressVisited  = 0;
        AddToOpen(0);
    }

    private VolumeSearchTermination Execute(CancellationToken cancel, int maxSteps = DEFAULT_MAX_SEARCH_STEPS)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
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
        if (!useGuidedCorridor || goalReached || visitedNodes < GUIDED_CORRIDOR_EARLY_ABORT_MIN_VISITED || bestNodeIndex < 0 || bestNodeIndex >= nodes.Count)
            return false;

        var bestNode            = NodeSpan[bestNodeIndex];
        var bestDistance        = bestNode.HScore;
        var bestAboveGoal       = MathF.Max(bestNode.Position.Y - goalPos.Y, 0f);
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
        Start(fromVoxel, toVoxel, fromPos, toPos, lateralBias);
        useGuidedCorridor        = corridor.HasValue;
        guidedCorridor           = corridor.GetValueOrDefault();
        heuristicWeight          = heuristicWeightOverride ?? SHORT_RANGE_HEURISTIC_WEIGHT;
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
        longRangeLateralBias     = default;
        debugReturnedLongRangeBestEffortPath = false;
        guidedCorridorEarlyAbortTriggered = false;
        currentL1CorridorRadius  = 0;
        lastTermination          = VolumeSearchTermination.SearchExhausted;
        lastSearchAttempts       = 0;
        lastPathDebug            = null;
        pendingLongRangeProxyDebug = null;
        guidedCorridorInitialAboveGoal    = 0;
        guidedCorridorInitialGoalDistance = 0;
        guidedCorridorLastProgressAboveGoal = 0;
        guidedCorridorLastProgressDistance = 0;
        guidedCorridorLastProgressVisited  = 0;
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
        var minDrop            = MathF.Max(leafVerticalSize * SHORT_RANGE_EXPLORATION_MIN_DROP_LEAF_CELLS, SHORT_RANGE_EXPLORATION_MIN_DROP_DISTANCE);
        return horizontalDistance >= minHorizontal && verticalDrop >= minDrop;
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
        lastPathDebug = BuildFlightPathDebugPayload(refined, debugInfos, finalPath, pendingLongRangeProxyDebug);
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

        horizontalForward = !TryNormalize(horizontalForward, out var normalizedHorizontalForward) ? Vector2.UnitX : normalizedHorizontalForward;

        var horizontalRight = new Vector2(-horizontalForward.Y, horizontalForward.X);
        var forward3        = new Vector3(horizontalForward.X, 0, horizontalForward.Y);
        var right3          = new Vector3(horizontalRight.X,   0, horizontalRight.Y);
        var forwardRight3   = Vector3.Normalize(forward3 + right3);
        var forwardLeft3    = Vector3.Normalize(forward3 - right3);
        var backwardRight3  = Vector3.Normalize(-forward3 + right3);
        var backwardLeft3   = Vector3.Normalize(-forward3 - right3);
        var goalAdjacentRearBias = next.voxel == goalVoxel;

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
        var forwardWeight         = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_WEIGHT          : FLIGHT_PUSH_FORWARD_WEIGHT;
        var backwardWeight        = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_BACKWARD_WEIGHT         : FLIGHT_PUSH_BACKWARD_WEIGHT;
        var forwardDiagonalWeight = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_DIAGONAL_WEIGHT : FLIGHT_PUSH_DIAGONAL_WEIGHT;
        var backwardDiagonalWeight = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_BACKWARD_DIAGONAL_WEIGHT : FLIGHT_PUSH_DIAGONAL_WEIGHT;

        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forward3,        forwardClearance,       forwardWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -forward3,       backwardClearance,      backwardWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, right3,          rightClearance,         1f);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, rightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -right3,         leftClearance,          1f);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, leftClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardRight3,   forwardRightClearance,  forwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardRightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardLeft3,    forwardLeftClearance,   forwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardLeftClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardRight3,  backwardRightClearance, backwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardRightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardLeft3,   backwardLeftClearance,  backwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardLeftClearance);
        var sweepSamples = BuildHorizontalSweepSamples
        (
            current,
            forward3,
            right3,
            horizontalScanDistance,
            horizontalSampleStep,
            goalAdjacentRearBias,
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
            sweepSamples,
            goalAdjacentRearBias
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
            if (goalAdjacentRearBias)
            {
                var forwardComponent = Vector2.Dot(horizontalPushDirection, normalizedHorizontalForward);
                if (forwardComponent > FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT)
                {
                    var rearProjectedDirection = horizontalPushDirection - normalizedHorizontalForward * forwardComponent;
                    if (!TryNormalize(rearProjectedDirection, out horizontalPushDirection))
                        horizontalPushDirection = default;
                }
            }

            var desiredHorizontalPush = horizontalBiasMagnitude * FLIGHT_PUSH_HORIZONTAL_BIAS_SCALE;
            var horizontalPushDistance = MathF.Min(maxHorizontalPush, desiredHorizontalPush);
            if (horizontalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE &&
                horizontalPushDirection.LengthSquared() > SCORE_EPSILON * SCORE_EPSILON)
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
            else if (TryAcceptDirectionalHorizontalCandidatesWithFixedVertical(previous, current, next, directionalHorizontalCandidates, maxHorizontalPush, verticalOffset, forward3, goalAdjacentRearBias, out adjusted))
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
            else if (TryAcceptDirectionalHorizontalCandidatesWithFixedVertical(previous, current, next, directionalHorizontalCandidates, maxHorizontalPush, verticalOffset, forward3, goalAdjacentRearBias, out adjusted))
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
        Vector3                                  forward,
        bool                                     goalAdjacentRearBias,
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
            if (goalAdjacentRearBias &&
                Vector3.Dot(candidate.Direction, forward) > FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT)
                continue;

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
        directionHint = !TryNormalize(directionHint, out var normalizedDirectionHint) ? Vector3.UnitY : normalizedDirectionHint;

        var attemptLift = requiredLift + FLIGHT_PUSH_HEIGHT_STRICT_BIAS;
        Span<float> scales = [1.0f, 0.85f, 0.70f, 0.55f];
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
        IReadOnlyList<FlightPathDebugSample> sweepSamples,
        bool                               goalAdjacentRearBias
    )
    {
        List<FlightPushHorizontalCandidate> candidates = new(FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT);

        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, rightSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, leftSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardRightSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardLeftSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardRightSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardLeftSample, goalAdjacentRearBias);

        foreach (var sample in sweepSamples)
            TryAddDirectionalHorizontalCandidate(candidates, origin, forward, new(sample.Clearance, sample.Endpoint), goalAdjacentRearBias);

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
        FlightPushProbeResult               sample,
        bool                                goalAdjacentRearBias
    )
    {
        if (sample.Clearance <= FLIGHT_PUSH_MIN_DISTANCE)
            return;

        var delta = sample.Endpoint - origin;
        delta.Y = 0;
        if (!TryNormalize(delta, out var direction))
            return;

        var forwardDot = Vector3.Dot(direction, forward);
        var scoreScale = goalAdjacentRearBias
                             ? MathF.Max
                               (
                                   FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_MIN_SCALE,
                                   1f -
                                   MathF.Max(0f, forwardDot) * FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_FORWARD_PENALTY +
                                   MathF.Max(0f, -forwardDot) * FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_BACKWARD_BONUS
                               )
                             : 1f + MathF.Max(0f, forwardDot) * FLIGHT_PUSH_DIRECTIONAL_FORWARD_BONUS;
        var score = sample.Clearance * scoreScale;

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
        bool                     goalAdjacentRearBias,
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
            if (goalAdjacentRearBias)
            {
                if (forwardDot > FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT)
                    weight *= MathF.Max(FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_FORWARD_MIN_SCALE, 1f - forwardDot * FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_FORWARD_PENALTY);
                else
                    weight += MathF.Max(0f, -forwardDot) * FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_BACKWARD_BONUS;
            }
            else if (forwardDot > 0f)
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
        IReadOnlyList<(ulong voxel, Vector3 p)> finalPath,
        FlightLongRangeProxyDebug?              proxyDebug = null
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

        if (remapped.Count == 0 && proxyDebug == null)
            return null;

        return new()
        {
            Waypoints  = remapped,
            CoarsePath = [],
            ProxyDebug = proxyDebug
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
        var coarseOnlyLongRangeSearch = ShouldRestrictLongRangeSearchToCoarseLevels();

        if (TryAddPreferredCoarseNeighbour(l0Index, l1Index, l0Coords, l1Coords, dx, dy, dz))
            return;

        if (!coarseOnlyLongRangeSearch && l2Index != VoxelMap.INDEX_LEVEL_MASK)
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
                else if (!coarseOnlyLongRangeSearch && l2Index != VoxelMap.INDEX_LEVEL_MASK)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    AddNeighbourIfEmpty(l2NeighbourVoxel);
                }
                else if (!coarseOnlyLongRangeSearch)
                    CollectBorder(neighbourVoxel, l2Desc, 2, dx, dy, dz);

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
            else if (!coarseOnlyLongRangeSearch && l2Index != VoxelMap.INDEX_LEVEL_MASK)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                AddNeighbourIfEmpty(l2NeighbourVoxel);
            }
            else if (!coarseOnlyLongRangeSearch)
                CollectBorder(l1NeighbourVoxel, l2Desc, 2, dx, dy, dz);

            return;
        }

        if (coarseOnlyLongRangeSearch) CollectBorderWithSubdivisionsL1Only(l0NeighbourVoxel, dx, dy, dz);
        else CollectBorderWithSubdivisions(l0NeighbourVoxel, dx, dy, dz);
    }

    private bool TryAddPreferredCoarseNeighbour
    (
        ushort                l0Index,
        ushort                l1Index,
        (int x, int y, int z) l0Coords,
        (int x, int y, int z) l1Coords,
        int                   dx,
        int                   dy,
        int                   dz
    )
    {
        if (!ShouldPreferCoarseDescentNeighbour(dx, dy, dz))
            return false;

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);
            if (l1Desc.InBounds(l1Neighbour))
            {
                var l1NeighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                l1NeighbourVoxel = VoxelMap.EncodeIndex(l0Index, l1NeighbourVoxel);
                if (CanUsePreferredCoarseNeighbour(l1NeighbourVoxel, dy))
                {
                    AddNeighbourIfEmpty(l1NeighbourVoxel);
                    return true;
                }
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!l0Desc.InBounds(l0Neighbour))
            return false;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));
        if (!CanUsePreferredCoarseNeighbour(l0NeighbourVoxel, dy))
            return false;

        AddNeighbourIfEmpty(l0NeighbourVoxel);
        return true;
    }

    private bool ShouldPreferCoarseDescentNeighbour(int dx, int dy, int dz)
    {
        if (!longRangeLateralBias.Enabled || !longRangeLateralBias.PreferDescending || dy > 0)
            return false;

        if (dy < 0)
            return true;

        var horizontal = new Vector2(dx, dz);
        if (!TryNormalize(horizontal, out var direction))
            return false;

        return Vector2.Dot(direction, longRangeLateralBias.Forward) >= LONG_RANGE_LATERAL_COARSE_PROMOTION_MIN_FORWARD_DOT;
    }

    private bool CanUsePreferredCoarseNeighbour(ulong voxel, int dy)
    {
        if (!Volume.IsEmpty(voxel))
            return false;

        return dy < 0 ? HasVerifiedTopEntry(voxel) : HasDownwardOpening(voxel);
    }

    private bool ShouldRestrictLongRangeSearchToCoarseLevels() => longRangeLateralBias.Enabled;

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

    private void CollectBorderWithSubdivisionsL1Only(ulong voxel, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (l1Desc.NumCellsX - 1, l1Desc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (l1Desc.NumCellsY - 1, l1Desc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (l1Desc.NumCellsZ - 1, l1Desc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel))
                AddNeighbourIfEmpty(l1Voxel);
        }
    }

    private void AddNeighbourIfEmpty(ulong voxel)
    {
        if (l1CorridorDistance is { } corridorDistance)
        {
            if (!TryGetVoxelCorridorDistance(voxel, out var distance) || distance > currentL1CorridorRadius)
                return;
        }
        else if (!IsVoxelInsidePathConstraint(voxel))
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
               CalculateLongRangeLateralTraversalPenalty(nodeSpan[parentIndex].Voxel, nodeSpan[parentIndex].Position, voxel, destination) +
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

    private static ushort ExtractL0Index(ulong voxel)
    {
        var temp = voxel;
        return VoxelMap.DecodeIndex(ref temp);
    }

    private static bool TryExtractL1Parent(ulong voxel, out ulong l1Voxel)
    {
        var temp = voxel;
        var l0   = VoxelMap.DecodeIndex(ref temp);
        var l1   = VoxelMap.DecodeIndex(ref temp);
        if (l1 == VoxelMap.INDEX_LEVEL_MASK)
        {
            l1Voxel = VoxelMap.INVALID_VOXEL;
            return false;
        }

        l1Voxel = VoxelMap.EncodeIndex(l1);
        l1Voxel = VoxelMap.EncodeIndex(l0, l1Voxel);
        return true;
    }

    private bool HasTraversableL1FaceTransition(ulong currentL1, ulong neighbourL1, int dx, int dy, int dz)
    {
        var currentX   = dx > 0 ? l2Desc.NumCellsX - 1 : 0;
        var currentY   = dy > 0 ? l2Desc.NumCellsY - 1 : 0;
        var currentZ   = dz > 0 ? l2Desc.NumCellsZ - 1 : 0;
        var neighbourX = dx > 0 ? 0 : l2Desc.NumCellsX - 1;
        var neighbourY = dy > 0 ? 0 : l2Desc.NumCellsY - 1;
        var neighbourZ = dz > 0 ? 0 : l2Desc.NumCellsZ - 1;

        var xMin = dx == 0 ? 0 : currentX;
        var xMax = dx == 0 ? l2Desc.NumCellsX - 1 : currentX;
        var yMin = dy == 0 ? 0 : currentY;
        var yMax = dy == 0 ? l2Desc.NumCellsY - 1 : currentY;
        var zMin = dz == 0 ? 0 : currentZ;
        var zMax = dz == 0 ? l2Desc.NumCellsZ - 1 : currentZ;

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var otherX = dx == 0 ? x : neighbourX;
            var otherY = dy == 0 ? y : neighbourY;
            var otherZ = dz == 0 ? z : neighbourZ;

            var currentLeaf   = VoxelMap.EncodeSubIndex(currentL1,   l2Desc.VoxelToIndex(x,      y,      z),      2);
            var neighbourLeaf = VoxelMap.EncodeSubIndex(neighbourL1, l2Desc.VoxelToIndex(otherX, otherY, otherZ), 2);
            if (Volume.IsEmpty(currentLeaf) && Volume.IsEmpty(neighbourLeaf))
                return true;
        }

        return false;
    }

    private static int OppositeFace(int face) => face ^ 1;

    private static ushort L1FaceBit(int face) => (ushort)(1 << face);

    private static bool HasL1Face(ushort faceMask, int face) => (faceMask & L1FaceBit(face)) != 0;

    private static bool CanTraverseMixedL1Cell(bool includeNonEmpty, ulong candidateL1, ulong goalL1)
        => includeNonEmpty && candidateL1 == goalL1;

    private static ushort GetPackedReachableL1Faces(ulong packedConnectivity, int face) => (ushort)((packedConnectivity >> (face * 6)) & L1AllFacesMask);

    private static ulong SetPackedReachableL1Faces(ulong packedConnectivity, int face, ushort reachableFaces)
    {
        var shift = face * 6;
        packedConnectivity &= ~((ulong)L1AllFacesMask << shift);
        packedConnectivity |= (ulong)(reachableFaces & L1AllFacesMask) << shift;
        return packedConnectivity;
    }

    private ushort GetReachableL1FacesFromEntry(ulong l1Voxel, int entryFace)
    {
        if (entryFace is < 0 or > 5)
            return 0;
        if (Volume.IsEmpty(l1Voxel))
            return L1AllFacesMask;

        var packedConnectivity = l1FaceConnectivityCache.GetOrAdd(l1Voxel, BuildPackedL1FaceConnectivity);
        return GetPackedReachableL1Faces(packedConnectivity, entryFace);
    }

    private ushort GetReachableL1FacesFromPoint(ulong l1Voxel, ulong actualVoxel, Vector3 point)
    {
        if (Volume.IsEmpty(l1Voxel))
            return L1AllFacesMask;
        if (!TryResolveL1SeedIndex(l1Voxel, actualVoxel, point, out var seedIndex))
            return 0;

        return FloodFillL1ReachableFaces(l1Voxel, seedIndex);
    }

    private bool ArePointsConnectedWithinL1(ulong l1Voxel, ulong fromVoxel, Vector3 fromPoint, ulong toVoxel, Vector3 toPoint)
    {
        if (Volume.IsEmpty(l1Voxel))
            return true;
        if (!TryResolveL1SeedIndex(l1Voxel, fromVoxel, fromPoint, out var fromSeedIndex) ||
            !TryResolveL1SeedIndex(l1Voxel, toVoxel,   toPoint,   out var toSeedIndex))
            return false;
        if (fromSeedIndex == toSeedIndex)
            return true;

        return FloodFillL1SeedConnectivity(l1Voxel, fromSeedIndex, toSeedIndex);
    }

    private ulong BuildPackedL1FaceConnectivity(ulong l1Voxel)
    {
        if (Volume.IsEmpty(l1Voxel))
        {
            ulong packed = 0;
            for (var face = 0; face < 6; ++face)
                packed = SetPackedReachableL1Faces(packed, face, L1AllFacesMask);
            return packed;
        }

        var totalCellCount = l2Desc.NumCellsTotal;
        var visited        = new bool[totalCellCount];
        ulong packedConnectivity = 0;

        for (ushort seedIndex = 0; seedIndex < totalCellCount; ++seedIndex)
        {
            if (visited[seedIndex])
                continue;

            var leafVoxel = VoxelMap.EncodeSubIndex(l1Voxel, seedIndex, 2);
            if (!Volume.IsEmpty(leafVoxel))
            {
                visited[seedIndex] = true;
                continue;
            }

            var reachableFaces = FloodFillL1Component(l1Voxel, seedIndex, visited, null, out _);
            if (reachableFaces == 0)
                continue;

            for (var face = 0; face < 6; ++face)
            {
                if (!HasL1Face(reachableFaces, face))
                    continue;

                var combinedReachable = (ushort)(GetPackedReachableL1Faces(packedConnectivity, face) | reachableFaces);
                packedConnectivity = SetPackedReachableL1Faces(packedConnectivity, face, combinedReachable);
            }
        }

        return packedConnectivity;
    }

    private ushort FloodFillL1ReachableFaces(ulong l1Voxel, ushort seedIndex) => FloodFillL1Component(l1Voxel, seedIndex, null, null, out _);

    private bool FloodFillL1SeedConnectivity(ulong l1Voxel, ushort fromSeedIndex, ushort toSeedIndex)
    {
        FloodFillL1Component(l1Voxel, fromSeedIndex, null, toSeedIndex, out var reachedTarget);
        return reachedTarget;
    }

    private ushort FloodFillL1Component(ulong l1Voxel, ushort seedIndex, bool[]? visited, ushort? targetSeedIndex, out bool reachedTarget)
    {
        reachedTarget = false;
        visited ??= new bool[l2Desc.NumCellsTotal];

        if (visited[seedIndex])
        {
            reachedTarget = targetSeedIndex == seedIndex;
            return 0;
        }

        var seedLeaf = VoxelMap.EncodeSubIndex(l1Voxel, seedIndex, 2);
        if (!Volume.IsEmpty(seedLeaf))
        {
            visited[seedIndex] = true;
            return 0;
        }

        Queue<ushort> frontier = new();
        frontier.Enqueue(seedIndex);
        visited[seedIndex] = true;

        ushort reachableFaces = 0;
        while (frontier.TryDequeue(out var currentIndex))
        {
            if (targetSeedIndex == currentIndex)
                reachedTarget = true;

            var (x, y, z) = l2Desc.IndexToVoxel(currentIndex);
            if (y == 0)
                reachableFaces |= L1FaceBit(L1FaceNegY);
            if (y == l2Desc.NumCellsY - 1)
                reachableFaces |= L1FaceBit(L1FacePosY);
            if (x == 0)
                reachableFaces |= L1FaceBit(L1FaceNegX);
            if (x == l2Desc.NumCellsX - 1)
                reachableFaces |= L1FaceBit(L1FacePosX);
            if (z == 0)
                reachableFaces |= L1FaceBit(L1FaceNegZ);
            if (z == l2Desc.NumCellsZ - 1)
                reachableFaces |= L1FaceBit(L1FacePosZ);

            for (var dir = 0; dir < 6; ++dir)
            {
                var dx = dir == L1FaceNegX ? -1 : dir == L1FacePosX ? 1 : 0;
                var dy = dir == L1FaceNegY ? -1 : dir == L1FacePosY ? 1 : 0;
                var dz = dir == L1FaceNegZ ? -1 : dir == L1FacePosZ ? 1 : 0;
                var nx = x + dx;
                var ny = y + dy;
                var nz = z + dz;
                if (!l2Desc.InBounds(nx, ny, nz))
                    continue;

                var neighbourIndex = l2Desc.VoxelToIndex(nx, ny, nz);
                if (visited[neighbourIndex])
                    continue;

                var neighbourLeaf = VoxelMap.EncodeSubIndex(l1Voxel, neighbourIndex, 2);
                if (!Volume.IsEmpty(neighbourLeaf))
                {
                    visited[neighbourIndex] = true;
                    continue;
                }

                visited[neighbourIndex] = true;
                frontier.Enqueue(neighbourIndex);
            }
        }

        return reachableFaces;
    }

    private bool TryResolveL1SeedIndex(ulong l1Voxel, ulong actualVoxel, Vector3 point, out ushort seedIndex)
    {
        seedIndex = 0;

        if (TryExtractL2IndexWithinL1(actualVoxel, l1Voxel, out seedIndex))
            return true;

        seedIndex = ResolvePointL2IndexWithinL1(l1Voxel, point);
        if (Volume.IsEmpty(VoxelMap.EncodeSubIndex(l1Voxel, seedIndex, 2)))
            return true;

        return TryFindNearestEmptyL1SeedIndex(l1Voxel, point, out seedIndex);
    }

    private static bool TryExtractL2IndexWithinL1(ulong voxel, ulong expectedL1, out ushort l2Index)
    {
        var temp    = voxel;
        var l0Index = VoxelMap.DecodeIndex(ref temp);
        var l1Index = VoxelMap.DecodeIndex(ref temp);
        l2Index     = VoxelMap.DecodeIndex(ref temp);
        if (l2Index == VoxelMap.INDEX_LEVEL_MASK)
            return false;

        var l1Voxel = VoxelMap.EncodeIndex(l1Index);
        l1Voxel = VoxelMap.EncodeIndex(l0Index, l1Voxel);
        return l1Voxel == expectedL1;
    }

    private ushort ResolvePointL2IndexWithinL1(ulong l1Voxel, Vector3 point)
    {
        var bounds   = Volume.VoxelBounds(l1Voxel, 0);
        var span     = bounds.max - bounds.min;
        var clamped  = Vector3.Clamp(point, bounds.min, bounds.max - new Vector3(SCORE_EPSILON));
        var relative = clamped - bounds.min;
        var x        = span.X > SCORE_EPSILON ? Math.Clamp((int)(relative.X / span.X * l2Desc.NumCellsX), 0, l2Desc.NumCellsX - 1) : 0;
        var y        = span.Y > SCORE_EPSILON ? Math.Clamp((int)(relative.Y / span.Y * l2Desc.NumCellsY), 0, l2Desc.NumCellsY - 1) : 0;
        var z        = span.Z > SCORE_EPSILON ? Math.Clamp((int)(relative.Z / span.Z * l2Desc.NumCellsZ), 0, l2Desc.NumCellsZ - 1) : 0;
        return l2Desc.VoxelToIndex(x, y, z);
    }

    private bool TryFindNearestEmptyL1SeedIndex(ulong l1Voxel, Vector3 point, out ushort seedIndex)
    {
        seedIndex = 0;
        var bestDistance = float.MaxValue;
        var found        = false;

        for (ushort currentIndex = 0; currentIndex < l2Desc.NumCellsTotal; ++currentIndex)
        {
            var leafVoxel = VoxelMap.EncodeSubIndex(l1Voxel, currentIndex, 2);
            if (!Volume.IsEmpty(leafVoxel))
                continue;

            var center           = ResolveVoxelCenter(leafVoxel);
            var distanceSquared  = Vector3.DistanceSquared(center, point);
            if (distanceSquared + SCORE_EPSILON >= bestDistance)
                continue;

            bestDistance = distanceSquared;
            seedIndex    = currentIndex;
            found        = true;
        }

        return found;
    }

    private ulong ResolveRepresentativeL1Voxel(ulong voxel, Vector3 referencePoint)
    {
        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1Voxel;

        var l0Index = ExtractL0Index(voxel);
        var bounds = Volume.VoxelBounds(voxel, 0);
        var span = bounds.max - bounds.min;
        var clamped = Vector3.Clamp(referencePoint, bounds.min, bounds.max - new Vector3(SCORE_EPSILON));
        var relative = clamped - bounds.min;
        var l1x = span.X > SCORE_EPSILON ? Math.Clamp((int)(relative.X / span.X * l1Desc.NumCellsX), 0, l1Desc.NumCellsX - 1) : 0;
        var l1y = span.Y > SCORE_EPSILON ? Math.Clamp((int)(relative.Y / span.Y * l1Desc.NumCellsY), 0, l1Desc.NumCellsY - 1) : 0;
        var l1z = span.Z > SCORE_EPSILON ? Math.Clamp((int)(relative.Z / span.Z * l1Desc.NumCellsZ), 0, l1Desc.NumCellsZ - 1) : 0;
        return VoxelMap.EncodeIndex(l0Index, VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1x, l1y, l1z)));
    }

    private bool IsVoxelInsidePathConstraint(ulong voxel)
    {
        if (l1PathSet == null)
            return true;

        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1PathSet.Contains(l1Voxel);

        return l0PathSet?.Contains(ExtractL0Index(voxel)) ?? false;
    }

    private bool TryGetVoxelCorridorDistance(ulong voxel, out int distance)
    {
        distance = int.MaxValue;
        if (l1CorridorDistance == null)
            return false;

        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1CorridorDistance.TryGetValue(l1Voxel, out distance);

        return l0CorridorDistance?.TryGetValue(ExtractL0Index(voxel), out distance) ?? false;
    }

    private bool TryGetVoxelL1DistanceFloor(ulong voxel, out float distance)
    {
        distance = float.MaxValue;
        if (l1DistanceField == null)
            return false;

        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1DistanceField.TryGetValue(l1Voxel, out distance);

        return l0DistanceField?.TryGetValue(ExtractL0Index(voxel), out distance) ?? false;
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

    private void VisitL1Neighbours(ulong voxel, Action<ulong> visitor)
    {
        var t     = voxel;
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
            visitor(neighbour);
        }
    }

    private void ComputeL1DistanceField(ulong goalVoxel, Vector3 goalPoint)
    {
        var goalL1 = ResolveRepresentativeL1Voxel(goalVoxel, goalPoint);
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
                if (!HasTraversableL1FaceTransition(current, neighbour, dx, dy, dz))
                    continue;

                var edgeCost = dx != 0 ? l1Desc.CellSize.X : dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                l1DistanceField[neighbour] = currentDist + edgeCost;
                frontier.Enqueue(neighbour);
            }
        }

        l0DistanceField = new();
        foreach (var (l1Voxel, l1Dist) in l1DistanceField)
        {
            var l0Index = ExtractL0Index(l1Voxel);
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

        var expanded = 0;
        while (frontier.TryDequeue(out var current, out var queuedDistance) && expanded < L1_DISTANCE_FIELD_BUDGET)
        {
            if (!l1DistanceField.TryGetValue(current, out var currentDistance) || queuedDistance > currentDistance + SCORE_EPSILON)
                continue;
            expanded++;

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

                if (!Volume.IsEmpty(neighbour))
                    continue;
                if (!HasTraversableL1FaceTransition(current, neighbour, dx, dy, dz))
                    continue;

                var edgeCost          = dx != 0 ? l1Desc.CellSize.X : dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                var neighbourDistance = currentDistance + edgeCost;
                if (l1DistanceField.TryGetValue(neighbour, out var existingDistance) && existingDistance <= neighbourDistance + SCORE_EPSILON)
                    continue;

                l1DistanceField[neighbour] = neighbourDistance;
                frontier.Enqueue(neighbour, neighbourDistance);
            }
        }

        l0DistanceField = new();
        foreach (var (l1Voxel, l1Dist) in l1DistanceField)
        {
            var l0Index = ExtractL0Index(l1Voxel);
            if (!l0DistanceField.TryGetValue(l0Index, out var existing) || l1Dist < existing)
                l0DistanceField[l0Index] = l1Dist;
        }
    }

    private float CalculateL1TransitionCost(ulong fromL1, ulong toL1)
    {
        var fromTemp = fromL1;
        var fromL0   = l0Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref fromTemp));
        var fromL1c  = l1Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref fromTemp));
        var toTemp   = toL1;
        var toL0     = l0Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref toTemp));
        var toL1c    = l1Desc.IndexToVoxel(VoxelMap.DecodeIndex(ref toTemp));

        var globalFromX = fromL0.x * l1Desc.NumCellsX + fromL1c.x;
        var globalFromY = fromL0.y * l1Desc.NumCellsY + fromL1c.y;
        var globalFromZ = fromL0.z * l1Desc.NumCellsZ + fromL1c.z;
        var globalToX   = toL0.x   * l1Desc.NumCellsX + toL1c.x;
        var globalToY   = toL0.y   * l1Desc.NumCellsY + toL1c.y;
        var globalToZ   = toL0.z   * l1Desc.NumCellsZ + toL1c.z;

        return MathF.Abs(globalFromX - globalToX) * l1Desc.CellSize.X +
               MathF.Abs(globalFromY - globalToY) * l1Desc.CellSize.Y +
               MathF.Abs(globalFromZ - globalToZ) * l1Desc.CellSize.Z;
    }

    private HashSet<ulong>? SearchL1CoarsePath(ulong fromVoxel, Vector3 fromPoint, ulong toVoxel, Vector3 toPoint)
    {
        var fromL1 = ResolveRepresentativeL1Voxel(fromVoxel, fromPoint);
        var toL1   = ResolveRepresentativeL1Voxel(toVoxel, toPoint);
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
        var fromL1        = ResolveRepresentativeL1Voxel(fromVoxel, fromPoint);
        var toL1          = ResolveRepresentativeL1Voxel(toVoxel, toPoint);
        var strictResult  = TrySearchL1BestEffortPath(fromL1, toL1, fromVoxel, fromPoint, toVoxel, toPoint, false);
        if (strictResult.ReachedGoal)
            return strictResult;

        if (Volume.IsEmpty(toL1))
            return strictResult;

        var relaxedResult = TrySearchL1BestEffortPath(fromL1, toL1, fromVoxel, fromPoint, toVoxel, toPoint, true);
        return IsBetterL1BestEffortResult(relaxedResult, strictResult) ? relaxedResult : strictResult;
    }

    private void BuildL1Corridor(HashSet<ulong> pathSet, int corridorRadius)
    {
        currentL1CorridorRadius = corridorRadius;
        l0PathSet = new();
        foreach (var l1Voxel in pathSet)
            l0PathSet.Add(ExtractL0Index(l1Voxel));

        if (corridorRadius <= 0)
        {
            l1PathSet          = pathSet;
            l1CorridorDistance = null;
            l0CorridorDistance = null;
            return;
        }

        l1PathSet = null;
        var distances = new Dictionary<ulong, int>(pathSet.Count * 2);
        var frontier = new Queue<ulong>();
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
            var l0Index = ExtractL0Index(l1Voxel);
            if (!l0CorridorDistance.TryGetValue(l0Index, out var existing) || distance < existing)
                l0CorridorDistance[l0Index] = distance;
        }
    }

    private HashSet<ulong> BuildL1LateralExplorationArea(ulong fromVoxel, Vector3 fromPos, Vector3 toPos, int attempt)
    {
        var startL1 = ResolveRepresentativeL1Voxel(fromVoxel, fromPos);
        var horizontalDelta = new Vector2(toPos.X - fromPos.X, toPos.Z - fromPos.Z);
        var horizontalDistance = horizontalDelta.Length();
        var forward = horizontalDistance > SCORE_EPSILON ? horizontalDelta / horizontalDistance : Vector2.UnitX;
        var right = new Vector2(-forward.Y, forward.X);
        var l1Horizontal = MathF.Max(l1Desc.CellSize.X, l1Desc.CellSize.Z);
        var l1Vertical = l1Desc.CellSize.Y;
        var distanceInL1Cells = MathF.Max(1f, horizontalDistance / MathF.Max(l1Horizontal, SCORE_EPSILON));
        var widthScale = LONG_RANGE_LATERAL_WIDTH_SCALE_BASE + LONG_RANGE_LATERAL_WIDTH_SCALE_STEP * attempt;
        var growthScale = LONG_RANGE_LATERAL_GROWTH_SCALE_BASE + LONG_RANGE_LATERAL_GROWTH_SCALE_STEP * attempt;
        var halfWidth = MathF.Max
        (
            l1Horizontal * (LONG_RANGE_LATERAL_MIN_HALF_WIDTH_L1_CELLS + attempt * LONG_RANGE_LATERAL_MIN_HALF_WIDTH_ATTEMPT_CELLS),
            horizontalDistance * widthScale
        );
        var forwardLimit = MathF.Max
        (
            horizontalDistance * (1f + attempt * LONG_RANGE_LATERAL_FORWARD_ATTEMPT_SCALE) + l1Horizontal * LONG_RANGE_LATERAL_FORWARD_SLACK_L1_CELLS,
            l1Horizontal * LONG_RANGE_LATERAL_MIN_FORWARD_L1_CELLS
        );
        var backwardLimit = MathF.Max
        (
            l1Horizontal * (LONG_RANGE_LATERAL_BACKWARD_SLACK_L1_CELLS + attempt * LONG_RANGE_LATERAL_BACKWARD_ATTEMPT_CELLS),
            horizontalDistance * (LONG_RANGE_LATERAL_BACKWARD_SCALE + attempt * LONG_RANGE_LATERAL_BACKWARD_SCALE_STEP)
        );
        var downwardLimit = MathF.Max(MathF.Max(0f, fromPos.Y - toPos.Y) + l1Vertical * LONG_RANGE_LATERAL_DOWNWARD_SLACK_L1_CELLS, l1Vertical * LONG_RANGE_LATERAL_MIN_VERTICAL_L1_CELLS);
        var upwardLimit = MathF.Max(MathF.Max(0f, toPos.Y - fromPos.Y) + l1Vertical * LONG_RANGE_LATERAL_UPWARD_SLACK_L1_CELLS, l1Vertical * LONG_RANGE_LATERAL_MIN_VERTICAL_L1_CELLS);
        var maxCells = LONG_RANGE_LATERAL_AREA_BASE_CELLS +
                       attempt * LONG_RANGE_LATERAL_AREA_STEP_CELLS +
                       (int)(distanceInL1Cells * LONG_RANGE_LATERAL_AREA_DISTANCE_CELLS_SCALE);

        HashSet<ulong> result = [startL1];
        Queue<ulong> frontier = new();
        frontier.Enqueue(startL1);

        while (frontier.TryDequeue(out var current) && result.Count < maxCells)
        {
            VisitL1Neighbours
            (
                current,
                neighbour =>
                {
                    if (result.Count >= maxCells || result.Contains(neighbour))
                        return;

                    if (!IsInsideL1LateralExplorationWindow(neighbour, fromPos, forward, right, halfWidth, growthScale, forwardLimit, backwardLimit, upwardLimit, downwardLimit))
                        return;

                    result.Add(neighbour);
                    frontier.Enqueue(neighbour);
                }
            );
        }

        return result;
    }

    private void ApplyL1AreaConstraint(HashSet<ulong> area)
    {
        currentL1CorridorRadius = 0;
        l1PathSet = area;
        l1CorridorDistance = null;
        l0CorridorDistance = null;
        l0PathSet = new();
        foreach (var l1Voxel in area)
            l0PathSet.Add(ExtractL0Index(l1Voxel));
    }

    private void ClearL1AreaConstraint()
    {
        currentL1CorridorRadius = 0;
        l1PathSet = null;
        l0PathSet = null;
        l1CorridorDistance = null;
        l0CorridorDistance = null;
    }

    private bool IsInsideL1LateralExplorationWindow
    (
        ulong   l1Voxel,
        Vector3 origin,
        Vector2 forward,
        Vector2 right,
        float   baseHalfWidth,
        float   forwardGrowth,
        float   forwardLimit,
        float   backwardLimit,
        float   upwardLimit,
        float   downwardLimit
    )
    {
        var center = ResolveVoxelCenter(l1Voxel);
        var relative = center - origin;
        if (relative.Y > upwardLimit || relative.Y < -downwardLimit)
            return false;

        var horizontal = new Vector2(relative.X, relative.Z);
        var forwardDistance = Vector2.Dot(horizontal, forward);
        if (forwardDistance < -backwardLimit || forwardDistance > forwardLimit)
            return false;

        var lateralDistance = MathF.Abs(Vector2.Dot(horizontal, right));
        var allowedHalfWidth = baseHalfWidth +
                               MathF.Max(forwardDistance, 0f) * forwardGrowth +
                               MathF.Sqrt(MathF.Max(forwardDistance, 0f)) * LONG_RANGE_LATERAL_FORWARD_SQRT_WIDTH_SCALE;
        return lateralDistance <= allowedHalfWidth;
    }

    private float ResolveLongRangeHeuristicWeight(int corridorRadius)
    {
        var t = Math.Clamp((float)corridorRadius / LONG_RANGE_L1_CORRIDOR_RELAX_STEPS, 0f, 1f);
        return LONG_RANGE_HEURISTIC_WEIGHT + (LONG_RANGE_L1_CORRIDOR_MAX_RADIUS_HEURISTIC_WEIGHT - LONG_RANGE_HEURISTIC_WEIGHT) * t;
    }

    private float ResolveLongRangeLateralHeuristicWeight(int attempt)
    {
        var t = Math.Clamp((float)attempt / (LONG_RANGE_LATERAL_EXPLORATION_ATTEMPTS - 1), 0f, 1f);
        return LONG_RANGE_LATERAL_HEURISTIC_WEIGHT_BASE + (LONG_RANGE_LATERAL_HEURISTIC_WEIGHT_MIN - LONG_RANGE_LATERAL_HEURISTIC_WEIGHT_BASE) * t;
    }

    private LongRangeLateralBias BuildLongRangeLateralBias(Vector3 fromPos, Vector3 toPos, int attempt)
    {
        var horizontalDelta    = new Vector2(toPos.X - fromPos.X, toPos.Z - fromPos.Z);
        var horizontalDistance = horizontalDelta.Length();
        var forward            = TryNormalize(horizontalDelta, out var normalizedForward) ? normalizedForward : Vector2.UnitX;
        var right              = new Vector2(-forward.Y, forward.X);
        var verticalDrop       = MathF.Max(fromPos.Y - toPos.Y, 0f);
        var leafVertical       = MathF.Max(l2Desc.CellSize.Y, SCORE_EPSILON);
        var preferDescending   = verticalDrop >= leafVertical * LONG_RANGE_LATERAL_DESCENT_ENABLE_MIN_DROP_LEAF_CELLS;
        var heightPriority     = 1f + Math.Clamp(verticalDrop / (leafVertical * LONG_RANGE_LATERAL_DESCENT_PRIORITY_DROP_LEAF_CELLS), 0f, LONG_RANGE_LATERAL_DESCENT_PRIORITY_MAX_BONUS);
        var directionalPenalty = MathF.Max(LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_MIN, 1f - attempt * LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_ATTEMPT_STEP);
        return new(true, fromPos, forward, right, horizontalDistance, preferDescending, heightPriority, directionalPenalty);
    }

    private int ResolveLongRangeL1BestEffortStepBudget(Vector3 fromPos, Vector3 toPos, bool includeNonEmpty)
    {
        var horizontalL1Cells = HorizontalDistanceXZ(fromPos, toPos) / MathF.Max(MathF.Max(l1Desc.CellSize.X, l1Desc.CellSize.Z), SCORE_EPSILON);
        var verticalL1Cells   = MathF.Abs(fromPos.Y - toPos.Y) / MathF.Max(l1Desc.CellSize.Y, SCORE_EPSILON);
        var estimatedCells    = horizontalL1Cells + verticalL1Cells * LONG_RANGE_L1_BEST_EFFORT_VERTICAL_DISTANCE_BUDGET_SCALE;
        var distanceBudget    = (int)(estimatedCells * LONG_RANGE_L1_BEST_EFFORT_DISTANCE_BUDGET_PER_CELL);
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
        var maxL1Extent    = MathF.Max(l1Desc.CellSize.X, MathF.Max(l1Desc.CellSize.Y, l1Desc.CellSize.Z));
        var gapCells       = bestDistance / MathF.Max(maxL1Extent, SCORE_EPSILON);
        var dynamicRadius  = LONG_RANGE_L1_GUIDED_FULLSEARCH_BASE_CORRIDOR_RADIUS + (int)MathF.Ceiling(gapCells * LONG_RANGE_L1_GUIDED_FULLSEARCH_GAP_RADIUS_SCALE);
        return Math.Clamp(dynamicRadius, LONG_RANGE_L1_GUIDED_FULLSEARCH_BASE_CORRIDOR_RADIUS, LONG_RANGE_L1_GUIDED_FULLSEARCH_MAX_CORRIDOR_RADIUS);
    }

    private L1BestEffortSearchResult TrySearchL1BestEffortPath
    (
        ulong   fromL1,
        ulong   toL1,
        ulong   fromVoxel,
        Vector3 fromPoint,
        ulong   toVoxel,
        Vector3 toPoint,
        bool    includeNonEmpty
    )
    {
        var tGoal = toL1;
        var gL0   = VoxelMap.DecodeIndex(ref tGoal);
        var gL1   = VoxelMap.DecodeIndex(ref tGoal);
        var gL0c  = l0Desc.IndexToVoxel(gL0);
        var gL1c  = l1Desc.IndexToVoxel(gL1);
        var rootMin = Volume.RootTile.BoundsMin;
        var coarseBias = BuildLongRangeLateralBias(fromPoint, toPoint, 0);
        var stepBudget = ResolveLongRangeL1BestEffortStepBudget(fromPoint, toPoint, includeNonEmpty);
        var totalBudget = stepBudget;
        var startReachableFaces = GetReachableL1FacesFromPoint(fromL1, fromVoxel, fromPoint);
        var goalReachableFaces  = GetReachableL1FacesFromPoint(toL1,   toVoxel,   toPoint);
        var directGoalConnected = fromL1 == toL1 && ArePointsConnectedWithinL1(fromL1, fromVoxel, fromPoint, toVoxel, toPoint);
        var goalFacePenalty     = MathF.Min(l1Desc.CellSize.X, MathF.Min(l1Desc.CellSize.Y, l1Desc.CellSize.Z));

        Vector3 CoarseCellCenter((int x, int y, int z) l0c, (int x, int y, int z) l1c)
            => rootMin +
               new Vector3(l0c.x * l0Desc.CellSize.X, l0c.y * l0Desc.CellSize.Y, l0c.z * l0Desc.CellSize.Z) +
               new Vector3((l1c.x + 0.5f) * l1Desc.CellSize.X, (l1c.y + 0.5f) * l1Desc.CellSize.Y, (l1c.z + 0.5f) * l1Desc.CellSize.Z);

        float H((int x, int y, int z) l0c, (int x, int y, int z) l1c)
        {
            var dx = (l0c.x - gL0c.x) * l0Desc.CellSize.X + (l1c.x - gL1c.x) * l1Desc.CellSize.X;
            var dy = (l0c.y - gL0c.y) * l0Desc.CellSize.Y + (l1c.y - gL1c.y) * l1Desc.CellSize.Y;
            var dz = (l0c.z - gL0c.z) * l0Desc.CellSize.Z + (l1c.z - gL1c.z) * l1Desc.CellSize.Z;
            return MathF.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        float StateHeuristic(L1TraversalState state)
        {
            var temp  = state.Voxel;
            var l0Idx = VoxelMap.DecodeIndex(ref temp);
            var l1Idx = VoxelMap.DecodeIndex(ref temp);
            var l0c   = l0Desc.IndexToVoxel(l0Idx);
            var l1c   = l1Desc.IndexToVoxel(l1Idx);
            var position = state.Voxel == fromL1
                               ? fromPoint
                               : state.Voxel == toL1
                                   ? toPoint
                                   : CoarseCellCenter(l0c, l1c);
            var baseH = coarseBias.Enabled
                            ? ComputeLongRangeLateralHeuristic(position, state.Voxel, coarseBias, toPoint)
                            : H(l0c, l1c);
            if (state.Voxel != toL1)
                return baseH;
            if (state.EntryFace == L1FaceInside)
                return directGoalConnected ? 0f : goalFacePenalty;

            return HasL1Face(goalReachableFaces, state.EntryFace) ? 0f : goalFacePenalty;
        }

        var startState = new L1TraversalState(fromL1, L1FaceInside);
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
            var phaseExpanded = 0;
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
                var currentPosition = current.Voxel == fromL1
                                          ? fromPoint
                                          : current.Voxel == toL1
                                              ? toPoint
                                              : CoarseCellCenter(l0c, l1c);

                if (h + SCORE_EPSILON < bestDistance || MathF.Abs(h - bestDistance) <= SCORE_EPSILON && cg < gScore.GetValueOrDefault(bestState, float.MaxValue))
                {
                    bestState    = current;
                    bestDistance = h;
                }

                var reachedGoal = current.Voxel == toL1 &&
                                  (current.EntryFace == L1FaceInside
                                       ? directGoalConnected
                                       : HasL1Face(goalReachableFaces, current.EntryFace));

                if (reachedGoal)
                {
                    var orderedPath = ReconstructL1OrderedPath(cameFrom, current);
                    result = new(new HashSet<ulong>(orderedPath), orderedPath, true, expanded, 0f, totalBudget);
                    return true;
                }

                var exitFaceMask = current.EntryFace == L1FaceInside
                                       ? startReachableFaces
                                       : GetReachableL1FacesFromEntry(current.Voxel, current.EntryFace);
                if (exitFaceMask == 0)
                    continue;

                for (var dir = 0; dir < 6; dir++)
                {
                    if (!HasL1Face(exitFaceMask, dir))
                        continue;

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

                    var nextState = new L1TraversalState(neighbour, (byte)OppositeFace(dir));
                    if (closed.Contains(nextState))
                        continue;
                    if (!HasTraversableL1FaceTransition(current.Voxel, neighbour, dx, dy, dz))
                        continue;

                    var isEmpty = Volume.IsEmpty(neighbour);
                    if (!isEmpty && !CanTraverseMixedL1Cell(includeNonEmpty, neighbour, toL1))
                        continue;
                    if (!isEmpty)
                    {
                        var entryReachableFaces = GetReachableL1FacesFromEntry(neighbour, nextState.EntryFace);
                        var canReachGoal        = neighbour == toL1 && HasL1Face(goalReachableFaces, nextState.EntryFace);
                        if (entryReachableFaces == 0 && !canReachGoal)
                            continue;
                    }

                    var neighbourPosition = neighbour == toL1
                                                ? toPoint
                                                : CoarseCellCenter((nl0x, nl0y, nl0z), (nl1x, nl1y, nl1z));
                    var edgeCost          = dx != 0 ? l1Desc.CellSize.X : dy != 0 ? l1Desc.CellSize.Y : l1Desc.CellSize.Z;
                    var mixedPenalty      = isEmpty ? 0f : edgeCost * LONG_RANGE_L1_BEST_EFFORT_MIXED_CELL_PENALTY_SCALE;
                    var traversalPenalty  = coarseBias.Enabled
                                                ? CalculateLongRangeLateralTraversalPenalty(current.Voxel, currentPosition, neighbour, neighbourPosition, coarseBias, toPoint)
                                                : 0f;
                    var tentativeG        = cg + edgeCost + mixedPenalty + traversalPenalty;

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
        if (expanded >= stepBudget && openQ.Count > 0 && bestDistance <= goalCaptureThreshold)
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
        return new(new HashSet<ulong>(bestOrderedPath), bestOrderedPath, false, expanded, bestDistance, totalBudget);
    }

    private static HashSet<ulong> ReconstructL1PathSet(Dictionary<ulong, ulong> cameFrom, ulong endNode)
    {
        HashSet<ulong> pathSet = [];
        var node = endNode;

        while (true)
        {
            pathSet.Add(node);
            if (!cameFrom.TryGetValue(node, out var parent))
                break;
            node = parent;
        }

        return pathSet;
    }

    private static List<ulong> ReconstructL1OrderedPath(Dictionary<ulong, ulong> cameFrom, ulong endNode)
    {
        List<ulong> path = [];
        var node = endNode;

        while (true)
        {
            path.Add(node);
            if (!cameFrom.TryGetValue(node, out var parent))
                break;
            node = parent;
        }

        path.Reverse();
        return path;
    }

    private static List<ulong> ReconstructL1OrderedPath(Dictionary<L1TraversalState, L1TraversalState> cameFrom, L1TraversalState endState)
    {
        List<ulong> path = [];
        var state = endState;

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

    private static List<(ulong voxel, Vector3 p)> MergePathSegments(List<(ulong voxel, Vector3 p)> head, List<(ulong voxel, Vector3 p)> tail)
    {
        if (head.Count == 0)
            return tail;
        if (tail.Count == 0)
            return head;

        List<(ulong voxel, Vector3 p)> merged = new(head.Count + tail.Count);
        merged.AddRange(head);

        var tailStartIndex = head[^1].voxel == tail[0].voxel && Vector3.DistanceSquared(head[^1].p, tail[0].p) <= SCORE_EPSILON * SCORE_EPSILON ? 1 : 0;
        for (var i = tailStartIndex; i < tail.Count; ++i)
            merged.Add(tail[i]);

        return merged;
    }

    private FlightPathDebugPayload BuildCoarsePathDebugPayload(IReadOnlyList<ulong> orderedPath, Vector3 fromPos, Vector3 toPos, bool reachedGoal)
    {
        List<FlightCoarsePathDebugNode> result = new(orderedPath.Count);
        for (var i = 0; i < orderedPath.Count; ++i)
        {
            var voxel = orderedPath[i];
            var point = i == 0
                            ? fromPos
                            : i == orderedPath.Count - 1 && reachedGoal
                                ? toPos
                                : ResolveVoxelCenter(voxel);
            result.Add(new(i, voxel, point));
        }

        return new()
        {
            Waypoints  = [],
            CoarsePath = result,
            ProxyDebug = pendingLongRangeProxyDebug
        };
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
                if (!HasTraversableL1FaceTransition(current, neighbour, dx, dy, dz))
                    continue;
                var isEmpty = Volume.IsEmpty(neighbour);
                if (!isEmpty && !CanTraverseMixedL1Cell(includeNonEmpty, neighbour, toL1))
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
        var h = longRangeLateralBias.Enabled ? ComputeLongRangeLateralHeuristic(position, voxel) : Vector3.Distance(position, goalPos);
        if (TryGetVoxelL1DistanceFloor(voxel, out var l1Dist))
            h = MathF.Max(h, l1Dist);
        return h;
    }

    private float ComputeLongRangeLateralHeuristic(Vector3 position, ulong voxel)
        => ComputeLongRangeLateralHeuristic(position, voxel, longRangeLateralBias, goalPos);

    private float ComputeLongRangeLateralHeuristic(Vector3 position, ulong voxel, LongRangeLateralBias bias, Vector3 goal)
    {
        var horizontalGoalDist  = HorizontalDistanceXZ(position, goal);
        var aboveGoal           = MathF.Max(position.Y - goal.Y, 0f);
        var belowGoal           = MathF.Max(goal.Y - position.Y, 0f);
        var relativeFromStart   = position - bias.Start;
        var horizontalFromStart = new Vector2(relativeFromStart.X, relativeFromStart.Z);
        var forwardProgress     = Vector2.Dot(horizontalFromStart, bias.Forward);
        var lateralOffset       = MathF.Abs(Vector2.Dot(horizontalFromStart, bias.Right));
        var forwardRemaining    = MathF.Max(0f, bias.HorizontalDistance - forwardProgress);
        var horizontalTerm      = horizontalGoalDist * LONG_RANGE_LATERAL_GOAL_DISTANCE_HEURISTIC_WEIGHT +
                                  forwardRemaining * LONG_RANGE_LATERAL_FORWARD_REMAINING_HEURISTIC_WEIGHT +
                                  lateralOffset * LONG_RANGE_LATERAL_SIDE_OFFSET_HEURISTIC_WEIGHT;

        float verticalTerm;
        if (bias.PreferDescending)
        {
            verticalTerm = aboveGoal * (LONG_RANGE_LATERAL_DESCENT_ABOVE_GOAL_HEURISTIC_WEIGHT * bias.HeightPriority) +
                           belowGoal * LONG_RANGE_LATERAL_DESCENT_BELOW_GOAL_HEURISTIC_WEIGHT;
        }
        else
        {
            verticalTerm = belowGoal * LONG_RANGE_LATERAL_ASCENT_BELOW_GOAL_HEURISTIC_WEIGHT +
                           aboveGoal * LONG_RANGE_LATERAL_ASCENT_ABOVE_GOAL_HEURISTIC_WEIGHT;
        }

        if (bias.PreferDescending &&
            aboveGoal >= l2Desc.CellSize.Y * LONG_RANGE_LATERAL_DOWNWARD_OPENING_MIN_ABOVE_GOAL_LEAF_CELLS &&
            HasDownwardOpening(voxel))
        {
            horizontalTerm *= LONG_RANGE_LATERAL_DOWNWARD_OPENING_HORIZONTAL_HEURISTIC_SCALE;
            verticalTerm   *= LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_HEURISTIC_SCALE;
        }

        return horizontalTerm + verticalTerm;
    }

    private float CalculateLongRangeLateralTraversalPenalty(ulong parentVoxel, Vector3 parentPosition, ulong destinationVoxel, Vector3 destination)
        => CalculateLongRangeLateralTraversalPenalty(parentVoxel, parentPosition, destinationVoxel, destination, longRangeLateralBias, goalPos);

    private float CalculateLongRangeLateralTraversalPenalty
    (
        ulong               parentVoxel,
        Vector3             parentPosition,
        ulong               destinationVoxel,
        Vector3             destination,
        LongRangeLateralBias bias,
        Vector3             goal
    )
    {
        if (!bias.Enabled)
            return 0f;

        var step           = destination - parentPosition;
        var horizontalStep = new Vector2(step.X, step.Z);
        var horizontalStepLength = horizontalStep.Length();
        var forwardStep    = Vector2.Dot(horizontalStep, bias.Forward);
        var lateralStep    = MathF.Abs(Vector2.Dot(horizontalStep, bias.Right));
        var penalty        = 0f;

        if (forwardStep < -SCORE_EPSILON)
            penalty += -forwardStep * LONG_RANGE_LATERAL_REVERSE_STEP_PENALTY * bias.DirectionalPenaltyScale;

        if (bias.PreferDescending)
        {
            var climbStep          = MathF.Max(step.Y, 0f);
            var descentStep        = MathF.Max(-step.Y, 0f);
            var parentAboveGoal    = MathF.Max(parentPosition.Y - goal.Y, 0f);
            var destinationAboveGoal = MathF.Max(destination.Y - goal.Y, 0f);
            var descentTowardGoal  = MathF.Max(parentAboveGoal - destinationAboveGoal, 0f);
            var progressRoom = MathF.Max(forwardStep, 0f) * LONG_RANGE_LATERAL_FORWARD_PROGRESS_CREDIT +
                               descentStep * LONG_RANGE_LATERAL_DESCENT_PROGRESS_CREDIT;

            if (climbStep > SCORE_EPSILON)
                penalty += climbStep * LONG_RANGE_LATERAL_CLIMB_STEP_PENALTY * bias.HeightPriority;
            if (lateralStep > progressRoom + SCORE_EPSILON)
                penalty += (lateralStep - progressRoom) * LONG_RANGE_LATERAL_LATERAL_STALL_PENALTY * bias.DirectionalPenaltyScale;

            if (parentAboveGoal >= l2Desc.CellSize.Y * LONG_RANGE_LATERAL_DOWNWARD_OPENING_MIN_ABOVE_GOAL_LEAF_CELLS)
            {
                var parentHasDownwardOpening      = HasDownwardOpening(parentVoxel);
                var destinationHasDownwardOpening = HasDownwardOpening(destinationVoxel);

                if (destinationHasDownwardOpening)
                    penalty *= LONG_RANGE_LATERAL_DOWNWARD_OPENING_DESTINATION_PENALTY_SCALE;

                if (parentHasDownwardOpening)
                {
                    var requiredDescent = MathF.Min
                    (
                        parentAboveGoal,
                        l2Desc.CellSize.Y * LONG_RANGE_LATERAL_DOWNWARD_OPENING_REQUIRED_DESCENT_LEAF_CELLS
                    );

                    if (descentTowardGoal + SCORE_EPSILON < requiredDescent)
                    {
                        var missedDescent = requiredDescent - descentTowardGoal;
                        penalty += horizontalStepLength * LONG_RANGE_LATERAL_DOWNWARD_OPENING_MISSED_DESCENT_PENALTY * bias.HeightPriority;
                        penalty += missedDescent * LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_MISS_PENALTY * bias.HeightPriority;
                    }
                }
            }
        }
        else
        {
            var progressRoom = MathF.Max(forwardStep, 0f) * LONG_RANGE_LATERAL_FORWARD_PROGRESS_CREDIT;
            if (lateralStep > progressRoom + SCORE_EPSILON)
                penalty += (lateralStep - progressRoom) * LONG_RANGE_LATERAL_LATERAL_STALL_PENALTY * LONG_RANGE_LATERAL_NON_DESCENT_STALL_SCALE;
        }

        return penalty;
    }

    private bool HasDownwardOpening(ulong voxel)
    {
        if (verifiedDownwardOpeningCache.TryGetValue(voxel, out var cached))
            return cached == 1;

        var open = HasVerifiedVerticalAccessThroughFace(voxel, throughLowerFace: true);
        verifiedDownwardOpeningCache.TryAdd(voxel, open ? (byte)1 : (byte)2);
        return open;
    }

    private bool HasVerifiedTopEntry(ulong voxel)
    {
        if (verifiedTopEntryCache.TryGetValue(voxel, out var cached))
            return cached == 1;

        var open = HasVerifiedVerticalAccessThroughFace(voxel, throughLowerFace: false);
        verifiedTopEntryCache.TryAdd(voxel, open ? (byte)1 : (byte)2);
        return open;
    }

    private bool HasVerifiedVerticalAccessThroughFace(ulong voxel, bool throughLowerFace)
    {
        var wallMask = GetVoxelWallMask(voxel);
        if (throughLowerFace)
        {
            if ((wallMask & SearchWallNegY) != 0)
                return false;
        }
        else if ((wallMask & SearchWallPosY) != 0)
            return false;

        var (min, max)  = Volume.VoxelBounds(voxel, 0);
        var size        = max - min;
        var rootMinY    = Volume.RootTile.BoundsMin.Y;
        var rootMaxY    = Volume.RootTile.BoundsMax.Y;
        var epsX        = MathF.Max(SCORE_EPSILON, MathF.Min(size.X * 0.18f, l2Desc.CellSize.X * 1.25f));
        var epsY        = MathF.Max(SCORE_EPSILON, MathF.Min(size.Y * 0.18f, l2Desc.CellSize.Y * 1.25f));
        var epsZ        = MathF.Max(SCORE_EPSILON, MathF.Min(size.Z * 0.18f, l2Desc.CellSize.Z * 1.25f));
        var minX        = min.X + epsX;
        var maxX        = max.X - epsX;
        var minZ        = min.Z + epsZ;
        var maxZ        = max.Z - epsZ;
        var probeDepth  = MathF.Max(l2Desc.CellSize.Y * LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_LEAF_CELLS, size.Y * LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_HEIGHT_SCALE);
        var insideY     = throughLowerFace
                              ? min.Y + Math.Clamp(size.Y * 0.60f, epsY, MathF.Max(epsY, size.Y - epsY))
                              : max.Y - Math.Clamp(size.Y * 0.25f, epsY, MathF.Max(epsY, size.Y - epsY));
        var outsideY    = throughLowerFace
                              ? MathF.Max(min.Y - probeDepth, rootMinY + epsY)
                              : MathF.Min(max.Y + probeDepth, rootMaxY - epsY);

        if (throughLowerFace)
        {
            if (outsideY >= insideY - SCORE_EPSILON)
                return false;
        }
        else if (outsideY <= insideY + SCORE_EPSILON)
            return false;

        Span<Vector2> offsets = stackalloc Vector2[5];
        offsets[0] = Vector2.Zero;
        var offsetX = MathF.Max(0f, (maxX - minX) * 0.35f);
        var offsetZ = MathF.Max(0f, (maxZ - minZ) * 0.35f);
        offsets[1] = new(+offsetX, 0f);
        offsets[2] = new(-offsetX, 0f);
        offsets[3] = new(0f, +offsetZ);
        offsets[4] = new(0f, -offsetZ);

        var centerX = (minX + maxX) * 0.5f;
        var centerZ = (minZ + maxZ) * 0.5f;

        for (var i = 0; i < offsets.Length; ++i)
        {
            var x = Math.Clamp(centerX + offsets[i].X, minX, maxX);
            var z = Math.Clamp(centerZ + offsets[i].Y, minZ, maxZ);
            var inside = new Vector3(x, insideY,  z);
            var outside = new Vector3(x, outsideY, z);
            var from = throughLowerFace ? inside : outside;
            var to   = throughLowerFace ? outside : inside;
            var startLeaf = Volume.FindLeafVoxel(from);
            var endLeaf   = Volume.FindLeafVoxel(to);
            if (!startLeaf.empty || !endLeaf.empty)
                continue;
            if (VoxelSearch.LineOfSight(Volume, startLeaf.voxel, endLeaf.voxel, from, to))
                return true;
        }

        return false;
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

    private readonly record struct LongRangeLateralBias
    (
        bool    Enabled,
        Vector3 Start,
        Vector2 Forward,
        Vector2 Right,
        float   HorizontalDistance,
        bool    PreferDescending,
        float   HeightPriority,
        float   DirectionalPenaltyScale
    );

    private readonly record struct L1TraversalState(ulong Voxel, byte EntryFace);

    private readonly record struct L1BestEffortSearchResult
    (
        HashSet<ulong> PathSet,
        IReadOnlyList<ulong> OrderedPath,
        bool           ReachedGoal,
        int            ExpandedNodes,
        float          BestDistance,
        int            StepBudget
    );

    private readonly record struct FlightPushProbeResult(float Clearance, Vector3 Endpoint);
    private readonly record struct FlightPushHorizontalCandidate(Vector3 Direction, float Clearance, float Score);
    private static readonly bool EnableLongRangeGlobalFallback = false;
    private static readonly bool DebugReturnLongRangeBestEffortPath = false;

    private const float SCORE_EPSILON                                = 0.00001f;
    private const int   DEFAULT_MAX_SEARCH_STEPS                     = 1_0000_0000;
    private const int   RAYCAST_SEARCH_STEP_BUDGET                   = 200000;
    private const int   GUIDED_CORRIDOR_SEARCH_STEP_BUDGET           = 2_000_000;
    private const int   GUIDED_CORRIDOR_EARLY_ABORT_MIN_VISITED      = 80_000;
    private const int   GUIDED_CORRIDOR_EARLY_ABORT_HARD_VISITED_THRESHOLD = 220_000;
    private const int   GUIDED_CORRIDOR_EARLY_ABORT_STALL_WINDOW     = 120_000;
    private const int   GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_HARD_VISITED_THRESHOLD = 120_000;
    private const int   GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_STALL_WINDOW = 60_000;
    private const int   MAX_ANCESTOR_LOOK_BACK                       = 6;
    private const int   RAYCAST_PARALLEL_NEIGHBOUR_THRESHOLD         = 12;
    private const float MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS    = 96f;
    private const float SHORT_RANGE_HEURISTIC_WEIGHT                 = 1.05f;
    private const float SHORT_RANGE_EXPLORATORY_HEURISTIC_WEIGHT     = 0.80f;
    private const float LONG_RANGE_HEURISTIC_WEIGHT                  = 0.70f;
    private const float LONG_RANGE_L1_CORRIDOR_MAX_RADIUS_HEURISTIC_WEIGHT = 0.30f;
    private const float LONG_RANGE_GLOBAL_FALLBACK_HEURISTIC_WEIGHT  = 0.85f;
    private const float LONG_RANGE_LATERAL_HEURISTIC_WEIGHT_BASE     = 0.22f;
    private const float LONG_RANGE_LATERAL_HEURISTIC_WEIGHT_MIN      = 0.05f;
    private const float GUIDED_CORRIDOR_HEURISTIC_WEIGHT             = 1.90f;
    private const float LONG_RANGE_HEURISTIC_BLEND_DISTANCE          = 4f;
    private const float GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS = 48f;
    private const int   L1_A_STAR_MAX_EXPANSIONS                     = 200_000;
    private const int   L1_DISTANCE_FIELD_BUDGET                     = 500_000;
    private const int   LONG_RANGE_L1_BEST_EFFORT_STEP_BUDGET        = 750_000;
    private const int   LONG_RANGE_L1_BEST_EFFORT_MAX_STEP_BUDGET    = 2_500_000;
    private const int   LONG_RANGE_L1_GOAL_CAPTURE_BASE_STEP_BUDGET  = 300_000;
    private const int   LONG_RANGE_L1_GOAL_CAPTURE_MAX_STEP_BUDGET   = 1_200_000;
    private const int   LONG_RANGE_L1_GUIDED_FULLSEARCH_BASE_CORRIDOR_RADIUS = 6;
    private const int   LONG_RANGE_L1_GUIDED_FULLSEARCH_MAX_CORRIDOR_RADIUS  = 18;
    private const int   LONG_RANGE_PROXY_COARSE_REENTRY_MAX_DEPTH    = 1;
    private const int   LONG_RANGE_L1_CORRIDOR_STEP_BUDGET           = 350_000;
    private const int   LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET         = 1_500_000;
    private const int   LONG_RANGE_L1_CORRIDOR_RELAX_STEPS           = 3;
    private const int   LONG_RANGE_LATERAL_SEARCH_STEP_BUDGET        = 1_200_000;
    private const int   LONG_RANGE_LATERAL_EXPLORATION_ATTEMPTS      = 4;
    private const int   LONG_RANGE_LATERAL_AREA_BASE_CELLS           = 9000;
    private const int   LONG_RANGE_LATERAL_AREA_STEP_CELLS           = 6000;
    private const float LONG_RANGE_LATERAL_AREA_DISTANCE_CELLS_SCALE = 220f;
    private const float LONG_RANGE_LATERAL_WIDTH_SCALE_BASE          = 0.90f;
    private const float LONG_RANGE_LATERAL_WIDTH_SCALE_STEP          = 0.45f;
    private const float LONG_RANGE_LATERAL_GROWTH_SCALE_BASE         = 0.30f;
    private const float LONG_RANGE_LATERAL_GROWTH_SCALE_STEP         = 0.12f;
    private const float LONG_RANGE_LATERAL_FORWARD_SQRT_WIDTH_SCALE  = 0.90f;
    private const float LONG_RANGE_LATERAL_MIN_HALF_WIDTH_L1_CELLS   = 12f;
    private const float LONG_RANGE_LATERAL_MIN_HALF_WIDTH_ATTEMPT_CELLS = 3f;
    private const float LONG_RANGE_LATERAL_MIN_FORWARD_L1_CELLS      = 12f;
    private const float LONG_RANGE_LATERAL_FORWARD_SLACK_L1_CELLS    = 6f;
    private const float LONG_RANGE_LATERAL_FORWARD_ATTEMPT_SCALE     = 0.35f;
    private const float LONG_RANGE_LATERAL_BACKWARD_SLACK_L1_CELLS   = 4f;
    private const float LONG_RANGE_LATERAL_BACKWARD_SCALE            = 0.12f;
    private const float LONG_RANGE_LATERAL_BACKWARD_SCALE_STEP       = 0.08f;
    private const float LONG_RANGE_LATERAL_BACKWARD_ATTEMPT_CELLS    = 2f;
    private const float LONG_RANGE_LATERAL_MIN_VERTICAL_L1_CELLS     = 6f;
    private const float LONG_RANGE_LATERAL_UPWARD_SLACK_L1_CELLS     = 4f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_SLACK_L1_CELLS   = 8f;
    private const float LONG_RANGE_LATERAL_DESCENT_ENABLE_MIN_DROP_LEAF_CELLS = 2f;
    private const float LONG_RANGE_LATERAL_DESCENT_PRIORITY_DROP_LEAF_CELLS    = 18f;
    private const float LONG_RANGE_LATERAL_DESCENT_PRIORITY_MAX_BONUS          = 1.40f;
    private const float LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_ATTEMPT_STEP    = 0.18f;
    private const float LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_MIN             = 0.50f;
    private const float LONG_RANGE_LATERAL_GOAL_DISTANCE_HEURISTIC_WEIGHT      = 0.32f;
    private const float LONG_RANGE_LATERAL_FORWARD_REMAINING_HEURISTIC_WEIGHT  = 0.70f;
    private const float LONG_RANGE_LATERAL_SIDE_OFFSET_HEURISTIC_WEIGHT        = 0.16f;
    private const float LONG_RANGE_LATERAL_DESCENT_ABOVE_GOAL_HEURISTIC_WEIGHT = 1.85f;
    private const float LONG_RANGE_LATERAL_DESCENT_BELOW_GOAL_HEURISTIC_WEIGHT = 0.55f;
    private const float LONG_RANGE_LATERAL_ASCENT_BELOW_GOAL_HEURISTIC_WEIGHT  = 1.35f;
    private const float LONG_RANGE_LATERAL_ASCENT_ABOVE_GOAL_HEURISTIC_WEIGHT  = 0.65f;
    private const float LONG_RANGE_LATERAL_CLIMB_STEP_PENALTY                  = 1.25f;
    private const float LONG_RANGE_LATERAL_REVERSE_STEP_PENALTY                = 0.80f;
    private const float LONG_RANGE_LATERAL_LATERAL_STALL_PENALTY               = 0.48f;
    private const float LONG_RANGE_LATERAL_FORWARD_PROGRESS_CREDIT             = 0.70f;
    private const float LONG_RANGE_LATERAL_DESCENT_PROGRESS_CREDIT             = 0.95f;
    private const float LONG_RANGE_LATERAL_NON_DESCENT_STALL_SCALE             = 0.60f;
    private const float LONG_RANGE_L1_BEST_EFFORT_MIXED_CELL_PENALTY_SCALE     = 8.00f;
    private const float LONG_RANGE_L1_BEST_EFFORT_DISTANCE_BUDGET_PER_CELL     = 3200f;
    private const float LONG_RANGE_L1_BEST_EFFORT_VERTICAL_DISTANCE_BUDGET_SCALE = 1.50f;
    private const float LONG_RANGE_L1_BEST_EFFORT_RELAXED_BUDGET_SCALE         = 1.10f;
    private const float LONG_RANGE_L1_GOAL_CAPTURE_DISTANCE_THRESHOLD_L1_CELLS = 18f;
    private const float LONG_RANGE_L1_GOAL_CAPTURE_DIRECT_DISTANCE_RATIO        = 0.18f;
    private const float LONG_RANGE_L1_GOAL_CAPTURE_BUDGET_PER_CELL             = 16000f;
    private const float LONG_RANGE_L1_GUIDED_FULLSEARCH_GAP_RADIUS_SCALE       = 1.0f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_MIN_ABOVE_GOAL_LEAF_CELLS = 1.5f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_HORIZONTAL_HEURISTIC_SCALE = 0.82f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_HEURISTIC_SCALE   = 0.50f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_DESTINATION_PENALTY_SCALE  = 0.72f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_REQUIRED_DESCENT_LEAF_CELLS = 1.25f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_MISSED_DESCENT_PENALTY      = 0.95f;
    private const float LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_MISS_PENALTY        = 1.35f;
    private const float LONG_RANGE_LATERAL_COARSE_PROMOTION_MIN_FORWARD_DOT              = -0.15f;
    private const float LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_LEAF_CELLS              = 3.0f;
    private const float LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_HEIGHT_SCALE             = 0.35f;
    private const float SHORT_RANGE_EXPLORATION_MIN_HORIZONTAL_LEAF_CELLS = 8f;
    private const float SHORT_RANGE_EXPLORATION_MIN_HORIZONTAL_DISTANCE   = 4f;
    private const float SHORT_RANGE_EXPLORATION_MIN_DROP_LEAF_CELLS       = 4f;
    private const float SHORT_RANGE_EXPLORATION_MIN_DROP_DISTANCE         = 2f;
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
    private const float GUIDED_CORRIDOR_EARLY_ABORT_PROGRESS_RATIO         = 0.04f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_PROGRESS_MIN_DISTANCE  = 8.00f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_SUFFICIENT_PROGRESS_RATIO = 0.55f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_MIN_DROP_LEAF_CELLS = 4.0f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_MIN_DROP_DISTANCE = 2.0f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_PROGRESS_LEAF_CELLS = 2.0f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_PROGRESS_MIN_DISTANCE = 1.0f;
    private const float GUIDED_CORRIDOR_EARLY_ABORT_DESCENT_SUFFICIENT_PROGRESS_RATIO = 0.72f;
    private const byte  L1FaceNegY                                    = 0;
    private const byte  L1FacePosY                                    = 1;
    private const byte  L1FaceNegX                                    = 2;
    private const byte  L1FacePosX                                    = 3;
    private const byte  L1FaceNegZ                                    = 4;
    private const byte  L1FacePosZ                                    = 5;
    private const byte  L1FaceInside                                  = byte.MaxValue;
    private const ushort L1AllFacesMask                               = 0x3f;
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
    private const float FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_WEIGHT     = 0.10f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_BACKWARD_WEIGHT    = 1.10f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_DIAGONAL_WEIGHT = 0.45f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_BACKWARD_DIAGONAL_WEIGHT = 1.20f;
    private const float FLIGHT_PUSH_MIN_DISTANCE                     = 0.02f;
    private const float FLIGHT_PUSH_VOXEL_INSET_RATIO                = 0.10f;
    private const float FLIGHT_PUSH_VOXEL_INSET_MIN                  = 0.01f;
    private const float FLIGHT_PUSH_VOXEL_INSET_MAX                  = 0.08f;
    private const int   FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT    = 24;
    private const float FLIGHT_PUSH_HORIZONTAL_SWEEP_WEIGHT          = 0.70f;
    private const float FLIGHT_PUSH_HORIZONTAL_SWEEP_FORWARD_BONUS   = 0.20f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_FORWARD_PENALTY = 0.90f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_FORWARD_MIN_SCALE = 0.12f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_BACKWARD_BONUS = 0.60f;
    private const float FLIGHT_PUSH_DIRECTIONAL_FORWARD_BONUS        = 0.85f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_FORWARD_PENALTY = 0.95f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_BACKWARD_BONUS = 0.70f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_MIN_SCALE = 0.10f;
    private const float FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT = 0.05f;
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
