using System.Numerics;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume.Models;
using vnavmesh.Navigation.Volume.Utils;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
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

        var guidedCorridorRadius = ResolveLongRangeGuidedFullSearchCorridorRadius(coarsePath.BestDistance);
        if (coarsePath.ReachedGoal)
            guidedCorridorRadius = Math.Min(guidedCorridorRadius * 2, LONG_RANGE_L1_GUIDED_FULL_SEARCH_MAX_CORRIDOR_RADIUS * 2);
        var proxyGoalVoxel       = coarsePath.ReachedGoal ? toVoxel : coarsePath.OrderedPath[^1];
        var proxyGoalPos         = ResolveVoxelCenter(proxyGoalVoxel);

        // 粗搜索未达终点时，尝试把 L1-only 代理目标解析为真实 L2 体素
        if (!coarsePath.ReachedGoal)
        {
            var (resolvedLeaf, leafEmpty) = Volume.FindLeafVoxel(proxyGoalPos);
            if (leafEmpty)
                proxyGoalVoxel = resolvedLeaf;
        }

        BuildL1Corridor(coarsePath.PathSet, guidedCorridorRadius);
        ComputeL1DistanceFieldFromCoarsePath(coarsePath.OrderedPath, 0f);

        var proxySearchBudget = coarsePath.ReachedGoal ? LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET * 2 : LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET;
        var proxyPath = RunSearchAttempt
        (
            fromVoxel,
            proxyGoalVoxel,
            fromPos,
            proxyGoalPos,
            returnIntermediatePoints,
            proxySearchBudget,
            1,
            cancel,
            heuristicWeightOverride: coarsePath.ReachedGoal ? GUIDED_CORRIDOR_HEURISTIC_WEIGHT : LONG_RANGE_GLOBAL_FALLBACK_HEURISTIC_WEIGHT
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
                return VoxelPathUtil.MergePathSegments(proxyPath, directTail, SCORE_EPSILON);
            }

            var tailPath = RunTailSearchFromProxy
            (
                proxyEndpoint.voxel,
                toVoxel,
                proxyEndpoint.p,
                toPos,
                returnIntermediatePoints,
                coarsePath.ReachedGoal,
                recursionDepth,
                out var usedLongRangeReentry,
                cancel
            );

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
                return VoxelPathUtil.MergePathSegments(proxyPath, tailPath, SCORE_EPSILON);
            }

            if (!(usedLongRangeReentry && pendingLongRangeProxyDebug is not null))
                pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyEndpoint.p, toPos, FlightLongRangeTailKind.ShortRangeRelayPartial);
            Service.Log.Debug
            (
                usedLongRangeReentry
                    ? $"[算路] 飞行体素粗层代理搜索后二次粗搜接力未达终点（{lastTermination}），访问节点 = {visitedNodes}"
                    : $"[算路] 飞行体素粗层代理搜索后短程接力未达终点（{lastTermination}），访问节点 = {visitedNodes}"
            );
            return VoxelPathUtil.MergePathSegments(proxyPath, tailPath, SCORE_EPSILON);
        }

        pendingLongRangeProxyDebug = new(proxyGoalVoxel, proxyGoalPos, proxyPath.Count > 0 ? proxyPath[^1].p : fromPos, toPos, FlightLongRangeTailKind.None);
        Service.Log.Debug
        (
            $"[算路] 飞行体素粗层代理搜索未达终点（{lastTermination}），粗路径单元 = {coarsePath.PathSet.Count}，引导半径 = {guidedCorridorRadius}，访问节点 = {visitedNodes}"
        );
        ClearL1AreaConstraint();

        // 最终兜底：代理搜索穷尽时，去掉走廊约束用真实目标做全搜索
        if (!coarsePath.ReachedGoal && lastTermination == VolumeSearchTermination.SearchExhausted)
        {
            Service.Log.Debug("[算路] 飞行体素兜底全搜索：代理搜索穷尽，回退无约束距离场全搜索");
            ComputeL1DistanceField(toVoxel, toPos);
            var fallbackPath = RunSearchAttempt
            (
                fromVoxel,
                toVoxel,
                fromPos,
                toPos,
                returnIntermediatePoints,
                LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET,
                2,
                cancel,
                heuristicWeightOverride: LONG_RANGE_GLOBAL_FALLBACK_HEURISTIC_WEIGHT
            );
            return fallbackPath.Count > 0 ? fallbackPath : proxyPath;
        }

        return proxyPath;
    }

    private List<(ulong voxel, Vector3 p)> RunTailSearchFromProxy
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        bool              coarseReachedGoal,
        int               recursionDepth,
        out bool          usedLongRangeReentry,
        CancellationToken cancel
    )
    {
        usedLongRangeReentry = false;
        ClearL1AreaConstraint();
        l1DistanceField = null;
        l0DistanceField = null;

        if (TryBuildDirectPath(fromVoxel, toVoxel, fromPos, toPos, out var directPath))
            return directPath;

        var remainingDistance           = Vector3.Distance(fromPos, toPos);
        var searchRaycast               = remainingDistance <= maxSearchRaycastDistance;
        var allowLongRangeCoarseReentry = recursionDepth < LONG_RANGE_PROXY_COARSE_REENTRY_MAX_DEPTH && !searchRaycast;

        if (searchRaycast)
        {
            var path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, RAYCAST_SEARCH_STEP_BUDGET, 1, cancel);
            if (lastTermination != VolumeSearchTermination.ReachedGoal)
                path = RunShortRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel);
            return path;
        }

        if (!coarseReachedGoal)
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
                GUIDED_CORRIDOR_SEARCH_STEP_BUDGET,
                1,
                cancel,
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
            LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET,
            2,
            cancel,
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
        var attempts                = 2;
        var useExploratoryHeuristic = ShouldUseShortRangeExploratoryHeuristic(fromPos, toPos);
        var fallbackHeuristicWeight = useExploratoryHeuristic ? SHORT_RANGE_EXPLORATORY_HEURISTIC_WEIGHT : SHORT_RANGE_HEURISTIC_WEIGHT;

        Service.Log.Debug
        (
            $"[算路] 飞行体素短距搜索进入拓扑回退：直线距离 = {Vector3.Distance(fromPos, toPos):f3}，首轮终止 = {lastTermination}，首轮访问节点 = {visitedNodes}，LoS 检查 = {lineOfSightChecks}，探索式启发 = {(useExploratoryHeuristic ? "是" : "否")}"
        );

        var l1Path = SearchL1CoarsePath(fromVoxel, fromPos, toVoxel, toPos);

        switch (l1Path)
        {
            case { Count: > 1 }:
            {
                BuildL1Corridor(l1Path, 4);
                var constrainedPath = RunSearchAttempt
                (
                    fromVoxel,
                    toVoxel,
                    fromPos,
                    toPos,
                    returnIntermediatePoints,
                    DEFAULT_MAX_SEARCH_STEPS,
                    attempts++,
                    cancel,
                    heuristicWeightOverride: fallbackHeuristicWeight
                );

                if (lastTermination == VolumeSearchTermination.ReachedGoal)
                {
                    Service.Log.Debug
                    (
                        $"[算路] 飞行体素短距 L1 约束搜索完成：访问节点 = {visitedNodes}，L1 路径单元 = {l1Path.Count}，启发式权重 = {heuristicWeight:f2}"
                    );
                    return constrainedPath;
                }

                Service.Log.Debug
                (
                    $"[算路] 飞行体素短距 L1 约束搜索未达终点（{lastTermination}），回退 L1 距离场全搜索"
                );
                l1PathSet = null;
                l0PathSet = null;
                break;
            }
            case { Count: 1 }:
                Service.Log.Debug("[算路] 飞行体素短距起终点位于同一 L1 单元，跳过 L1 约束，直接使用距离场回退");
                break;
            default:
                Service.Log.Debug("[算路] 飞行体素短距 L1 粗搜索未找到路径，直接使用距离场回退");
                break;
        }

        if (l1DistanceField is not { Count: > 0 })
            ComputeL1DistanceField(toVoxel, toPos);

        return RunSearchAttempt
        (
            fromVoxel,
            toVoxel,
            fromPos,
            toPos,
            returnIntermediatePoints,
            DEFAULT_MAX_SEARCH_STEPS,
            attempts,
            cancel,
            heuristicWeightOverride: fallbackHeuristicWeight
        );
    }
}
