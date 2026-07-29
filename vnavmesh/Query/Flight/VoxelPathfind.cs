using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight.Models;
using vnavmesh.Query.Flight.Utils;

namespace vnavmesh.Query.Flight;

public partial class VoxelPathfind
{
    private readonly VolumeLevel l0Desc;
    private readonly VolumeLevel l1Desc;
    private readonly VolumeLevel l2Desc;

    private readonly float       maxSearchRaycastDistance;
    private readonly float       goalVisibilityProbeDistance;
    private readonly float       bestNodeRelativeHTolerance;
    private readonly List<ulong> neighbourScratch = new(64);

    private readonly List<VolumePathfindNode>                nodes                        = new(1024);
    private          VoxelNodeLookup                         nodeLookup                   = new(2048);
    private readonly Dictionary<ulong, byte>                 voxelWallMaskCache           = new(4096);
    private readonly Dictionary<ulong, byte>                 verifiedDownwardOpeningCache = new(2048);
    private readonly Dictionary<ulong, byte>                 verifiedTopEntryCache        = new(2048);
    private readonly Dictionary<ulong, ulong>                l1FaceConnectivityCache      = new(2048);
    private readonly Dictionary<(ulong, ulong), bool>        l1FaceTransitionCache        = new(2048);
    private readonly List<int>                               openList                     = new(256);
    private readonly Dictionary<VolumeVisibilityKey, bool>[] visibilityCaches;
    private readonly object[]                                visibilityLocks;
    private readonly Dictionary<VolumeVisibilityKey, bool>   pathLoSCache             = new(1024);
    private readonly Dictionary<(ulong, int), float>         clearanceCache           = new(1024);
    private readonly object                                  cacheLock                = new();
    private          VolumeNeighbourEvaluation?[]            parallelEvaluationBuffer = Array.Empty<VolumeNeighbourEvaluation?>();
    private readonly ParallelOptions                         parallelOptions          = new() { MaxDegreeOfParallelism = Math.Max(1, Environment.ProcessorCount) };
    private          bool[]?                                 l1FloodFillVisited;
    private          Queue<ushort>?                          l1BfsQueue;

    private int                        bestNodeIndex;
    private ulong                      goalVoxel;
    private Vector3                    goalPos;
    private bool                       goalReached;
    private bool                       useGuidedCorridor;
    private float                      heuristicWeight;
    private int                        visitedNodes;
    private int                        generatedNodes;
    private int                        lineOfSightChecks;
    private int                        lineOfSightHits;
    private int                        peakOpenListSize;
    private VolumeSearchTermination    lastTermination;
    private int                        lastSearchAttempts;
    private bool                       guidedCorridorEarlyAbortTriggered;
    private int                        currentL1CorridorRadius;
    private HashSet<ulong>?            l1PathSet;
    private HashSet<ushort>?           l0PathSet;
    private Dictionary<ulong, int>?    l1CorridorDistance;
    private Dictionary<ushort, int>?   l0CorridorDistance;
    private Dictionary<ulong, float>?  l1DistanceField;
    private Dictionary<ushort, float>? l0DistanceField;
    private GuidedSearchCorridor       guidedCorridor;
    private LongRangeLateralBias       longRangeLateralBias;
    private float                      guidedCorridorInitialGoalDistance;
    private float                      guidedCorridorInitialAboveGoal;
    private float                      guidedCorridorLastProgressDistance;
    private float                      guidedCorridorLastProgressAboveGoal;
    private int                        guidedCorridorLastProgressVisited;
    private bool                       allowCoarseL1Stepping;
    private bool                       allowAncestorLookBack = true;
    private HashSet<ulong>?            previouslyVisitedVoxels;
    private int                        completedVisitedNodes;
    private int                        completedGeneratedNodes;
    private int                        completedLineOfSightChecks;
    private int                        completedLineOfSightHits;
    private int                        completedPeakOpenListSize;
    private int                        completedSearchAttempts;
    private int                        coarseExpandedNodes;

    public VoxelMap Volume { get; }

    public Span<VolumePathfindNode> NodeSpan => CollectionsMarshal.AsSpan(nodes);

    internal VolumeSearchTelemetry LastTelemetry => new
    (
        completedVisitedNodes      + visitedNodes,
        completedGeneratedNodes    + generatedNodes,
        completedLineOfSightChecks + lineOfSightChecks,
        completedLineOfSightHits   + lineOfSightHits,
        Math.Max(completedPeakOpenListSize, peakOpenListSize),
        coarseExpandedNodes,
        lastTermination,
        completedSearchAttempts + lastSearchAttempts,
        heuristicWeight
    );

    public VoxelPathfind
    (
        VoxelMap volume
    )
    {
        Volume                      = volume;
        l0Desc                      = volume.Levels[0];
        l1Desc                      = volume.Levels[1];
        l2Desc                      = volume.Levels[2];
        maxSearchRaycastDistance    = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS;
        goalVisibilityProbeDistance = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS;
        bestNodeRelativeHTolerance  = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * BEST_NODE_RELATIVE_H_TOLERANCE_LEAF_CELLS;

        visibilityCaches = new Dictionary<VolumeVisibilityKey, bool>[VISIBILITY_CACHE_STRIPES];
        visibilityLocks  = new object[VISIBILITY_CACHE_STRIPES];

        for (var i = 0; i < VISIBILITY_CACHE_STRIPES; ++i)
        {
            visibilityCaches[i] = new(256);
            visibilityLocks[i]  = new();
        }
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
        BeginQuery();
        return FindPathInternal(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel, true);
    }

    private List<(ulong voxel, Vector3 p)> FindPathInternal
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel,
        bool              allowRelay
    )
    {
        l1PathSet               = null;
        l0PathSet               = null;
        l1CorridorDistance      = null;
        l0CorridorDistance      = null;
        l1DistanceField         = null;
        l0DistanceField         = null;
        previouslyVisitedVoxels = null;

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
            nodeLookup.SetOrUpdate(toVoxel, 0);
            return [(toVoxel, toPos)];
        }

        if (TryBuildDirectPath(fromVoxel, toVoxel, fromPos, toPos, out var directPath))
            return directPath;

        var straightLineDistance = Vector3.Distance(fromPos, toPos);
        var searchRaycast        = straightLineDistance <= maxSearchRaycastDistance;

        if (searchRaycast)
        {
            var fastSearchBudget = ResolveShortRangeFastSearchStepBudget(straightLineDistance);
            var path = RunSearchAttempt
            (
                fromVoxel,
                toVoxel,
                fromPos,
                toPos,
                returnIntermediatePoints,
                fastSearchBudget,
                cancel,
                heuristicWeightOverride: SHORT_RANGE_FAST_HEURISTIC_WEIGHT,
                disableAncestorLookBack: true,
                allowCoarseSteppingOverride: true
            );

            if (lastTermination == VolumeSearchTermination.ReachedGoal)
                return RefineSimplifiedPath(path, cancel);

            RetainClosedSetKnowledge();
            path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, RAYCAST_SEARCH_STEP_BUDGET, cancel);

            if (lastTermination != VolumeSearchTermination.ReachedGoal)
            {
                RetainClosedSetKnowledge();
                path = RunShortRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel);
            }

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
                LONG_RANGE_GUIDED_PROBE_STEP_BUDGET,
                cancel,
                corridor,
                SHORT_RANGE_FAST_HEURISTIC_WEIGHT,
                disableAncestorLookBack: true,
                allowCoarseSteppingOverride: true
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
                guidedCorridorEarlyAbortTriggered ?
                    $"[算路] 飞行体素定向走廊搜索提前回退（进展停滞），访问节点 = {visitedNodes}，最佳距离 = {NodeSpan[bestNodeIndex].HScore:f3}，当前高差余量 = {MathF.Max(NodeSpan[bestNodeIndex].Position.Y - goalPos.Y, 0f):f3}" :
                    $"[算路] 飞行体素定向走廊搜索未达终点（{lastTermination}），回退侧向探测/全搜索"
            );
        }

        RetainClosedSetKnowledge();
        var longRangePath = RunLongRangeFallback(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel);

        if (lastTermination == VolumeSearchTermination.ReachedGoal)
            return RefineSimplifiedPath(longRangePath, cancel);

        // Partial 接力：用当前路径终点作为新起点重新搜索
        if (allowRelay && longRangePath.Count > 0)
        {
            var relayPoint = longRangePath[^1];
            var (relayVoxel, relayEmpty) = Volume.FindLeafVoxel(relayPoint.p);

            if (relayEmpty && relayVoxel != VoxelMap.INVALID_VOXEL && relayVoxel != fromVoxel)
            {
                var remainingDistance = Vector3.Distance(relayPoint.p, toPos);
                Service.Log.Debug($"[算路] 飞行体素 Partial 接力搜索：接力点 = {relayPoint.p:f3}，剩余距离 = {remainingDistance:f3}");

                var relayPath = FindPathInternal(relayVoxel, toVoxel, relayPoint.p, toPos, returnIntermediatePoints, cancel, false);

                if (relayPath.Count > 0)
                {
                    var mergedPath = VoxelPathUtil.MergePathSegments(longRangePath, relayPath, SCORE_EPSILON);
                    return RefineSimplifiedPath(mergedPath, cancel);
                }
            }
        }

        return longRangePath.Count > 0 ?
                   RefineSimplifiedPath(longRangePath, cancel) :
                   [];
    }
}
