using System.Collections.Concurrent;
using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume.Models;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private readonly VolumeLevel l0Desc;
    private readonly VolumeLevel l1Desc;
    private readonly VolumeLevel l2Desc;

    private readonly float       maxSearchRaycastDistance;
    private readonly float       goalVisibilityProbeDistance;
    private readonly float       bestNodeRelativeHTolerance;
    private readonly List<ulong> neighbourScratch = new(64);

    private readonly List<VolumePathfindNode>                        nodes                        = new(1024);
    private          VoxelNodeLookup                                 nodeLookup                   = new(2048);
    private readonly ConcurrentDictionary<ulong, byte>               voxelWallMaskCache           = new(Environment.ProcessorCount, 4096);
    private readonly ConcurrentDictionary<ulong, byte>               verifiedDownwardOpeningCache = new(Environment.ProcessorCount, 2048);
    private readonly ConcurrentDictionary<ulong, byte>               verifiedTopEntryCache        = new(Environment.ProcessorCount, 2048);
    private readonly ConcurrentDictionary<ulong, ulong>              l1FaceConnectivityCache      = new(Environment.ProcessorCount, 2048);
    private readonly List<int>                                       openList                     = new(256);
    private readonly ConcurrentDictionary<VolumeVisibilityKey, bool> visibilityCache              = new(Environment.ProcessorCount, 4096);

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
    private bool                       debugReturnedLongRangeBestEffortPath;
    private bool                       guidedCorridorEarlyAbortTriggered;
    private int                        currentL1CorridorRadius;
    private HashSet<ulong>?            l1PathSet;
    private HashSet<ushort>?           l0PathSet;
    private Dictionary<ulong, int>?    l1CorridorDistance;
    private Dictionary<ushort, int>?   l0CorridorDistance;
    private Dictionary<ulong, float>?  l1DistanceField;
    private Dictionary<ushort, float>? l0DistanceField;
    private FlightLongRangeProxyDebug? pendingLongRangeProxyDebug;
    private GuidedSearchCorridor       guidedCorridor;
    private LongRangeLateralBias       longRangeLateralBias;
    private float                      guidedCorridorInitialGoalDistance;
    private float                      guidedCorridorInitialAboveGoal;
    private float                      guidedCorridorLastProgressDistance;
    private float                      guidedCorridorLastProgressAboveGoal;
    private int                        guidedCorridorLastProgressVisited;
    private bool                       allowCoarseL1Stepping;
    private HashSet<ulong>?            previouslyVisitedVoxels;

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

    internal FlightPathDebugPayload? LastPathDebug { get; private set; }

    public VoxelPathfind(VoxelMap volume)
    {
        Volume                      = volume;
        l0Desc                      = volume.Levels[0];
        l1Desc                      = volume.Levels[1];
        l2Desc                      = volume.Levels[2];
        maxSearchRaycastDistance    = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS;
        goalVisibilityProbeDistance = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS;
        bestNodeRelativeHTolerance  = MathF.Max(l2Desc.CellSize.X, MathF.Max(l2Desc.CellSize.Y, l2Desc.CellSize.Z)) * BEST_NODE_RELATIVE_H_TOLERANCE_LEAF_CELLS;
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
        l1PathSet                  = null;
        l0PathSet                  = null;
        l1CorridorDistance         = null;
        l0CorridorDistance         = null;
        l1DistanceField            = null;
        l0DistanceField            = null;
        LastPathDebug              = null;
        pendingLongRangeProxyDebug = null;
        previouslyVisitedVoxels    = null;

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
        if (straightLineDistance > maxSearchRaycastDistance * 0.5f)
            ComputeL1DistanceField(toVoxel, toPos, L1_DISTANCE_FIELD_PRECOMPUTE_BUDGET);

        var searchRaycast = Vector3.Distance(fromPos, toPos) <= maxSearchRaycastDistance;

        if (searchRaycast)
        {
            var path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, RAYCAST_SEARCH_STEP_BUDGET, 1, cancel);

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

        return longRangePath.Count > 0 ? RefineSimplifiedPath(longRangePath, cancel) : [];
    }
}
