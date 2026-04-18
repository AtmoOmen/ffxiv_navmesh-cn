using System.Numerics;
using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;

namespace vnavmesh.Navigation.Volume;

public class VoxelPathfind
{
    private const float ScoreEpsilon = 0.00001f;
    private const int   DefaultMaxSearchSteps = 1_0000_0000;
    private const int   RaycastSearchStepBudget = 200000;
    private const int   MaxAncestorLookback = 6;
    private const int   RaycastParallelNeighbourThreshold = 12;
    private const float MaxSearchRaycastDistanceInLeafCells = 96f;
    private const float ShortRangeHeuristicWeight = 1.05f;
    private const float LongRangeHeuristicWeight = 1.45f;
    private const float LongRangeHeuristicBlendDistance = 4f;
    private const float GoalVisibilityProbeDistanceInLeafCells = 48f;

    private enum CandidateKind : byte
    {
        Projected,
        GoalAligned,
        CenterBiased
    }

    internal enum SearchTermination : byte
    {
        ReachedGoal,
        SearchExhausted,
        StepBudgetReached
    }

    private readonly record struct VisibilityKey
    (
        int           FromNodeIndex,
        int           FromNodeRevision,
        ulong         ToVoxel,
        CandidateKind CandidateKind
    );

    internal readonly record struct SearchTelemetry
    (
        int VisitedNodes,
        int GeneratedNodes,
        int LineOfSightChecks,
        int LineOfSightHits,
        int PeakOpenListSize,
        SearchTermination Termination,
        bool SearchRaycastEnabled,
        int SearchAttempts,
        float HeuristicWeight
    );

    private readonly record struct NeighbourEvaluation
    (
        ulong   Voxel,
        int     BestParentIndex,
        Vector3 BestPosition,
        float   BestScore
    );

    public struct Node
    {
        public float   GScore;
        public float   HScore;
        public ulong   Voxel;
        public int     ParentIndex;
        public int     OpenHeapIndex;
        public int     Revision;
        public bool    Closed;
        public Vector3 Position;
    }

    private readonly VoxelMap.Level _l0Desc;
    private readonly VoxelMap.Level _l1Desc;
    private readonly VoxelMap.Level _l2Desc;
    private readonly float          _maxSearchRaycastDistance;
    private readonly float          _goalVisibilityProbeDistance;
    private readonly List<ulong>    _neighbourScratch = new(64);

    private readonly List<Node>                          _nodes      = new(1024);
    private readonly Dictionary<ulong, int>              _nodeLookup = new(1024);
    private readonly List<int>                           _openList   = new(256);
    private readonly ConcurrentDictionary<VisibilityKey, bool> _visibilityCache = new(Environment.ProcessorCount, 4096);
    private int                             _bestNodeIndex;
    private ulong                           _goalVoxel;
    private Vector3                         _goalPos;
    private bool                            _goalReached;
    private bool                            _useSearchRaycast;
    private float                           _heuristicWeight;
    private int                             _visitedNodes;
    private int                             _generatedNodes;
    private int                             _lineOfSightChecks;
    private int                             _lineOfSightHits;
    private int                             _peakOpenListSize;
    private SearchTermination               _lastTermination;
    private bool                            _lastSearchRaycastEnabled;
    private int                             _lastSearchAttempts;

    public VoxelMap Volume { get; }

    public   Span<Node>      NodeSpan      => CollectionsMarshal.AsSpan(_nodes);
    internal SearchTelemetry LastTelemetry => new
    (
        _visitedNodes,
        _generatedNodes,
        _lineOfSightChecks,
        _lineOfSightHits,
        _peakOpenListSize,
        _lastTermination,
        _lastSearchRaycastEnabled,
        _lastSearchAttempts,
        _heuristicWeight
    );

    public VoxelPathfind(VoxelMap volume, Config _)
    {
        Volume                    = volume;
        _l0Desc                   = volume.Levels[0];
        _l1Desc                   = volume.Levels[1];
        _l2Desc                   = volume.Levels[2];
        _maxSearchRaycastDistance = MathF.Max(_l2Desc.CellSize.X, MathF.Max(_l2Desc.CellSize.Y, _l2Desc.CellSize.Z)) * MaxSearchRaycastDistanceInLeafCells;
        _goalVisibilityProbeDistance = MathF.Max(_l2Desc.CellSize.X, MathF.Max(_l2Desc.CellSize.Y, _l2Desc.CellSize.Z)) * GoalVisibilityProbeDistanceInLeafCells;
    }

    public List<(ulong voxel, Vector3 p)> FindPath
        (ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, CancellationToken cancel)
    {
        if (fromVoxel == toVoxel)
        {
            ResetSearchState();
            _goalVoxel       = toVoxel;
            _goalPos         = toPos;
            _bestNodeIndex   = 0;
            _goalReached     = true;
            _visitedNodes    = 1;
            _generatedNodes  = 1;
            _lastTermination = SearchTermination.ReachedGoal;
            _lastSearchAttempts = 1;
            _nodes.Add
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
            _nodeLookup[toVoxel] = 0;
            return [(toVoxel, toPos)];
        }

        if (useRaycast && TryBuildDirectPath(fromVoxel, toVoxel, fromPos, toPos, out var directPath))
            return directPath;

        var useSearchRaycast = useRaycast && ShouldUseSearchRaycast(fromPos, toPos);
        var maxSteps         = useSearchRaycast ? RaycastSearchStepBudget : DefaultMaxSearchSteps;
        var path             = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, useSearchRaycast, returnIntermediatePoints, cancel, maxSteps, 1);

        if (_lastTermination == SearchTermination.StepBudgetReached && useSearchRaycast)
        {
            Service.Log.Debug
            (
                $"[算路] 飞行体素搜索触发降级重试：直线距离 = {Vector3.Distance(fromPos, toPos):f3}，首轮访问节点 = {_visitedNodes}，LoS 检查 = {_lineOfSightChecks}"
            );
            path = RunSearchAttempt(fromVoxel, toVoxel, fromPos, toPos, false, returnIntermediatePoints, cancel, DefaultMaxSearchSteps, 2);
        }

        return useRaycast ? SimplifyPath(path, cancel) : path;
    }

    public void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        ResetSearchState();

        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"输入体素非法：{fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        _goalVoxel = toVoxel;
        _goalPos   = toPos;

        _nodes.Add
        (
            new()
            {
                GScore        = 0,
                HScore        = HeuristicDistance(fromPos),
                Voxel         = fromVoxel,
                ParentIndex   = 0,
                OpenHeapIndex = -1,
                Closed        = false,
                Position      = fromPos
            }
        );
        _nodeLookup[fromVoxel] = 0;
        _generatedNodes        = 1;
        AddToOpen(0);
    }

    private SearchTermination Execute(CancellationToken cancel, int maxSteps = DefaultMaxSearchSteps)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return _goalReached ? SearchTermination.ReachedGoal : SearchTermination.SearchExhausted;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }

        return SearchTermination.StepBudgetReached;
    }

    private List<(ulong voxel, Vector3 p)> RunSearchAttempt
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              useSearchRaycast,
        bool              returnIntermediatePoints,
        CancellationToken cancel,
        int               maxSteps,
        int               attempts
    )
    {
        Start(fromVoxel, toVoxel, fromPos, toPos);
        _useSearchRaycast         = useSearchRaycast;
        _heuristicWeight          = ResolveHeuristicWeight(fromPos, toPos, useSearchRaycast);
        _lastSearchRaycastEnabled = useSearchRaycast;
        _lastSearchAttempts       = attempts;
        _lastTermination = Execute(cancel, maxSteps);
        return BuildPathToVisitedNode(_bestNodeIndex, returnIntermediatePoints);
    }

    private bool ShouldUseSearchRaycast(Vector3 fromPos, Vector3 toPos) => Vector3.Distance(fromPos, toPos) <= _maxSearchRaycastDistance;

    private float ResolveHeuristicWeight(Vector3 fromPos, Vector3 toPos, bool useSearchRaycast)
    {
        if (useSearchRaycast)
            return ShortRangeHeuristicWeight;

        var distance = Vector3.Distance(fromPos, toPos);
        var blend    = Math.Clamp(distance / (_maxSearchRaycastDistance * LongRangeHeuristicBlendDistance), 0f, 1f);
        return ShortRangeHeuristicWeight + (LongRangeHeuristicWeight - ShortRangeHeuristicWeight) * blend;
    }

    private bool TryConnectGoal(int currentIndex)
    {
        var nodeSpan = NodeSpan;
        ref var current = ref nodeSpan[currentIndex];
        if (current.HScore > _goalVisibilityProbeDistance)
            return false;
        if (!TryLineOfSight(currentIndex, _goalVoxel, _goalPos, CandidateKind.GoalAligned))
            return false;

        var goalScore = current.GScore + CalculateEdgeCost(current.Position, _goalPos);
        var goalIndex = GetOrCreateNode(_goalVoxel, currentIndex);
        nodeSpan = NodeSpan;
        ref var goal = ref nodeSpan[goalIndex];
        if (goal.Closed || goalScore + ScoreEpsilon >= goal.GScore)
            return false;

        goal.GScore      = goalScore;
        goal.HScore      = 0;
        goal.ParentIndex = currentIndex;
        goal.Position    = _goalPos;
        unchecked
        {
            ++goal.Revision;
        }

        _bestNodeIndex = goalIndex;
        _goalReached   = true;
        return true;
    }

    public bool ExecuteStep()
    {
        if (_openList.Count == 0)
            return false;

        var currentIndex = PopMinOpen();
        var nodeSpan     = NodeSpan;
        ref var current  = ref nodeSpan[currentIndex];
        current.Closed   = true;
        ++_visitedNodes;
        UpdateBestNode(currentIndex);

        if (current.Voxel == _goalVoxel)
        {
            _bestNodeIndex = currentIndex;
            _goalReached   = true;
            return false;
        }

        if (TryConnectGoal(currentIndex))
            return false;

        var neighbours = CollectNeighbours(current.Voxel);
        if (ShouldParallelizeNeighbourEvaluation(neighbours.Count))
        {
            var evaluations = new NeighbourEvaluation?[neighbours.Count];
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
                VisitNeighbour(currentIndex, neighbourVoxel);
        }

        return true;
    }

    private void ResetSearchState()
    {
        _nodes.Clear();
        _nodeLookup.Clear();
        _openList.Clear();
        _visibilityCache.Clear();
        _bestNodeIndex     = 0;
        _goalReached       = false;
        _visitedNodes      = 0;
        _generatedNodes    = 0;
        _lineOfSightChecks = 0;
        _lineOfSightHits   = 0;
        _peakOpenListSize  = 0;
        _heuristicWeight   = 1;
        _lastTermination   = SearchTermination.SearchExhausted;
        _lastSearchRaycastEnabled = false;
        _lastSearchAttempts = 0;
    }

    internal void ReleaseRetainedState()
    {
        ResetSearchState();
        _neighbourScratch.Clear();
        _neighbourScratch.TrimExcess();
        _nodes.TrimExcess();
        _nodeLookup.TrimExcess();
        _openList.TrimExcess();
    }

    private bool TryBuildDirectPath(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, out List<(ulong voxel, Vector3 p)> path)
    {
        ++_lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, fromVoxel, toVoxel, fromPos, toPos))
        {
            path = [];
            return false;
        }

        ++_lineOfSightHits;
        _goalVoxel      = toVoxel;
        _goalPos        = toPos;
        _bestNodeIndex  = 0;
        _goalReached    = true;
        _visitedNodes   = 1;
        _generatedNodes = 1;
        _lastTermination = SearchTermination.ReachedGoal;
        _lastSearchAttempts = 1;
        path            = [(toVoxel, toPos)];
        return true;
    }

    private List<(ulong voxel, Vector3 p)> SimplifyPath(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return path;

        List<(ulong voxel, Vector3 p)> simplified = [path[0]];
        var anchorIndex = 0;

        while (anchorIndex < path.Count - 1)
        {
            var furthestVisibleIndex = FindFurthestVisibleIndex(path, anchorIndex, cancel);

            simplified.Add(path[furthestVisibleIndex]);
            anchorIndex = furthestVisibleIndex;
        }

        return simplified;
    }

    private int FindFurthestVisibleIndex(List<(ulong voxel, Vector3 p)> path, int anchorIndex, CancellationToken cancel)
    {
        var pathLastIndex = path.Count - 1;
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

            furthestVisibleIndex = probeIndex;
            step <<= 1;
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
            if (HasLineOfSight(path, anchorIndex, mid, cancel))
            {
                visibleIndex = mid;
            }
            else
            {
                blockedIndex = mid - 1;
            }
        }

        return visibleIndex;
    }

    private bool HasLineOfSight(List<(ulong voxel, Vector3 p)> path, int anchorIndex, int probeIndex, CancellationToken cancel)
    {
        if ((probeIndex & 0x3f) == 0)
            cancel.ThrowIfCancellationRequested();

        var anchor = path[anchorIndex];
        var probe  = path[probeIndex];
        ++_lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, anchor.voxel, probe.voxel, anchor.p, probe.p))
            return false;

        ++_lineOfSightHits;
        return true;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var result = new List<(ulong voxel, Vector3 p)>();

        if ((uint)nodeIndex >= (uint)_nodes.Count)
            return result;

        var nodeSpan = NodeSpan;
        result.Add((nodeSpan[nodeIndex].Voxel, nodeSpan[nodeIndex].Position));

        while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
        {
            ref var child      = ref nodeSpan[nodeIndex];
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
            else
            {
                result.Add((parent.Voxel, parent.Position));
            }

            nodeIndex = parentIndex;
        }

        result.Reverse();
        return result;
    }

    private List<ulong> CollectNeighbours(ulong voxel)
    {
        _neighbourScratch.Clear();

        var encodedVoxel = voxel;
        var l0Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l1Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l2Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l0Coords     = _l0Desc.IndexToVoxel(l0Index);
        var l1Coords     = l1Index != VoxelMap.IndexLevelMask ? _l1Desc.IndexToVoxel(l1Index) : default;
        var l2Coords     = l2Index != VoxelMap.IndexLevelMask ? _l2Desc.IndexToVoxel(l2Index) : default;

        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  -1, 0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  +1, 0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, -1, 0,  0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, +1, 0,  0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  0,  -1);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  0,  +1);

        return _neighbourScratch;
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
        if (l2Index != VoxelMap.IndexLevelMask)
        {
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);
            if (_l2Desc.InBounds(l2Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(_l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                AddNeighbourIfEmpty(neighbourVoxel);
                return;
            }
        }

        if (l1Index != VoxelMap.IndexLevelMask)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);
            if (_l1Desc.InBounds(l1Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(_l1Desc.VoxelToIndex(l1Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);

                if (Volume.IsEmpty(neighbourVoxel))
                {
                    AddNeighbourIfEmpty(neighbourVoxel);
                }
                else if (l2Index != VoxelMap.IndexLevelMask)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : _l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : _l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : _l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, _l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    AddNeighbourIfEmpty(l2NeighbourVoxel);
                }
                else
                {
                    CollectBorder(neighbourVoxel, _l2Desc, 2, dx, dy, dz);
                }

                return;
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!_l0Desc.InBounds(l0Neighbour))
            return;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(_l0Desc.VoxelToIndex(l0Neighbour));
        if (Volume.IsEmpty(l0NeighbourVoxel))
        {
            AddNeighbourIfEmpty(l0NeighbourVoxel);
            return;
        }

        if (l1Index != VoxelMap.IndexLevelMask)
        {
            var l1X              = dx == 0 ? l1Coords.x : dx > 0 ? 0 : _l1Desc.NumCellsX - 1;
            var l1Y              = dy == 0 ? l1Coords.y : dy > 0 ? 0 : _l1Desc.NumCellsY - 1;
            var l1Z              = dz == 0 ? l1Coords.z : dz > 0 ? 0 : _l1Desc.NumCellsZ - 1;
            var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(l0NeighbourVoxel, _l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);

            if (Volume.IsEmpty(l1NeighbourVoxel))
            {
                AddNeighbourIfEmpty(l1NeighbourVoxel);
            }
            else if (l2Index != VoxelMap.IndexLevelMask)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : _l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : _l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : _l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, _l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                AddNeighbourIfEmpty(l2NeighbourVoxel);
            }
            else
            {
                CollectBorder(l1NeighbourVoxel, _l2Desc, 2, dx, dy, dz);
            }

            return;
        }

        CollectBorderWithSubdivisions(l0NeighbourVoxel, dx, dy, dz);
    }

    private void CollectBorder(ulong voxel, VoxelMap.Level levelDesc, int level, int dx, int dy, int dz)
    {
        var (xmin, xmax) = dx == 0 ? (0, levelDesc.NumCellsX - 1) : dx > 0 ? (0, 0) : (levelDesc.NumCellsX - 1, levelDesc.NumCellsX - 1);
        var (ymin, ymax) = dy == 0 ? (0, levelDesc.NumCellsY - 1) : dy > 0 ? (0, 0) : (levelDesc.NumCellsY - 1, levelDesc.NumCellsY - 1);
        var (zmin, zmax) = dz == 0 ? (0, levelDesc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (levelDesc.NumCellsZ - 1, levelDesc.NumCellsZ - 1);

        for (var z = zmin; z <= zmax; ++z)
        for (var x = xmin; x <= xmax; ++x)
        for (var y = ymin; y <= ymax; ++y)
            AddNeighbourIfEmpty(VoxelMap.EncodeSubIndex(voxel, levelDesc.VoxelToIndex(x, y, z), level));
    }

    private void CollectBorderWithSubdivisions(ulong voxel, int dx, int dy, int dz)
    {
        var (xmin, xmax) = dx == 0 ? (0, _l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (_l1Desc.NumCellsX - 1, _l1Desc.NumCellsX - 1);
        var (ymin, ymax) = dy == 0 ? (0, _l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (_l1Desc.NumCellsY - 1, _l1Desc.NumCellsY - 1);
        var (zmin, zmax) = dz == 0 ? (0, _l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (_l1Desc.NumCellsZ - 1, _l1Desc.NumCellsZ - 1);

        for (var z = zmin; z <= zmax; ++z)
        for (var x = xmin; x <= xmax; ++x)
        for (var y = ymin; y <= ymax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, _l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel))
            {
                AddNeighbourIfEmpty(l1Voxel);
            }
            else
            {
                CollectBorder(l1Voxel, _l2Desc, 2, dx, dy, dz);
            }
        }
    }

    private void AddNeighbourIfEmpty(ulong voxel)
    {
        if (Volume.IsEmpty(voxel))
            _neighbourScratch.Add(voxel);
    }

    private void VisitNeighbour(int currentIndex, ulong neighbourVoxel)
    {
        if (!TryGetBestCandidate(currentIndex, neighbourVoxel, out var bestParentIndex, out var bestPosition, out var bestScore))
            return;

        ApplyNeighbourEvaluation(new(neighbourVoxel, bestParentIndex, bestPosition, bestScore));
    }

    private void ApplyNeighbourEvaluation(NeighbourEvaluation evaluation)
    {
        var nodeIndex = GetOrCreateNode(evaluation.Voxel, evaluation.BestParentIndex);
        var nodeSpan  = NodeSpan;
        ref var node  = ref nodeSpan[nodeIndex];
        if (node.Closed || evaluation.BestScore + ScoreEpsilon >= node.GScore)
            return;

        node.GScore      = evaluation.BestScore;
        node.HScore      = HeuristicDistance(evaluation.BestPosition);
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
        if (_nodeLookup.TryGetValue(voxel, out var nodeIndex))
            return nodeIndex;

        nodeIndex = _nodes.Count;
        _nodes.Add
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
        _nodeLookup[voxel] = nodeIndex;
        ++_generatedNodes;
        return nodeIndex;
    }

    private bool TryGetBestCandidate(int currentIndex, ulong neighbourVoxel, out int bestParentIndex, out Vector3 bestPosition, out float bestScore)
    {
        bestParentIndex = -1;
        bestPosition    = default;
        bestScore       = float.MaxValue;

        if (!TryEvaluateCandidate(currentIndex, neighbourVoxel, false, ref bestParentIndex, ref bestPosition, ref bestScore))
            return false;

        if (!_useSearchRaycast)
            return true;

        var nodeSpan      = NodeSpan;
        var ancestorIndex = nodeSpan[currentIndex].ParentIndex;
        var lookbackCount = 0;
        var lastEvaluated = -1;
        while (ancestorIndex >= 0 && lookbackCount < MaxAncestorLookback)
        {
            TryEvaluateCandidate(ancestorIndex, neighbourVoxel, true, ref bestParentIndex, ref bestPosition, ref bestScore);
            lastEvaluated = ancestorIndex;

            ref var ancestor = ref nodeSpan[ancestorIndex];
            if (ancestor.ParentIndex == ancestorIndex)
                return bestParentIndex >= 0;

            ancestorIndex = ancestor.ParentIndex;
            ++lookbackCount;
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
        Span<Vector3> candidatePositions = stackalloc Vector3[3];
        Span<CandidateKind> candidateKinds = stackalloc CandidateKind[3];
        candidatePositions[0] = ResolveProjectedPosition(voxel, parentIndex);
        candidateKinds[0]     = CandidateKind.Projected;
        var projectedScore    = CalculateNodeScore(parentIndex, candidatePositions[0]);
        if (bestParentIndex >= 0 && projectedScore > bestScore + ScoreEpsilon)
            return false;

        candidatePositions[1] = ResolveGoalAlignedPosition(voxel);
        candidateKinds[1]     = CandidateKind.GoalAligned;
        candidatePositions[2] = ResolveCenterBiasedPosition(voxel, parentIndex);
        candidateKinds[2]     = CandidateKind.CenterBiased;

        var found = false;
        for (var i = 0; i < candidatePositions.Length; ++i)
        {
            var candidatePosition = candidatePositions[i];
            if (i > 0 && IsSamePosition(candidatePositions[..i], candidatePosition))
                continue;
            if (requireVisibility && !TryLineOfSight(parentIndex, voxel, candidatePosition, candidateKinds[i]))
                continue;

            var candidateScore = i == 0 ? projectedScore : CalculateNodeScore(parentIndex, candidatePosition);
            if (!IsBetterCandidate(parentIndex, candidatePosition, candidateScore, bestParentIndex, bestPosition, bestScore))
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
        if (voxel == _goalVoxel)
            return _goalPos;

        var nodeSpan = NodeSpan;
        return Volume.ClampPointToVoxel(voxel, nodeSpan[parentIndex].Position);
    }

    private Vector3 ResolveGoalAlignedPosition(ulong voxel)
    {
        if (voxel == _goalVoxel)
            return _goalPos;

        return Volume.ClampPointToVoxel(voxel, _goalPos);
    }

    private Vector3 ResolveCenterBiasedPosition(ulong voxel, int parentIndex)
    {
        if (voxel == _goalVoxel)
            return _goalPos;

        var nodeSpan      = NodeSpan;
        var projected     = ResolveProjectedPosition(voxel, parentIndex);
        var goalAligned   = ResolveGoalAlignedPosition(voxel);
        var blendedTarget = Vector3.Lerp(projected, goalAligned, 0.5f);
        return Volume.ClampPointToVoxel(voxel, blendedTarget);
    }

    private static bool IsSamePosition(ReadOnlySpan<Vector3> existingPositions, Vector3 candidatePosition)
    {
        for (var i = 0; i < existingPositions.Length; ++i)
        {
            if (Vector3.DistanceSquared(existingPositions[i], candidatePosition) <= ScoreEpsilon * ScoreEpsilon)
                return true;
        }

        return false;
    }

    private bool IsBetterCandidate
    (
        int     candidateParentIndex,
        Vector3 candidatePosition,
        float   candidateScore,
        int     currentBestParentIndex,
        Vector3 currentBestPosition,
        float   currentBestScore
    )
    {
        if (candidateScore + ScoreEpsilon < currentBestScore)
            return true;
        if (currentBestScore + ScoreEpsilon < candidateScore)
            return false;

        var candidateF = TotalScore(candidateScore, HeuristicDistance(candidatePosition));
        var currentF   = TotalScore(currentBestScore, HeuristicDistance(currentBestPosition));
        if (candidateF + ScoreEpsilon < currentF)
            return true;
        if (currentF + ScoreEpsilon < candidateF)
            return false;

        return candidateParentIndex < currentBestParentIndex;
    }

    private float CalculateNodeScore(int parentIndex, Vector3 destination)
    {
        var nodeSpan = NodeSpan;
        return nodeSpan[parentIndex].GScore + CalculateEdgeCost(nodeSpan[parentIndex].Position, destination);
    }

    private static float CalculateEdgeCost(Vector3 from, Vector3 to) => Vector3.Distance(from, to);

    private bool TryLineOfSight(int fromNodeIndex, ulong toVoxel, Vector3 toPosition, CandidateKind candidateKind)
    {
        var nodeSpan     = NodeSpan;
        ref var fromNode = ref nodeSpan[fromNodeIndex];
        var cacheKey     = new VisibilityKey(fromNodeIndex, fromNode.Revision, toVoxel, candidateKind);
        if (_visibilityCache.TryGetValue(cacheKey, out var cached))
            return cached;

        Interlocked.Increment(ref _lineOfSightChecks);
        var visible = VoxelSearch.LineOfSight(Volume, fromNode.Voxel, toVoxel, fromNode.Position, toPosition);
        if (!_visibilityCache.TryAdd(cacheKey, visible))
            return _visibilityCache[cacheKey];

        if (visible)
            Interlocked.Increment(ref _lineOfSightHits);
        return visible;
    }

    private float HeuristicDistance(Vector3 position) => Vector3.Distance(position, _goalPos);

    private float TotalScore(float gScore, float hScore) => gScore + hScore * _heuristicWeight;

    private bool ShouldParallelizeNeighbourEvaluation(int count) =>
        _useSearchRaycast && count >= RaycastParallelNeighbourThreshold && Environment.ProcessorCount > 1;

    private void UpdateBestNode(int nodeIndex)
    {
        if (_nodes.Count == 0 || IsBetterBestNode(nodeIndex, _bestNodeIndex))
            _bestNodeIndex = nodeIndex;
    }

    private bool IsBetterBestNode(int candidateIndex, int currentIndex)
    {
        var nodeSpan      = NodeSpan;
        ref var candidate = ref nodeSpan[candidateIndex];
        ref var current   = ref nodeSpan[currentIndex];

        if (candidate.HScore + ScoreEpsilon < current.HScore)
            return true;
        if (current.HScore + ScoreEpsilon < candidate.HScore)
            return false;
        return candidate.GScore + ScoreEpsilon < current.GScore;
    }

    private void AddToOpen(int nodeIndex)
    {
        ref var node = ref NodeSpan[nodeIndex];
        if (node.OpenHeapIndex < 0)
        {
            node.OpenHeapIndex = _openList.Count;
            _openList.Add(nodeIndex);
            if (_openList.Count > _peakOpenListSize)
                _peakOpenListSize = _openList.Count;
        }

        PercolateUp(node.OpenHeapIndex);
    }

    private int PopMinOpen()
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = _openList[0];
        _openList[0] = _openList[^1];
        _openList.RemoveAt(_openList.Count - 1);
        nodeSpan[nodeIndex].OpenHeapIndex = -1;

        if (_openList.Count > 0)
        {
            nodeSpan[_openList[0]].OpenHeapIndex = 0;
            PercolateDown(0);
        }

        return nodeIndex;
    }

    private void PercolateUp(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = _openList[heapIndex];
        var parent    = heapIndex - 1 >> 1;

        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[_openList[parent]]))
        {
            _openList[heapIndex]                          = _openList[parent];
            nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex                                     = parent;
            parent                                        = heapIndex - 1 >> 1;
        }

        _openList[heapIndex]              = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private void PercolateDown(int heapIndex)
    {
        var nodeSpan  = NodeSpan;
        var nodeIndex = _openList[heapIndex];
        var maxSize   = _openList.Count;

        while (true)
        {
            var child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            var child2 = child1 + 1;

            if (child2 == maxSize || HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[_openList[child2]]))
            {
                if (HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[nodeIndex]))
                {
                    _openList[heapIndex]                          = _openList[child1];
                    nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex                                     = child1;
                }
                else
                {
                    break;
                }
            }
            else if (HeapLess(ref nodeSpan[_openList[child2]], ref nodeSpan[nodeIndex]))
            {
                _openList[heapIndex]                          = _openList[child2];
                nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex                                     = child2;
            }
            else
            {
                break;
            }
        }

        _openList[heapIndex]              = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private bool HeapLess(ref Node left, ref Node right)
    {
        var leftF  = TotalScore(left.GScore, left.HScore);
        var rightF = TotalScore(right.GScore, right.HScore);
        if (leftF + ScoreEpsilon < rightF)
            return true;
        if (rightF + ScoreEpsilon < leftF)
            return false;
        if (left.HScore + ScoreEpsilon < right.HScore)
            return true;
        if (right.HScore + ScoreEpsilon < left.HScore)
            return false;
        return left.GScore > right.GScore;
    }
}
