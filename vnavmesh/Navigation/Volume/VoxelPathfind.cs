using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;

namespace vnavmesh.Navigation.Volume;

public class VoxelPathfind
{
    private const float ScoreEpsilon = 0.00001f;

    private readonly Config _config;

    internal readonly record struct SearchTelemetry
    (
        int VisitedNodes,
        int GeneratedNodes,
        int LineOfSightChecks,
        int LineOfSightHits,
        int PeakOpenListSize
    );

    public struct Node
    {
        public float   GScore;
        public float   HScore;
        public ulong   Voxel;         // voxel map index corresponding to this node
        public int     ParentIndex;   // index in the node list of the node we entered from
        public int     OpenHeapIndex; // -1 if in closed list, otherwise index in open list
        public Vector3 Position;
    }

    private readonly VoxelMap.Level _l0Desc;
    private readonly VoxelMap.Level _l1Desc;
    private readonly VoxelMap.Level _l2Desc;
    private readonly List<ulong>    _neighbourScratch = new(64);

    private List<Node>             _nodes      = new(1024); // grow only (TODO: consider chunked vector)
    private Dictionary<ulong, int> _nodeLookup = new(1024); // voxel -> node index
    private List<int>              _openList   = new(256);  // heap containing node indices
    private int                    _bestNodeIndex;
    private ulong                  _goalVoxel;
    private Vector3                _goalPos;
    private bool                   _useRaycast;
    private float                  _randomnessMultiplier;
    private bool                   _allowReopen = false; // this is extremely expensive and doesn't seem to actually improve the result
    private int                    _visitedNodes;
    private int                    _generatedNodes;
    private int                    _lineOfSightChecks;
    private int                    _lineOfSightHits;
    private int                    _peakOpenListSize;

    public VoxelMap Volume { get; }

    public   Span<Node>      NodeSpan      => CollectionsMarshal.AsSpan(_nodes);
    internal SearchTelemetry LastTelemetry => new(_visitedNodes, _generatedNodes, _lineOfSightChecks, _lineOfSightHits, _peakOpenListSize);

    public VoxelPathfind(VoxelMap volume, Config config)
    {
        _config = config;
        Volume  = volume;
        _l0Desc = volume.Levels[0];
        _l1Desc = volume.Levels[1];
        _l2Desc = volume.Levels[2];
    }

    public List<(ulong voxel, Vector3 p)> FindPath
        (ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, CancellationToken cancel)
    {
        _useRaycast = useRaycast;
        Start(fromVoxel, toVoxel, fromPos, toPos);
        Execute(cancel);
        return BuildPathToVisitedNode(_bestNodeIndex, returnIntermediatePoints);
    }

    public void Start(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos)
    {
        _nodes.Clear();
        _nodeLookup.Clear();
        _openList.Clear();
        _bestNodeIndex        = 0;
        _randomnessMultiplier = _config.RandomnessMultiplier;
        _visitedNodes         = 0;
        _generatedNodes       = 0;
        _lineOfSightChecks    = 0;
        _lineOfSightHits      = 0;
        _peakOpenListSize     = 0;

        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"Bad input cells: {fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        _goalVoxel = toVoxel;
        _goalPos   = toPos;

        _nodes.Add
        (
            new()
            {
                GScore        = 0,
                HScore        = HeuristicDistance(fromVoxel, fromPos),
                Voxel         = fromVoxel,
                ParentIndex   = 0,
                OpenHeapIndex = -1,
                Position      = fromPos
            }
        ); // start's parent is self
        _nodeLookup[fromVoxel] = 0;
        _generatedNodes        = 1;
        AddToOpen(0);
    }

    public void Execute(CancellationToken cancel, int maxSteps = 1000000)
    {
        for (var i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }
    }

    public bool ExecuteStep()
    {
        if (_openList.Count == 0)
            return false;

        var curNodeIndex = PopMinOpen();
        ++_visitedNodes;

        if (_useRaycast)
            SetVertex(curNodeIndex);

        var nodeSpan = NodeSpan;
        ref var curNode = ref nodeSpan[curNodeIndex];
        UpdateBestNode(curNodeIndex);

        if (curNode.Voxel == _goalVoxel && curNode.HScore <= 0)
        {
            _bestNodeIndex = curNodeIndex;
            return false;
        }

        VisitNeighbours(curNodeIndex, curNode.Voxel);
        return true;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var res = new List<(ulong voxel, Vector3 p)>();

        if (nodeIndex < _nodes.Count)
        {
            var     nodeSpan = NodeSpan;
            ref var lastNode = ref nodeSpan[nodeIndex];
            res.Add((lastNode.Voxel, lastNode.Position));

            while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
            {
                ref var prevNode  = ref nodeSpan[nodeIndex];
                var     nextIndex = prevNode.ParentIndex;
                ref var nextNode  = ref nodeSpan[nextIndex];

                if (returnIntermediatePoints)
                {
                    var delta = nextNode.Position - prevNode.Position;
                    foreach (var v in VoxelSearch.EnumerateVoxelsInLine(Volume, prevNode.Voxel, nextNode.Voxel, prevNode.Position, nextNode.Position))
                        res.Add((v.voxel, prevNode.Position + v.t * delta));
                }
                else res.Add((nextNode.Voxel, nextNode.Position));

                nodeIndex = nextIndex;
            }

            res.Reverse();
        }

        return res;
    }

    private void VisitNeighbours(int currentIndex, ulong voxel)
    {
        foreach (var neighbourVoxel in CollectNeighbours(voxel))
            VisitNeighbour(currentIndex, neighbourVoxel);
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

                if (Volume.IsEmpty(neighbourVoxel)) AddNeighbourIfEmpty(neighbourVoxel);
                else if (l2Index != VoxelMap.IndexLevelMask)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : _l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : _l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : _l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, _l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    AddNeighbourIfEmpty(l2NeighbourVoxel);
                }
                else CollectBorder(neighbourVoxel, _l2Desc, 2, dx, dy, dz);

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

            if (Volume.IsEmpty(l1NeighbourVoxel)) AddNeighbourIfEmpty(l1NeighbourVoxel);
            else if (l2Index != VoxelMap.IndexLevelMask)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : _l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : _l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : _l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, _l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                AddNeighbourIfEmpty(l2NeighbourVoxel);
            }
            else CollectBorder(l1NeighbourVoxel, _l2Desc, 2, dx, dy, dz);

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
            if (Volume.IsEmpty(l1Voxel)) AddNeighbourIfEmpty(l1Voxel);
            else CollectBorder(l1Voxel, _l2Desc, 2, dx, dy, dz);
        }
    }

    private void AddNeighbourIfEmpty(ulong voxel)
    {
        if (Volume.IsEmpty(voxel))
            _neighbourScratch.Add(voxel);
    }

    private void VisitNeighbour(int currentIndex, ulong nodeVoxel)
    {
        if (!_nodeLookup.TryGetValue(nodeVoxel, out var nodeIndex))
        {
            nodeIndex = _nodes.Count;
            _nodes.Add(new() { GScore = float.MaxValue, HScore = float.MaxValue, Voxel = nodeVoxel, ParentIndex = currentIndex, OpenHeapIndex = -1 });
            _nodeLookup[nodeVoxel] = nodeIndex;
            ++_generatedNodes;
        }
        else if (!_allowReopen && _nodes[nodeIndex].OpenHeapIndex < 0)
        {
            return;
        }

        var candidateParentIndex = ResolveExpandedParentIndex(currentIndex);
        var candidatePosition    = ResolveNodePosition(nodeVoxel, candidateParentIndex);
        var candidateScore       = CalculateNodeScore(candidateParentIndex, candidatePosition);
        var nodeSpan            = NodeSpan;
        ref var candidateNode   = ref nodeSpan[nodeIndex];

        if (candidateScore + ScoreEpsilon < candidateNode.GScore)
        {
            candidateNode.GScore      = candidateScore;
            candidateNode.HScore      = HeuristicDistance(nodeVoxel, candidatePosition);
            candidateNode.ParentIndex = candidateParentIndex;
            candidateNode.Position    = candidatePosition;
            AddToOpen(nodeIndex);
            UpdateBestNode(nodeIndex);
        }
    }

    private int ResolveExpandedParentIndex(int currentIndex)
    {
        if (!_useRaycast)
            return currentIndex;

        var nodeSpan = NodeSpan;
        return nodeSpan[currentIndex].ParentIndex;
    }

    private Vector3 ResolveNodePosition(ulong nodeVoxel, int parentIndex)
    {
        if (nodeVoxel == _goalVoxel)
            return _goalPos;

        var nodeSpan = NodeSpan;
        return Volume.ClampPointToVoxel(nodeVoxel, nodeSpan[parentIndex].Position);
    }

    private float CalculateNodeScore(int parentIndex, Vector3 destination)
    {
        var nodeSpan      = NodeSpan;
        ref var parent    = ref nodeSpan[parentIndex];
        var edgeCost      = CalculateEdgeCost(parent.Position, destination);
        var randomFactor  = _randomnessMultiplier > 0 ? (float)Random.Shared.NextDouble() * _randomnessMultiplier : 0;
        return parent.GScore + edgeCost + randomFactor;
    }

    private static float CalculateEdgeCost(Vector3 from, Vector3 to)
    {
        var distance          = (from - to).Length();
        var verticalDifference = MathF.Abs(from.Y - to.Y);
        var verticalPenalty    = 0.2f * verticalDifference;
        return distance + verticalPenalty;
    }

    private void SetVertex(int nodeIndex)
    {
        var nodeSpan = NodeSpan;
        ref var node = ref nodeSpan[nodeIndex];
        if (node.ParentIndex == nodeIndex)
            return;

        if (TryLineOfSight(node.ParentIndex, node.Voxel, node.Position))
            return;

        var bestParentIndex = -1;
        var bestPosition    = default(Vector3);
        var bestScore       = float.MaxValue;

        foreach (var neighbourVoxel in CollectNeighbours(node.Voxel))
        {
            if (!_nodeLookup.TryGetValue(neighbourVoxel, out var neighbourIndex))
                continue;

            var refreshedSpan = NodeSpan;
            ref var neighbour = ref refreshedSpan[neighbourIndex];
            if (neighbour.OpenHeapIndex >= 0)
                continue;

            var candidatePosition = ResolveNodePosition(node.Voxel, neighbourIndex);
            if (!TryLineOfSight(neighbourIndex, node.Voxel, candidatePosition))
                continue;

            var candidateScore = refreshedSpan[neighbourIndex].GScore + CalculateEdgeCost(refreshedSpan[neighbourIndex].Position, candidatePosition);
            if (candidateScore + ScoreEpsilon < bestScore)
            {
                bestParentIndex = neighbourIndex;
                bestPosition    = candidatePosition;
                bestScore       = candidateScore;
            }
        }

        if (bestParentIndex < 0)
            return;

        var finalSpan = NodeSpan;
        ref var current = ref finalSpan[nodeIndex];
        current.ParentIndex = bestParentIndex;
        current.Position    = bestPosition;
        current.GScore      = bestScore;
        current.HScore      = HeuristicDistance(current.Voxel, bestPosition);
        UpdateBestNode(nodeIndex);
    }

    private bool TryLineOfSight(int fromNodeIndex, ulong toVoxel, Vector3 toPosition)
    {
        var nodeSpan = NodeSpan;
        ref var fromNode = ref nodeSpan[fromNodeIndex];
        ++_lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, fromNode.Voxel, toVoxel, fromNode.Position, toPosition))
            return false;

        ++_lineOfSightHits;
        return true;
    }

    private float HeuristicDistance(ulong nodeVoxel, Vector3 v) => nodeVoxel != _goalVoxel ? (v - _goalPos).Length() * 0.999f : 0;

    private void UpdateBestNode(int nodeIndex)
    {
        if (IsBetterBestNode(nodeIndex, _bestNodeIndex))
            _bestNodeIndex = nodeIndex;
    }

    private bool IsBetterBestNode(int candidateIndex, int currentIndex)
    {
        var nodeSpan    = NodeSpan;
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
                else break;
            }
            else if (HeapLess(ref nodeSpan[_openList[child2]], ref nodeSpan[nodeIndex]))
            {
                _openList[heapIndex]                          = _openList[child2];
                nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex                                     = child2;
            }
            else break;
        }

        _openList[heapIndex]              = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private static bool HeapLess(ref Node nodeL, ref Node nodeR)
    {
        var fl = nodeL.GScore + nodeL.HScore;
        var fr = nodeR.GScore + nodeR.HScore;
        if (fl + ScoreEpsilon < fr)
            return true;
        if (fr + ScoreEpsilon < fl)
            return false;
        return nodeL.GScore > nodeR.GScore; // tie-break towards larger g-values
    }
}
