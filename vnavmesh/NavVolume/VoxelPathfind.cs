using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading;

namespace vnavmesh.NavVolume;

public class VoxelPathfind
{
    internal readonly record struct SearchTelemetry(int VisitedNodes, int GeneratedNodes, int LineOfSightChecks, int LineOfSightHits, int PeakOpenListSize);

    public struct Node
    {
        public float GScore;
        public float HScore;
        public ulong Voxel; // voxel map index corresponding to this node
        public int ParentIndex; // index in the node list of the node we entered from
        public int OpenHeapIndex; // -1 if in closed list, otherwise index in open list
        public Vector3 Position;
    }

    private VoxelMap _volume;
    private readonly VoxelMap.Level _l0Desc;
    private readonly VoxelMap.Level _l1Desc;
    private readonly VoxelMap.Level _l2Desc;
    private List<Node> _nodes = new(1024); // grow only (TODO: consider chunked vector)
    private Dictionary<ulong, int> _nodeLookup = new(1024); // voxel -> node index
    private List<int> _openList = new(256); // heap containing node indices
    private int _bestNodeIndex;
    private ulong _goalVoxel;
    private Vector3 _goalPos;
    private bool _useRaycast;
    private float _randomnessMultiplier;
    private bool _allowReopen = false; // this is extremely expensive and doesn't seem to actually improve the result
    private float _raycastLimitSq = float.MaxValue;
    private int _visitedNodes;
    private int _generatedNodes;
    private int _lineOfSightChecks;
    private int _lineOfSightHits;
    private int _peakOpenListSize;

    public VoxelMap Volume => _volume;
    public Span<Node> NodeSpan => CollectionsMarshal.AsSpan(_nodes);
    internal SearchTelemetry LastTelemetry => new(_visitedNodes, _generatedNodes, _lineOfSightChecks, _lineOfSightHits, _peakOpenListSize);

    public VoxelPathfind(VoxelMap volume)
    {
        _volume = volume;
        _l0Desc = volume.Levels[0];
        _l1Desc = volume.Levels[1];
        _l2Desc = volume.Levels[2];
    }

    public List<(ulong voxel, Vector3 p)> FindPath(ulong fromVoxel, ulong toVoxel, Vector3 fromPos, Vector3 toPos, bool useRaycast, bool returnIntermediatePoints, CancellationToken cancel)
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
        _bestNodeIndex = 0;
        _randomnessMultiplier = Service.Config.RandomnessMultiplier;
        _visitedNodes = 0;
        _generatedNodes = 0;
        _lineOfSightChecks = 0;
        _lineOfSightHits = 0;
        _peakOpenListSize = 0;
        if (fromVoxel == VoxelMap.InvalidVoxel || toVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"Bad input cells: {fromVoxel:X} -> {toVoxel:X}");
            return;
        }

        _goalVoxel = toVoxel;
        _goalPos = toPos;

        _nodes.Add(new() { HScore = HeuristicDistance(fromVoxel, fromPos), Voxel = fromVoxel, ParentIndex = 0, OpenHeapIndex = -1, Position = fromPos }); // start's parent is self
        _nodeLookup[fromVoxel] = 0;
        _generatedNodes = 1;
        AddToOpen(0);
        //Service.Log.Debug($"volume pathfind: {fromPos} ({fromVoxel:X}) to {toPos} ({toVoxel:X})");
    }

    public void Execute(CancellationToken cancel, int maxSteps = 1000000)
    {
        for (int i = 0; i < maxSteps; ++i)
        {
            if (!ExecuteStep())
                return;
            if ((i & 0x3ff) == 0)
                cancel.ThrowIfCancellationRequested();
        }
    }

    // returns whether search is to be terminated; on success, first node of the open list would contain found goal
    public bool ExecuteStep()
    {
        var nodeSpan = NodeSpan;
        if (_openList.Count == 0 || nodeSpan[_bestNodeIndex].HScore <= 0)
            return false;

        var curNodeIndex = PopMinOpen();
        ref var curNode = ref nodeSpan[curNodeIndex];
        ++_visitedNodes;
        //Service.Log.Debug($"volume pathfind: considering {curNode.Voxel:X} (#{curNodeIndex}), g={curNode.GScore:f3}, h={curNode.HScore:f3}");

        VisitNeighbours(curNodeIndex, curNode.Voxel);
        return true;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var res = new List<(ulong voxel, Vector3 p)>();
        if (nodeIndex < _nodes.Count)
        {
            var nodeSpan = NodeSpan;
            ref var lastNode = ref nodeSpan[nodeIndex];
            res.Add((lastNode.Voxel, lastNode.Position));
            //Service.Log.Debug($"volume pathfind: backpath from {lastNode.Voxel:X} (#{nodeIndex})");
            while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
            {
                ref var prevNode = ref nodeSpan[nodeIndex];
                var nextIndex = prevNode.ParentIndex;
                ref var nextNode = ref nodeSpan[nextIndex];
                //Service.Log.Debug($"volume pathfind: backpath next {nextNode.Voxel:X} (#{nextIndex})");
                if (returnIntermediatePoints)
                {
                    var delta = nextNode.Position - prevNode.Position;
                    foreach (var v in VoxelSearch.EnumerateVoxelsInLine(_volume, prevNode.Voxel, nextNode.Voxel, prevNode.Position, nextNode.Position))
                    {
                        //Service.Log.Debug($"volume pathfind: intermediate {v}");
                        res.Add((v.voxel, prevNode.Position + v.t * delta));
                    }
                }
                else
                {
                    res.Add((nextNode.Voxel, nextNode.Position));
                }
                nodeIndex = nextIndex;
            }
            res.Reverse();
        }
        return res;
    }

    private void VisitNeighbours(int parentIndex, ulong voxel)
    {
        var encodedVoxel = voxel;
        var l0Index = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l1Index = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l2Index = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l0Coords = _l0Desc.IndexToVoxel(l0Index);
        var l1Coords = l1Index != VoxelMap.IndexLevelMask ? _l1Desc.IndexToVoxel(l1Index) : default;
        var l2Coords = l2Index != VoxelMap.IndexLevelMask ? _l2Desc.IndexToVoxel(l2Index) : default;

        VisitDirection(parentIndex, l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0, -1, 0);
        VisitDirection(parentIndex, l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0, +1, 0);
        VisitDirection(parentIndex, l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, -1, 0, 0);
        VisitDirection(parentIndex, l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, +1, 0, 0);
        VisitDirection(parentIndex, l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0, 0, -1);
        VisitDirection(parentIndex, l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0, 0, +1);
    }

    private void VisitDirection(int parentIndex, ushort l0Index, ushort l1Index, ushort l2Index, (int x, int y, int z) l0Coords, (int x, int y, int z) l1Coords, (int x, int y, int z) l2Coords, int dx, int dy, int dz)
    {
        if (l2Index != VoxelMap.IndexLevelMask)
        {
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);
            if (_l2Desc.InBounds(l2Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(_l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                VisitNeighbourIfEmpty(parentIndex, neighbourVoxel);
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
                if (_volume.IsEmpty(neighbourVoxel))
                {
                    VisitNeighbour(parentIndex, neighbourVoxel);
                }
                else if (l2Index != VoxelMap.IndexLevelMask)
                {
                    int l2X = dx == 0 ? l2Coords.x : dx > 0 ? 0 : _l2Desc.NumCellsX - 1;
                    int l2Y = dy == 0 ? l2Coords.y : dy > 0 ? 0 : _l2Desc.NumCellsY - 1;
                    int l2Z = dz == 0 ? l2Coords.z : dz > 0 ? 0 : _l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, _l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    VisitNeighbourIfEmpty(parentIndex, l2NeighbourVoxel);
                }
                else
                {
                    VisitBorder(parentIndex, neighbourVoxel, _l2Desc, 2, dx, dy, dz);
                }

                return;
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!_l0Desc.InBounds(l0Neighbour))
            return;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(_l0Desc.VoxelToIndex(l0Neighbour));
        if (_volume.IsEmpty(l0NeighbourVoxel))
        {
            VisitNeighbour(parentIndex, l0NeighbourVoxel);
            return;
        }

        if (l1Index != VoxelMap.IndexLevelMask)
        {
            int l1X = dx == 0 ? l1Coords.x : dx > 0 ? 0 : _l1Desc.NumCellsX - 1;
            int l1Y = dy == 0 ? l1Coords.y : dy > 0 ? 0 : _l1Desc.NumCellsY - 1;
            int l1Z = dz == 0 ? l1Coords.z : dz > 0 ? 0 : _l1Desc.NumCellsZ - 1;
            var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(l0NeighbourVoxel, _l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);
            if (_volume.IsEmpty(l1NeighbourVoxel))
            {
                VisitNeighbour(parentIndex, l1NeighbourVoxel);
            }
            else if (l2Index != VoxelMap.IndexLevelMask)
            {
                int l2X = dx == 0 ? l2Coords.x : dx > 0 ? 0 : _l2Desc.NumCellsX - 1;
                int l2Y = dy == 0 ? l2Coords.y : dy > 0 ? 0 : _l2Desc.NumCellsY - 1;
                int l2Z = dz == 0 ? l2Coords.z : dz > 0 ? 0 : _l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, _l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                VisitNeighbourIfEmpty(parentIndex, l2NeighbourVoxel);
            }
            else
            {
                VisitBorder(parentIndex, l1NeighbourVoxel, _l2Desc, 2, dx, dy, dz);
            }

            return;
        }

        VisitBorderWithSubdivisions(parentIndex, l0NeighbourVoxel, dx, dy, dz);
    }

    private void VisitBorder(int parentIndex, ulong voxel, VoxelMap.Level levelDesc, int level, int dx, int dy, int dz)
    {
        var (xmin, xmax) = dx == 0 ? (0, levelDesc.NumCellsX - 1) : dx > 0 ? (0, 0) : (levelDesc.NumCellsX - 1, levelDesc.NumCellsX - 1);
        var (ymin, ymax) = dy == 0 ? (0, levelDesc.NumCellsY - 1) : dy > 0 ? (0, 0) : (levelDesc.NumCellsY - 1, levelDesc.NumCellsY - 1);
        var (zmin, zmax) = dz == 0 ? (0, levelDesc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (levelDesc.NumCellsZ - 1, levelDesc.NumCellsZ - 1);
        for (int z = zmin; z <= zmax; ++z)
        {
            for (int x = xmin; x <= xmax; ++x)
            {
                for (int y = ymin; y <= ymax; ++y)
                {
                    VisitNeighbourIfEmpty(parentIndex, VoxelMap.EncodeSubIndex(voxel, levelDesc.VoxelToIndex(x, y, z), level));
                }
            }
        }
    }

    private void VisitBorderWithSubdivisions(int parentIndex, ulong voxel, int dx, int dy, int dz)
    {
        var (xmin, xmax) = dx == 0 ? (0, _l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (_l1Desc.NumCellsX - 1, _l1Desc.NumCellsX - 1);
        var (ymin, ymax) = dy == 0 ? (0, _l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (_l1Desc.NumCellsY - 1, _l1Desc.NumCellsY - 1);
        var (zmin, zmax) = dz == 0 ? (0, _l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (_l1Desc.NumCellsZ - 1, _l1Desc.NumCellsZ - 1);
        for (int z = zmin; z <= zmax; ++z)
        {
            for (int x = xmin; x <= xmax; ++x)
            {
                for (int y = ymin; y <= ymax; ++y)
                {
                    var l1Voxel = VoxelMap.EncodeSubIndex(voxel, _l1Desc.VoxelToIndex(x, y, z), 1);
                    if (_volume.IsEmpty(l1Voxel))
                    {
                        VisitNeighbour(parentIndex, l1Voxel);
                    }
                    else
                    {
                        VisitBorder(parentIndex, l1Voxel, _l2Desc, 2, dx, dy, dz);
                    }
                }
            }
        }
    }

    private void VisitNeighbourIfEmpty(int parentIndex, ulong nodeVoxel)
    {
        if (_volume.IsEmpty(nodeVoxel))
            VisitNeighbour(parentIndex, nodeVoxel);
    }

    private void VisitNeighbour(int parentIndex, ulong nodeVoxel)
    {
        if (!_nodeLookup.TryGetValue(nodeVoxel, out var nodeIndex))
        {
            // first time we're visiting this node, calculate heuristic
            nodeIndex = _nodes.Count;
            _nodes.Add(new() { GScore = float.MaxValue, HScore = float.MaxValue, Voxel = nodeVoxel, ParentIndex = parentIndex, OpenHeapIndex = -1 });
            _nodeLookup[nodeVoxel] = nodeIndex;
            ++_generatedNodes;
        }
        else if (!_allowReopen && _nodes[nodeIndex].OpenHeapIndex < 0)
        {
            // in closed list already - TODO: is it possible to visit again with lower cost?..
            return;
        }

        var nodeSpan = NodeSpan;
        ref var parentNode = ref nodeSpan[parentIndex];
        var enterPos = nodeVoxel == _goalVoxel ? _goalPos : _volume.ClampPointToVoxel(nodeVoxel, parentNode.Position);
        var nodeG = CalculateGScore(ref parentNode, nodeVoxel, enterPos, ref parentIndex);
        ref var curNode = ref nodeSpan[nodeIndex];
        if (nodeG + 0.00001f < curNode.GScore)
        {
            // new path is better
            curNode.GScore = nodeG;
            curNode.HScore = HeuristicDistance(nodeVoxel, enterPos);
            curNode.ParentIndex = parentIndex;
            curNode.Position = enterPos;
            AddToOpen(nodeIndex);
            //Service.Log.Debug($"volume pathfind: adding {nodeVoxel:X} (#{nodeIndex}), parent={parentIndex}, g={nodeG:f3}, h={_nodes[nodeIndex].HScore:f3}");

            if (curNode.HScore < _nodes[_bestNodeIndex].HScore)
                _bestNodeIndex = nodeIndex;
        }
    }

    private float CalculateGScore(ref Node parent, ulong destVoxel, Vector3 destPos, ref int parentIndex)
    {
        float randomFactor = _randomnessMultiplier > 0 ? (float)Random.Shared.NextDouble() * _randomnessMultiplier : 0;

        float baseDistance;
        float parentBaseG;
        Vector3 fromPos;

        if (_useRaycast)
        {
            // check LoS from grandparent
            int grandParentIndex = parent.ParentIndex;
            ref var grandParentNode = ref NodeSpan[grandParentIndex];
            // TODO: invert LoS check to match path reconstruction step?
            var distanceSquared = (grandParentNode.Position - destPos).LengthSquared();
            if (distanceSquared <= _raycastLimitSq)
            {
                ++_lineOfSightChecks;
                if (VoxelSearch.LineOfSight(_volume, grandParentNode.Voxel, destVoxel, grandParentNode.Position, destPos))
                {
                    ++_lineOfSightHits;
                    parentIndex = grandParentIndex;
                    baseDistance = MathF.Sqrt(distanceSquared);
                    parentBaseG = grandParentNode.GScore;
                    fromPos = grandParentNode.Position;
                }
                else
                {
                    baseDistance = (parent.Position - destPos).Length();
                    parentBaseG = parent.GScore;
                    fromPos = parent.Position;
                }
            }
            else
            {
                baseDistance = (parent.Position - destPos).Length();
                parentBaseG = parent.GScore;
                fromPos = parent.Position;
            }
        }
        else
        {
            baseDistance = (parent.Position - destPos).Length();
            parentBaseG = parent.GScore;
            fromPos = parent.Position;
        }

        float verticalDifference = MathF.Abs(fromPos.Y - destPos.Y);
        float verticalPenalty = 0.2f * verticalDifference;

        return parentBaseG + baseDistance + randomFactor + verticalPenalty;
    }

    private float HeuristicDistance(ulong nodeVoxel, Vector3 v) => nodeVoxel != _goalVoxel ? (v - _goalPos).Length() * 0.999f : 0;

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
        // update location
        PercolateUp(node.OpenHeapIndex);
    }

    // remove first (minimal) node from open heap and mark as closed
    private int PopMinOpen()
    {
        var nodeSpan = NodeSpan;
        int nodeIndex = _openList[0];
        _openList[0] = _openList[_openList.Count - 1];
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
        var nodeSpan = NodeSpan;
        int nodeIndex = _openList[heapIndex];
        int parent = (heapIndex - 1) >> 1;
        while (heapIndex > 0 && HeapLess(ref nodeSpan[nodeIndex], ref nodeSpan[_openList[parent]]))
        {
            _openList[heapIndex] = _openList[parent];
            nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
            heapIndex = parent;
            parent = (heapIndex - 1) >> 1;
        }
        _openList[heapIndex] = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private void PercolateDown(int heapIndex)
    {
        var nodeSpan = NodeSpan;
        int nodeIndex = _openList[heapIndex];
        int maxSize = _openList.Count;
        while (true)
        {
            int child1 = (heapIndex << 1) + 1;
            if (child1 >= maxSize)
                break;
            int child2 = child1 + 1;
            if (child2 == maxSize || HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[_openList[child2]]))
            {
                if (HeapLess(ref nodeSpan[_openList[child1]], ref nodeSpan[nodeIndex]))
                {
                    _openList[heapIndex] = _openList[child1];
                    nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                    heapIndex = child1;
                }
                else
                {
                    break;
                }
            }
            else if (HeapLess(ref nodeSpan[_openList[child2]], ref nodeSpan[nodeIndex]))
            {
                _openList[heapIndex] = _openList[child2];
                nodeSpan[_openList[heapIndex]].OpenHeapIndex = heapIndex;
                heapIndex = child2;
            }
            else
            {
                break;
            }
        }
        _openList[heapIndex] = nodeIndex;
        nodeSpan[nodeIndex].OpenHeapIndex = heapIndex;
    }

    private bool HeapLess(ref Node nodeL, ref Node nodeR)
    {
        var fl = nodeL.GScore + nodeL.HScore;
        var fr = nodeR.GScore + nodeR.HScore;
        if (fl + 0.00001f < fr)
            return true;
        else if (fr + 0.00001f < fl)
            return false;
        else
            return nodeL.GScore > nodeR.GScore; // tie-break towards larger g-values
    }
}
