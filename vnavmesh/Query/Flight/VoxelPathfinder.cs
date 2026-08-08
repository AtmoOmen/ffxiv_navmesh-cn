using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight.Models;

namespace vnavmesh.Query.Flight;

public sealed partial class VoxelPathfinder
{
    public SparseVoxelOctree Volume { get; }

    private readonly List<VolumePathfindNode> nodes = new(1024);
    private VoxelNodeLookup nodeLookup = new(VoxelPathfinderConstants.MAX_NODE_LOOKUP_CAPACITY);
    private readonly List<int> openList = new(256);
    private readonly Dictionary<(ulong From, ulong To), bool> losCache = new(1024);

    private readonly int[]   layerDepths;
    private readonly float[] layerCellSizes;

    private int visitedNodes;
    private int generatedNodes;
    private int lineOfSightChecks;
    private int lineOfSightHits;
    private int peakOpenListSize;
    private int coarseExpandedNodes;
    private int searchAttempts;
    private VolumeSearchTermination lastTermination;

    private int                        layerDepth;
    private float                      layerCellSize;
    private (int MinX, int MinY, int MinZ, int MaxX, int MaxY, int MaxZ)? corridor;
    private ulong                      layerGoalVoxel;
    private Vector3                    layerGoalPos;
    private int                        goalNodeIndex;
    private int                        bestNodeIndex;
    private int                        globalRemainingBudget;

    public Span<VolumePathfindNode> NodeSpan => CollectionsMarshal.AsSpan(nodes);

    internal VolumeSearchTelemetry LastTelemetry => new
    (
        visitedNodes,
        generatedNodes,
        lineOfSightChecks,
        lineOfSightHits,
        peakOpenListSize,
        coarseExpandedNodes,
        lastTermination,
        searchAttempts,
        1f
    );

    public VoxelPathfinder
    (
        SparseVoxelOctree volume
    )
    {
        Volume         = volume;
        layerDepths    = volume.LayerDepths;
        layerCellSizes = volume.LayerCellSizes;
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

        if (fromVoxel == toVoxel || Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
        {
            lastTermination = VolumeSearchTermination.ReachedGoal;
            searchAttempts  = 1;
            visitedNodes    = 1;
            generatedNodes  = 1;
            nodes.Add
            (
                new()
                {
                    Voxel             = toVoxel,
                    Position          = toPos,
                    GScore            = 0,
                    HScore            = 0,
                    ParentIndex       = 0,
                    DiscoveredByIndex = 0,
                    Closed            = true
                }
            );
            nodeLookup.SetOrUpdate(toVoxel, 0);
            return [(toVoxel, toPos)];
        }

        if (TryLineOfSight(fromVoxel, fromPos, toVoxel, toPos))
        {
            lastTermination = VolumeSearchTermination.ReachedGoal;
            searchAttempts  = 1;
            visitedNodes    = 1;
            generatedNodes  = 1;
            return [(fromVoxel, fromPos), (toVoxel, toPos)];
        }

        var macroPath = SearchLayer(0, fromPos, toPos, null, VoxelPathfinderConstants.MACRO_EXPANSION_BUDGET, cancel);

        if (macroPath.Count == 0)
            return [];

        var midPath = RefinePathSegments
        (
            macroPath,
            1,
            layerCellSizes[0],
            VoxelPathfinderConstants.MID_SEGMENT_EXPANSION_BUDGET,
            cancel
        );
        var microPath = RefinePathSegments
        (
            midPath,
            2,
            layerCellSizes[1],
            VoxelPathfinderConstants.MICRO_SEGMENT_EXPANSION_BUDGET,
            cancel
        );
        var pulled     = StringPull(microPath, cancel);
        var normalized = NormalizeWaypoints(pulled, cancel);

        return returnIntermediatePoints ?
                   ExpandIntermediatePoints(normalized, cancel) :
                   normalized;
    }

    private List<(ulong voxel, Vector3 p)> SearchLayer
    (
        int                                                        layerIndex,
        Vector3                                                    fromPos,
        Vector3                                                    toPos,
        (int MinX, int MinY, int MinZ, int MaxX, int MaxY, int MaxZ)? corridorBox,
        int                                                        budget,
        CancellationToken                                          cancel
    )
    {
        ResetLayerState();
        layerDepth    = layerDepths[layerIndex];
        layerCellSize = layerCellSizes[layerIndex];
        corridor      = corridorBox;
        layerGoalPos  = toPos;
        ++searchAttempts;

        if (!TryResolveLayerPoint(fromPos, out var startVoxel, out var startPos))
            return [];
        if (!TryResolveLayerPoint(toPos, out var goalCellVoxel, out var resolvedGoalPos))
            return [];

        if (startVoxel == goalCellVoxel)
            return [(startVoxel, startPos), (goalCellVoxel, resolvedGoalPos)];

        layerGoalVoxel = goalCellVoxel | VoxelPathfinderConstants.VIRTUAL_GOAL_FLAG;
        layerGoalPos   = resolvedGoalPos;
        AddStartNode(startVoxel, startPos);

        var stepBudget = Math.Min(budget, globalRemainingBudget);
        lastTermination = Execute(cancel, stepBudget);

        if (goalNodeIndex >= 0)
            return BuildPathTo(goalNodeIndex);
        if (bestNodeIndex >= 0 && nodes[bestNodeIndex].ParentIndex != bestNodeIndex)
            return BuildPathTo(bestNodeIndex);
        return [];
    }

    private List<(ulong voxel, Vector3 p)> RefinePathSegments
    (
        List<(ulong voxel, Vector3 p)> waypoints,
        int                             layerIndex,
        float                           expandWorld,
        int                             segmentBudget,
        CancellationToken               cancel
    )
    {
        var result = new List<(ulong voxel, Vector3 p)>(waypoints.Count);
        result.Add(waypoints[0]);

        for (var i = 0; i < waypoints.Count - 1; ++i)
        {
            cancel.ThrowIfCancellationRequested();
            var a = waypoints[i];
            var b = waypoints[i + 1];
            var corridorBox = ComputeCorridor(a.p, b.p, expandWorld, layerDepths[layerIndex], layerCellSizes[layerIndex]);
            var segment = SearchLayer(layerIndex, a.p, b.p, corridorBox, segmentBudget, cancel);

            if (segment.Count > 1 && lastTermination == VolumeSearchTermination.ReachedGoal)
            {
                for (var j = 1; j < segment.Count; ++j)
                    result.Add(segment[j]);
            }
            else
            {
                result.Add(b);
            }
        }

        return result;
    }

    private (int MinX, int MinY, int MinZ, int MaxX, int MaxY, int MaxZ)? ComputeCorridor
    (
        Vector3 a,
        Vector3 b,
        float   expand,
        int     depth,
        float   cellSize
    )
    {
        var min = Vector3.Max(Vector3.Min(a, b) - new Vector3(expand), Volume.BoundsMin);
        var max = Vector3.Min(Vector3.Max(a, b) + new Vector3(expand), Volume.BoundsMax);
        var cellCount = 1 << depth;

        return
        (
            Math.Clamp((int)((min.X - Volume.BoundsMin.X) / cellSize), 0, cellCount - 1),
            Math.Clamp((int)((min.Y - Volume.BoundsMin.Y) / cellSize), 0, cellCount - 1),
            Math.Clamp((int)((min.Z - Volume.BoundsMin.Z) / cellSize), 0, cellCount - 1),
            Math.Clamp((int)((max.X - Volume.BoundsMin.X) / cellSize), 0, cellCount - 1),
            Math.Clamp((int)((max.Y - Volume.BoundsMin.Y) / cellSize), 0, cellCount - 1),
            Math.Clamp((int)((max.Z - Volume.BoundsMin.Z) / cellSize), 0, cellCount - 1)
        );
    }

    private void BeginQuery()
    {
        visitedNodes          = 0;
        generatedNodes        = 0;
        lineOfSightChecks     = 0;
        lineOfSightHits       = 0;
        peakOpenListSize      = 0;
        coarseExpandedNodes   = 0;
        searchAttempts        = 0;
        lastTermination       = VolumeSearchTermination.SearchExhausted;
        globalRemainingBudget = VoxelPathfinderConstants.GLOBAL_EXPANSION_BUDGET;
        nodes.Clear();
        nodeLookup.Clear();
        openList.Clear();
        losCache.Clear();
    }

    private void ResetLayerState()
    {
        nodes.Clear();
        nodeLookup.Clear();
        openList.Clear();
        goalNodeIndex = -1;
        bestNodeIndex = -1;
    }

    private void AddStartNode
    (
        ulong   startVoxel,
        Vector3 startPos
    )
    {
        var index = nodes.Count;
        nodes.Add
        (
            new()
            {
                Voxel             = startVoxel,
                Position          = startPos,
                GScore            = 0,
                HScore            = Vector3.Distance(startPos, layerGoalPos),
                ParentIndex       = index,
                DiscoveredByIndex = index,
                OpenHeapIndex     = -1,
                Closed            = false
            }
        );
        nodeLookup.Set(startVoxel, index);
        ++generatedNodes;
        bestNodeIndex = index;
        AddToOpen(index);
    }

    private bool TryResolveLayerPoint
    (
        Vector3        p,
        out ulong      voxel,
        out Vector3    resolved
    )
    {
        var cellCount = 1 << layerDepth;
        var rel = p - Volume.BoundsMin;
        var x = Math.Clamp((int)(rel.X / layerCellSize), 0, cellCount - 1);
        var y = Math.Clamp((int)(rel.Y / layerCellSize), 0, cellCount - 1);
        var z = Math.Clamp((int)(rel.Z / layerCellSize), 0, cellCount - 1);
        var cellVoxel = SparseVoxelOctree.EncodeCoord(layerDepth, x, y, z);

        if (!Volume.IsCellTraversable(layerDepth, x, y, z))
        {
            if (TryFindNearestTraversableCell(x, y, z, out var nx, out var ny, out var nz))
            {
                voxel    = SparseVoxelOctree.EncodeCoord(layerDepth, nx, ny, nz);
                resolved = LayerCellCenter(nx, ny, nz);
                return true;
            }

            voxel    = SparseVoxelOctree.INVALID_VOXEL;
            resolved = default;
            return false;
        }

        var leaf = Volume.FindLeafVoxel(p);

        if (leaf.empty && leaf.voxel != SparseVoxelOctree.INVALID_VOXEL)
        {
            voxel    = cellVoxel;
            resolved = p;
            return true;
        }

        if (TryFindNearestEmptyPointInCell(x, y, z, p, out var emptyPoint))
        {
            voxel    = cellVoxel;
            resolved = emptyPoint;
            return true;
        }

        if (TryFindNearestTraversableCell(x, y, z, out var rx, out var ry, out var rz))
        {
            voxel    = SparseVoxelOctree.EncodeCoord(layerDepth, rx, ry, rz);
            resolved = LayerCellCenter(rx, ry, rz);
            return true;
        }

        voxel    = SparseVoxelOctree.INVALID_VOXEL;
        resolved = default;
        return false;
    }

    private bool TryFindNearestEmptyPointInCell
    (
        int         cellX,
        int         cellY,
        int         cellZ,
        Vector3     p,
        out Vector3 emptyPoint
    )
    {
        var shift    = Volume.MaxDepth - layerDepth;
        var leafSpan = 1 << shift;
        var minX = cellX << shift;
        var minY = cellY << shift;
        var minZ = cellZ << shift;
        var maxX = minX + leafSpan - 1;
        var maxY = minY + leafSpan - 1;
        var maxZ = minZ + leafSpan - 1;

        var leafSize = Volume.LeafSize;
        var startX = Math.Clamp((int)((p.X - Volume.BoundsMin.X) / leafSize), minX, maxX);
        var startY = Math.Clamp((int)((p.Y - Volume.BoundsMin.Y) / leafSize), minY, maxY);
        var startZ = Math.Clamp((int)((p.Z - Volume.BoundsMin.Z) / leafSize), minZ, maxZ);

        var queue   = new Queue<(int X, int Y, int Z)>();
        var visited = new HashSet<int>();
        var cellCount = 1 << Volume.MaxDepth;
        queue.Enqueue((startX, startY, startZ));
        visited.Add(((startX * cellCount) + startY) * cellCount + startZ);
        var popped = 0;

        while (queue.Count > 0 && popped < VoxelPathfinderConstants.MAX_NEAREST_TRAVERSABLE_CELLS)
        {
            var (x, y, z) = queue.Dequeue();
            ++popped;
            var center = Volume.BoundsMin + new Vector3((x + 0.5f) * leafSize, (y + 0.5f) * leafSize, (z + 0.5f) * leafSize);
            var located = Volume.FindLeafVoxel(center);

            if (located.empty)
            {
                emptyPoint = center;
                return true;
            }

            foreach (var (dx, dy, dz) in VoxelNeighbourOffsets.All)
            {
                var nx = x + dx;
                var ny = y + dy;
                var nz = z + dz;

                if (nx < minX || nx > maxX || ny < minY || ny > maxY || nz < minZ || nz > maxZ)
                    continue;

                var key = ((nx * cellCount) + ny) * cellCount + nz;

                if (!visited.Add(key))
                    continue;

                queue.Enqueue((nx, ny, nz));
            }
        }

        emptyPoint = default;
        return false;
    }

    private bool TryFindNearestTraversableCell
    (
        int  x,
        int  y,
        int  z,
        out int nx,
        out int ny,
        out int nz
    )
    {
        var cellCount = 1 << layerDepth;
        var queue = new Queue<(int X, int Y, int Z)>();
        var visited = new HashSet<int>();
        queue.Enqueue((x, y, z));
        visited.Add(((x * cellCount) + y) * cellCount + z);
        var popped = 0;

        while (queue.Count > 0 && popped < VoxelPathfinderConstants.MAX_NEAREST_TRAVERSABLE_CELLS)
        {
            var (cx, cy, cz) = queue.Dequeue();
            ++popped;

            if (Volume.IsCellTraversable(layerDepth, cx, cy, cz))
            {
                nx = cx;
                ny = cy;
                nz = cz;
                return true;
            }

            foreach (var (dx, dy, dz) in VoxelNeighbourOffsets.All)
            {
                var qx = cx + dx;
                var qy = cy + dy;
                var qz = cz + dz;

                if ((uint)qx >= (uint)cellCount || (uint)qy >= (uint)cellCount || (uint)qz >= (uint)cellCount)
                    continue;
                if (corridor is { } c && !IsWithinCorridor(qx, qy, qz, c))
                    continue;

                var key = ((qx * cellCount) + qy) * cellCount + qz;
                if (!visited.Add(key))
                    continue;

                queue.Enqueue((qx, qy, qz));
            }
        }

        nx = 0;
        ny = 0;
        nz = 0;
        return false;
    }

    private bool IsWithinCorridor
    (
        int x,
        int y,
        int z
    )
    {
        if (corridor is not { } c)
            return true;

        return x >= c.MinX && x <= c.MaxX &&
               y >= c.MinY && y <= c.MaxY &&
               z >= c.MinZ && z <= c.MaxZ;
    }

    private bool IsWithinCorridor
    (
        int x,
        int y,
        int z,
        (int MinX, int MinY, int MinZ, int MaxX, int MaxY, int MaxZ) c
    ) => x >= c.MinX && x <= c.MaxX &&
        y >= c.MinY && y <= c.MaxY &&
        z >= c.MinZ && z <= c.MaxZ;

    private Vector3 LayerCellCenter
    (
        int x,
        int y,
        int z
    ) => Volume.BoundsMin + (new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * layerCellSize);

    private bool TryLineOfSight
    (
        ulong   fromVoxel,
        Vector3 fromPos,
        ulong   toVoxel,
        Vector3 toPos
    )
    {
        if (fromVoxel == toVoxel)
            return true;
        if (Vector3.DistanceSquared(fromPos, toPos) <= float.Epsilon)
            return false;

        var key = fromVoxel < toVoxel ?
                      (fromVoxel, toVoxel) :
                      (toVoxel, fromVoxel);

        if (losCache.TryGetValue(key, out var cached))
            return cached;

        ++lineOfSightChecks;
        var visible = VoxelSearch.LineOfSight(Volume, fromVoxel, toVoxel, fromPos, toPos);

        if (visible)
            ++lineOfSightHits;

        if (losCache.Count < VoxelPathfinderConstants.MAX_LOS_CACHE_SIZE)
            losCache[key] = visible;
        return visible;
    }

    public void ReleaseRetainedState()
    {
        nodes.Clear();
        nodes.TrimExcess();
        nodeLookup.Clear();
        nodeLookup.TrimExcess();
        openList.Clear();
        openList.TrimExcess();
        losCache.Clear();
    }
}
