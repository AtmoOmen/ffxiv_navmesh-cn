using System.Numerics;
using vnavmesh.Query.Flight.Models;

namespace vnavmesh.Query.Flight;

public partial class VoxelPathfind
{
    private List<(ulong voxel, Vector3 p)> RunIntervalSearch(Vector3 from, Vector3 to, CancellationToken cancel)
    {
        lock (sync)
        {
            ResetSearchState();
            lastSearchAttempts = 1;
            var source               = LocateInterval(from);
            var destination          = LocateInterval(to);
            var requestedDestination = destination;
            var corridor             = BuildIntervalCorridor(source, ref destination, ref to, cancel);
            var sourceFrontier       = new PriorityQueue<IntervalKey, float>();
            var destinationFrontier  = new PriorityQueue<IntervalKey, float>();
            var sourceRecords        = new Dictionary<IntervalKey, IntervalVisit>();
            var destinationRecords   = new Dictionary<IntervalKey, IntervalVisit>();
            var sourceBounds         = IntervalBounds(source);
            var sourcePosition       = (sourceBounds.Min + sourceBounds.Max) * 0.5f;
            sourcePosition.Y = from.Y;
            var destinationBounds   = IntervalBounds(destination);
            var destinationPosition = (destinationBounds.Min + destinationBounds.Max) * 0.5f;
            destinationPosition.Y = to.Y;
            sourceRecords.Add(source, new(0, sourcePosition, source));
            destinationRecords.Add(destination, new(0, destinationPosition, destination));
            sourceFrontier.Enqueue(source, Vector3.Distance(sourcePosition,                to));
            destinationFrontier.Enqueue(destination, Vector3.Distance(destinationPosition, from));
            if (corridor.Count == 0)
                destinationFrontier.Clear();
            var best         = source;
            var bestDistance = float.PositiveInfinity;
            var joined       = false;

            while (sourceFrontier.Count > 0 && destinationFrontier.Count > 0)
            {
                var forward = sourceFrontier.Count <= destinationFrontier.Count;
                var frontier = forward ?
                                   sourceFrontier :
                                   destinationFrontier;
                var records = forward ?
                                  sourceRecords :
                                  destinationRecords;
                var opposite = forward ?
                                   destinationRecords :
                                   sourceRecords;
                var target = forward ?
                                 to :
                                 from;
                var current = frontier.Dequeue();
                var visit   = records[current];
                if (visit.Closed)
                    continue;

                visit.Closed = true;
                visitedNodes++;
                if ((visitedNodes & 0xff) == 0)
                    cancel.ThrowIfCancellationRequested();

                var distance = Vector3.DistanceSquared(visit.Position, to);

                if (forward && distance < bestDistance)
                {
                    best         = current;
                    bestDistance = distance;
                }

                if (opposite.ContainsKey(current))
                {
                    best        = current;
                    joined      = true;
                    goalReached = destination == requestedDestination;
                    break;
                }

                if (sourceRecords.Count + destinationRecords.Count >= MAX_NODE_COUNT)
                    break;

                var bounds = IntervalBounds(current);

                for (var direction = 0; direction < 6; direction++)
                {
                    var offsetX = direction switch { L1_FACE_NEG_X => -1, L1_FACE_POS_X => 1, _ => 0 };
                    var offsetZ = direction switch { L1_FACE_NEG_Z => -1, L1_FACE_POS_Z => 1, _ => 0 };
                    var offsetY = direction switch { L1_FACE_NEG_Y => -1, L1_FACE_POS_Y => 1, _ => 0 };
                    var tile    = topologyTiles[current.Tile];
                    var nextX   = (current.Column % tile.SizeX) + offsetX;
                    var nextZ   = (current.Column / tile.SizeX) + offsetZ;
                    var root    = l0Desc.IndexToVoxel(current.Tile);

                    if (nextX < 0)
                    {
                        root.x--;
                        nextX += tile.SizeX;
                    }
                    else if (nextX >= tile.SizeX)
                    {
                        root.x++;
                        nextX -= tile.SizeX;
                    }

                    if (nextZ < 0)
                    {
                        root.z--;
                        nextZ += tile.SizeZ;
                    }
                    else if (nextZ >= tile.SizeZ)
                    {
                        root.z++;
                        nextZ -= tile.SizeZ;
                    }

                    var reachesFace = offsetY switch
                    {
                        < 0 => tile.Runs[current.Run].Start == 0,
                        > 0 => tile.Runs[current.Run].End   == tile.SizeY,
                        _   => true
                    };
                    if (!reachesFace)
                        continue;
                    root.y += offsetY;

                    if (!l0Desc.InBounds(root))
                        continue;

                    var nextTileIndex = l0Desc.VoxelToIndex(root);
                    var nextTile      = GetTopologyTile(nextTileIndex);
                    var column        = (nextZ * nextTile.SizeX) + nextX;
                    var first = nextTile.Columns.Length == 0 ?
                                    0 :
                                    nextTile.Columns[column];
                    var end = nextTile.Columns.Length == 0 ?
                                  nextTile.Runs.Length :
                                  nextTile.Columns[column + 1];

                    for (var run = first; run < end; run++)
                    {
                        if (!corridor.Contains(new(nextTileIndex, nextTile.Runs[run].Component)))
                            continue;

                        var next       = new IntervalKey(nextTileIndex, column, run);
                        var nextBounds = IntervalBounds(next);
                        var minY       = MathF.Max(bounds.Min.Y, nextBounds.Min.Y);
                        var maxY       = MathF.Min(bounds.Max.Y, nextBounds.Max.Y);
                        var connected = offsetY switch
                        {
                            < 0 => nextTile.Runs[run].End   == nextTile.SizeY,
                            > 0 => nextTile.Runs[run].Start == 0,
                            _   => minY                     < maxY
                        };
                        if (!connected)
                            continue;

                        var position = (nextBounds.Min + nextBounds.Max) * 0.5f;
                        position.Y = offsetY == 0 ?
                                         Math.Clamp(visit.Position.Y, minY + (l2Desc.CellSize.Y * 0.25f), maxY - (l2Desc.CellSize.Y * 0.25f)) :
                                         Math.Clamp
                                             (visit.Position.Y, nextBounds.Min.Y + (l2Desc.CellSize.Y * 0.25f), nextBounds.Max.Y - (l2Desc.CellSize.Y * 0.25f));
                        var cost = visit.Cost                               +
                                   MathF.Abs(position.Y - visit.Position.Y) +
                                   Vector2.Distance(new(position.X, position.Z), new(visit.Position.X, visit.Position.Z));
                        if (records.TryGetValue(next, out var previous) && previous.Cost <= cost)
                            continue;

                        records[next] = new(cost, position, current);
                        frontier.Enqueue(next, cost + Vector3.Distance(position, target));
                    }
                }

                peakOpenListSize = Math.Max(peakOpenListSize, sourceFrontier.Count + destinationFrontier.Count);
            }

            generatedNodes = sourceRecords.Count + destinationRecords.Count;
            if (goalReached)
                lastTermination = VolumeSearchTermination.ReachedGoal;
            else if (joined || sourceFrontier.Count == 0 || destinationFrontier.Count == 0)
                lastTermination = VolumeSearchTermination.SearchExhausted;
            else
                lastTermination = VolumeSearchTermination.StepBudgetReached;
            var positions = BuildIntervalBranch(sourceRecords, best);
            positions.Add(from);
            positions.Reverse();

            if (joined)
            {
                positions.AddRange(BuildIntervalBranch(destinationRecords, best));
                positions.Add(to);
            }

            List<(ulong voxel, Vector3 p)> path = new(positions.Count);
            foreach (var position in positions)
                path.Add((Volume.FindLeafVoxel(position).voxel, position));
            Service.Log.Debug($"[算路] 飞行体素双向区间搜索完成：访问区间 = {visitedNodes}，生成区间 = {generatedNodes}，到达终点 = {goalReached}");
            return path;
        }
    }

    private HashSet<TopologyRegion> BuildIntervalCorridor(IntervalKey source, ref IntervalKey destination, ref Vector3 target, CancellationToken cancel)
    {
        var first    = new TopologyRegion(source.Tile,      topologyTiles[source.Tile].Runs[source.Run].Component);
        var last     = new TopologyRegion(destination.Tile, topologyTiles[destination.Tile].Runs[destination.Run].Component);
        var frontier = new PriorityQueue<TopologyRegion, float>();
        var parents  = new Dictionary<TopologyRegion, TopologyRegion> { [first] = first };
        var costs    = new Dictionary<TopologyRegion, float> { [first]          = 0 };
        var closed   = new HashSet<TopologyRegion>();
        frontier.Enqueue(first, 0);

        while (frontier.TryDequeue(out var current, out _))
        {
            if (!closed.Add(current))
                continue;

            cancel.ThrowIfCancellationRequested();
            coarseExpandedNodes++;

            if (current == last)
                break;

            var info   = topologyTiles[current.Tile].Regions[current.Component];
            var center = (info.Min + info.Max) * 0.5f;

            foreach (var neighbour in GetTopologyNeighbours(current))
            {
                if (closed.Contains(neighbour))
                    continue;

                var other       = topologyTiles[neighbour.Tile].Regions[neighbour.Component];
                var otherCenter = (other.Min + other.Max) * 0.5f;
                var cost        = costs[current] + Vector3.Distance(center, otherCenter);
                if (costs.TryGetValue(neighbour, out var previous) && previous <= cost)
                    continue;

                costs[neighbour]   = cost;
                parents[neighbour] = current;
                frontier.Enqueue(neighbour, cost + Vector3.Distance(otherCenter, target));
            }
        }

        if (!closed.Contains(last))
        {
            var bounds      = IntervalBounds(source);
            var sourcePoint = (bounds.Min + bounds.Max) * 0.5f;
            var sourceVoxel = Volume.FindLeafVoxel(sourcePoint).voxel;
            var root        = Volume.RootTile;
            var extent      = Vector3.Max(Vector3.Abs(root.BoundsMin - target), Vector3.Abs(root.BoundsMax - target));
            var nearest     = FindNearestConnectedPoint(sourceVoxel, target, extent, closed);
            if (!nearest.HasValue)
                return [];

            Service.Log.Debug($"[算路] 飞行终点与起点不连通：最近可达终点 = {nearest.Value:f3}，距请求终点 = {Vector3.Distance(nearest.Value, target):f3}，可达连通分量 = {closed.Count}");
            target      = nearest.Value;
            destination = LocateInterval(target);
            last        = new(destination.Tile, topologyTiles[destination.Tile].Runs[destination.Run].Component);
        }

        HashSet<TopologyRegion> corridor = [last];

        while (last != first)
        {
            last = parents[last];
            corridor.Add(last);
        }

        return corridor;
    }

    private static List<Vector3> BuildIntervalBranch(Dictionary<IntervalKey, IntervalVisit> records, IntervalKey current)
    {
        List<Vector3> positions = [];

        while (true)
        {
            var visit = records[current];
            positions.Add(visit.Position);
            if (visit.Parent == current)
                return positions;

            var departure = records[visit.Parent].Position;
            departure.Y = visit.Position.Y;
            if (Vector2.DistanceSquared(new(departure.X, departure.Z), new(visit.Position.X, visit.Position.Z)) > 0)
                positions.Add(departure);
            current = visit.Parent;
        }
    }

    private IntervalKey LocateInterval(Vector3 point)
    {
        var root   = l0Desc.VoxelToIndex(Volume.RootTile.WorldToVoxel(point));
        var tile   = GetTopologyTile(root);
        var cell   = (point - tile.Min) * l2Desc.InvCellSize;
        var column = ((int)cell.Z * tile.SizeX) + (int)cell.X;
        var first = tile.Columns.Length == 0 ?
                        0 :
                        tile.Columns[column];
        var end = tile.Columns.Length == 0 ?
                      tile.Runs.Length :
                      tile.Columns[column + 1];
        for (var run = first; run < end; run++)
            if (cell.Y >= tile.Runs[run].Start && cell.Y < tile.Runs[run].End)
                return new(root, column, run);

        throw new InvalidOperationException("路径端点未包含在空体素区间中");
    }

    private (Vector3 Min, Vector3 Max) IntervalBounds(IntervalKey key)
    {
        var tile  = topologyTiles[key.Tile];
        var run   = tile.Runs[key.Run];
        var cellX = key.Column % tile.SizeX;
        var cellZ = key.Column / tile.SizeX;
        return (tile.Min    + (new Vector3(cellX,     run.Start, cellZ)     * l2Desc.CellSize),
                   tile.Min + (new Vector3(cellX + 1, run.End,   cellZ + 1) * l2Desc.CellSize));
    }

    private readonly record struct IntervalKey
    (
        ushort Tile,
        int    Column,
        int    Run
    );

    private sealed class IntervalVisit
    (
        float       cost,
        Vector3     position,
        IntervalKey parent
    )
    {
        public float       Cost     { get; } = cost;
        public Vector3     Position { get; } = position;
        public IntervalKey Parent   { get; } = parent;
        public bool        Closed   { get; set; }
    }
}
