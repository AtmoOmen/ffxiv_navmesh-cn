using System.Numerics;
using vnavmesh.Common.Build.Flight;

namespace vnavmesh.Query.Flight;

public partial class VoxelPathfind
{
    private readonly Lock                            sync         = new();
    private readonly Dictionary<ulong, ulong[]>      regionByTile = [];
    private readonly Dictionary<ulong, List<ushort>> regionCells  = [];
    private readonly Dictionary<ulong, ushort>       regionFaces  = [];

    internal Vector3? FindNearestConnectedPoint(ulong source, Vector3 point, Vector3 halfExtent)
    {
        lock (sync)
        {
            var searchMin    = point - halfExtent;
            var searchMax    = point + halfExtent;
            var sourceRegion = ResolveTopologyRegion(source);
            var sourceBounds = Volume.VoxelBounds(source, 0);
            var sourcePoint  = (sourceBounds.min + sourceBounds.max) * 0.5f;
            var candidates   = new PriorityQueue<TopologyCandidate, float>();

            foreach (var tileCandidate in EnumerateRootTiles(point, searchMin, searchMax))
            {
                var connected = TakeConnectedCandidate(sourceRegion, sourcePoint, candidates, tileCandidate.Distance);
                if (connected.HasValue)
                    return connected;

                var tile      = GetTopologyTile(tileCandidate.Tile);
                var points    = new Vector3[tile.Regions.Length];
                var distances = new float[tile.Regions.Length];
                Array.Fill(distances, float.PositiveInfinity);

                if (tile.Columns.Length == 0)
                    AddCandidate(VoxelMap.EncodeIndex(tileCandidate.Tile), 0);
                else
                {
                    var inset     = l2Desc.CellSize * 0.25f;
                    var intervals = new Stack<(int Start, int End)>();

                    for (var column = 0; column < tile.Columns.Length - 1; column++)
                    for (var runIndex = tile.Columns[column]; runIndex < tile.Columns[column + 1]; runIndex++)
                    {
                        var run   = tile.Runs[runIndex];
                        var cellX = column % tile.SizeX;
                        var cellZ = column / tile.SizeX;
                        intervals.Push((run.Start, run.End));

                        while (intervals.TryPop(out var interval))
                        {
                            var runMin = tile.Min + (new Vector3(cellX,     interval.Start, cellZ)     * l2Desc.CellSize);
                            var runMax = tile.Min + (new Vector3(cellX + 1, interval.End,   cellZ + 1) * l2Desc.CellSize);
                            var min    = Vector3.Max(runMin, searchMin);
                            var max    = Vector3.Min(runMax, searchMax);
                            if (min.X > max.X || min.Y > max.Y || min.Z > max.Z)
                                continue;
                            var nearest = Vector3.Clamp(point, min, max);
                            if (Vector3.DistanceSquared(point, nearest) >= distances[run.Component])
                                continue;

                            var probe = Vector3.Clamp(nearest, runMin + inset, runMax - inset);
                            var voxel = Volume.FindLeafVoxel(probe).voxel;
                            AddCandidate(voxel, run.Component);
                            var bounds = Volume.VoxelBounds(voxel, 0);
                            var first  = Math.Clamp((int)MathF.Round((bounds.min.Y - tile.Min.Y) * l2Desc.InvCellSize.Y), interval.Start, interval.End);
                            var last   = Math.Clamp((int)MathF.Round((bounds.max.Y - tile.Min.Y) * l2Desc.InvCellSize.Y), interval.Start, interval.End);
                            if (first > interval.Start)
                                intervals.Push((interval.Start, first));
                            if (last < interval.End)
                                intervals.Push((last, interval.End));
                        }
                    }
                }

                for (var component = 0; component < distances.Length; component++)
                    if (float.IsFinite(distances[component]))
                        candidates.Enqueue(new(new(tileCandidate.Tile, component), points[component]), distances[component]);

                continue;

                void AddCandidate(ulong voxel, int component)
                {
                    var bounds = Volume.VoxelBounds(voxel, 0.1f);
                    var min    = Vector3.Max(bounds.min, searchMin);
                    var max    = Vector3.Min(bounds.max, searchMax);
                    if (min.X > max.X || min.Y > max.Y || min.Z > max.Z)
                        return;

                    var candidate = Vector3.Clamp(point, min, max);
                    var distance  = Vector3.DistanceSquared(point, candidate);
                    if (distance >= distances[component])
                        return;
                    distances[component] = distance;
                    points[component]    = candidate;
                }
            }

            return TakeConnectedCandidate(sourceRegion, sourcePoint, candidates, float.PositiveInfinity);
        }
    }

    private Vector3? TakeConnectedCandidate(TopologyRegion source, Vector3 sourcePoint, PriorityQueue<TopologyCandidate, float> candidates, float maxDistance)
    {
        while (candidates.TryPeek(out var candidate, out var distance) && distance <= maxDistance)
        {
            candidates.Dequeue();
            if (AreTopologyRegionsConnected(source, candidate.Region, sourcePoint, candidate.Point))
                return candidate.Point;
        }

        return null;
    }

    private IEnumerable<(ushort Tile, float Distance)> EnumerateRootTiles(Vector3 point, Vector3 searchMin, Vector3 searchMax)
    {
        var root     = Volume.RootTile;
        var minCell  = l0Desc.ClampMin(root.WorldToVoxel(searchMin));
        var maxCell  = l0Desc.ClampMax(root.WorldToVoxel(searchMax));
        var frontier = new PriorityQueue<VoxelRange, float>();
        Enqueue(new(new(minCell.x, minCell.y, minCell.z), new(maxCell.x, maxCell.y, maxCell.z)));

        while (frontier.TryDequeue(out var current, out var distance))
        {
            if (current.Min == current.Max)
            {
                var index = l0Desc.VoxelToIndex((int)current.Min.X, (int)current.Min.Y, (int)current.Min.Z);
                yield return (index, distance);
                continue;
            }

            var extent   = (current.Max - current.Min) * l0Desc.CellSize;
            var lowerMax = current.Max;
            var upperMin = current.Min;

            if (extent.X >= extent.Y && extent.X >= extent.Z)
            {
                lowerMax.X = MathF.Floor((current.Min.X + current.Max.X) * 0.5f);
                upperMin.X = lowerMax.X + 1;
            }
            else if (extent.Y >= extent.Z)
            {
                lowerMax.Y = MathF.Floor((current.Min.Y + current.Max.Y) * 0.5f);
                upperMin.Y = lowerMax.Y + 1;
            }
            else
            {
                lowerMax.Z = MathF.Floor((current.Min.Z + current.Max.Z) * 0.5f);
                upperMin.Z = lowerMax.Z + 1;
            }

            Enqueue(current with { Max = lowerMax });
            Enqueue(current with { Min = upperMin });
        }

        yield break;

        void Enqueue(VoxelRange range)
        {
            if (range.Min.X > range.Max.X || range.Min.Y > range.Max.Y || range.Min.Z > range.Max.Z)
                return;

            var inset = Vector3.Zero;

            if (range.Min == range.Max)
            {
                var index = l0Desc.VoxelToIndex((int)range.Min.X, (int)range.Min.Y, (int)range.Min.Z);
                var cell  = root.GetCell(index);
                if (cell == ushort.MaxValue)
                    return;

                if ((cell & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                    inset = new(0.1f);
            }

            var min = Vector3.Max(root.BoundsMin                                                 + (range.Min * l0Desc.CellSize) + inset, searchMin);
            var max = Vector3.Min(root.BoundsMin + ((range.Max + Vector3.One) * l0Desc.CellSize) - inset, searchMax);
            if (min.X > max.X || min.Y > max.Y || min.Z > max.Z)
                return;

            frontier.Enqueue(range, Vector3.DistanceSquared(point, Vector3.Clamp(point, min, max)));
        }
    }

    private ulong ResolveRegion(ulong voxel)
    {
        var encoded   = voxel;
        var rootIndex = VoxelMap.DecodeIndex(ref encoded);
        var tileIndex = VoxelMap.DecodeIndex(ref encoded);
        var root      = Volume.RootTile;

        if ((root.GetCell(rootIndex) & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
            return VoxelMap.EncodeIndex(rootIndex);

        var middle = root.GetSubdivision(root.GetSubdivisionIndex(rootIndex));
        var parent = VoxelMap.EncodeIndex(rootIndex, VoxelMap.EncodeIndex(tileIndex));

        if ((middle.GetCell(tileIndex) & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
            return parent;

        var tile  = middle.GetSubdivision(middle.GetSubdivisionIndex(tileIndex));
        var level = tile.LevelDesc;
        var seed  = VoxelMap.DecodeIndex(ref encoded);

        if (!regionByTile.TryGetValue(parent, out var labels))
        {
            labels = new ulong[level.NumCellsTotal];
            regionByTile.Add(parent, labels);
        }

        if (labels[seed] != 0)
            return labels[seed];

        var frontier = new Queue<ushort>();
        labels[seed] = voxel;
        var cells = new List<ushort>();
        regionCells.Add(voxel, cells);
        ushort faces = 0;
        frontier.Enqueue(seed);

        while (frontier.TryDequeue(out var current))
        {
            var coordinates = level.IndexToVoxel(current);
            cells.Add(current);
            if (coordinates.y == 0)
                faces |= 1 << L1_FACE_NEG_Y;
            if (coordinates.y == level.NumCellsY - 1)
                faces |= 1 << L1_FACE_POS_Y;
            if (coordinates.x == 0)
                faces |= 1 << L1_FACE_NEG_X;
            if (coordinates.x == level.NumCellsX - 1)
                faces |= 1 << L1_FACE_POS_X;
            if (coordinates.z == 0)
                faces |= 1 << L1_FACE_NEG_Z;
            if (coordinates.z == level.NumCellsZ - 1)
                faces |= 1 << L1_FACE_POS_Z;

            for (var direction = 0; direction < 6; direction++)
            {
                var offsetX = direction switch { 0 => -1, 1 => 1, _ => 0 };
                var offsetY = direction switch { 2 => -1, 3 => 1, _ => 0 };
                var offsetZ = direction switch { 4 => -1, 5 => 1, _ => 0 };
                var nextX   = coordinates.x + offsetX;
                var nextY   = coordinates.y + offsetY;
                var nextZ   = coordinates.z + offsetZ;

                if (!level.InBounds(nextX, nextY, nextZ))
                    continue;

                var index = level.VoxelToIndex(nextX, nextY, nextZ);
                if ((tile.GetCell(index) & VoxelMap.VOXEL_OCCUPIED_BIT) != 0)
                    continue;

                if (labels[index] != 0)
                    continue;

                labels[index] = voxel;
                frontier.Enqueue(index);
            }
        }

        regionFaces.Add(voxel, faces);
        return voxel;
    }

    private readonly record struct VoxelRange
    (
        Vector3 Min,
        Vector3 Max
    );

    private readonly record struct TopologyCandidate
    (
        TopologyRegion Region,
        Vector3        Point
    );
}
