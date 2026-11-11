using System.Buffers;
using System.Numerics;
using vnavmesh.Common.Build.Flight;

namespace vnavmesh.Query.Flight;

public partial class VoxelPathfind
{
    private readonly Dictionary<ushort, TopologyTile>             topologyTiles      = [];
    private readonly Dictionary<TopologyRegion, TopologyRegion[]> topologyNeighbours = [];
    private readonly Dictionary<TopologyRegion, int>              componentByRegion  = [];
    private readonly List<ConnectedComponent>                     components         = [];

    internal bool IsConnected(ulong source, ulong destination)
    {
        lock (sync)
        {
            if (source == destination)
                return true;

            var sourceRegion      = ResolveTopologyRegion(source);
            var destinationRegion = ResolveTopologyRegion(destination);
            var sourceBounds      = Volume.VoxelBounds(source,      0);
            var destinationBounds = Volume.VoxelBounds(destination, 0);
            return AreTopologyRegionsConnected
            (
                sourceRegion,
                destinationRegion,
                (sourceBounds.min      + sourceBounds.max)      * 0.5f,
                (destinationBounds.min + destinationBounds.max) * 0.5f
            );
        }
    }

    private bool AreTopologyRegionsConnected(TopologyRegion source, TopologyRegion destination, Vector3 sourcePoint, Vector3 destinationPoint)
    {
        var sourceComponent      = GetComponent(source);
        var destinationComponent = GetComponent(destination);
        if (sourceComponent == destinationComponent)
            return true;
        if (components[sourceComponent].Pending.Count == 0 || components[destinationComponent].Pending.Count == 0)
            return false;

        var sourceVoxel      = Volume.FindLeafVoxel(sourcePoint).voxel;
        var destinationVoxel = Volume.FindLeafVoxel(destinationPoint).voxel;

        if (VoxelSearch.LineOfSight(Volume, sourceVoxel, destinationVoxel, sourcePoint, destinationPoint))
        {
            Merge(sourceComponent, destinationComponent);
            return true;
        }

        var sourceFrontier      = new PriorityQueue<TopologyRegion, float>();
        var destinationFrontier = new PriorityQueue<TopologyRegion, float>();

        while (sourceComponent != destinationComponent)
        {
            if (components[sourceComponent].Pending.Count == 0 || components[destinationComponent].Pending.Count == 0)
                return false;

            var expandSource = components[sourceComponent].Count <= components[destinationComponent].Count;
            var frontier = expandSource ?
                               sourceFrontier :
                               destinationFrontier;
            var component = expandSource ?
                                sourceComponent :
                                destinationComponent;
            var targetPoint = expandSource ?
                                  destinationPoint :
                                  sourcePoint;

            if (!frontier.TryDequeue(out var current, out _))
            {
                foreach (var region in components[component].Pending)
                    frontier.Enqueue(region, TopologyDistanceSquared(region, targetPoint));
                continue;
            }

            if (!components[component].Pending.Contains(current))
                continue;

            var tile = topologyTiles[current.Tile];

            if (tile.Columns.Length == 0)
            {
                var probe      = Vector3.Clamp(targetPoint, tile.Min + new Vector3(0.1f), tile.Max - new Vector3(0.1f));
                var probeVoxel = Volume.FindLeafVoxel(probe).voxel;
                var targetVoxel = expandSource ?
                                      destinationVoxel :
                                      sourceVoxel;

                if (VoxelSearch.LineOfSight(Volume, probeVoxel, targetVoxel, probe, targetPoint))
                {
                    Merge(sourceComponent, destinationComponent);
                    return true;
                }
            }

            foreach (var neighbour in GetTopologyNeighbours(current))
            {
                if (componentByRegion.TryGetValue(neighbour, out var existing))
                {
                    component = Merge(component, FindComponent(existing));
                    if (FindComponent(sourceComponent) == FindComponent(destinationComponent))
                        return true;
                    continue;
                }

                componentByRegion.Add(neighbour, component);
                components[component].Count++;
                components[component].Pending.Add(neighbour);
                frontier.Enqueue(neighbour, TopologyDistanceSquared(neighbour, targetPoint));
            }

            components[component].Pending.Remove(current);
            sourceComponent      = FindComponent(sourceComponent);
            destinationComponent = FindComponent(destinationComponent);
        }

        return true;
    }

    private TopologyRegion ResolveTopologyRegion(ulong voxel)
    {
        var encoded   = voxel;
        var rootIndex = VoxelMap.DecodeIndex(ref encoded);
        var bounds    = Volume.VoxelBounds(voxel, 0);
        var point     = (bounds.min + bounds.max) * 0.5f;
        var tile      = GetTopologyTile(rootIndex);
        if (tile.Columns.Length == 0)
            return new(rootIndex, 0);

        var coordinates = (point - tile.Min) * l2Desc.InvCellSize;
        var column      = tile.GetColumn((int)coordinates.X, (int)coordinates.Z);
        foreach (var run in column)
            if (coordinates.Y >= run.Start && coordinates.Y < run.End)
                return new(rootIndex, run.Component);

        throw new InvalidOperationException("空体素未包含在局部连通分量中");
    }

    private TopologyTile GetTopologyTile(ushort rootIndex)
    {
        if (topologyTiles.TryGetValue(rootIndex, out var existing))
            return existing;

        var root   = Volume.RootTile;
        var bounds = root.CalculateSubdivisionBounds(l0Desc.IndexToVoxel(rootIndex));
        var sizeX  = l1Desc.NumCellsX * l2Desc.NumCellsX;
        var sizeY  = l1Desc.NumCellsY * l2Desc.NumCellsY;
        var sizeZ  = l1Desc.NumCellsZ * l2Desc.NumCellsZ;
        var cell   = root.GetCell(rootIndex);

        if ((cell & VoxelMap.VOXEL_OCCUPIED_BIT) == 0 || cell == ushort.MaxValue)
        {
            var empty = cell != ushort.MaxValue;
            TopologyRun[] uniformRuns = empty ?
                                            [new(0, sizeY, 0)] :
                                            [];
            TopologyRegionInfo[] uniformRegions = empty ?
                                                      [new(bounds.min, bounds.max, sizeX * sizeY * sizeZ)] :
                                                      [];
            var uniform = new TopologyTile(sizeX, sizeY, sizeZ, bounds.min, bounds.max, uniformRuns, [], uniformRegions, []);
            topologyTiles.Add(rootIndex, uniform);
            return uniform;
        }

        var   middle         = root.GetSubdivision(cell & VoxelMap.VOXEL_ID_MASK);
        var   wordsPerColumn = (sizeY + 63) / 64;
        var   columnCount    = sizeX        * sizeZ;
        var   emptyColumns   = ArrayPool<ulong>.Shared.Rent(columnCount * wordsPerColumn);
        int[] parents        = [];
        int[] sizes          = [];

        try
        {
            Array.Clear(emptyColumns, 0, columnCount * wordsPerColumn);

            for (var middleZ = 0; middleZ < l1Desc.NumCellsZ; middleZ++)
            for (var middleX = 0; middleX < l1Desc.NumCellsX; middleX++)
            for (var middleY = 0; middleY < l1Desc.NumCellsY; middleY++)
            {
                var middleCell = middle.GetCell(l1Desc.VoxelToIndex(middleX, middleY, middleZ));
                if (middleCell == ushort.MaxValue)
                    continue;

                var baseY = middleY * l2Desc.NumCellsY;
                var baseX = middleX * l2Desc.NumCellsX;
                var baseZ = middleZ * l2Desc.NumCellsZ;

                for (var leafZ = 0; leafZ < l2Desc.NumCellsZ; leafZ++)
                for (var leafX = 0; leafX < l2Desc.NumCellsX; leafX++)
                {
                    var column = ((baseZ + leafZ) * sizeX) + baseX + leafX;

                    if ((middleCell & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                    {
                        SetEmptySpan(column, baseY, baseY + l2Desc.NumCellsY);
                        continue;
                    }

                    var leaf  = middle.GetSubdivision(middleCell & VoxelMap.VOXEL_ID_MASK);
                    var start = -1;

                    for (var leafY = 0; leafY < l2Desc.NumCellsY; leafY++)
                    {
                        var empty = (leaf.GetCell(l2Desc.VoxelToIndex(leafX, leafY, leafZ)) & VoxelMap.VOXEL_OCCUPIED_BIT) == 0;

                        if (empty)
                        {
                            if (start < 0)
                                start = leafY;
                        }
                        else if (start >= 0)
                        {
                            SetEmptySpan(column, baseY + start, baseY + leafY);
                            start = -1;
                        }
                    }

                    if (start >= 0)
                        SetEmptySpan(column, baseY + start, baseY + l2Desc.NumCellsY);
                }
            }

            var columns  = new int[columnCount + 1];
            var runCount = 0;

            for (var column = 0; column < columnCount; column++)
            {
                columns[column] = runCount;
                var previousFilled = false;

                for (var wordIndex = 0; wordIndex < wordsPerColumn; wordIndex++)
                {
                    var word   = emptyColumns[(column * wordsPerColumn) + wordIndex];
                    var starts = word & ~(word << 1);
                    if (previousFilled)
                        starts &= ~1UL;
                    runCount       += BitOperations.PopCount(starts);
                    previousFilled =  (word & (1UL << 63)) != 0;
                }
            }

            columns[^1] = runCount;
            var runs    = new TopologyRun[runCount];
            var written = 0;

            for (var column = 0; column < columnCount; column++)
            for (var wordIndex = 0; wordIndex < wordsPerColumn; wordIndex++)
            {
                var word = emptyColumns[(column * wordsPerColumn) + wordIndex];

                while (word != 0)
                {
                    var start     = BitOperations.TrailingZeroCount(word);
                    var end       = start            + BitOperations.TrailingZeroCount(~(word >> start));
                    var firstCell = (wordIndex * 64) + start;
                    var lastCell  = (wordIndex * 64) + end;
                    if (written > columns[column] && runs[written - 1].End == firstCell)
                        runs[written - 1] = runs[written          - 1] with { End = lastCell };
                    else
                        runs[written++] = new(firstCell, lastCell, -1);
                    word = end == 64 ?
                               0 :
                               word & (ulong.MaxValue << end);
                }
            }

            parents = ArrayPool<int>.Shared.Rent(runCount);
            sizes   = ArrayPool<int>.Shared.Rent(runCount);

            for (var index = 0; index < runCount; index++)
            {
                parents[index] = index;
                sizes[index]   = 1;
            }

            for (var column = 0; column < columns.Length - 1; column++)
            {
                if (column % sizeX != 0)
                    UnionColumns(column, column - 1);
                if (column >= sizeX)
                    UnionColumns(column, column - sizeX);
            }

            var regionByLabel = new Dictionary<int, int>();
            var regions       = new List<TopologyRegionInfo>();

            for (var column = 0; column < columns.Length - 1; column++)
            for (var index = columns[column]; index < columns[column + 1]; index++)
            {
                var label = FindRun(index);

                if (!regionByLabel.TryGetValue(label, out var component))
                {
                    component = regions.Count;
                    regionByLabel.Add(label, component);
                    regions.Add(new(new(float.PositiveInfinity), new(float.NegativeInfinity), 0));
                }

                var run    = runs[index];
                var cellX  = column % sizeX;
                var cellZ  = column / sizeX;
                var min    = bounds.min + (new Vector3(cellX,     run.Start, cellZ)     * l2Desc.CellSize);
                var max    = bounds.min + (new Vector3(cellX + 1, run.End,   cellZ + 1) * l2Desc.CellSize);
                var region = regions[component];
                region.Min   =  Vector3.Min(region.Min, min);
                region.Max   =  Vector3.Max(region.Max, max);
                region.Cells += run.End - run.Start;
                runs[index]  =  run with { Component = component };
            }

            var faceBits = new ulong[6][];

            for (var direction = 0; direction < 6; direction++)
            {
                var faceSize = direction switch
                {
                    < 2 => sizeX * sizeZ,
                    < 4 => sizeY * sizeZ,
                    _   => sizeX * sizeY
                };
                faceBits[direction] = new ulong[(faceSize + 63) / 64];
            }

            for (var column = 0; column < columnCount; column++)
            {
                var cellX     = column % sizeX;
                var cellZ     = column / sizeX;
                var firstWord = emptyColumns[column * wordsPerColumn];
                var lastWord  = emptyColumns[(column * wordsPerColumn) + ((sizeY - 1) / 64)];
                if ((firstWord & 1) != 0)
                    faceBits[L1_FACE_NEG_Y][column / 64] |= 1UL << (column % 64);
                if ((lastWord & (1UL << ((sizeY - 1) % 64))) != 0)
                    faceBits[L1_FACE_POS_Y][column / 64] |= 1UL << (column % 64);
                if (cellX != 0 && cellX != sizeX - 1 && cellZ != 0 && cellZ != sizeZ - 1)
                    continue;

                for (var wordIndex = 0; wordIndex < wordsPerColumn; wordIndex++)
                {
                    var word = emptyColumns[(column * wordsPerColumn) + wordIndex];

                    while (word != 0)
                    {
                        var cellY  = (wordIndex * 64)    + BitOperations.TrailingZeroCount(word);
                        var indexX = (cellZ     * sizeY) + cellY;
                        var indexZ = (cellX     * sizeY) + cellY;
                        if (cellX == 0)
                            faceBits[L1_FACE_NEG_X][indexX / 64] |= 1UL << (indexX % 64);
                        if (cellX == sizeX - 1)
                            faceBits[L1_FACE_POS_X][indexX / 64] |= 1UL << (indexX % 64);
                        if (cellZ == 0)
                            faceBits[L1_FACE_NEG_Z][indexZ / 64] |= 1UL << (indexZ % 64);
                        if (cellZ == sizeZ - 1)
                            faceBits[L1_FACE_POS_Z][indexZ / 64] |= 1UL << (indexZ % 64);
                        word &= word - 1;
                    }
                }
            }

            var result = new TopologyTile(sizeX, sizeY, sizeZ, bounds.min, bounds.max, runs, columns, [.. regions], faceBits);
            topologyTiles.Add(rootIndex, result);
            return result;

            void SetEmptySpan(int column, int start, int end)
            {
                var firstWord = start     / 64;
                var lastWord  = (end - 1) / 64;

                for (var wordIndex = firstWord; wordIndex <= lastWord; wordIndex++)
                {
                    var firstBit = wordIndex == firstWord ?
                                       start % 64 :
                                       0;
                    var endBit = wordIndex == lastWord ?
                                     ((end - 1) % 64) + 1 :
                                     64;
                    var mask = ulong.MaxValue << firstBit;
                    if (endBit < 64)
                        mask &= (1UL << endBit) - 1;
                    emptyColumns[(column * wordsPerColumn) + wordIndex] |= mask;
                }
            }

            int FindRun(int index)
            {
                while (parents[index] != index)
                {
                    parents[index] = parents[parents[index]];
                    index          = parents[index];
                }

                return index;
            }

            void UnionColumns(int firstColumn, int secondColumn)
            {
                var first  = columns[firstColumn];
                var second = columns[secondColumn];

                while (first < columns[firstColumn + 1] && second < columns[secondColumn + 1])
                {
                    var firstRun  = runs[first];
                    var secondRun = runs[second];

                    if (firstRun.Start < secondRun.End && secondRun.Start < firstRun.End)
                    {
                        var firstRoot  = FindRun(first);
                        var secondRoot = FindRun(second);

                        if (firstRoot != secondRoot)
                        {
                            if (sizes[firstRoot] < sizes[secondRoot])
                                (firstRoot, secondRoot) = (secondRoot, firstRoot);
                            parents[secondRoot] =  firstRoot;
                            sizes[firstRoot]    += sizes[secondRoot];
                        }
                    }

                    if (firstRun.End <= secondRun.End)
                        first++;
                    else
                        second++;
                }
            }
        }
        finally
        {
            ArrayPool<ulong>.Shared.Return(emptyColumns);
            if (parents.Length != 0)
                ArrayPool<int>.Shared.Return(parents);
            if (sizes.Length != 0)
                ArrayPool<int>.Shared.Return(sizes);
        }
    }

    private TopologyRegion[] GetTopologyNeighbours(TopologyRegion region)
    {
        if (topologyNeighbours.TryGetValue(region, out var existing))
            return existing;

        var tile        = topologyTiles[region.Tile];
        var info        = tile.Regions[region.Component];
        var coordinates = l0Desc.IndexToVoxel(region.Tile);
        var neighbours  = new HashSet<TopologyRegion>();

        for (var direction = 0; direction < 6; direction++)
        {
            var offsetX = direction switch { L1_FACE_NEG_X => -1, L1_FACE_POS_X => 1, _ => 0 };
            var offsetY = direction switch { L1_FACE_NEG_Y => -1, L1_FACE_POS_Y => 1, _ => 0 };
            var offsetZ = direction switch { L1_FACE_NEG_Z => -1, L1_FACE_POS_Z => 1, _ => 0 };
            if ((offsetX < 0 && info.Min.X > tile.Min.X) ||
                (offsetX > 0 && info.Max.X < tile.Max.X) ||
                (offsetY < 0 && info.Min.Y > tile.Min.Y) ||
                (offsetY > 0 && info.Max.Y < tile.Max.Y) ||
                (offsetZ < 0 && info.Min.Z > tile.Min.Z) ||
                (offsetZ > 0 && info.Max.Z < tile.Max.Z))
                continue;

            var neighbourX = coordinates.x + offsetX;
            var neighbourY = coordinates.y + offsetY;
            var neighbourZ = coordinates.z + offsetZ;
            if (!l0Desc.InBounds(neighbourX, neighbourY, neighbourZ))
                continue;

            var neighbourIndex = l0Desc.VoxelToIndex(neighbourX, neighbourY, neighbourZ);
            var other          = GetTopologyTile(neighbourIndex);
            if (other.Regions.Length == 0)
                continue;

            if (tile.Regions.Length == 1 && other.Regions.Length == 1)
            {
                if (tile.Columns.Length == 0 && other.Columns.Length == 0)
                    neighbours.Add(new(neighbourIndex, 0));
                else
                {
                    var face = tile.Faces.Length == 0 ?
                                   other.Faces[direction ^ 1] :
                                   tile.Faces[direction];

                    for (var wordIndex = 0; wordIndex < face.Length; wordIndex++)
                    {
                        var first = tile.Faces.Length == 0 ?
                                        ulong.MaxValue :
                                        tile.Faces[direction][wordIndex];
                        var second = other.Faces.Length == 0 ?
                                         ulong.MaxValue :
                                         other.Faces[direction ^ 1][wordIndex];
                        if ((first & second) == 0)
                            continue;
                        neighbours.Add(new(neighbourIndex, 0));
                        break;
                    }
                }

                continue;
            }

            var columnsX = offsetX == 0 ?
                               tile.SizeX :
                               1;
            var columnsZ = offsetZ == 0 ?
                               tile.SizeZ :
                               1;

            for (var columnZ = 0; columnZ < columnsZ; columnZ++)
            for (var columnX = 0; columnX < columnsX; columnX++)
            {
                var currentX = columnX;
                var currentZ = columnZ;
                var nextX    = columnX;
                var nextZ    = columnZ;

                if (offsetX != 0)
                {
                    currentX = offsetX < 0 ?
                                   0 :
                                   tile.SizeX - 1;
                    nextX = offsetX < 0 ?
                                other.SizeX - 1 :
                                0;
                }

                if (offsetZ != 0)
                {
                    currentZ = offsetZ < 0 ?
                                   0 :
                                   tile.SizeZ - 1;
                    nextZ = offsetZ < 0 ?
                                other.SizeZ - 1 :
                                0;
                }

                var currentColumn = tile.GetColumn(currentX, currentZ);
                var nextColumn    = other.GetColumn(nextX, nextZ);
                if (currentColumn.IsEmpty || nextColumn.IsEmpty)
                    continue;

                if (offsetY != 0)
                {
                    var currentRun = offsetY < 0 ?
                                         currentColumn[0] :
                                         currentColumn[^1];
                    var nextRun = offsetY < 0 ?
                                      nextColumn[^1] :
                                      nextColumn[0];
                    var currentTouches = offsetY              < 0 ?
                                             currentRun.Start == 0 :
                                             currentRun.End   == tile.SizeY;
                    var nextTouches = offsetY           < 0 ?
                                          nextRun.End   == other.SizeY :
                                          nextRun.Start == 0;
                    if (currentRun.Component == region.Component && currentTouches && nextTouches)
                        neighbours.Add(new(neighbourIndex, nextRun.Component));
                    continue;
                }

                var currentOffset = 0;
                var nextOffset    = 0;

                while (currentOffset < currentColumn.Length && nextOffset < nextColumn.Length)
                {
                    var currentRun = currentColumn[currentOffset];
                    var nextRun    = nextColumn[nextOffset];
                    if (currentRun.Component == region.Component && currentRun.Start < nextRun.End && nextRun.Start < currentRun.End)
                        neighbours.Add(new(neighbourIndex, nextRun.Component));
                    if (currentRun.End <= nextRun.End)
                        currentOffset++;
                    else
                        nextOffset++;
                }
            }
        }

        var result = neighbours.ToArray();
        topologyNeighbours.Add(region, result);
        return result;
    }

    private float TopologyDistanceSquared(TopologyRegion region, Vector3 point)
    {
        var info = topologyTiles[region.Tile].Regions[region.Component];
        return Vector3.DistanceSquared(Vector3.Clamp(point, info.Min, info.Max), point);
    }

    private int GetComponent(TopologyRegion region)
    {
        if (componentByRegion.TryGetValue(region, out var component))
            return FindComponent(component);

        component = components.Count;
        var created = new ConnectedComponent { Parent = component };
        created.Pending.Add(region);
        components.Add(created);
        componentByRegion.Add(region, component);
        return component;
    }

    private int FindComponent(int component)
    {
        while (components[component].Parent != component)
        {
            components[component].Parent = components[components[component].Parent].Parent;
            component                    = components[component].Parent;
        }

        return component;
    }

    private int Merge(int first, int second)
    {
        first  = FindComponent(first);
        second = FindComponent(second);
        if (first == second)
            return first;
        if (components[first].Count < components[second].Count)
            (first, second) = (second, first);

        var retained = components[first];
        var merged   = components[second];
        merged.Parent  =  first;
        retained.Count += merged.Count;
        retained.Pending.UnionWith(merged.Pending);
        merged.Pending.Clear();
        return first;
    }

    private readonly record struct TopologyRegion
    (
        ushort Tile,
        int    Component
    );

    private readonly record struct TopologyRun
    (
        int Start,
        int End,
        int Component
    );

    private sealed class TopologyRegionInfo
    (
        Vector3 min,
        Vector3 max,
        int     cells
    )
    {
        public Vector3 Min   = min;
        public Vector3 Max   = max;
        public int     Cells = cells;
    }

    private sealed record TopologyTile
    (
        int                  SizeX,
        int                  SizeY,
        int                  SizeZ,
        Vector3              Min,
        Vector3              Max,
        TopologyRun[]        Runs,
        int[]                Columns,
        TopologyRegionInfo[] Regions,
        ulong[][]            Faces
    )
    {
        public ReadOnlySpan<TopologyRun> GetColumn(int cellX, int cellZ)
        {
            if (Columns.Length == 0)
                return Runs;
            var column = (cellZ * SizeX)                            + cellX;
            return Runs.AsSpan(Columns[column], Columns[column + 1] - Columns[column]);
        }
    }

    private sealed class ConnectedComponent
    {
        public int Parent;
        public int Count = 1;

        public HashSet<TopologyRegion> Pending { get; } = [];
    }
}
