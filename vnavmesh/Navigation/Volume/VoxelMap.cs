using System.Numerics;
using vnavmesh.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Volume;

public class VoxelMap
{
    public Level[] Levels   { get; init; }
    public Tile    RootTile { get; init; }

    private readonly int[]         leafScaleX;
    private readonly int[]         leafScaleY;
    private readonly int[]         leafScaleZ;
    private readonly SemaphoreSlim materializationGate = new(1, 1);

    private byte[]?           deferredTreePayload;
    private int               deferredTreeOffset;
    private int               deferredTreeLength;
    private Action<VoxelMap>? deferredTreeMaterializer;

    private static readonly byte[] SSubtreeCountByPackedState       = BuildSubtreeCountByPackedState();
    private static readonly byte[] SSubtreePrefixCountByPackedState = BuildSubtreePrefixCountByPackedState();

    public static ulong EncodeIndex(ushort tileIndex, ulong subIndex = INVALID_VOXEL) =>
        (subIndex << INDEX_LEVEL_SHIFT) + tileIndex;

    public static ulong EncodeSubIndex(ulong voxel, ushort tileIndex, int level)
    {
        var shift = INDEX_LEVEL_SHIFT * level;
        voxel &= ~((ulong)INDEX_LEVEL_MASK << shift);
        voxel |= (ulong)tileIndex << shift;
        return voxel;
    }

    public static ushort DecodeIndex(ref ulong index)
    {
        var tileIndex = (ushort)(index & INDEX_LEVEL_MASK);
        index >>= INDEX_LEVEL_SHIFT;
        return tileIndex;
    }

    private static int PackedStateBytes(int numCells) => numCells + 3 >> 2;

    private static PackedCellState ClassifyPackedCellState(ushort value) => value switch
    {
        0               => PackedCellState.Empty,
        ushort.MaxValue => PackedCellState.SolidLeaf,
        _               => PackedCellState.Subtree
    };

    private static ushort[]? BuildSubtreePrefixCounts(byte[] packedStates)
    {
        if (packedStates.Length == 0)
            return null;

        var    counts  = new ushort[packedStates.Length];
        ushort running = 0;

        for (var i = 0; i < packedStates.Length; ++i)
        {
            counts[i] =  running;
            running   += SSubtreeCountByPackedState[packedStates[i]];
        }

        return running == 0 ? null : counts;
    }

    private static byte[] BuildSubtreeCountByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>(byte.MaxValue + 1);

        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            byte count = 0;
            for (var offset = 0; offset < 4; ++offset)
                if ((PackedCellState)(packedState >> offset * 2 & 0x3) == PackedCellState.Subtree)
                    ++count;

            table[packedState] = count;
        }

        return table;
    }

    private static byte[] BuildSubtreePrefixCountByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>((byte.MaxValue + 1) * 4);

        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            byte prefix = 0;

            for (var offset = 0; offset < 4; ++offset)
            {
                table[packedState * 4 + offset] = prefix;
                if ((PackedCellState)(packedState >> offset * 2 & 0x3) == PackedCellState.Subtree)
                    ++prefix;
            }
        }

        return table;
    }

    public VoxelMap(Vector3 boundsMin, Vector3 boundsMax, int[] tilesPerLevel)
    {
        Levels = new Level[tilesPerLevel.Length];
        var levelExtent = boundsMax - boundsMin;

        for (var i = 0; i < Levels.Length; ++i)
        {
            Levels[i]   = new(levelExtent, tilesPerLevel[i]);
            levelExtent = Levels[i].CellSize;
        }

        RootTile       = new(this, boundsMin, boundsMax, 0);
        leafScaleX     = GC.AllocateUninitializedArray<int>(Levels.Length);
        leafScaleY     = GC.AllocateUninitializedArray<int>(Levels.Length);
        leafScaleZ     = GC.AllocateUninitializedArray<int>(Levels.Length);
        leafScaleX[^1] = 1;
        leafScaleY[^1] = 1;
        leafScaleZ[^1] = 1;

        for (var i = Levels.Length - 2; i >= 0; --i)
        {
            leafScaleX[i] = leafScaleX[i + 1] * Levels[i + 1].NumCellsX;
            leafScaleY[i] = leafScaleY[i + 1] * Levels[i + 1].NumCellsY;
            leafScaleZ[i] = leafScaleZ[i + 1] * Levels[i + 1].NumCellsZ;
        }
    }

    public bool IsEmpty(ulong voxel)
    {
        EnsureMaterialized();
        var tile = RootTile;

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == INDEX_LEVEL_MASK)
                return false; // asking for non-leaf => consider non-empty

            var data = tile.GetCell(tileIndex);
            if ((data & VOXEL_OCCUPIED_BIT) == 0)
                return true; // found empty voxel
            data &= VOXEL_ID_MASK;
            if (data == VOXEL_ID_MASK)
                return false; // found non-empty leaf
            tile = tile.GetSubdivision(data);
        }
    }

    public (ulong voxel, bool empty) FindLeafVoxel(Vector3 p)
    {
        EnsureMaterialized();
        return RootTile.FindLeafVoxel(p);
    }

    public (Vector3 min, Vector3 max) VoxelBounds(ulong voxel, float eps)
    {
        EnsureMaterialized();
        var tile = RootTile;
        var eps3 = new Vector3(eps);

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == INDEX_LEVEL_MASK) return (tile.BoundsMin + eps3, tile.BoundsMax - eps3);

            var data = tile.GetCell(tileIndex);
            var id   = data & VOXEL_ID_MASK;

            if ((data & VOXEL_OCCUPIED_BIT) == 0 || id == VOXEL_ID_MASK)
            {
                var bb = tile.CalculateSubdivisionBounds(Levels[tile.Level].IndexToVoxel(tileIndex));
                return (bb.min + eps3, bb.max - eps3);
            }

            tile = tile.GetSubdivision(id);
        }
    }

    internal Vector3 ClampPointToVoxel(ulong voxel, Vector3 p, float eps = 0.1f)
    {
        EnsureMaterialized();
        var tile = RootTile;

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == INDEX_LEVEL_MASK)
                return Vector3.Clamp(p, tile.BoundsMin + new Vector3(eps), tile.BoundsMax - new Vector3(eps));

            var data = tile.GetCell(tileIndex);
            var id   = data & VOXEL_ID_MASK;

            if ((data & VOXEL_OCCUPIED_BIT) == 0 || id == VOXEL_ID_MASK)
            {
                var (min, max) = tile.CalculateSubdivisionBounds(Levels[tile.Level].IndexToVoxel(tileIndex));
                var eps3 = new Vector3(eps);
                return Vector3.Clamp(p, min + eps3, max - eps3);
            }

            tile = tile.GetSubdivision(id);
        }
    }

    internal bool HasDeferredTree => deferredTreePayload != null || deferredTreeMaterializer != null;

    internal void SetDeferredTreePayload(byte[] payload, int offset, int length)
    {
        deferredTreePayload      = payload;
        deferredTreeOffset       = offset;
        deferredTreeLength       = length;
        deferredTreeMaterializer = null;
    }

    internal void SetDeferredTreeMaterializer(Action<VoxelMap> materializer)
    {
        deferredTreePayload      = null;
        deferredTreeOffset       = 0;
        deferredTreeLength       = 0;
        deferredTreeMaterializer = materializer;
    }

    internal void EnsureMaterialized()
    {
        if (!HasDeferredTree)
            return;

        materializationGate.Wait();

        try
        {
            if (!HasDeferredTree)
                return;

            if (deferredTreePayload is { } payload)
            {
                Navmesh.MaterializeDeferredVolumeTree(this, payload, deferredTreeOffset, deferredTreeLength);
                deferredTreePayload = null;
                deferredTreeOffset  = 0;
                deferredTreeLength  = 0;
                return;
            }

            if (deferredTreeMaterializer is { } materializer)
            {
                materializer(this);
                deferredTreeMaterializer = null;
            }
        }
        finally
        {
            materializationGate.Release();
        }
    }

    internal void ReleaseRetainedState()
    {
        deferredTreePayload      = null;
        deferredTreeOffset       = 0;
        deferredTreeLength       = 0;
        deferredTreeMaterializer = null;
        RootTile.ReleaseRetainedState();
    }

    internal void CompactRetainedState()
    {
        if (HasDeferredTree)
            return;

        RootTile.CompactRetainedState();
    }

    public RootColumnBuildResult BuildRootColumn(Voxelizer vox, int tx, int tz)
    {
        var ny          = Levels[0].NumCellsY;
        var contents    = GC.AllocateUninitializedArray<ushort>(ny);
        var subdivision = new List<Tile>(64);
        for (var ty = 0; ty < ny; ++ty)
            contents[ty] = BuildTileContent(vox, RootTile, subdivision, null, tx, ty, tz, 0, ty * leafScaleY[0], 0);
        return new() { Contents = contents, Subdivision = subdivision };
    }

    private static int AppendSubdivision(List<Tile>? list, Tile? tile, Tile child)
    {
        if (tile != null)
        {
            var localId = tile.SubdivisionCount;
            tile.AddSubdivision(child);
            return localId;
        }

        if (list == null)
            throw new InvalidOperationException("体积子树写入目标缺失");

        var rootId = list.Count;
        list.Add(child);
        return rootId;
    }

    private ushort BuildTileContent
        (Voxelizer vox, Tile parent, List<Tile>? rootSubdivision, Tile? tileSubdivision, int rootX, int rootY, int rootZ, int leafX, int leafY, int leafZ)
    {
        var level = parent.Level;
        var (solid, empty) = vox.ClassifyRegion(leafX, leafY, leafZ, leafScaleX[level], leafScaleY[level], leafScaleZ[level]);
        if (!solid) return 0;

        if (!empty) return VOXEL_OCCUPIED_BIT | VOXEL_ID_MASK;

        var index = parent.LevelDesc.VoxelToIndex(rootX, rootY, rootZ);
        var (min, max) = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel(index));
        if (parent.Level + 1 >= Levels.Length)
            throw new InvalidOperationException("体积列构建遇到超出层级的混合体素");
        var tile    = new Tile(this, min, max, parent.Level + 1, false);
        var localId = AppendSubdivision(rootSubdivision, tileSubdivision, tile);

        ref var l           = ref Levels[tile.Level];
        var     childScaleX = leafScaleX[tile.Level];
        var     childScaleY = leafScaleY[tile.Level];
        var     childScaleZ = leafScaleZ[tile.Level];
        ushort  i           = 0;
        for (var z = 0; z < l.NumCellsZ; ++z)
        for (var x = 0; x < l.NumCellsX; ++x)
        for (var y = 0; y < l.NumCellsY; ++y, ++i)
            tile.Contents[i] = BuildTileContent
            (
                vox,
                tile,
                null,
                tile,
                x,
                y,
                z,
                leafX + x * childScaleX,
                leafY + y * childScaleY,
                leafZ + z * childScaleZ
            );

        return (ushort)(VOXEL_OCCUPIED_BIT | localId);
    }

    private ushort BuildTileContent
    (
        Voxelizer   vox,
        Voxelizer[] mipScratch,
        Tile        parent,
        List<Tile>? rootSubdivision,
        Tile?       tileSubdivision,
        int         rootX,
        int         rootY,
        int         rootZ,
        int         cellX,
        int         cellY,
        int         cellZ
    )
    {
        var level  = parent.Level;
        var source = level == Levels.Length - 1 ? vox : mipScratch[level];
        var (solid, empty) = source.Get(cellX, cellY, cellZ);
        if (!solid) return 0;

        if (!empty) return VOXEL_OCCUPIED_BIT | VOXEL_ID_MASK;

        var index = parent.LevelDesc.VoxelToIndex(rootX, rootY, rootZ);
        var (min, max) = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel(index));
        if (parent.Level + 1 >= Levels.Length)
            throw new InvalidOperationException("体积列构建遇到超出层级的混合体素");
        var tile    = new Tile(this, min, max, parent.Level + 1, false);
        var localId = AppendSubdivision(rootSubdivision, tileSubdivision, tile);

        ref var l = ref Levels[tile.Level];
        ushort  i = 0;
        for (var z = 0; z < l.NumCellsZ; ++z)
        for (var x = 0; x < l.NumCellsX; ++x)
        for (var y = 0; y < l.NumCellsY; ++y, ++i)
            tile.Contents[i] = BuildTileContent
            (
                vox,
                mipScratch,
                tile,
                null,
                tile,
                x,
                y,
                z,
                cellX * l.NumCellsX + x,
                cellY * l.NumCellsY + y,
                cellZ * l.NumCellsZ + z
            );

        return (ushort)(VOXEL_OCCUPIED_BIT | localId);
    }

    public class Level
    {
        public Vector3 CellSize    { get; init; }
        public Vector3 InvCellSize { get; init; }
        public int     NumCellsX   { get; init; }
        public int     NumCellsY   { get; init; }
        public int     NumCellsZ   { get; init; }
        public int     ShiftYX     { get; init; }
        public int     ShiftXZ     { get; init; }

        public int NumCellsTotal => NumCellsX * NumCellsY * NumCellsZ;

        public Level(Vector3 extent, int nx, int ny, int nz)
        {
            if ((nx & nx - 1) != 0 || (ny & ny - 1) != 0 || (nz & nz - 1) != 0)
                throw new ArgumentException($"Non-power-of-2 cell size not supported: got {nx}*{ny}*{nz}");

            CellSize    = extent         / new Vector3(nx, ny, nz);
            InvCellSize = new Vector3(1) / CellSize;
            NumCellsX   = nx;
            NumCellsY   = ny;
            NumCellsZ   = nz;
            ShiftYX     = BitOperations.Log2((uint)ny);
            ShiftXZ     = BitOperations.Log2((uint)nx);
        }

        public Level(Vector3 extent, int ncells) : this(extent, ncells, ncells, ncells) { }

        public bool InBounds(int x, int y, int z) => x >= 0 && x < NumCellsX && y >= 0 && y < NumCellsY && z >= 0 && z < NumCellsZ;

        public bool InBounds((int x, int y, int z) v) => InBounds(v.x, v.y, v.z);

        public (int x, int y, int z) ClampMin(int x, int y, int z) => (Math.Max(x, 0), Math.Max(y, 0), Math.Max(z, 0));

        public (int x, int y, int z) ClampMin((int x, int y, int z) v) => ClampMin(v.x, v.y, v.z);

        public (int x, int y, int z) ClampMax(int x, int y, int z) => (Math.Min(x, NumCellsX - 1), Math.Min(y, NumCellsY - 1), Math.Min(z, NumCellsZ - 1));

        public (int x, int y, int z) ClampMax((int x, int y, int z) v) => ClampMax(v.x, v.y, v.z);

        public (int x, int y, int z) Clamp(int x, int y, int z) => ClampMax(ClampMin(x, y, z));

        public (int x, int y, int z) Clamp((int x, int y, int z) v) => ClampMax(ClampMin(v.x, v.y, v.z));

        public (int x, int y, int z) IndexToVoxel(ushort index)
        {
            var y  = index & NumCellsY - 1;
            var xz = index >> ShiftYX;
            var x  = xz & NumCellsX - 1;
            var z  = xz >> ShiftXZ;
            return (x, y, z);
        }

        public ushort VoxelToIndex(int x, int y, int z) => (ushort)(((z << ShiftXZ) + x << ShiftYX) + y);

        public ushort VoxelToIndex((int x, int y, int z) v) => VoxelToIndex(v.x, v.y, v.z);
    }

    public class Tile
    (
        VoxelMap owner,
        Vector3  boundsMin,
        Vector3  boundsMax,
        int      level,
        bool     clearContents = true
    )
    {
        private Tile[]? subdivision;

        private ushort[]? contentCleared = clearContents
                                               ? new ushort[owner.Levels[level].NumCellsTotal]
                                               : GC.AllocateUninitializedArray<ushort>(owner.Levels[level].NumCellsTotal);

        private byte[]?   packedStates;
        private ushort[]? subtreePrefixCounts;

        public VoxelMap Owner     { get; init; } = owner;
        public Vector3  BoundsMin { get; init; } = boundsMin;
        public Vector3  BoundsMax { get; init; } = boundsMax;
        public int      Level     { get; init; } = level;

        public ushort[] Contents
        {
            get => contentCleared ??= MaterializeContents();
            private set
            {
                contentCleared      = value;
                packedStates        = null;
                subtreePrefixCounts = null;
                StorageKind         = TileStorageKind.Dense;
            }
        } // high bit unset: empty voxel (TODO: region id in low bits?); high bit set: voxel with solid geometry (VoxelIdMask if leaf, subvoxel index otherwise); order is (y,x,z)

        public int SubdivisionCount { get; private set; }

        public int CellCount => LevelDesc.NumCellsTotal;

        public ReadOnlySpan<Tile> Subdivisions => subdivision == null ? [] : subdivision.AsSpan(0, SubdivisionCount);

        public Level LevelDesc => Owner.Levels[Level];

        internal TileStorageKind StorageKind { get; private set; } = TileStorageKind.Dense;

        internal ReadOnlySpan<byte> PackedStates => packedStates == null ? [] : packedStates.AsSpan();

        public (int x, int y, int z) WorldToVoxel(Vector3 v)
        {
            var frac = (v - BoundsMin) * LevelDesc.InvCellSize;
            return ((int)frac.X, (int)frac.Y, (int)frac.Z);
        }

        public Vector3 VoxelToWorld(int x, int y, int z) => BoundsMin + new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * LevelDesc.CellSize;

        public Vector3 VoxelToWorld((int x, int y, int z) v) => VoxelToWorld(v.x, v.y, v.z);

        public (Vector3 min, Vector3 max) CalculateSubdivisionBounds(int x, int y, int z)
        {
            var ld  = LevelDesc;
            var min = BoundsMin + new Vector3(x, y, z) * ld.CellSize;
            return (min, min    + ld.CellSize);
        }

        public (Vector3 min, Vector3 max) CalculateSubdivisionBounds((int x, int y, int z) v) => CalculateSubdivisionBounds(v.x, v.y, v.z);

        public ushort GetCell(int index) => StorageKind switch
        {
            TileStorageKind.Dense       => contentCleared![index],
            TileStorageKind.AllEmpty    => 0,
            TileStorageKind.SolidLeaf   => ushort.MaxValue,
            TileStorageKind.PackedMixed => GetPackedCellValue(index),
            _                           => throw new InvalidOperationException($"未知的体素瓦片存储类型: {StorageKind}")
        };

        public void SetCell(int index, ushort value)
        {
            if (StorageKind != TileStorageKind.Dense)
            {
                _                   = Contents;
                StorageKind         = TileStorageKind.Dense;
                packedStates        = null;
                subtreePrefixCounts = null;
            }

            contentCleared![index] = value;
        }

        public bool IsSubdividedCell(int index)
        {
            if (StorageKind == TileStorageKind.Dense)
            {
                var cell = contentCleared![index];
                return (cell & VOXEL_OCCUPIED_BIT) != 0 && (cell & VOXEL_ID_MASK) != VOXEL_ID_MASK;
            }

            if (StorageKind != TileStorageKind.PackedMixed)
                return false;

            return GetPackedCellState(index) == PackedCellState.Subtree;
        }

        public int GetSubdivisionIndex(int index)
        {
            if (StorageKind == TileStorageKind.Dense)
            {
                var cell = contentCleared![index];
                if ((cell & VOXEL_OCCUPIED_BIT) == 0)
                    throw new InvalidOperationException("空体素不存在子树索引");

                var childIndex = cell & VOXEL_ID_MASK;
                if (childIndex == VOXEL_ID_MASK)
                    throw new InvalidOperationException("叶子体素不存在子树索引");
                return childIndex;
            }

            if (StorageKind != TileStorageKind.PackedMixed)
                throw new InvalidOperationException("当前瓦片未存储子树索引");

            if (GetPackedCellState(index) != PackedCellState.Subtree)
                throw new InvalidOperationException("当前单元不是子树节点");

            var byteIndex = index >> 2;
            return subtreePrefixCounts![byteIndex] + SSubtreePrefixCountByPackedState[packedStates![byteIndex] * 4 + (index & 3)];
        }

        public void SetUniformEmpty()
        {
            StorageKind         = TileStorageKind.AllEmpty;
            contentCleared      = null;
            packedStates        = null;
            subtreePrefixCounts = null;
            ClearSubdivision();
        }

        public void SetUniformSolidLeaf()
        {
            StorageKind         = TileStorageKind.SolidLeaf;
            contentCleared      = null;
            packedStates        = null;
            subtreePrefixCounts = null;
            ClearSubdivision();
        }

        public void SetPackedStates(byte[] states)
        {
            StorageKind         = TileStorageKind.PackedMixed;
            contentCleared      = null;
            packedStates        = states;
            subtreePrefixCounts = BuildSubtreePrefixCounts(states);
        }

        public (ulong index, bool empty) FindLeafVoxel(Vector3 p, bool checkBounds = true)
        {
            Owner.EnsureMaterialized();
            var v = WorldToVoxel(p);
            if (checkBounds && !LevelDesc.InBounds(v))
                return (INVALID_VOXEL, false); // out of bounds; consider everything outside to be occupied

            var idx  = LevelDesc.VoxelToIndex(v);
            var data = GetCell(idx);
            if ((data & VOXEL_OCCUPIED_BIT) == 0)
                return (EncodeIndex(idx), true); // empty at this level

            var childIndex = data & VOXEL_ID_MASK;
            if (childIndex == VOXEL_ID_MASK)
                return (EncodeIndex(idx), false); // occupied leaf

            var sub = GetSubdivision(childIndex).FindLeafVoxel(p, false); // guaranteed to be in bounds
            return (EncodeIndex(idx, sub.index), sub.empty);
        }

        public IEnumerable<(ulong index, bool empty)> EnumerateLeafVoxels(Vector3 bmin, Vector3 bmax)
        {
            Owner.EnsureMaterialized();
            var ld   = LevelDesc;
            var vmin = ld.ClampMin(WorldToVoxel(bmin));
            var vmax = ld.ClampMax(WorldToVoxel(bmax));

            for (var z = vmin.z; z <= vmax.z; z++)
            for (var x = vmin.x; x <= vmax.x; ++x)
            for (var y = vmin.y; y <= vmax.y; ++y)
            {
                var idx  = ld.VoxelToIndex(x, y, z);
                var data = GetCell(idx);

                if ((data & VOXEL_OCCUPIED_BIT) == 0)
                {
                    yield return (EncodeIndex(idx), true); // empty at this level
                    continue;
                }

                var childIndex = data & VOXEL_ID_MASK;

                if (childIndex == VOXEL_ID_MASK)
                {
                    yield return (EncodeIndex(idx), false); // occupied leaf
                    continue;
                }

                foreach (var sub in GetSubdivision(childIndex).EnumerateLeafVoxels(bmin, bmax))
                    yield return (EncodeIndex(idx, sub.index), sub.empty);
            }
        }

        public void ClearSubdivision() => SubdivisionCount = 0;

        public void EnsureSubdivisionCapacity(int capacity)
        {
            if (capacity <= 0 || capacity <= (subdivision?.Length ?? 0))
                return;

            var newCapacity = subdivision == null ? Math.Max(capacity, 4) : Math.Max(capacity, subdivision.Length * 2);
            var resized     = GC.AllocateUninitializedArray<Tile>(newCapacity);
            if (SubdivisionCount > 0)
                Array.Copy(subdivision!, resized, SubdivisionCount);
            subdivision = resized;
        }

        public void AddSubdivision(Tile child)
        {
            EnsureSubdivisionCapacity(SubdivisionCount + 1);
            subdivision![SubdivisionCount++] = child;
        }

        public void AddSubdivisions(IEnumerable<Tile> children)
        {
            if (children is List<Tile> list && list.Count > 0)
            {
                EnsureSubdivisionCapacity(SubdivisionCount + list.Count);
                for (var i = 0; i < list.Count; ++i)
                    subdivision![SubdivisionCount + i] = list[i];
                SubdivisionCount += list.Count;
                return;
            }

            foreach (var child in children)
                AddSubdivision(child);
        }

        public Tile GetSubdivision(int index)
        {
            if ((uint)index >= (uint)SubdivisionCount)
                throw new ArgumentOutOfRangeException(nameof(index), index, $"体积子树索引越界: {index} / {SubdivisionCount}");
            return subdivision![index];
        }

        internal void CompactRetainedState()
        {
            if (SubdivisionCount > 0)
            {
                for (var i = 0; i < SubdivisionCount; ++i)
                    subdivision![i].CompactRetainedState();
            }

            if (StorageKind != TileStorageKind.Dense || contentCleared == null)
            {
                contentCleared = null;
                TrimSubdivisionStorage();
                return;
            }

            var dense        = contentCleared;
            var allEmpty     = true;
            var allSolidLeaf = SubdivisionCount == 0;
            var states       = new byte[PackedStateBytes(dense.Length)];

            for (var i = 0; i < dense.Length; ++i)
            {
                var state = ClassifyPackedCellState(dense[i]);
                states[i >> 2] |= (byte)((byte)state << (i & 3) * 2);
                allEmpty       &= state == PackedCellState.Empty;
                allSolidLeaf   &= state == PackedCellState.SolidLeaf;
            }

            if (allEmpty) SetUniformEmpty();
            else if (allSolidLeaf) SetUniformSolidLeaf();
            else SetPackedStates(states);

            TrimSubdivisionStorage();
        }

        internal void ReleaseRetainedState()
        {
            if (SubdivisionCount > 0)
            {
                for (var i = 0; i < SubdivisionCount; ++i)
                    subdivision![i].ReleaseRetainedState();
            }

            subdivision         = null;
            SubdivisionCount    = 0;
            contentCleared      = null;
            packedStates        = null;
            subtreePrefixCounts = null;
            StorageKind         = TileStorageKind.AllEmpty;
        }

        private ushort[] MaterializeContents()
        {
            var contents = new ushort[CellCount];

            switch (StorageKind)
            {
                case TileStorageKind.AllEmpty:
                    return contents;
                case TileStorageKind.SolidLeaf:
                    Array.Fill(contents, ushort.MaxValue);
                    return contents;
                case TileStorageKind.PackedMixed:
                    for (var i = 0; i < contents.Length; ++i)
                        contents[i] = GetPackedCellValue(i);
                    return contents;
                case TileStorageKind.Dense:
                    return contentCleared ?? [];
                default:
                    throw new InvalidOperationException($"未知的体素瓦片存储类型: {StorageKind}");
            }
        }

        private ushort GetPackedCellValue(int index) => GetPackedCellState(index) switch
        {
            PackedCellState.Empty     => 0,
            PackedCellState.SolidLeaf => ushort.MaxValue,
            PackedCellState.Subtree   => (ushort)(VOXEL_OCCUPIED_BIT | GetSubdivisionIndex(index)),
            _                         => throw new InvalidOperationException("体素状态无效")
        };

        private PackedCellState GetPackedCellState(int index)
            => (PackedCellState)(packedStates![index >> 2] >> (index & 3) * 2 & 0x3);

        private void TrimSubdivisionStorage()
        {
            if (SubdivisionCount == 0)
            {
                subdivision = null;
                return;
            }

            if (subdivision == null || subdivision.Length == SubdivisionCount)
                return;

            var trimmed = GC.AllocateUninitializedArray<Tile>(SubdivisionCount);
            Array.Copy(subdivision, trimmed, SubdivisionCount);
            subdivision = trimmed;
        }
    }

    internal enum TileStorageKind : byte
    {
        Dense,
        AllEmpty,
        SolidLeaf,
        PackedMixed
    }

    private enum PackedCellState : byte
    {
        Empty     = 0,
        SolidLeaf = 1,
        Subtree   = 2
    }

    public sealed class RootColumnBuildResult
    {
        public required ushort[]   Contents    { get; init; }
        public required List<Tile> Subdivision { get; init; }
    }

    #region 常量

    public const ushort VOXEL_OCCUPIED_BIT = 0x8000;
    public const ushort VOXEL_ID_MASK      = 0x7fff;
    public const ushort INDEX_LEVEL_MASK   = 0xffff;
    public const int    INDEX_LEVEL_SHIFT  = 16;
    public const ulong  INVALID_VOXEL      = ulong.MaxValue;

    #endregion
}
