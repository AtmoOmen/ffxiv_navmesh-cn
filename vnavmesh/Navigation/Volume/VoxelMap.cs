using System.Numerics;
using vnavmesh.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Volume;

public class VoxelMap
{
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
    {
        private Tile[]?   _subdivision;
        private ushort[]? _contents;
        private byte[]?   _packedStates;
        private ushort[]? _subtreePrefixCounts;

        public VoxelMap Owner     { get; init; }
        public Vector3  BoundsMin { get; init; }
        public Vector3  BoundsMax { get; init; }
        public int      Level     { get; init; }

        public ushort[] Contents
        {
            get => _contents ??= MaterializeContents();
            private set
            {
                _contents            = value;
                _packedStates        = null;
                _subtreePrefixCounts = null;
                StorageKind          = TileStorageKind.Dense;
            }
        } // high bit unset: empty voxel (TODO: region id in low bits?); high bit set: voxel with solid geometry (VoxelIdMask if leaf, subvoxel index otherwise); order is (y,x,z)

        public int SubdivisionCount { get; private set; }

        public int CellCount => LevelDesc.NumCellsTotal;

        public ReadOnlySpan<Tile> Subdivisions => _subdivision == null ? [] : _subdivision.AsSpan(0, SubdivisionCount);

        public Level LevelDesc => Owner.Levels[Level];

        internal TileStorageKind StorageKind { get; private set; }

        internal ReadOnlySpan<byte> PackedStates => _packedStates == null ? [] : _packedStates.AsSpan();

        public Tile(VoxelMap owner, Vector3 boundsMin, Vector3 boundsMax, int level, bool clearContents = true)
        {
            Owner       = owner;
            BoundsMin   = boundsMin;
            BoundsMax   = boundsMax;
            Level       = level;
            StorageKind = TileStorageKind.Dense;
            _contents = clearContents
                            ? new ushort[owner.Levels[level].NumCellsTotal]
                            : GC.AllocateUninitializedArray<ushort>(owner.Levels[level].NumCellsTotal);
        }

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
            TileStorageKind.Dense       => _contents![index],
            TileStorageKind.AllEmpty    => 0,
            TileStorageKind.SolidLeaf   => ushort.MaxValue,
            TileStorageKind.PackedMixed => GetPackedCellValue(index),
            _                           => throw new InvalidOperationException($"未知的体素瓦片存储类型: {StorageKind}")
        };

        public void SetCell(int index, ushort value)
        {
            if (StorageKind != TileStorageKind.Dense)
            {
                _                    = Contents;
                StorageKind          = TileStorageKind.Dense;
                _packedStates        = null;
                _subtreePrefixCounts = null;
            }

            _contents![index] = value;
        }

        public bool IsSubdividedCell(int index)
        {
            if (StorageKind == TileStorageKind.Dense)
            {
                var cell = _contents![index];
                return (cell & VoxelOccupiedBit) != 0 && (cell & VoxelIdMask) != VoxelIdMask;
            }

            if (StorageKind != TileStorageKind.PackedMixed)
                return false;

            return GetPackedCellState(index) == PackedCellState.Subtree;
        }

        public int GetSubdivisionIndex(int index)
        {
            if (StorageKind == TileStorageKind.Dense)
            {
                var cell = _contents![index];
                if ((cell & VoxelOccupiedBit) == 0)
                    throw new InvalidOperationException("空体素不存在子树索引");

                var childIndex = cell & VoxelIdMask;
                if (childIndex == VoxelIdMask)
                    throw new InvalidOperationException("叶子体素不存在子树索引");
                return childIndex;
            }

            if (StorageKind != TileStorageKind.PackedMixed)
                throw new InvalidOperationException("当前瓦片未存储子树索引");

            if (GetPackedCellState(index) != PackedCellState.Subtree)
                throw new InvalidOperationException("当前单元不是子树节点");

            var byteIndex = index >> 2;
            return _subtreePrefixCounts![byteIndex] + s_subtreePrefixCountByPackedState[_packedStates![byteIndex] * 4 + (index & 3)];
        }

        public void SetUniformEmpty()
        {
            StorageKind          = TileStorageKind.AllEmpty;
            _contents            = null;
            _packedStates        = null;
            _subtreePrefixCounts = null;
            ClearSubdivision();
        }

        public void SetUniformSolidLeaf()
        {
            StorageKind          = TileStorageKind.SolidLeaf;
            _contents            = null;
            _packedStates        = null;
            _subtreePrefixCounts = null;
            ClearSubdivision();
        }

        public void SetPackedStates(byte[] packedStates)
        {
            StorageKind          = TileStorageKind.PackedMixed;
            _contents            = null;
            _packedStates        = packedStates;
            _subtreePrefixCounts = BuildSubtreePrefixCounts(packedStates);
        }

        public (ulong index, bool empty) FindLeafVoxel(Vector3 p, bool checkBounds = true)
        {
            Owner.EnsureMaterialized();
            var v = WorldToVoxel(p);
            if (checkBounds && !LevelDesc.InBounds(v))
                return (InvalidVoxel, false); // out of bounds; consider everything outside to be occupied

            var idx  = LevelDesc.VoxelToIndex(v);
            var data = GetCell(idx);
            if ((data & VoxelOccupiedBit) == 0)
                return (EncodeIndex(idx), true); // empty at this level

            var childIndex = data & VoxelIdMask;
            if (childIndex == VoxelIdMask)
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

                if ((data & VoxelOccupiedBit) == 0)
                {
                    yield return (EncodeIndex(idx), true); // empty at this level
                    continue;
                }

                var childIndex = data & VoxelIdMask;

                if (childIndex == VoxelIdMask)
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
            if (capacity <= 0 || capacity <= (_subdivision?.Length ?? 0))
                return;

            var newCapacity = _subdivision == null ? Math.Max(capacity, 4) : Math.Max(capacity, _subdivision.Length * 2);
            var resized     = GC.AllocateUninitializedArray<Tile>(newCapacity);
            if (SubdivisionCount > 0)
                Array.Copy(_subdivision!, resized, SubdivisionCount);
            _subdivision = resized;
        }

        public void AddSubdivision(Tile child)
        {
            EnsureSubdivisionCapacity(SubdivisionCount + 1);
            _subdivision![SubdivisionCount++] = child;
        }

        public void AddSubdivisions(IEnumerable<Tile> children)
        {
            if (children is List<Tile> list && list.Count > 0)
            {
                EnsureSubdivisionCapacity(SubdivisionCount + list.Count);
                for (var i = 0; i < list.Count; ++i)
                    _subdivision![SubdivisionCount + i] = list[i];
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
            return _subdivision![index];
        }

        internal void CompactRetainedState()
        {
            if (SubdivisionCount > 0)
            {
                for (var i = 0; i < SubdivisionCount; ++i)
                    _subdivision![i].CompactRetainedState();
            }

            if (StorageKind != TileStorageKind.Dense || _contents == null)
            {
                _contents = null;
                TrimSubdivisionStorage();
                return;
            }

            var dense        = _contents;
            var allEmpty     = true;
            var allSolidLeaf = SubdivisionCount == 0;
            var packedStates = new byte[PackedStateBytes(dense.Length)];

            for (var i = 0; i < dense.Length; ++i)
            {
                var state = ClassifyPackedCellState(dense[i]);
                packedStates[i >> 2] |= (byte)((byte)state << (i & 3) * 2);
                allEmpty             &= state == PackedCellState.Empty;
                allSolidLeaf         &= state == PackedCellState.SolidLeaf;
            }

            if (allEmpty) SetUniformEmpty();
            else if (allSolidLeaf) SetUniformSolidLeaf();
            else SetPackedStates(packedStates);

            TrimSubdivisionStorage();
        }

        internal void ReleaseRetainedState()
        {
            if (SubdivisionCount > 0)
            {
                for (var i = 0; i < SubdivisionCount; ++i)
                    _subdivision![i].ReleaseRetainedState();
            }

            _subdivision         = null;
            SubdivisionCount     = 0;
            _contents            = null;
            _packedStates        = null;
            _subtreePrefixCounts = null;
            StorageKind          = TileStorageKind.AllEmpty;
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
                    return _contents ?? [];
                default:
                    throw new InvalidOperationException($"未知的体素瓦片存储类型: {StorageKind}");
            }
        }

        private ushort GetPackedCellValue(int index) => GetPackedCellState(index) switch
        {
            PackedCellState.Empty     => 0,
            PackedCellState.SolidLeaf => ushort.MaxValue,
            PackedCellState.Subtree   => (ushort)(VoxelOccupiedBit | GetSubdivisionIndex(index)),
            _                         => throw new InvalidOperationException("体素状态无效")
        };

        private PackedCellState GetPackedCellState(int index)
            => (PackedCellState)(_packedStates![index >> 2] >> (index & 3) * 2 & 0x3);

        private void TrimSubdivisionStorage()
        {
            if (SubdivisionCount == 0)
            {
                _subdivision = null;
                return;
            }

            if (_subdivision == null || _subdivision.Length == SubdivisionCount)
                return;

            var trimmed = GC.AllocateUninitializedArray<Tile>(SubdivisionCount);
            Array.Copy(_subdivision, trimmed, SubdivisionCount);
            _subdivision = trimmed;
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

    public Level[] Levels   { get; init; }
    public Tile    RootTile { get; init; }

    private readonly int[]         _leafScaleX;
    private readonly int[]         _leafScaleY;
    private readonly int[]         _leafScaleZ;
    private readonly SemaphoreSlim _materializationGate = new(1, 1);

    private byte[]?           _deferredTreePayload;
    private int               _deferredTreeOffset;
    private int               _deferredTreeLength;
    private Action<VoxelMap>? _deferredTreeMaterializer;

    public const ushort VoxelOccupiedBit = 0x8000;
    public const ushort VoxelIdMask      = 0x7fff;

    // voxel index is stored as (... L2 L1 L0)
    public const ushort IndexLevelMask  = 0xffff;
    public const int    IndexLevelShift = 16;

    public const ulong InvalidVoxel = ulong.MaxValue;

    private static readonly byte[] s_subtreeCountByPackedState       = BuildSubtreeCountByPackedState();
    private static readonly byte[] s_subtreePrefixCountByPackedState = BuildSubtreePrefixCountByPackedState();

    public static ulong EncodeIndex(ushort tileIndex, ulong subIndex = InvalidVoxel) => (subIndex << IndexLevelShift) + tileIndex;

    public static ulong EncodeSubIndex(ulong voxel, ushort tileIndex, int level)
    {
        var shift = IndexLevelShift * level;
        voxel &= ~((ulong)IndexLevelMask << shift);
        voxel |= (ulong)tileIndex << shift;
        return voxel;
    }

    public static ushort DecodeIndex(ref ulong index)
    {
        var tileIndex = (ushort)(index & IndexLevelMask);
        index >>= IndexLevelShift;
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
            running   += s_subtreeCountByPackedState[packedStates[i]];
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

        RootTile        = new(this, boundsMin, boundsMax, 0);
        _leafScaleX     = GC.AllocateUninitializedArray<int>(Levels.Length);
        _leafScaleY     = GC.AllocateUninitializedArray<int>(Levels.Length);
        _leafScaleZ     = GC.AllocateUninitializedArray<int>(Levels.Length);
        _leafScaleX[^1] = 1;
        _leafScaleY[^1] = 1;
        _leafScaleZ[^1] = 1;

        for (var i = Levels.Length - 2; i >= 0; --i)
        {
            _leafScaleX[i] = _leafScaleX[i + 1] * Levels[i + 1].NumCellsX;
            _leafScaleY[i] = _leafScaleY[i + 1] * Levels[i + 1].NumCellsY;
            _leafScaleZ[i] = _leafScaleZ[i + 1] * Levels[i + 1].NumCellsZ;
        }
    }

    public bool IsEmpty(ulong voxel)
    {
        EnsureMaterialized();
        var tile = RootTile;

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == IndexLevelMask)
                return false; // asking for non-leaf => consider non-empty

            var data = tile.GetCell(tileIndex);
            if ((data & VoxelOccupiedBit) == 0)
                return true; // found empty voxel
            data &= VoxelIdMask;
            if (data == VoxelIdMask)
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
            if (tileIndex == IndexLevelMask) return (tile.BoundsMin + eps3, tile.BoundsMax - eps3);

            var data = tile.GetCell(tileIndex);
            var id   = data & VoxelIdMask;

            if ((data & VoxelOccupiedBit) == 0 || id == VoxelIdMask)
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
            if (tileIndex == IndexLevelMask)
                return Vector3.Clamp(p, tile.BoundsMin + new Vector3(eps), tile.BoundsMax - new Vector3(eps));

            var data = tile.GetCell(tileIndex);
            var id   = data & VoxelIdMask;

            if ((data & VoxelOccupiedBit) == 0 || id == VoxelIdMask)
            {
                var (min, max) = tile.CalculateSubdivisionBounds(Levels[tile.Level].IndexToVoxel(tileIndex));
                var eps3 = new Vector3(eps);
                return Vector3.Clamp(p, min + eps3, max - eps3);
            }

            tile = tile.GetSubdivision(id);
        }
    }

    internal bool HasDeferredTree => _deferredTreePayload != null || _deferredTreeMaterializer != null;

    internal void SetDeferredTreePayload(byte[] payload, int offset, int length)
    {
        _deferredTreePayload      = payload;
        _deferredTreeOffset       = offset;
        _deferredTreeLength       = length;
        _deferredTreeMaterializer = null;
    }

    internal void SetDeferredTreeMaterializer(Action<VoxelMap> materializer)
    {
        _deferredTreePayload      = null;
        _deferredTreeOffset       = 0;
        _deferredTreeLength       = 0;
        _deferredTreeMaterializer = materializer;
    }

    internal void EnsureMaterialized()
    {
        if (!HasDeferredTree)
            return;

        _materializationGate.Wait();

        try
        {
            if (!HasDeferredTree)
                return;

            if (_deferredTreePayload is { } payload)
            {
                Navmesh.MaterializeDeferredVolumeTree(this, payload, _deferredTreeOffset, _deferredTreeLength);
                _deferredTreePayload = null;
                _deferredTreeOffset  = 0;
                _deferredTreeLength  = 0;
                return;
            }

            if (_deferredTreeMaterializer is { } materializer)
            {
                materializer(this);
                _deferredTreeMaterializer = null;
            }
        }
        finally
        {
            _materializationGate.Release();
        }
    }

    internal void ReleaseRetainedState()
    {
        _deferredTreePayload      = null;
        _deferredTreeOffset       = 0;
        _deferredTreeLength       = 0;
        _deferredTreeMaterializer = null;
        RootTile.ReleaseRetainedState();
    }

    internal void CompactRetainedState()
    {
        if (HasDeferredTree)
            return;

        RootTile.CompactRetainedState();
    }

    public void Build(Voxelizer vox, int tx, int tz)
    {
        var column = BuildRootColumn(vox, tx, tz);
        var ny     = Levels[0].NumCellsY;
        var idx    = Levels[0].VoxelToIndex(tx, 0, tz);
        Array.Copy(column.Contents, 0, RootTile.Contents, idx, ny);
        RootTile.AddSubdivisions(column.Subdivision);
    }

    public RootColumnBuildResult BuildRootColumn(Voxelizer vox, int tx, int tz)
    {
        var ny          = Levels[0].NumCellsY;
        var contents    = GC.AllocateUninitializedArray<ushort>(ny);
        var subdivision = new List<Tile>(64);
        for (var ty = 0; ty < ny; ++ty)
            contents[ty] = BuildTileContent(vox, RootTile, subdivision, null, tx, ty, tz, 0, ty * _leafScaleY[0], 0);
        return new() { Contents = contents, Subdivision = subdivision };
    }

    public RootColumnBuildResult BuildRootColumn(Voxelizer vox, Voxelizer[] mipScratch, int tx, int tz)
    {
        BuildMipChain(vox, mipScratch);
        var ny          = Levels[0].NumCellsY;
        var contents    = GC.AllocateUninitializedArray<ushort>(ny);
        var subdivision = new List<Tile>(64);
        for (var ty = 0; ty < ny; ++ty)
            contents[ty] = BuildTileContent(vox, mipScratch, RootTile, subdivision, null, tx, ty, tz, 0, ty, 0);
        return new() { Contents = contents, Subdivision = subdivision };
    }

    private void BuildMipChain(Voxelizer leaf, Voxelizer[] mipScratch)
    {
        if (mipScratch.Length != Levels.Length - 1)
            throw new ArgumentException("体积分层缓存长度与体积层级不匹配", nameof(mipScratch));

        var current = leaf;

        for (var level = Levels.Length - 2; level >= 0; --level)
        {
            ref readonly var nextLevel = ref Levels[level + 1];
            current.DownsampleInto(mipScratch[level], nextLevel.NumCellsX, nextLevel.NumCellsY, nextLevel.NumCellsZ);
            current = mipScratch[level];
        }
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
        var (solid, empty) = vox.ClassifyRegion(leafX, leafY, leafZ, _leafScaleX[level], _leafScaleY[level], _leafScaleZ[level]);
        if (!solid) return 0;

        if (!empty) return VoxelOccupiedBit | VoxelIdMask;

        var index = parent.LevelDesc.VoxelToIndex(rootX, rootY, rootZ);
        var (min, max) = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel(index));
        if (parent.Level + 1 >= Levels.Length)
            throw new InvalidOperationException("体积列构建遇到超出层级的混合体素");
        var tile    = new Tile(this, min, max, parent.Level + 1, false);
        var localId = AppendSubdivision(rootSubdivision, tileSubdivision, tile);

        ref var l           = ref Levels[tile.Level];
        var     childScaleX = _leafScaleX[tile.Level];
        var     childScaleY = _leafScaleY[tile.Level];
        var     childScaleZ = _leafScaleZ[tile.Level];
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

        return (ushort)(VoxelOccupiedBit | localId);
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

        if (!empty) return VoxelOccupiedBit | VoxelIdMask;

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

        return (ushort)(VoxelOccupiedBit | localId);
    }
}
