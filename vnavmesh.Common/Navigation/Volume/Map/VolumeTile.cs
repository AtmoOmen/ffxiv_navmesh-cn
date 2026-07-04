using System.Numerics;

namespace vnavmesh.Common.Navigation.Volume.Map;

public sealed class VolumeTile
(
    VoxelMap owner,
    Vector3  boundsMin,
    Vector3  boundsMax,
    int      level,
    bool     clearContents = true
)
{
    private VolumeTile[]? subdivision;

    private ushort[]? contentCleared = clearContents ?
                                           new ushort[owner.Levels[level].NumCellsTotal] :
                                           GC.AllocateUninitializedArray<ushort>(owner.Levels[level].NumCellsTotal);

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
            StorageKind         = VolumeTileStorageKind.Dense;
        }
    }

    public int SubdivisionCount { get; private set; }

    public int CellCount => LevelDesc.NumCellsTotal;

    public ReadOnlySpan<VolumeTile> Subdivisions => subdivision == null ?
                                                        [] :
                                                        subdivision.AsSpan(0, SubdivisionCount);

    public VolumeLevel LevelDesc => Owner.Levels[Level];

    internal VolumeTileStorageKind StorageKind { get; private set; } = VolumeTileStorageKind.Dense;

    internal ReadOnlySpan<byte> PackedStates => packedStates == null ?
                                                    [] :
                                                    packedStates.AsSpan();

    public (int x, int y, int z) WorldToVoxel(Vector3 v)
    {
        var frac = (v - BoundsMin) * LevelDesc.InvCellSize;
        return ((int)frac.X, (int)frac.Y, (int)frac.Z);
    }

    public Vector3 VoxelToWorld(int x, int y, int z) => BoundsMin + new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * LevelDesc.CellSize;

    public Vector3 VoxelToWorld((int x, int y, int z) v) => VoxelToWorld(v.x, v.y, v.z);

    public (Vector3 min, Vector3 max) CalculateSubdivisionBounds(int x, int y, int z)
    {
        var min = BoundsMin + new Vector3(x, y, z) * LevelDesc.CellSize;
        return (min, min    + LevelDesc.CellSize);
    }

    public (Vector3 min, Vector3 max) CalculateSubdivisionBounds((int x, int y, int z) v) => CalculateSubdivisionBounds(v.x, v.y, v.z);

    public ushort GetCell(int index) => StorageKind switch
    {
        VolumeTileStorageKind.Dense       => contentCleared![index],
        VolumeTileStorageKind.AllEmpty    => 0,
        VolumeTileStorageKind.SolidLeaf   => ushort.MaxValue,
        VolumeTileStorageKind.PackedMixed => GetPackedCellValue(index),
        _                                 => throw new InvalidOperationException($"未知的体素瓦片存储类型: {StorageKind}")
    };

    public void SetCell(int index, ushort value)
    {
        if (StorageKind != VolumeTileStorageKind.Dense)
            _ = Contents;

        contentCleared![index] = value;
    }

    public bool IsSubdividedCell(int index)
    {
        if (StorageKind == VolumeTileStorageKind.Dense)
        {
            var cell = contentCleared![index];
            return (cell & VoxelMap.VOXEL_OCCUPIED_BIT) != 0 && (cell & VoxelMap.VOXEL_ID_MASK) != VoxelMap.VOXEL_ID_MASK;
        }

        return StorageKind == VolumeTileStorageKind.PackedMixed && GetPackedCellState(index) == VolumePackedCellState.Subtree;
    }

    public int GetSubdivisionIndex(int index)
    {
        if (StorageKind == VolumeTileStorageKind.Dense)
        {
            var cell = contentCleared![index];
            if ((cell & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                throw new InvalidOperationException("空体素不存在子树索引");

            var childIndex = cell & VoxelMap.VOXEL_ID_MASK;
            if (childIndex == VoxelMap.VOXEL_ID_MASK)
                throw new InvalidOperationException("叶子体素不存在子树索引");
            return childIndex;
        }

        if (StorageKind != VolumeTileStorageKind.PackedMixed)
            throw new InvalidOperationException("当前瓦片未存储子树索引");

        if (GetPackedCellState(index) != VolumePackedCellState.Subtree)
            throw new InvalidOperationException("当前单元不是子树节点");

        var byteIndex = index >> 2;
        return subtreePrefixCounts![byteIndex] + VoxelMap.SubtreePrefixCountByPackedState[packedStates![byteIndex] * 4 + (index & 3)];
    }

    public void SetUniformEmpty()
    {
        StorageKind         = VolumeTileStorageKind.AllEmpty;
        contentCleared      = null;
        packedStates        = null;
        subtreePrefixCounts = null;
        ClearSubdivision();
    }

    public void SetUniformSolidLeaf()
    {
        StorageKind         = VolumeTileStorageKind.SolidLeaf;
        contentCleared      = null;
        packedStates        = null;
        subtreePrefixCounts = null;
        ClearSubdivision();
    }

    public void SetPackedStates(byte[] states)
    {
        StorageKind         = VolumeTileStorageKind.PackedMixed;
        contentCleared      = null;
        packedStates        = states;
        subtreePrefixCounts = VoxelMap.BuildSubtreePrefixCounts(states);
    }

    public (ulong index, bool empty) FindLeafVoxel(Vector3 p, bool checkBounds = true)
    {
        Owner.EnsureMaterialized();
        var v = WorldToVoxel(p);
        if (checkBounds && !LevelDesc.InBounds(v))
            return (VoxelMap.INVALID_VOXEL, false);

        var idx  = LevelDesc.VoxelToIndex(v);
        var data = GetCell(idx);
        if ((data & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
            return (VoxelMap.EncodeIndex(idx), true);

        var childIndex = data & VoxelMap.VOXEL_ID_MASK;
        if (childIndex == VoxelMap.VOXEL_ID_MASK)
            return (VoxelMap.EncodeIndex(idx), false);

        var sub = GetSubdivision(childIndex).FindLeafVoxel(p, false);
        return (VoxelMap.EncodeIndex(idx, sub.index), sub.empty);
    }

    public IEnumerable<(ulong index, bool empty)> EnumerateLeafVoxels(Vector3 bmin, Vector3 bmax)
    {
        Owner.EnsureMaterialized();
        var vmin = LevelDesc.ClampMin(WorldToVoxel(bmin));
        var vmax = LevelDesc.ClampMax(WorldToVoxel(bmax));

        for (var z = vmin.z; z <= vmax.z; ++z)
        for (var x = vmin.x; x <= vmax.x; ++x)
        for (var y = vmin.y; y <= vmax.y; ++y)
        {
            var idx  = LevelDesc.VoxelToIndex(x, y, z);
            var data = GetCell(idx);

            if ((data & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
            {
                yield return (VoxelMap.EncodeIndex(idx), true);
                continue;
            }

            var childIndex = data & VoxelMap.VOXEL_ID_MASK;

            if (childIndex == VoxelMap.VOXEL_ID_MASK)
            {
                yield return (VoxelMap.EncodeIndex(idx), false);
                continue;
            }

            foreach (var sub in GetSubdivision(childIndex).EnumerateLeafVoxels(bmin, bmax))
                yield return (VoxelMap.EncodeIndex(idx, sub.index), sub.empty);
        }
    }

    public void ClearSubdivision() => SubdivisionCount = 0;

    public void EnsureSubdivisionCapacity(int capacity)
    {
        if (capacity <= 0 || capacity <= (subdivision?.Length ?? 0))
            return;

        var newCapacity = subdivision == null ?
                              Math.Max(capacity, 4) :
                              Math.Max(capacity, subdivision.Length * 2);
        var resized = GC.AllocateUninitializedArray<VolumeTile>(newCapacity);
        if (SubdivisionCount > 0)
            Array.Copy(subdivision!, resized, SubdivisionCount);
        subdivision = resized;
    }

    public void AddSubdivision(VolumeTile child)
    {
        EnsureSubdivisionCapacity(SubdivisionCount + 1);
        subdivision![SubdivisionCount++] = child;
    }

    public void AddSubdivisions(IEnumerable<VolumeTile> children)
    {
        if (children is List<VolumeTile> list && list.Count > 0)
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

    public VolumeTile GetSubdivision(int index)
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

        if (StorageKind != VolumeTileStorageKind.Dense || contentCleared == null)
        {
            contentCleared = null;
            TrimSubdivisionStorage();
            return;
        }

        var dense        = contentCleared;
        var allEmpty     = true;
        var allSolidLeaf = SubdivisionCount == 0;
        var states       = new byte[VoxelMap.PackedStateBytes(dense.Length)];

        for (var i = 0; i < dense.Length; ++i)
        {
            var state = VoxelMap.ClassifyPackedCellState(dense[i]);
            states[i >> 2] |= (byte)((byte)state << (i & 3) * 2);
            allEmpty       &= state == VolumePackedCellState.Empty;
            allSolidLeaf   &= state == VolumePackedCellState.SolidLeaf;
        }

        if (allEmpty)
            SetUniformEmpty();
        else if (allSolidLeaf)
            SetUniformSolidLeaf();
        else
            SetPackedStates(states);

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
        StorageKind         = VolumeTileStorageKind.AllEmpty;
    }

    private ushort[] MaterializeContents()
    {
        var contents = new ushort[CellCount];

        switch (StorageKind)
        {
            case VolumeTileStorageKind.AllEmpty:
                return contents;
            case VolumeTileStorageKind.SolidLeaf:
                Array.Fill(contents, ushort.MaxValue);
                return contents;
            case VolumeTileStorageKind.PackedMixed:
                for (var i = 0; i < contents.Length; ++i)
                    contents[i] = GetPackedCellValue(i);
                return contents;
            case VolumeTileStorageKind.Dense:
                return contentCleared ?? [];
            default:
                throw new InvalidOperationException($"未知的体素瓦片存储类型: {StorageKind}");
        }
    }

    private ushort GetPackedCellValue(int index) => GetPackedCellState(index) switch
    {
        VolumePackedCellState.Empty     => 0,
        VolumePackedCellState.SolidLeaf => ushort.MaxValue,
        VolumePackedCellState.Subtree   => (ushort)(VoxelMap.VOXEL_OCCUPIED_BIT | GetSubdivisionIndex(index)),
        _                               => throw new InvalidOperationException("体素状态无效")
    };

    private VolumePackedCellState GetPackedCellState(int index)
        => (VolumePackedCellState)(packedStates![index >> 2] >> (index & 3) * 2 & 0x3);

    private void TrimSubdivisionStorage()
    {
        if (SubdivisionCount == 0)
        {
            subdivision = null;
            return;
        }

        if (subdivision == null || subdivision.Length == SubdivisionCount)
            return;

        var trimmed = GC.AllocateUninitializedArray<VolumeTile>(SubdivisionCount);
        Array.Copy(subdivision, trimmed, SubdivisionCount);
        subdivision = trimmed;
    }
}
