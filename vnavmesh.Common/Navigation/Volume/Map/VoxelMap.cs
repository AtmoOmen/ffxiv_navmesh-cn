using System.Numerics;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Volume.Search;

namespace vnavmesh.Common.Navigation.Volume.Map;

public class VoxelMap
{
    public VolumeLevel[] Levels   { get; init; }
    public VolumeTile    RootTile { get; init; }

    private readonly int[]         leafScaleX;
    private readonly int[]         leafScaleY;
    private readonly int[]         leafScaleZ;
    private readonly int           l2ShiftZ;
    private readonly int           l1ShiftZ;
    private readonly SemaphoreSlim materializationGate = new(1, 1);

    private byte[]?           deferredTreePayload;
    private int               deferredTreeOffset;
    private int               deferredTreeLength;
    private Action<VoxelMap>? deferredTreeMaterializer;

    internal static readonly byte[] SubtreeCountByPackedState       = BuildSubtreeCountByPackedState();
    internal static readonly byte[] SubtreePrefixCountByPackedState = BuildSubtreePrefixCountByPackedState();

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

    internal static int PackedStateBytes(int numCells) => numCells + 3 >> 2;

    internal static VolumePackedCellState ClassifyPackedCellState(ushort value) => value switch
    {
        0               => VolumePackedCellState.Empty,
        ushort.MaxValue => VolumePackedCellState.SolidLeaf,
        _               => VolumePackedCellState.Subtree
    };

    internal static ushort[]? BuildSubtreePrefixCounts(byte[] packedStates)
    {
        if (packedStates.Length == 0)
            return null;

        var    counts  = new ushort[packedStates.Length];
        ushort running = 0;

        for (var i = 0; i < packedStates.Length; ++i)
        {
            counts[i] =  running;
            running   += SubtreeCountByPackedState[packedStates[i]];
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
                if ((VolumePackedCellState)(packedState >> offset * 2 & 0x3) == VolumePackedCellState.Subtree)
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
                if ((VolumePackedCellState)(packedState >> offset * 2 & 0x3) == VolumePackedCellState.Subtree)
                    ++prefix;
            }
        }

        return table;
    }

    public VoxelMap(Vector3 boundsMin, Vector3 boundsMax, int[] tilesPerLevel)
    {
        Levels = new VolumeLevel[tilesPerLevel.Length];
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

        l2ShiftZ = BitOperations.Log2((uint)Levels[2].NumCellsZ);
        l1ShiftZ = BitOperations.Log2((uint)Levels[1].NumCellsZ);
    }

    public bool IsEmpty(ulong voxel)
    {
        EnsureMaterialized();
        var tile = RootTile;

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == INDEX_LEVEL_MASK)
                return false;

            var data = tile.GetCell(tileIndex);
            if ((data & VOXEL_OCCUPIED_BIT) == 0)
                return true;
            data &= VOXEL_ID_MASK;
            if (data == VOXEL_ID_MASK)
                return false;
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

    public bool TryGetLeafVoxelBounds(ulong voxel, out (Vector3 min, Vector3 max) bounds)
    {
        if (Levels.Length != 3)
        {
            bounds = default;
            return false;
        }

        var l0Index = (ushort)(voxel & INDEX_LEVEL_MASK);
        var l1Index = (ushort)((voxel >> INDEX_LEVEL_SHIFT) & INDEX_LEVEL_MASK);
        var l2Index = (ushort)((voxel >> (INDEX_LEVEL_SHIFT * 2)) & INDEX_LEVEL_MASK);

        if (l1Index == INDEX_LEVEL_MASK || l2Index == INDEX_LEVEL_MASK)
        {
            bounds = default;
            return false;
        }

        ref var l0 = ref Levels[0];
        ref var l1 = ref Levels[1];
        ref var l2 = ref Levels[2];

        var l0C = l0.IndexToVoxel(l0Index);
        var l1C = l1.IndexToVoxel(l1Index);
        var l2C = l2.IndexToVoxel(l2Index);

        var min = RootTile.BoundsMin
            + new Vector3
               (
                   l0C.x * l0.CellSize.X + l1C.x * l1.CellSize.X + l2C.x * l2.CellSize.X,
                   l0C.y * l0.CellSize.Y + l1C.y * l1.CellSize.Y + l2C.y * l2.CellSize.Y,
                   l0C.z * l0.CellSize.Z + l1C.z * l1.CellSize.Z + l2C.z * l2.CellSize.Z
               );

        bounds = (min, min + l2.CellSize);
        return true;
    }

    public bool TryFindLeafVoxelFast(Vector3 p, out ulong voxel, out bool empty)
    {
        if (Levels.Length != 3)
        {
            voxel = INVALID_VOXEL;
            empty = false;
            return false;
        }

        ref var l0 = ref Levels[0];
        ref var l1 = ref Levels[1];
        ref var l2 = ref Levels[2];

        var rel = p - RootTile.BoundsMin;
        var gx  = (int)(rel.X * l2.InvCellSize.X);
        var gy  = (int)(rel.Y * l2.InvCellSize.Y);
        var gz  = (int)(rel.Z * l2.InvCellSize.Z);

        if ((uint)gx >= (uint)leafScaleX[0] ||
            (uint)gy >= (uint)leafScaleY[0] ||
            (uint)gz >= (uint)leafScaleZ[0])
        {
            voxel = INVALID_VOXEL;
            empty = false;
            return false;
        }

        var l2x = gx & (l2.NumCellsX - 1);
        var l2y = gy & (l2.NumCellsY - 1);
        var l2z = gz & (l2.NumCellsZ - 1);

        var l1x = (gx >> l2.ShiftXZ) & (l1.NumCellsX - 1);
        var l1y = (gy >> l2.ShiftYX) & (l1.NumCellsY - 1);
        var l1z = (gz >> l2ShiftZ)   & (l1.NumCellsZ - 1);

        var l0x = gx >> (l2.ShiftXZ + l1.ShiftXZ);
        var l0y = gy >> (l2.ShiftYX + l1.ShiftYX);
        var l0z = gz >> (l2ShiftZ   + l1ShiftZ);

        var l0Index = l0.VoxelToIndex(l0x, l0y, l0z);
        var l1Index = l1.VoxelToIndex(l1x, l1y, l1z);
        var l2Index = l2.VoxelToIndex(l2x, l2y, l2z);

        var tile = RootTile;
        var data = tile.GetCell(l0Index);

        if ((data & VOXEL_OCCUPIED_BIT) == 0)
        {
            voxel = EncodeIndex(l0Index);
            empty = true;
            return true;
        }

        var id = data & VOXEL_ID_MASK;
        if (id == VOXEL_ID_MASK)
        {
            voxel = EncodeIndex(l0Index);
            empty = false;
            return true;
        }

        tile = tile.GetSubdivision(id);
        data = tile.GetCell(l1Index);

        if ((data & VOXEL_OCCUPIED_BIT) == 0)
        {
            voxel = EncodeIndex(l1Index);
            voxel = EncodeIndex(l0Index, voxel);
            empty = true;
            return true;
        }

        id = data & VOXEL_ID_MASK;
        if (id == VOXEL_ID_MASK)
        {
            voxel = EncodeIndex(l1Index);
            voxel = EncodeIndex(l0Index, voxel);
            empty = false;
            return true;
        }

        tile = tile.GetSubdivision(id);
        data = tile.GetCell(l2Index);

        voxel = EncodeIndex(l2Index);
        voxel = EncodeIndex(l1Index, voxel);
        voxel = EncodeIndex(l0Index, voxel);
        empty = (data & VOXEL_OCCUPIED_BIT) == 0;
        return true;
    }

    public Vector3 ClampPointToVoxel(ulong voxel, Vector3 p, float eps = 0.1f)
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

    public void ReleaseRetainedState()
    {
        deferredTreePayload      = null;
        deferredTreeOffset       = 0;
        deferredTreeLength       = 0;
        deferredTreeMaterializer = null;
        RootTile.ReleaseRetainedState();
    }

    public void CompactRetainedState()
    {
        if (HasDeferredTree)
            return;

        RootTile.CompactRetainedState();
    }

    public VolumeRootColumnBuildResult BuildRootColumn(Voxelizer vox, int tx, int tz)
    {
        var ny          = Levels[0].NumCellsY;
        var contents    = GC.AllocateUninitializedArray<ushort>(ny);
        var subdivision = new List<VolumeTile>(64);
        for (var ty = 0; ty < ny; ++ty)
            contents[ty] = BuildTileContent(vox, RootTile, subdivision, null, tx, ty, tz, 0, ty * leafScaleY[0], 0);
        return new() { Contents = contents, Subdivision = subdivision };
    }

    private static int AppendSubdivision(List<VolumeTile>? list, VolumeTile? tile, VolumeTile child)
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
        (Voxelizer vox, VolumeTile parent, List<VolumeTile>? rootSubdivision, VolumeTile? tileSubdivision, int rootX, int rootY, int rootZ, int leafX, int leafY, int leafZ)
    {
        var level = parent.Level;
        var (solid, empty) = vox.ClassifyBox(leafX, leafY, leafZ, leafScaleX[level], leafScaleY[level], leafScaleZ[level]);
        if (!solid) return 0;

        if (!empty) return VOXEL_OCCUPIED_BIT | VOXEL_ID_MASK;

        var index = parent.LevelDesc.VoxelToIndex(rootX, rootY, rootZ);
        var (min, max) = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel(index));
        if (parent.Level + 1 >= Levels.Length)
            throw new InvalidOperationException("体积列构建遇到超出层级的混合体素");
        var tile    = new VolumeTile(this, min, max, parent.Level + 1, false);
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
        Voxelizer           vox,
        Voxelizer[]         mipScratch,
        VolumeTile          parent,
        List<VolumeTile>? rootSubdivision,
        VolumeTile?       tileSubdivision,
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
        var (solid, empty) = source.GetCellState(cellX, cellY, cellZ);
        if (!solid) return 0;

        if (!empty) return VOXEL_OCCUPIED_BIT | VOXEL_ID_MASK;

        var index = parent.LevelDesc.VoxelToIndex(rootX, rootY, rootZ);
        var (min, max) = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel(index));
        if (parent.Level + 1 >= Levels.Length)
            throw new InvalidOperationException("体积列构建遇到超出层级的混合体素");
        var tile    = new VolumeTile(this, min, max, parent.Level + 1, false);
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

    #region 常量

    public const ushort VOXEL_OCCUPIED_BIT = 0x8000;
    public const ushort VOXEL_ID_MASK      = 0x7fff;
    public const ushort INDEX_LEVEL_MASK   = 0xffff;
    public const int    INDEX_LEVEL_SHIFT  = 16;
    public const ulong  INVALID_VOXEL      = ulong.MaxValue;

    #endregion
}
