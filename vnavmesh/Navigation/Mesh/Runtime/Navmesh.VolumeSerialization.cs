using System.Buffers;
using System.Buffers.Binary;
using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Runtime;

public partial record class Navmesh
{
    private static VoxelMap? DeserializeVolume(BinaryReader reader)
    {
        if (!reader.ReadBoolean())
            return null;

        var numLevels = reader.ReadInt32();
        if (numLevels <= 0)
            throw new Exception("体积缓存层级无效");

        var tilesPerLevel = new int[numLevels];
        foreach (ref var l in tilesPerLevel.AsSpan())
            l = reader.ReadInt32();

        var (min, max) = DeserializeBounds(reader);
        var volume = new VoxelMap(min, max, tilesPerLevel);
        if (TryCaptureDeferredVolumeTree(reader.BaseStream, out var payload, out var offset, out var length))
        {
            volume.SetDeferredTreePayload(payload, offset, length);
            reader.BaseStream.Position = reader.BaseStream.Length;
        }
        else
        {
            DeserializeVolumeTile(reader, volume.RootTile);
        }
        return volume;
    }

    private static void SerializeVolume(BinaryWriter writer, VoxelMap? volume)
    {
        writer.Write(volume != null);
        if (volume == null)
            return;

        writer.Write(volume.Levels.Length);
        foreach (ref var l in volume.Levels.AsSpan())
            writer.Write(l.NumCellsX);

        volume.EnsureMaterialized();
        SerializeBounds(writer, volume.RootTile.BoundsMin, volume.RootTile.BoundsMax);
        SerializeVolumeTile(writer, volume.RootTile);
    }

    internal static void MaterializeDeferredVolumeTree(VoxelMap volume, byte[] payload, int offset, int length)
    {
        using var stream = new MemoryStream(payload, offset, length, false);
        using var reader = new BinaryReader(stream);
        DeserializeVolumeTile(reader, volume.RootTile);
    }

    private static void DeserializeVolumeTile(BinaryReader reader, VoxelMap.Tile tile)
    {
        var encoding = (VolumeTileEncoding)reader.ReadByte();

        switch (encoding)
        {
            case VolumeTileEncoding.Empty:
                Array.Clear(tile.Contents);
                tile.ClearSubdivision();
                return;
            case VolumeTileEncoding.SolidLeaf:
                Array.Fill(tile.Contents, ushort.MaxValue);
                tile.ClearSubdivision();
                return;
            case VolumeTileEncoding.Mixed:
                break;
            default:
                throw new Exception($"未知的体积编码类型: {encoding}");
        }

        tile.ClearSubdivision();
        var packedBytes  = PackedStateBytes(tile.Contents.Length);
        var rentedStates = ArrayPool<byte>.Shared.Rent(packedBytes);
        var packedStates = rentedStates.AsSpan(0, packedBytes);

        try
        {
            reader.BaseStream.ReadExactly(packedStates);

            var subtreeCount = 0;
            var contentsBytes = MemoryMarshal.AsBytes(tile.Contents.AsSpan());
            for (var i = 0; i < packedStates.Length; ++i)
            {
                var packedState = packedStates[i];
                if (s_invalidPackedState[packedState])
                    throw new Exception($"未知的体积单元状态字节: 0x{packedState:X2}");

                subtreeCount += s_subtreeCountByPackedState[packedState];
                BinaryPrimitives.WriteUInt64LittleEndian(contentsBytes.Slice(i * sizeof(ulong), sizeof(ulong)), s_expandedContentsByPackedState[packedState]);
            }

            tile.EnsureSubdivisionCapacity(subtreeCount);

            if (subtreeCount == 0)
                return;

            var baseIndex = 0;
            for (var i = 0; i < packedStates.Length; ++i, baseIndex += 4)
            {
                var subtreeMask = s_subtreeMaskByPackedState[packedStates[i]];
                while (subtreeMask != 0)
                {
                    var localOffset = BitOperations.TrailingZeroCount((uint)subtreeMask);
                    subtreeMask = (byte)(subtreeMask & (subtreeMask - 1));
                    var flatIndex = baseIndex + localOffset;
                    tile.Contents[flatIndex] = DeserializeVolumeSubtile(reader, tile, flatIndex);
                }
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(rentedStates);
        }
    }

    private static ushort DeserializeVolumeSubtile(BinaryReader reader, VoxelMap.Tile parent, int flatIndex)
    {
        var localId = parent.SubdivisionCount;
        if (localId >= VoxelMap.VoxelIdMask)
            throw new Exception("体积子树数量超出上限");

        var subBounds = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel((ushort)flatIndex));
        var child     = new VoxelMap.Tile(parent.Owner, subBounds.min, subBounds.max, parent.Level + 1, false);
        parent.AddSubdivision(child);
        DeserializeVolumeTile(reader, child);
        return (ushort)(VoxelMap.VoxelOccupiedBit | localId);
    }

    private static void SerializeVolumeTile(BinaryWriter writer, VoxelMap.Tile tile)
    {
        var packedBytes  = PackedStateBytes(tile.Contents.Length);
        var rentedStates = ArrayPool<byte>.Shared.Rent(packedBytes);
        var packedStates = rentedStates.AsSpan(0, packedBytes);
        packedStates.Clear();
        var allEmpty     = true;
        var allSolidLeaf = tile.SubdivisionCount == 0;

        try
        {
            for (var i = 0; i < tile.Contents.Length; ++i)
            {
                var cell  = tile.Contents[i];
                var state = ClassifyCell(cell);
                packedStates[i >> 2] |= (byte)((byte)state << (i & 3) * 2);
                allEmpty             &= state == VolumeCellState.Empty;
                allSolidLeaf         &= state == VolumeCellState.SolidLeaf;
            }

            var encoding = allEmpty ? VolumeTileEncoding.Empty : allSolidLeaf ? VolumeTileEncoding.SolidLeaf : VolumeTileEncoding.Mixed;
            writer.Write((byte)encoding);
            if (encoding != VolumeTileEncoding.Mixed)
                return;

            writer.BaseStream.Write(packedStates);

            for (var i = 0; i < tile.Contents.Length; ++i)
            {
                if (ClassifyCell(tile.Contents[i]) != VolumeCellState.Subtree)
                    continue;

                var localId = tile.Contents[i] & VoxelMap.VoxelIdMask;
                if (localId >= tile.SubdivisionCount)
                    throw new Exception($"体积子树索引越界: {localId} / {tile.SubdivisionCount}");
                SerializeVolumeTile(writer, tile.GetSubdivision(localId));
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(rentedStates);
        }
    }

    private static VolumeCellState ClassifyCell(ushort value) => value switch
    {
        0               => VolumeCellState.Empty,
        ushort.MaxValue => VolumeCellState.SolidLeaf,
        _               => VolumeCellState.Subtree
    };

    private static readonly ulong[] s_expandedContentsByPackedState = BuildExpandedContentsByPackedState();
    private static readonly byte[]  s_subtreeMaskByPackedState      = BuildSubtreeMaskByPackedState();
    private static readonly byte[]  s_subtreeCountByPackedState     = BuildSubtreeCountByPackedState();
    private static readonly bool[]  s_invalidPackedState            = BuildInvalidPackedState();

    private static int PackedStateBytes(int numCells) => numCells + 3 >> 2;

    private static bool TryCaptureDeferredVolumeTree(Stream stream, out byte[] payload, out int offset, out int length)
    {
        if (stream is MemoryStream memoryStream && memoryStream.TryGetBuffer(out var buffer) && buffer.Array != null)
        {
            payload = buffer.Array;
            offset  = buffer.Offset + checked((int)memoryStream.Position);
            length  = checked((int)(memoryStream.Length - memoryStream.Position));
            return true;
        }

        payload = [];
        offset  = 0;
        length  = 0;
        return false;
    }

    private static ulong[] BuildExpandedContentsByPackedState()
    {
        var table = GC.AllocateUninitializedArray<ulong>(byte.MaxValue + 1);
        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            ulong expanded = 0;
            for (var offset = 0; offset < 4; ++offset)
            {
                var state = (VolumeCellState)(packedState >> offset * 2 & 0x3);
                var value = state switch
                {
                    VolumeCellState.Empty     => (ushort)0,
                    VolumeCellState.SolidLeaf => ushort.MaxValue,
                    VolumeCellState.Subtree   => VoxelMap.VoxelOccupiedBit,
                    _                         => (ushort)0
                };
                expanded |= (ulong)value << offset * 16;
            }

            table[packedState] = expanded;
        }

        return table;
    }

    private static byte[] BuildSubtreeMaskByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>(byte.MaxValue + 1);
        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            byte mask = 0;
            for (var offset = 0; offset < 4; ++offset)
                if ((VolumeCellState)(packedState >> offset * 2 & 0x3) == VolumeCellState.Subtree)
                    mask |= (byte)(1 << offset);
            table[packedState] = mask;
        }

        return table;
    }

    private static byte[] BuildSubtreeCountByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>(byte.MaxValue + 1);
        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
            table[packedState] = (byte)BitOperations.PopCount(s_subtreeMaskByPackedState[packedState]);
        return table;
    }

    private static bool[] BuildInvalidPackedState()
    {
        var table = GC.AllocateUninitializedArray<bool>(byte.MaxValue + 1);
        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            for (var offset = 0; offset < 4; ++offset)
            {
                if ((VolumeCellState)(packedState >> offset * 2 & 0x3) == (VolumeCellState)3)
                {
                    table[packedState] = true;
                    break;
                }
            }
        }

        return table;
    }
}
