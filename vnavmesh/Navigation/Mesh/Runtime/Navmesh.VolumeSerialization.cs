using System.Numerics;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

public partial record class Navmesh
{
    private static VoxelMap? DeserializeVolume(BinaryReader reader)
    {
        var volume = DeserializeVolumeHeader(reader);
        if (volume == null)
            return null;

        if (TryCaptureDeferredVolumeTree(reader.BaseStream, out var payload, out var offset, out var length))
        {
            volume.SetDeferredTreePayload(payload, offset, length);
            reader.BaseStream.Position = reader.BaseStream.Length;
        }
        else DeserializeVolumeTile(reader, volume.RootTile);

        return volume;
    }

    private static VoxelMap? DeserializeVolumeHeader(BinaryReader reader)
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
        return new(min, max, tilesPerLevel);
    }

    private static VoxelMap? DeserializeCompressedDeferredVolume(byte[] payload, long expectedBytes)
    {
        var       decodedPayload = DecompressFastLz(payload, expectedBytes);
        using var stream         = new MemoryStream(decodedPayload, 0, decodedPayload.Length, false, true);
        using var reader         = new BinaryReader(stream);
        var       volume         = DeserializeVolumeHeader(reader);
        if (volume == null)
            return null;

        var treeOffset = checked((int)stream.Position);
        volume.SetDeferredTreeMaterializer(v => MaterializeDeferredCompressedVolumeTree(v, payload, expectedBytes, treeOffset));
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
        volume.CompactRetainedState();
        SerializeBounds(writer, volume.RootTile.BoundsMin, volume.RootTile.BoundsMax);
        SerializeVolumeTile(writer, volume.RootTile);
    }

    internal static void MaterializeDeferredVolumeTree(VoxelMap volume, byte[] payload, int offset, int length)
    {
        using var stream = new MemoryStream(payload, offset, length, false);
        using var reader = new BinaryReader(stream);
        DeserializeVolumeTile(reader, volume.RootTile);
    }

    internal static void MaterializeDeferredCompressedVolumeTree(VoxelMap volume, byte[] payload, long expectedBytes, int offset)
    {
        var       decodedPayload = DecompressFastLz(payload, expectedBytes);
        using var stream         = new MemoryStream(decodedPayload, offset, decodedPayload.Length - offset, false);
        using var reader         = new BinaryReader(stream);
        DeserializeVolumeTile(reader, volume.RootTile);
    }

    private static (VoxelMap? Value, CacheSegmentTelemetry Telemetry) DecodeDeferredVolumeSegment(CacheSegmentDescriptor descriptor, Stream source)
    {
        var timer   = StopWatchTimer.Create();
        var payload = ReadSegmentPayload(source, descriptor);
        var volume  = DeserializeCompressedDeferredVolume(payload, descriptor.UncompressedBytes);
        return (volume, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static void DeserializeVolumeTile(BinaryReader reader, VoxelMap.Tile tile)
    {
        var encoding = (VolumeTileEncoding)reader.ReadByte();

        switch (encoding)
        {
            case VolumeTileEncoding.Empty:
                tile.SetUniformEmpty();
                return;
            case VolumeTileEncoding.SolidLeaf:
                tile.SetUniformSolidLeaf();
                return;
            case VolumeTileEncoding.Mixed:
                break;
            default:
                throw new Exception($"未知的体积编码类型: {encoding}");
        }

        tile.ClearSubdivision();
        var packedBytes  = PackedStateBytes(tile.CellCount);
        var packedStates = GC.AllocateUninitializedArray<byte>(packedBytes);
        reader.BaseStream.ReadExactly(packedStates);

        var subtreeCount = 0;

        for (var i = 0; i < packedStates.Length; ++i)
        {
            var packedState = packedStates[i];
            if (s_invalidPackedState[packedState])
                throw new Exception($"未知的体积单元状态字节: 0x{packedState:X2}");

            subtreeCount += s_subtreeCountByPackedState[packedState];
        }

        tile.SetPackedStates(packedStates);
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
                subtreeMask = (byte)(subtreeMask & subtreeMask - 1);
                DeserializeVolumeSubtile(reader, tile, baseIndex + localOffset);
            }
        }
    }

    private static void DeserializeVolumeSubtile(BinaryReader reader, VoxelMap.Tile parent, int flatIndex)
    {
        var localId = parent.SubdivisionCount;
        if (localId >= VoxelMap.VOXEL_ID_MASK)
            throw new Exception("体积子树数量超出上限");

        var subBounds = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel((ushort)flatIndex));
        var child     = new VoxelMap.Tile(parent.Owner, subBounds.min, subBounds.max, parent.Level + 1, false);
        parent.AddSubdivision(child);
        DeserializeVolumeTile(reader, child);
    }

    private static void SerializeVolumeTile(BinaryWriter writer, VoxelMap.Tile tile)
    {
        tile.CompactRetainedState();

        switch (tile.StorageKind)
        {
            case VoxelMap.TileStorageKind.AllEmpty:
                writer.Write((byte)VolumeTileEncoding.Empty);
                return;
            case VoxelMap.TileStorageKind.SolidLeaf:
                writer.Write((byte)VolumeTileEncoding.SolidLeaf);
                return;
            case VoxelMap.TileStorageKind.PackedMixed:
                writer.Write((byte)VolumeTileEncoding.Mixed);
                writer.BaseStream.Write(tile.PackedStates);
                break;
            case VoxelMap.TileStorageKind.Dense:
                throw new InvalidOperationException("体积瓦片序列化前未完成压缩");
            default:
                throw new InvalidOperationException($"未知的体积瓦片存储类型: {tile.StorageKind}");
        }

        for (var i = 0; i < tile.CellCount; ++i)
        {
            if (!tile.IsSubdividedCell(i))
                continue;

            var localId = tile.GetSubdivisionIndex(i);
            if (localId >= tile.SubdivisionCount)
                throw new Exception($"体积子树索引越界: {localId} / {tile.SubdivisionCount}");
            SerializeVolumeTile(writer, tile.GetSubdivision(localId));
        }
    }

    private static readonly byte[] s_subtreeMaskByPackedState  = BuildSubtreeMaskByPackedState();
    private static readonly byte[] s_subtreeCountByPackedState = BuildSubtreeCountByPackedState();
    private static readonly bool[] s_invalidPackedState        = BuildInvalidPackedState();

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

    private static byte[] BuildSubtreeMaskByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>(byte.MaxValue + 1);

        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            byte mask = 0;
            for (var offset = 0; offset < 4; ++offset)
                if ((VolumeCellState)(packedState >> offset * 2 & 0x3) == VolumeCellState.Subtree)
                    mask |= (byte)(1              << offset);
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
        for (var offset = 0; offset < 4; ++offset)
            if ((VolumeCellState)(packedState >> offset * 2 & 0x3) == (VolumeCellState)3)
            {
                table[packedState] = true;
                break;
            }

        return table;
    }
}
