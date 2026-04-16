using System.Buffers;
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
        DeserializeVolumeTile(reader, volume.RootTile);
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

        SerializeBounds(writer, volume.RootTile.BoundsMin, volume.RootTile.BoundsMax);
        SerializeVolumeTile(writer, volume.RootTile);
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
            for (var i = 0; i < tile.Contents.Length; ++i)
                if ((VolumeCellState)(packedStates[i >> 2] >> (i & 3) * 2 & 0x3) == VolumeCellState.Subtree)
                    ++subtreeCount;

            tile.EnsureSubdivisionCapacity(subtreeCount);

            for (var i = 0; i < tile.Contents.Length; ++i)
            {
                var state = (VolumeCellState)(packedStates[i >> 2] >> (i & 3) * 2 & 0x3);
                tile.Contents[i] = state switch
                {
                    VolumeCellState.Empty     => 0,
                    VolumeCellState.SolidLeaf => ushort.MaxValue,
                    VolumeCellState.Subtree   => DeserializeVolumeSubtile(reader, tile, i),
                    _                         => throw new Exception($"未知的体积单元状态: {state}")
                };
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
        var child     = new VoxelMap.Tile(parent.Owner, subBounds.min, subBounds.max, parent.Level + 1);
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

    private static int PackedStateBytes(int numCells) => numCells + 3 >> 2;
}
