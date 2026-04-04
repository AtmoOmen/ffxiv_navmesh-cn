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
                tile.Subdivision.Clear();
                return;
            case VolumeTileEncoding.SolidLeaf:
                Array.Fill(tile.Contents, ushort.MaxValue);
                tile.Subdivision.Clear();
                return;
            case VolumeTileEncoding.Mixed:
                break;
            default:
                throw new Exception($"未知的体积编码类型: {encoding}");
        }

        tile.Subdivision.Clear();
        var packedBytes  = PackedStateBytes(tile.Contents.Length);
        var packedStates = GC.AllocateUninitializedArray<byte>(packedBytes);
        reader.ReadExactly(packedStates);

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

    private static ushort DeserializeVolumeSubtile(BinaryReader reader, VoxelMap.Tile parent, int flatIndex)
    {
        var localId = parent.Subdivision.Count;
        if (localId >= VoxelMap.VoxelIdMask)
            throw new Exception("体积子树数量超出上限");

        var subBounds = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel((ushort)flatIndex));
        var child     = new VoxelMap.Tile(parent.Owner, subBounds.min, subBounds.max, parent.Level + 1);
        parent.Subdivision.Add(child);
        DeserializeVolumeTile(reader, child);
        return (ushort)(VoxelMap.VoxelOccupiedBit | localId);
    }

    private static void SerializeVolumeTile(BinaryWriter writer, VoxelMap.Tile tile)
    {
        var packedBytes  = PackedStateBytes(tile.Contents.Length);
        var packedStates = GC.AllocateUninitializedArray<byte>(packedBytes);
        var subtreeIds   = GC.AllocateUninitializedArray<ushort>(tile.Contents.Length);
        packedStates.AsSpan().Clear();
        var subtreeCount = 0;
        var allEmpty     = true;
        var allSolidLeaf = tile.Subdivision.Count == 0;

        for (var i = 0; i < tile.Contents.Length; ++i)
        {
            var cell  = tile.Contents[i];
            var state = ClassifyCell(cell);
            packedStates[i >> 2] |= (byte)((byte)state << (i & 3) * 2);
            allEmpty             &= state == VolumeCellState.Empty;
            allSolidLeaf         &= state == VolumeCellState.SolidLeaf;
            if (state == VolumeCellState.Subtree)
                subtreeIds[subtreeCount++] = (ushort)(cell & VoxelMap.VoxelIdMask);
        }

        var encoding = allEmpty ? VolumeTileEncoding.Empty : allSolidLeaf ? VolumeTileEncoding.SolidLeaf : VolumeTileEncoding.Mixed;
        writer.Write((byte)encoding);
        if (encoding != VolumeTileEncoding.Mixed)
            return;

        writer.Write(packedStates);

        for (var i = 0; i < subtreeCount; ++i)
        {
            var localId = subtreeIds[i];
            if (localId >= tile.Subdivision.Count)
                throw new Exception($"体积子树索引越界: {localId} / {tile.Subdivision.Count}");
            SerializeVolumeTile(writer, tile.Subdivision[localId]);
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
