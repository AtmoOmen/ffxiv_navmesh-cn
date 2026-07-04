using System.Numerics;

namespace vnavmesh.Common.Navigation.Volume.Search;

public class Voxelizer
{
    public int SizeX { get; }
    public int SizeY { get; }
    public int SizeZ { get; }

    private readonly ulong[]  solidWords;
    private readonly ulong[]? emptyWords;

    public Voxelizer(int nx, int ny, int nz, bool partial = false)
    {
        if (!BitOperations.IsPow2(nx) || !BitOperations.IsPow2(ny) || !BitOperations.IsPow2(nz))
            throw new Exception($"Non-power-of-two size not supported: {nx}x{ny}x{nz}");

        SizeX = nx;
        SizeY = ny;
        SizeZ = nz;

        var numCells  = nx * ny * nz;
        var wordCount = numCells + 63 >> 6;
        solidWords = GC.AllocateUninitializedArray<ulong>(wordCount);
        if (partial)
            emptyWords = GC.AllocateUninitializedArray<ulong>(wordCount);
    }

    public int VoxelToIndex(int x, int y, int z) => (z * SizeX + x) * SizeY + y;

    public (bool solid, bool empty) GetCellState(int index)
    {
        var solid = GetBit(solidWords, index);
        var empty = emptyWords != null ?
                        GetBit(emptyWords, index) :
                        !solid;
        return (solid, empty);
    }

    public (bool solid, bool empty) GetCellState(int x, int y, int z) => GetCellState(VoxelToIndex(x, y, z));

    public (bool solid, bool empty) ClassifyBox(int x0, int y0, int z0, int sizeX, int sizeY, int sizeZ)
    {
        var anySolid = false;

        if (emptyWords == null)
        {
            var allSolid = true;

            for (var z = 0; z < sizeZ; ++z)
            for (var x = 0; x < sizeX; ++x)
            {
                var startIndex = VoxelToIndex(x0 + x, y0, z0 + z);
                var (segmentAnySolid, segmentAllSolid) =  ClassifyRange(solidWords, startIndex, sizeY);
                anySolid                               |= segmentAnySolid;
                allSolid                               &= segmentAllSolid;
                if (anySolid && !allSolid)
                    return (true, true);
            }

            return anySolid ?
                       (true, !allSolid) :
                       (false, true);
        }

        var anyEmpty = false;

        for (var z = 0; z < sizeZ; ++z)
        for (var x = 0; x < sizeX; ++x)
        {
            var startIndex = VoxelToIndex(x0 + x, y0, z0 + z);
            var (segmentAnySolid, segmentAnyEmpty) =  ClassifyRangePair(solidWords, emptyWords, startIndex, sizeY);
            anySolid                               |= segmentAnySolid;
            anyEmpty                               |= segmentAnyEmpty;
            if (anySolid && anyEmpty)
                return (true, true);
        }

        return (anySolid, anyEmpty);
    }

    public void AddSpan(int x, int z, int y0, int y1)
    {
        if ((uint)x >= (uint)SizeX || (uint)z >= (uint)SizeZ)
            return;
        if (y1 < 0 || y0 >= SizeY)
            return;

        y0 = Math.Clamp(y0, 0,  SizeY - 1);
        y1 = Math.Clamp(y1, y0, SizeY - 1);
        var startIndex = VoxelToIndex(x, y0, z);
        SetRange(solidWords, startIndex, y1 - y0 + 1);
    }

    public void Clear()
    {
        Array.Clear(solidWords);
        if (emptyWords != null)
            Array.Clear(emptyWords);
    }

    private static bool GetBit(ulong[] words, int index)
    {
        var wordIndex = index >> 6;
        var bitMask   = 1UL   << (index & 63);
        return (words[wordIndex] & bitMask) != 0;
    }

    private static void SetRange(ulong[] words, int startIndex, int length)
    {
        if (length <= 0)
            return;

        var index     = startIndex;
        var remaining = length;

        while (remaining > 0)
        {
            var wordIndex = index >> 6;
            var bitOffset = index & 63;
            var bitCount  = Math.Min(64 - bitOffset, remaining);
            var mask = bitCount == 64 ?
                           ulong.MaxValue :
                           (1UL << bitCount) - 1 << bitOffset;
            words[wordIndex] |= mask;
            index            += bitCount;
            remaining        -= bitCount;
        }
    }

    private static (bool anySet, bool allSet) ClassifyRange(ulong[] words, int startIndex, int length)
    {
        var index     = startIndex;
        var remaining = length;
        var anySet    = false;
        var allSet    = true;

        while (remaining > 0)
        {
            var wordIndex = index >> 6;
            var bitOffset = index & 63;
            var bitCount  = Math.Min(64 - bitOffset, remaining);
            var mask = bitCount == 64 ?
                           ulong.MaxValue :
                           (1UL << bitCount) - 1 << bitOffset;
            var value = words[wordIndex] & mask;
            if (value != 0)
                anySet = true;
            if (value != mask)
                allSet = false;
            if (anySet && !allSet)
                return (true, false);
            index     += bitCount;
            remaining -= bitCount;
        }

        return (anySet, allSet);
    }

    private static (bool anySet1, bool anySet2) ClassifyRangePair(ulong[] words1, ulong[] words2, int startIndex, int length)
    {
        var index     = startIndex;
        var remaining = length;
        var anySet1   = false;
        var anySet2   = false;

        while (remaining > 0)
        {
            var wordIndex = index >> 6;
            var bitOffset = index & 63;
            var bitCount  = Math.Min(64 - bitOffset, remaining);
            var mask = bitCount == 64 ?
                           ulong.MaxValue :
                           (1UL << bitCount) - 1 << bitOffset;
            anySet1 |= (words1[wordIndex] & mask) != 0;
            anySet2 |= (words2[wordIndex] & mask) != 0;
            if (anySet1 && anySet2)
                return (true, true);
            index     += bitCount;
            remaining -= bitCount;
        }

        return (anySet1, anySet2);
    }
}
