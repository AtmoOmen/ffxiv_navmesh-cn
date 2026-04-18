using System.Numerics;

namespace vnavmesh.Navigation.Volume;

// raw 1/2-bit-per-voxel container
// for downsampled mips, w is 0 for 'has solids', 1 for 'has non-solids'
public class Voxelizer
{
    public int NumX;
    public int NumY;
    public int NumZ;

    private readonly ulong[]  solidWords;
    private readonly ulong[]? emptyWords;

    public Voxelizer(int nx, int ny, int nz, bool partial = false)
    {
        if (!BitOperations.IsPow2(nx) || !BitOperations.IsPow2(ny) || !BitOperations.IsPow2(nz))
            throw new Exception($"Non-power-of-two size not supported: {nx}x{ny}x{nz}");

        NumX = nx;
        NumY = ny;
        NumZ = nz;

        var numCells  = nx * ny * nz;
        var wordCount = numCells + 63 >> 6;
        solidWords = GC.AllocateUninitializedArray<ulong>(wordCount);
        if (partial)
            emptyWords = GC.AllocateUninitializedArray<ulong>(wordCount);
    }

    public int VoxelToIndex(int x, int y, int z) => (z * NumX + x) * NumY + y;

    public (bool solid, bool empty) Get(int idx)
    {
        var solid = GetBit(solidWords, idx);
        var empty = emptyWords != null ? GetBit(emptyWords, idx) : !solid;
        return (solid, empty);
    }

    public (bool solid, bool empty) Get(int x, int y, int z) => Get(VoxelToIndex(x, y, z));

    public (bool solid, bool empty) ClassifyRegion(int x0, int y0, int z0, int sizeX, int sizeY, int sizeZ)
    {
        var anySolid = false;

        if (emptyWords == null)
        {
            var allSolid = true;

            for (var z = 0; z < sizeZ; ++z)
            for (var x = 0; x < sizeX; ++x)
            {
                var startIndex = VoxelToIndex(x0 + x, y0, z0 + z);
                var (segmentAnySolid, segmentAllSolid) =  RangeClassify(solidWords, startIndex, sizeY);
                anySolid                               |= segmentAnySolid;
                allSolid                               &= segmentAllSolid;
                if (anySolid && !allSolid)
                    return (true, true);
            }

            return anySolid ? (true, !allSolid) : (false, true);
        }

        var anyEmpty = false;

        for (var z = 0; z < sizeZ; ++z)
        for (var x = 0; x < sizeX; ++x)
        {
            var startIndex = VoxelToIndex(x0 + x, y0, z0 + z);
            var (segmentAnySolid, segmentAnyEmpty) =  RangeClassifyPair(solidWords, emptyWords, startIndex, sizeY);
            anySolid                               |= segmentAnySolid;
            anyEmpty                               |= segmentAnyEmpty;
            if (anySolid && anyEmpty)
                return (true, true);
        }

        return (anySolid, anyEmpty);
    }

    public void AddSpan(int x, int z, int y0, int y1)
    {
        if ((uint)x >= (uint)NumX || (uint)z >= (uint)NumZ)
            return;
        if (y1 < 0 || y0 >= NumY)
            return;

        y0 = Math.Clamp(y0, 0,  NumY - 1);
        y1 = Math.Clamp(y1, y0, NumY - 1);
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
            var mask = bitCount == 64
                           ? ulong.MaxValue
                           : (1UL << bitCount) - 1 << bitOffset;
            words[wordIndex] |= mask;
            index            += bitCount;
            remaining        -= bitCount;
        }
    }

    private static (bool anySet, bool allSet) RangeClassify(ulong[] words, int startIndex, int length)
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
            var mask = bitCount == 64
                           ? ulong.MaxValue
                           : (1UL << bitCount) - 1 << bitOffset;
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

    private static (bool anySet1, bool anySet2) RangeClassifyPair(ulong[] words1, ulong[] words2, int startIndex, int length)
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
            var mask = bitCount == 64
                           ? ulong.MaxValue
                           : (1UL << bitCount) - 1 << bitOffset;
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
