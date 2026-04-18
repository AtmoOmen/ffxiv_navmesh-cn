using System.Numerics;

namespace vnavmesh.Navigation.Volume;

// raw 1/2-bit-per-voxel container
// for downsampled mips, w is 0 for 'has solids', 1 for 'has non-solids'
public class Voxelizer
{
    public int NumX;
    public int NumY;
    public int NumZ;
    public int NumW => _emptyWords == null ? 1 : 2;

    private readonly ulong[]  _solidWords;
    private readonly ulong[]? _emptyWords;

    public Voxelizer(int nx, int ny, int nz, bool partial = false)
    {
        if (!BitOperations.IsPow2(nx) || !BitOperations.IsPow2(ny) || !BitOperations.IsPow2(nz))
            throw new Exception($"Non-power-of-two size not supported: {nx}x{ny}x{nz}");

        NumX = nx;
        NumY = ny;
        NumZ = nz;

        var numCells  = nx * ny * nz;
        var wordCount = numCells + 63 >> 6;
        _solidWords = GC.AllocateUninitializedArray<ulong>(wordCount);
        if (partial)
            _emptyWords = GC.AllocateUninitializedArray<ulong>(wordCount);
    }

    public int VoxelToIndex(int x, int y, int z) => (z * NumX + x) * NumY + y;

    public (bool solid, bool empty) Get(int idx)
    {
        var solid = GetBit(_solidWords, idx);
        var empty = _emptyWords != null ? GetBit(_emptyWords, idx) : !solid;
        return (solid, empty);
    }

    public (bool solid, bool empty) Get(int x, int y, int z) => Get(VoxelToIndex(x, y, z));

    public (bool solid, bool empty) ClassifyRegion(int x0, int y0, int z0, int sizeX, int sizeY, int sizeZ)
    {
        var anySolid = false;

        if (_emptyWords == null)
        {
            var allSolid = true;

            for (var z = 0; z < sizeZ; ++z)
            for (var x = 0; x < sizeX; ++x)
            {
                var startIndex = VoxelToIndex(x0 + x, y0, z0 + z);
                var (segmentAnySolid, segmentAllSolid) = RangeClassify(_solidWords, startIndex, sizeY);
                anySolid |= segmentAnySolid;
                allSolid &= segmentAllSolid;
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
            var (segmentAnySolid, segmentAnyEmpty) = RangeClassifyPair(_solidWords, _emptyWords, startIndex, sizeY);
            anySolid |= segmentAnySolid;
            anyEmpty |= segmentAnyEmpty;
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

        y0 = Math.Clamp(y0, 0, NumY - 1);
        y1 = Math.Clamp(y1, y0, NumY - 1);
        var startIndex = VoxelToIndex(x, y0, z);
        SetRange(_solidWords, startIndex, y1 - y0 + 1);
    }

    public void Clear()
    {
        Array.Clear(_solidWords);
        if (_emptyWords != null)
            Array.Clear(_emptyWords);
    }

    public void DownsampleInto(Voxelizer result, int dx, int dy, int dz)
    {
        result.Clear();

        var shiftX = BitOperations.Log2((uint)dx);
        var shiftY = BitOperations.Log2((uint)dy);
        var shiftZ = BitOperations.Log2((uint)dz);
        var idx    = 0;

        for (var z = 0; z < NumZ; ++z)
        for (var x = 0; x < NumX; ++x)
        for (var y = 0; y < NumY; ++y, ++idx)
        {
            var solid    = GetBit(_solidWords, idx);
            var empty    = _emptyWords != null ? GetBit(_emptyWords, idx) : !solid;
            var resIndex = result.VoxelToIndex(x >> shiftX, y >> shiftY, z >> shiftZ);
            if (solid)
                SetBit(result._solidWords, resIndex);
            if (empty && result._emptyWords != null)
                SetBit(result._emptyWords, resIndex);
        }
    }

    public Voxelizer Downsample(int dx, int dy, int dz)
    {
        var result = new Voxelizer(NumX / dx, NumY / dy, NumZ / dz, true);
        DownsampleInto(result, dx, dy, dz);
        return result;
    }

    private static bool GetBit(ulong[] words, int index)
    {
        var wordIndex = index >> 6;
        var bitMask   = 1UL   << (index & 63);
        return (words[wordIndex] & bitMask) != 0;
    }

    private static void SetBit(ulong[] words, int index)
    {
        var wordIndex = index >> 6;
        var bitMask   = 1UL   << (index & 63);
        words[wordIndex] |= bitMask;
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

    private static bool RangeAnySet(ulong[] words, int startIndex, int length)
    {
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
            if ((words[wordIndex] & mask) != 0)
                return true;
            index     += bitCount;
            remaining -= bitCount;
        }

        return false;
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

    private static bool RangeAllSet(ulong[] words, int startIndex, int length)
    {
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
            if ((words[wordIndex] & mask) != mask)
                return false;
            index     += bitCount;
            remaining -= bitCount;
        }

        return true;
    }
}
