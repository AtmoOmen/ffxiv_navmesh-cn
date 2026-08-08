using System.Runtime.CompilerServices;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight.Models;

namespace vnavmesh.Query.Flight;

internal static class VoxelNeighbourOffsets
{
    public static readonly (int Dx, int Dy, int Dz)[] All = Build();

    private static (int Dx, int Dy, int Dz)[] Build()
    {
        var offsets = new (int Dx, int Dy, int Dz)[26];
        var i = 0;

        for (var dx = -1; dx <= 1; ++dx)
        for (var dy = -1; dy <= 1; ++dy)
        for (var dz = -1; dz <= 1; ++dz)
        {
            if (dx == 0 && dy == 0 && dz == 0)
                continue;

            offsets[i++] = (dx, dy, dz);
        }

        return offsets;
    }
}

internal struct VoxelNodeLookup
{
    private ulong[] keys;
    private int[]   values;
    private int     mask;

    public int Count { get; private set; }

    public VoxelNodeLookup
    (
        int capacity
    )
    {
        var size = RoundUpPowerOf2(Math.Max(capacity, 16));
        keys   = new ulong[size];
        values = new int[size];
        mask   = size - 1;
        Array.Fill(keys,   SparseVoxelOctree.INVALID_VOXEL);
        Array.Fill(values, -1);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetValue
    (
        ulong key,
        out int value
    )
    {
        var idx = (int)(Mix(key) & (uint)mask);

        while (true)
        {
            var k = keys[idx];

            if (k == key)
            {
                value = values[idx];
                return true;
            }

            if (k == SparseVoxelOctree.INVALID_VOXEL)
            {
                value = -1;
                return false;
            }

            idx = (idx + 1) & mask;
        }
    }

    public void Set
    (
        ulong key,
        int   value
    )
    {
        if (Count >= (keys.Length * 3) >> 2)
            Grow();

        var idx = (int)(Mix(key) & (uint)mask);

        while (keys[idx] != SparseVoxelOctree.INVALID_VOXEL)
            idx = (idx + 1) & mask;

        keys[idx]   = key;
        values[idx] = value;
        ++Count;
    }

    public void SetOrUpdate
    (
        ulong key,
        int   value
    )
    {
        if (Count >= (keys.Length * 3) >> 2)
            Grow();

        var idx = (int)(Mix(key) & (uint)mask);

        while (true)
        {
            var k = keys[idx];

            if (k == key)
            {
                values[idx] = value;
                return;
            }

            if (k == SparseVoxelOctree.INVALID_VOXEL)
            {
                keys[idx]   = key;
                values[idx] = value;
                ++Count;
                return;
            }

            idx = (idx + 1) & mask;
        }
    }

    public void Clear()
    {
        if (Count == 0)
            return;

        Array.Fill(keys,   SparseVoxelOctree.INVALID_VOXEL);
        Array.Fill(values, -1);
        Count = 0;
    }

    public void TrimExcess()
    {
        if (keys.Length <= 2048)
            return;

        var size = RoundUpPowerOf2(Math.Max(Count * 2, 16));

        if (size >= keys.Length)
            return;

        var newKeys   = new ulong[size];
        var newValues = new int[size];
        var newMask   = size - 1;
        Array.Fill(newKeys,   SparseVoxelOctree.INVALID_VOXEL);
        Array.Fill(newValues, -1);

        for (var i = 0; i < keys.Length; ++i)
            if (keys[i] != SparseVoxelOctree.INVALID_VOXEL)
                SetInternal(newKeys, newValues, newMask, keys[i], values[i]);

        keys   = newKeys;
        values = newValues;
        mask   = newMask;
    }

    private void Grow()
    {
        var newSize   = keys.Length << 1;
        var newKeys   = new ulong[newSize];
        var newValues = new int[newSize];
        var newMask   = newSize - 1;
        Array.Fill(newKeys,   SparseVoxelOctree.INVALID_VOXEL);
        Array.Fill(newValues, -1);

        for (var i = 0; i < keys.Length; ++i)
            if (keys[i] != SparseVoxelOctree.INVALID_VOXEL)
                SetInternal(newKeys, newValues, newMask, keys[i], values[i]);

        keys   = newKeys;
        values = newValues;
        mask   = newMask;
    }

    private static void SetInternal
    (
        ulong[] keys,
        int[]   values,
        int     mask,
        ulong   key,
        int     value
    )
    {
        var idx = (int)(Mix(key) & (uint)mask);

        while (keys[idx] != SparseVoxelOctree.INVALID_VOXEL)
            idx = (idx + 1) & mask;

        keys[idx]   = key;
        values[idx] = value;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static uint Mix
    (
        ulong key
    )
    {
        key ^= key >> 33;
        key *= 0xff51afd7ed558ccdUL;
        key ^= key >> 33;
        return (uint)key;
    }

    private static int RoundUpPowerOf2
    (
        int value
    )
    {
        var result = 1;

        while (result < value)
            result <<= 1;

        return result;
    }
}
