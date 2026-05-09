using System.Numerics;

namespace vnavmesh.Common.Navigation.Volume.Map;

public sealed class VolumeLevel
{
    public Vector3 CellSize    { get; init; }
    public Vector3 InvCellSize { get; init; }
    public int     NumCellsX   { get; init; }
    public int     NumCellsY   { get; init; }
    public int     NumCellsZ   { get; init; }
    public int     ShiftYX     { get; init; }
    public int     ShiftXZ     { get; init; }

    public int NumCellsTotal => NumCellsX * NumCellsY * NumCellsZ;

    public VolumeLevel(Vector3 extent, int nx, int ny, int nz)
    {
        if ((nx & nx - 1) != 0 || (ny & ny - 1) != 0 || (nz & nz - 1) != 0)
            throw new ArgumentException($"Non-power-of-2 cell size not supported: got {nx}*{ny}*{nz}");

        CellSize    = extent         / new Vector3(nx, ny, nz);
        InvCellSize = new Vector3(1) / CellSize;
        NumCellsX   = nx;
        NumCellsY   = ny;
        NumCellsZ   = nz;
        ShiftYX     = BitOperations.Log2((uint)ny);
        ShiftXZ     = BitOperations.Log2((uint)nx);
    }

    public VolumeLevel(Vector3 extent, int ncells) : this(extent, ncells, ncells, ncells) { }

    public bool InBounds(int x, int y, int z) => x >= 0 && x < NumCellsX && y >= 0 && y < NumCellsY && z >= 0 && z < NumCellsZ;

    public bool InBounds((int x, int y, int z) v) => InBounds(v.x, v.y, v.z);

    public (int x, int y, int z) ClampMin(int x, int y, int z) => (Math.Max(x, 0), Math.Max(y, 0), Math.Max(z, 0));

    public (int x, int y, int z) ClampMin((int x, int y, int z) v) => ClampMin(v.x, v.y, v.z);

    public (int x, int y, int z) ClampMax(int x, int y, int z) => (Math.Min(x, NumCellsX - 1), Math.Min(y, NumCellsY - 1), Math.Min(z, NumCellsZ - 1));

    public (int x, int y, int z) ClampMax((int x, int y, int z) v) => ClampMax(v.x, v.y, v.z);

    public (int x, int y, int z) Clamp(int x, int y, int z) => ClampMax(ClampMin(x, y, z));

    public (int x, int y, int z) Clamp((int x, int y, int z) v) => ClampMax(ClampMin(v.x, v.y, v.z));

    public (int x, int y, int z) IndexToVoxel(ushort index)
    {
        var y  = index & NumCellsY - 1;
        var xz = index >> ShiftYX;
        var x  = xz & NumCellsX - 1;
        var z  = xz >> ShiftXZ;
        return (x, y, z);
    }

    public ushort VoxelToIndex(int x, int y, int z) => (ushort)(((z << ShiftXZ) + x << ShiftYX) + y);

    public ushort VoxelToIndex((int x, int y, int z) v) => VoxelToIndex(v.x, v.y, v.z);
}
