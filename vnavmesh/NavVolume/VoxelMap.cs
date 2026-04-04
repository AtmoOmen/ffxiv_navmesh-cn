using System.Numerics;

namespace vnavmesh.NavVolume;

public class VoxelMap
{
    public class Level
    {
        public Vector3 CellSize    { get; init; }
        public Vector3 InvCellSize { get; init; }
        public int     NumCellsX   { get; init; }
        public int     NumCellsY   { get; init; }
        public int     NumCellsZ   { get; init; }
        public int     ShiftYX     { get; init; }
        public int     ShiftXZ     { get; init; }

        public int NumCellsTotal => NumCellsX * NumCellsY * NumCellsZ;

        public Level(Vector3 extent, int nx, int ny, int nz)
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

        public Level(Vector3 extent, int ncells) : this(extent, ncells, ncells, ncells) { }

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

    public class Tile
    {
        public VoxelMap Owner     { get; init; }
        public Vector3  BoundsMin { get; init; }
        public Vector3  BoundsMax { get; init; }
        public int      Level     { get; init; }

        public ushort[]
            Contents
        {
            get;
            init;
        } // high bit unset: empty voxel (TODO: region id in low bits?); high bit set: voxel with solid geometry (VoxelIdMask if leaf, subvoxel index otherwise); order is (y,x,z)

        public List<Tile> Subdivision { get; init; } = new();

        public Level LevelDesc => Owner.Levels[Level];

        public Tile(VoxelMap owner, Vector3 boundsMin, Vector3 boundsMax, int level)
        {
            Owner     = owner;
            BoundsMin = boundsMin;
            BoundsMax = boundsMax;
            Level     = level;
            Contents  = new ushort[owner.Levels[level].NumCellsTotal];
        }

        public (int x, int y, int z) WorldToVoxel(Vector3 v)
        {
            var frac = (v - BoundsMin) * LevelDesc.InvCellSize;
            return ((int)frac.X, (int)frac.Y, (int)frac.Z);
        }

        public Vector3 VoxelToWorld(int x, int y, int z) => BoundsMin + new Vector3(x + 0.5f, y + 0.5f, z + 0.5f) * LevelDesc.CellSize;

        public Vector3 VoxelToWorld((int x, int y, int z) v) => VoxelToWorld(v.x, v.y, v.z);

        public (Vector3 min, Vector3 max) CalculateSubdivisionBounds(int x, int y, int z)
        {
            var ld  = LevelDesc;
            var min = BoundsMin + new Vector3(x, y, z) * ld.CellSize;
            return (min, min    + ld.CellSize);
        }

        public (Vector3 min, Vector3 max) CalculateSubdivisionBounds((int x, int y, int z) v) => CalculateSubdivisionBounds(v.x, v.y, v.z);

        public (ulong index, bool empty) FindLeafVoxel(Vector3 p, bool checkBounds = true)
        {
            var v = WorldToVoxel(p);
            if (checkBounds && !LevelDesc.InBounds(v))
                return (InvalidVoxel, false); // out of bounds; consider everything outside to be occupied

            var idx  = LevelDesc.VoxelToIndex(v);
            var data = Contents[idx];
            if ((data & VoxelOccupiedBit) == 0) return (EncodeIndex(idx), true); // empty at this level
            data &= VoxelIdMask;
            if (data == VoxelIdMask) return (EncodeIndex(idx), false); // occupied leaf

            var sub = Subdivision[data].FindLeafVoxel(p, false); // guaranteed to be in bounds
            return (EncodeIndex(idx, sub.index), sub.empty);
        }

        public IEnumerable<(ulong index, bool empty)> EnumerateLeafVoxels(Vector3 bmin, Vector3 bmax)
        {
            var ld   = LevelDesc;
            var vmin = ld.ClampMin(WorldToVoxel(bmin));
            var vmax = ld.ClampMax(WorldToVoxel(bmax));

            for (var z = vmin.z; z <= vmax.z; z++)
            for (var x = vmin.x; x <= vmax.x; ++x)
            for (var y = vmin.y; y <= vmax.y; ++y)
            {
                var idx  = ld.VoxelToIndex(x, y, z);
                var data = Contents[idx];

                if ((data & VoxelOccupiedBit) == 0)
                {
                    yield return (EncodeIndex(idx), true); // empty at this level
                    continue;
                }

                data &= VoxelIdMask;

                if (data == VoxelIdMask) yield return (EncodeIndex(idx), false); // occupied leaf
                else
                {
                    foreach (var sub in Subdivision[data].EnumerateLeafVoxels(bmin, bmax))
                        yield return (EncodeIndex(idx, sub.index), sub.empty);
                }
            }
        }
    }

    public sealed class RootColumnBuildResult
    {
        public required ushort[]   Contents    { get; init; }
        public required List<Tile> Subdivision { get; init; }
    }

    public Level[] Levels   { get; init; }
    public Tile    RootTile { get; init; }

    private readonly int[] _leafScaleX;
    private readonly int[] _leafScaleY;
    private readonly int[] _leafScaleZ;

    public const ushort VoxelOccupiedBit = 0x8000;
    public const ushort VoxelIdMask      = 0x7fff;

    // voxel index is stored as (... L2 L1 L0)
    public const ushort IndexLevelMask  = 0xffff;
    public const int    IndexLevelShift = 16;

    public const ulong InvalidVoxel = ulong.MaxValue;

    public static ulong EncodeIndex(ushort tileIndex, ulong subIndex = InvalidVoxel) => (subIndex << IndexLevelShift) + tileIndex;

    public static ulong EncodeSubIndex(ulong voxel, ushort tileIndex, int level)
    {
        var shift = IndexLevelShift * level;
        voxel &= ~((ulong)IndexLevelMask << shift);
        voxel |= (ulong)tileIndex << shift;
        return voxel;
    }

    public static ushort DecodeIndex(ref ulong index)
    {
        var tileIndex = (ushort)(index & IndexLevelMask);
        index >>= IndexLevelShift;
        return tileIndex;
    }

    public VoxelMap(Vector3 boundsMin, Vector3 boundsMax, int[] tilesPerLevel)
    {
        Levels = new Level[tilesPerLevel.Length];
        var levelExtent = boundsMax - boundsMin;

        for (var i = 0; i < Levels.Length; ++i)
        {
            Levels[i]   = new(levelExtent, tilesPerLevel[i]);
            levelExtent = Levels[i].CellSize;
        }

        RootTile        = new(this, boundsMin, boundsMax, 0);
        _leafScaleX     = GC.AllocateUninitializedArray<int>(Levels.Length);
        _leafScaleY     = GC.AllocateUninitializedArray<int>(Levels.Length);
        _leafScaleZ     = GC.AllocateUninitializedArray<int>(Levels.Length);
        _leafScaleX[^1] = 1;
        _leafScaleY[^1] = 1;
        _leafScaleZ[^1] = 1;

        for (var i = Levels.Length - 2; i >= 0; --i)
        {
            _leafScaleX[i] = _leafScaleX[i + 1] * Levels[i + 1].NumCellsX;
            _leafScaleY[i] = _leafScaleY[i + 1] * Levels[i + 1].NumCellsY;
            _leafScaleZ[i] = _leafScaleZ[i + 1] * Levels[i + 1].NumCellsZ;
        }
    }

    public bool IsEmpty(ulong voxel)
    {
        var tile = RootTile;

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == IndexLevelMask)
                return false; // asking for non-leaf => consider non-empty

            var data = tile.Contents[tileIndex];
            if ((data & VoxelOccupiedBit) == 0)
                return true; // found empty voxel
            data &= VoxelIdMask;
            if (data == VoxelIdMask)
                return false; // found non-empty leaf
            tile = tile.Subdivision[data];
        }
    }

    public (ulong voxel, bool empty) FindLeafVoxel(Vector3 p) => RootTile.FindLeafVoxel(p);

    public (Vector3 min, Vector3 max) VoxelBounds(ulong voxel, float eps)
    {
        var tile = RootTile;
        var eps3 = new Vector3(eps);

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == IndexLevelMask) return (tile.BoundsMin + eps3, tile.BoundsMax - eps3);

            var data = tile.Contents[tileIndex];
            var id   = data & VoxelIdMask;

            if ((data & VoxelOccupiedBit) == 0 || id == VoxelIdMask)
            {
                var bb = tile.CalculateSubdivisionBounds(Levels[tile.Level].IndexToVoxel(tileIndex));
                return (bb.min + eps3, bb.max - eps3);
            }

            tile = tile.Subdivision[id];
        }
    }

    internal Vector3 ClampPointToVoxel(ulong voxel, Vector3 p, float eps = 0.1f)
    {
        var tile = RootTile;

        while (true)
        {
            var tileIndex = DecodeIndex(ref voxel);
            if (tileIndex == IndexLevelMask)
                return Vector3.Clamp(p, tile.BoundsMin + new Vector3(eps), tile.BoundsMax - new Vector3(eps));

            var data = tile.Contents[tileIndex];
            var id   = data & VoxelIdMask;

            if ((data & VoxelOccupiedBit) == 0 || id == VoxelIdMask)
            {
                var (min, max) = tile.CalculateSubdivisionBounds(Levels[tile.Level].IndexToVoxel(tileIndex));
                var eps3 = new Vector3(eps);
                return Vector3.Clamp(p, min + eps3, max - eps3);
            }

            tile = tile.Subdivision[id];
        }
    }

    public void Build(Voxelizer vox, int tx, int tz)
    {
        var column = BuildRootColumn(vox, tx, tz);
        var ny     = Levels[0].NumCellsY;
        var idx    = Levels[0].VoxelToIndex(tx, 0, tz);
        Array.Copy(column.Contents, 0, RootTile.Contents, idx, ny);
        RootTile.Subdivision.AddRange(column.Subdivision);
    }

    public RootColumnBuildResult BuildRootColumn(Voxelizer vox, int tx, int tz)
    {
        var ny          = Levels[0].NumCellsY;
        var contents    = new ushort[ny];
        var subdivision = new List<Tile>();
        for (var ty = 0; ty < ny; ++ty)
            contents[ty] = BuildTileContent(vox, RootTile, subdivision, tx, ty, tz, 0, ty * _leafScaleY[0], 0);
        return new() { Contents = contents, Subdivision = subdivision };
    }

    private ushort BuildTileContent(Voxelizer vox, Tile parent, List<Tile> rootSubdivision, int rootX, int rootY, int rootZ, int leafX, int leafY, int leafZ)
    {
        var level = parent.Level;
        var (solid, empty) = vox.ClassifyRegion(leafX, leafY, leafZ, _leafScaleX[level], _leafScaleY[level], _leafScaleZ[level]);
        if (!solid) return 0;

        if (!empty) return VoxelOccupiedBit | VoxelIdMask;

        var index = parent.LevelDesc.VoxelToIndex(rootX, rootY, rootZ);
        var (min, max) = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel(index));
        if (parent.Level + 1 >= Levels.Length)
            throw new InvalidOperationException("体积列构建遇到超出层级的混合体素");
        var tile    = new Tile(this, min, max, parent.Level + 1);
        var localId = rootSubdivision.Count;
        rootSubdivision.Add(tile);

        ref var l           = ref Levels[tile.Level];
        var     childScaleX = _leafScaleX[tile.Level];
        var     childScaleY = _leafScaleY[tile.Level];
        var     childScaleZ = _leafScaleZ[tile.Level];
        ushort  i           = 0;
        for (var z = 0; z < l.NumCellsZ; ++z)
        for (var x = 0; x < l.NumCellsX; ++x)
        for (var y = 0; y < l.NumCellsY; ++y, ++i)
            tile.Contents[i] = BuildTileContent
            (
                vox,
                tile,
                tile.Subdivision,
                x,
                y,
                z,
                leafX + x * childScaleX,
                leafY + y * childScaleY,
                leafZ + z * childScaleZ
            );

        return (ushort)(VoxelOccupiedBit | localId);
    }
}
