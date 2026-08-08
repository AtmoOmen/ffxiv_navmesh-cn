using System.Numerics;
using System.Runtime.CompilerServices;

namespace vnavmesh.Common.Build.Flight;

public sealed class SparseVoxelOctree
{
    public const ulong INVALID_VOXEL = ulong.MaxValue;
    public const byte  NODE_EMPTY      = 0;
    public const byte  NODE_SOLID_LEAF = 1;
    public const byte  NODE_MIXED      = 2;

    private const int  DEPTH_SHIFT       = 27;
    private const int  DEPTH_MASK        = 0xF;
    private const int  MAX_DDA_ITERATIONS = 8192;

    private readonly SemaphoreSlim            materializationGate = new(1, 1);
    private          byte[]?                  deferredTreePayload;
    private          int                      deferredTreeOffset;
    private          int                      deferredTreeLength;
    private          Action<SparseVoxelOctree>? deferredTreeMaterializer;

    private OctreeNodePool pool;

    public Vector3  BoundsMin        { get; }
    public Vector3  BoundsMax        { get; }
    public float    LeafSize         { get; }
    public int      MaxDepth         { get; }
    public int[]    LayerDepths      { get; }
    public float[]  LayerCellSizes   { get; }
    public Vector3  LeafCellSize     => new(LeafSize);
    public int      RootNodeIndex    => 0;
    public int      NodeCount        => pool.NodeCount;

    public SparseVoxelOctree
    (
        Vector3 boundsMin,
        Vector3 boundsMax,
        float   leafSize,
        int     maxDepth,
        int[]   layerDepths
    )
    {
        if (leafSize <= 0 || !float.IsFinite(leafSize))
            throw new ArgumentException($"无效的八叉树叶尺寸: {leafSize}");
        if (maxDepth <= 0 || maxDepth > 16)
            throw new ArgumentException($"无效的八叉树最大深度: {maxDepth}");

        var worldExtent = boundsMax - boundsMin;
        var cells = (int)(worldExtent.X / leafSize);
        if (worldExtent.Y != worldExtent.X || worldExtent.Z != worldExtent.X || cells != (1 << maxDepth))
            throw new ArgumentException($"八叉树包围盒 {worldExtent} 与叶尺寸 {leafSize}、深度 {maxDepth} 不匹配");
        if (layerDepths is not { Length: > 0 })
            throw new ArgumentException("八叉树分层深度不能为空");

        for (var i = 0; i < layerDepths.Length; ++i)
        {
            if (layerDepths[i] < 1 || layerDepths[i] > maxDepth)
                throw new ArgumentException($"八叉树分层深度越界: {layerDepths[i]}");
            if (i > 0 && layerDepths[i] <= layerDepths[i - 1])
                throw new ArgumentException($"八叉树分层深度必须严格递增: {string.Join(',', layerDepths)}");
        }

        BoundsMin    = boundsMin;
        BoundsMax    = boundsMax;
        LeafSize     = leafSize;
        MaxDepth     = maxDepth;
        LayerDepths  = (int[])layerDepths.Clone();
        LayerCellSizes = new float[layerDepths.Length];

        for (var i = 0; i < layerDepths.Length; ++i)
            LayerCellSizes[i] = leafSize * (1 << (maxDepth - layerDepths[i]));

        pool = new OctreeNodePool();
        pool.EnsureRoot();
    }

    public static int DepthOf
    (
        ulong voxel
    ) => (int)((voxel >> DEPTH_SHIFT) & DEPTH_MASK);

    public static ulong EncodeCoord
    (
        int depth,
        int x,
        int y,
        int z
    )
    {
        ulong index = (ulong)depth << DEPTH_SHIFT;

        for (var d = 1; d <= depth; ++d)
        {
            var shift = depth - d;
            var child = (((z >> shift) & 1) << 2) |
                        (((y >> shift) & 1) << 1) |
                        ((x >> shift) & 1);
            index |= (ulong)child << (3 * (d - 1));
        }

        return index;
    }

    public static (int x, int y, int z) CoordAtDepth
    (
        ulong voxel,
        int   depth
    )
    {
        var x = 0;
        var y = 0;
        var z = 0;

        for (var d = 1; d <= depth; ++d)
        {
            var child = (int)((voxel >> (3 * (d - 1))) & 0x7);
            x = (x << 1) | (child & 1);
            y = (y << 1) | ((child >> 1) & 1);
            z = (z << 1) | ((child >> 2) & 1);
        }

        return (x, y, z);
    }

    public byte GetNodeState
    (
        int nodeIndex
    ) => pool.States[nodeIndex];

    public int GetChildNode
    (
        int nodeIndex,
        int childIndex
    ) => pool.GetChild(nodeIndex, childIndex);

    public (ulong voxel, bool empty) FindLeafVoxel
    (
        Vector3 p
    )
    {
        EnsureMaterialized();

        if (!TryGetLeafCoord(p, out var gx, out var gy, out var gz))
            return (INVALID_VOXEL, false);

        var located = LocateLeaf(gx, gy, gz);
        return (located.Voxel, located.Empty);
    }

    public bool TryFindLeafVoxelFast
    (
        Vector3   p,
        out ulong voxel,
        out bool  empty
    )
    {
        EnsureMaterialized();

        if (!TryGetLeafCoord(p, out var gx, out var gy, out var gz))
        {
            voxel = INVALID_VOXEL;
            empty = false;
            return false;
        }

        var located = LocateLeaf(gx, gy, gz);
        voxel = located.Voxel;
        empty = located.Empty;
        return true;
    }

    public (Vector3 min, Vector3 max) VoxelBounds
    (
        ulong voxel,
        float eps
    )
    {
        EnsureMaterialized();
        var depth   = DepthOf(voxel);
        var (x, y, z) = CoordAtDepth(voxel, depth);
        return BoundsOfCoord(depth, x, y, z, eps);
    }

    public bool TryGetLeafVoxelBounds
    (
        ulong                          voxel,
        out (Vector3 min, Vector3 max) bounds
    )
    {
        if (voxel == INVALID_VOXEL || DepthOf(voxel) > MaxDepth)
        {
            bounds = default;
            return false;
        }

        var depth   = DepthOf(voxel);
        var (x, y, z) = CoordAtDepth(voxel, depth);
        bounds = BoundsOfCoord(depth, x, y, z, 0);
        return true;
    }

    public Vector3 ClampPointToVoxel
    (
        ulong   voxel,
        Vector3 p,
        float   eps = 0.1f
    )
    {
        var (min, max) = VoxelBounds(voxel, eps);
        return Vector3.Clamp(p, min, max);
    }

    public bool IsEmpty
    (
        ulong voxel
    )
    {
        EnsureMaterialized();

        if (voxel == INVALID_VOXEL)
            return false;

        var depth = DepthOf(voxel);

        if (depth == 0)
            return pool.States[0] == NODE_EMPTY;
        if (pool.States[0] == NODE_EMPTY)
            return true;
        if (pool.States[0] == NODE_SOLID_LEAF)
            return false;

        var node = 0;

        for (var d = 1; d <= depth; ++d)
        {
            var child = (int)((voxel >> (3 * (d - 1))) & 0x7);
            var slot  = pool.GetChild(node, child);

            if (slot == 0)
                return true;

            node = slot - 1;

            if (pool.States[node] == NODE_SOLID_LEAF)
                return false;
        }

        return false;
    }

    public bool IsCellTraversable
    (
        int depth,
        int x,
        int y,
        int z
    )
    {
        EnsureMaterialized();

        if (depth == 0)
            return pool.States[0] != NODE_SOLID_LEAF;
        if (pool.States[0] == NODE_EMPTY)
            return true;
        if (pool.States[0] == NODE_SOLID_LEAF)
            return false;

        var node = 0;

        for (var d = 1; d <= depth; ++d)
        {
            var shift = depth - d;
            var child = (((z >> shift) & 1) << 2) |
                        (((y >> shift) & 1) << 1) |
                        ((x >> shift) & 1);
            var slot = pool.GetChild(node, child);

            if (slot == 0)
                return true;

            node = slot - 1;

            if (pool.States[node] == NODE_SOLID_LEAF)
                return false;
        }

        return true;
    }

    public IEnumerable<(ulong voxel, bool empty)> EnumerateLeafVoxels
    (
        Vector3 bmin,
        Vector3 bmax
    )
    {
        EnsureMaterialized();

        if (pool.States[0] == NODE_EMPTY)
        {
            yield return (0, true);
            yield break;
        }

        if (pool.States[0] == NODE_SOLID_LEAF)
        {
            if (Intersects(bmin, bmax, BoundsMin, BoundsMax))
                yield return (0, false);
            yield break;
        }

        foreach (var item in EnumerateNode(0, 0, 0, 0, 0, bmin, bmax))
            yield return item;
    }

    public bool TryLineOfSightDDA
    (
        Vector3  fromPos,
        Vector3  toPos,
        out bool visible
    )
    {
        EnsureMaterialized();
        visible = false;

        if (pool.States[0] == NODE_EMPTY)
        {
            visible = true;
            return true;
        }

        var cells   = 1 << MaxDepth;
        var rel     = fromPos - BoundsMin;
        var gx      = (int)(rel.X / LeafSize);
        var gy      = (int)(rel.Y / LeafSize);
        var gz      = (int)(rel.Z / LeafSize);
        var goalRel = toPos - BoundsMin;
        var goalGx  = (int)(goalRel.X / LeafSize);
        var goalGy  = (int)(goalRel.Y / LeafSize);
        var goalGz  = (int)(goalRel.Z / LeafSize);

        if ((uint)(gx     + 1) > (uint)cells ||
            (uint)(gy     + 1) > (uint)cells ||
            (uint)(gz     + 1) > (uint)cells ||
            (uint)(goalGx + 1) > (uint)cells ||
            (uint)(goalGy + 1) > (uint)cells ||
            (uint)(goalGz + 1) > (uint)cells)
        {
            visible = false;
            return false;
        }

        if (gx < 0) gx = 0;
        else if (gx >= cells) gx = cells - 1;
        if (gy < 0) gy = 0;
        else if (gy >= cells) gy = cells - 1;
        if (gz < 0) gz = 0;
        else if (gz >= cells) gz = cells - 1;

        if (goalGx < 0) goalGx = 0;
        else if (goalGx >= cells) goalGx = cells - 1;
        if (goalGy < 0) goalGy = 0;
        else if (goalGy >= cells) goalGy = cells - 1;
        if (goalGz < 0) goalGz = 0;
        else if (goalGz >= cells) goalGz = cells - 1;

        var ab    = toPos - fromPos;
        var invX  = ab.X != 0 ? 1f / ab.X : 0;
        var invY  = ab.Y != 0 ? 1f / ab.Y : 0;
        var invZ  = ab.Z != 0 ? 1f / ab.Z : 0;
        var stepX = Math.Sign(ab.X);
        var stepY = Math.Sign(ab.Y);
        var stepZ = Math.Sign(ab.Z);

        float tMaxX, tMaxY, tMaxZ;
        float tDeltaX, tDeltaY, tDeltaZ;

        if (stepX > 0)
        {
            tMaxX   = (((gx + 1) * LeafSize) - rel.X) * invX;
            tDeltaX = LeafSize * invX;
        }
        else if (stepX < 0)
        {
            tMaxX   = ((gx * LeafSize) - rel.X) * invX;
            tDeltaX = -LeafSize * invX;
        }
        else
        {
            tMaxX   = float.MaxValue;
            tDeltaX = 0;
        }

        if (stepY > 0)
        {
            tMaxY   = (((gy + 1) * LeafSize) - rel.Y) * invY;
            tDeltaY = LeafSize * invY;
        }
        else if (stepY < 0)
        {
            tMaxY   = ((gy * LeafSize) - rel.Y) * invY;
            tDeltaY = -LeafSize * invY;
        }
        else
        {
            tMaxY   = float.MaxValue;
            tDeltaY = 0;
        }

        if (stepZ > 0)
        {
            tMaxZ   = (((gz + 1) * LeafSize) - rel.Z) * invZ;
            tDeltaZ = LeafSize * invZ;
        }
        else if (stepZ < 0)
        {
            tMaxZ   = ((gz * LeafSize) - rel.Z) * invZ;
            tDeltaZ = -LeafSize * invZ;
        }
        else
        {
            tMaxZ   = float.MaxValue;
            tDeltaZ = 0;
        }

        var iterations = 0;

        while (gx != goalGx || gy != goalGy || gz != goalGz)
        {
            if (++iterations > MAX_DDA_ITERATIONS)
            {
                visible = false;
                return true;
            }

            var located = LocateLeaf(gx, gy, gz);

            if (!located.Empty)
            {
                visible = false;
                return true;
            }

            if (located.Depth > 0 && located.Depth < MaxDepth)
            {
                var shift = MaxDepth - located.Depth;

                if ((goalGx >> shift) == (gx >> shift) &&
                    (goalGy >> shift) == (gy >> shift) &&
                    (goalGz >> shift) == (gz >> shift))
                {
                    visible = true;
                    return true;
                }

                FastForward
                (
                    shift,
                    ref gx,
                    ref gy,
                    ref gz,
                    goalGx,
                    goalGy,
                    goalGz,
                    stepX,
                    stepY,
                    stepZ,
                    ref tMaxX,
                    ref tMaxY,
                    ref tMaxZ,
                    tDeltaX,
                    tDeltaY,
                    tDeltaZ
                );
                continue;
            }

            var nextBoundaryX = gx == goalGx ? float.MaxValue : tMaxX;
            var nextBoundaryY = gy == goalGy ? float.MaxValue : tMaxY;
            var nextBoundaryZ = gz == goalGz ? float.MaxValue : tMaxZ;

            if (nextBoundaryX < nextBoundaryY)
            {
                if (nextBoundaryX < nextBoundaryZ)
                {
                    gx += stepX;
                    tMaxX += tDeltaX;
                }
                else
                {
                    gz += stepZ;
                    tMaxZ += tDeltaZ;
                }
            }
            else if (nextBoundaryY < nextBoundaryZ)
            {
                gy += stepY;
                tMaxY += tDeltaY;
            }
            else
            {
                gz += stepZ;
                tMaxZ += tDeltaZ;
            }
        }

        var goalLocated = LocateLeaf(goalGx, goalGy, goalGz);

        if (!goalLocated.Empty)
        {
            visible = false;
            return true;
        }

        visible = true;
        return true;
    }

    public VolumeTileBuildResult BuildTileSubtrees
    (
        Voxelizer vox,
        int       tx,
        int       tz,
        int       numTilesX
    )
    {
        EnsureMaterialized();

        if (!BitOperations.IsPow2((uint)numTilesX) || numTilesX > (1 << MaxDepth))
            throw new ArgumentException($"无效的地面瓦片数量: {numTilesX}");

        var depth      = BitOperations.Log2((uint)numTilesX);
        var tileLeaves = (1 << MaxDepth) / numTilesX;

        if (vox.SizeX != tileLeaves || vox.SizeZ != tileLeaves || vox.SizeY != (1 << MaxDepth))
            throw new ArgumentException($"体素网格尺寸与瓦片不匹配: {vox.SizeX}x{vox.SizeY}x{vox.SizeZ}, 期望 {tileLeaves}x{1 << MaxDepth}x{tileLeaves}");

        var buildPool = new OctreeNodePool();
        var roots     = new List<(int YCube, int RootNode)>();
        var yCubes    = 1 << depth;

        for (var yCube = 0; yCube < yCubes; ++yCube)
        {
            var root = BuildCube(vox, buildPool, depth, 0, yCube * tileLeaves, 0, tileLeaves);

            if (root >= 0)
                roots.Add((yCube, root));
        }

        return new() { Pool = buildPool, Depth = depth, Roots = roots };
    }

    public void AttachTileSubtree
    (
        VolumeTileBuildResult result,
        int                   tx,
        int                   tz
    )
    {
        EnsureMaterialized();

        var depth     = result.Depth;
        var cellCount = 1 << depth;

        if ((uint)tx >= (uint)cellCount || (uint)tz >= (uint)cellCount)
            throw new ArgumentException($"瓦片坐标越界: {tx},{tz} / {cellCount}");

        foreach (var (yCube, rootNode) in result.Roots)
        {
            var parent = EnsureAncestor(depth, tx, yCube, tz);
            var copied = CopySubtree(result.Pool, rootNode);
            pool.SetChild(parent, ChildIndexAtDepth(depth, depth, tx, yCube, tz), copied + 1);
        }
    }

    public void WriteTree
    (
        BinaryWriter writer
    )
    {
        EnsureMaterialized();
        WriteNode(writer, 0);
    }

    public void ReadTree
    (
        BinaryReader reader
    )
    {
        pool = new OctreeNodePool();
        pool.EnsureRoot();

        var packed = reader.ReadUInt16();

        if (packed == 0)
            return;

        if (packed == ushort.MaxValue)
        {
            pool.States[0] = NODE_SOLID_LEAF;
            return;
        }

        pool.States[0] = NODE_MIXED;
        pool.AllocateChildrenForNode(0);
        ReadNodeChildren(reader, 0, packed);
    }

    internal bool HasDeferredTree => deferredTreePayload != null || deferredTreeMaterializer != null;

    internal void SetDeferredTreePayload
    (
        byte[] payload,
        int    offset,
        int    length
    )
    {
        deferredTreePayload      = payload;
        deferredTreeOffset       = offset;
        deferredTreeLength       = length;
        deferredTreeMaterializer = null;
    }

    internal void SetDeferredTreeMaterializer
    (
        Action<SparseVoxelOctree> materializer
    )
    {
        deferredTreePayload      = null;
        deferredTreeOffset       = 0;
        deferredTreeLength       = 0;
        deferredTreeMaterializer = materializer;
    }

    internal void EnsureMaterialized()
    {
        if (!HasDeferredTree)
            return;

        materializationGate.Wait();

        try
        {
            if (!HasDeferredTree)
                return;

            if (deferredTreePayload is { } payload)
            {
                Navmesh.MaterializeDeferredVolumeTree(this, payload, deferredTreeOffset, deferredTreeLength);
                deferredTreePayload = null;
                deferredTreeOffset  = 0;
                deferredTreeLength  = 0;
                return;
            }

            if (deferredTreeMaterializer is { } materializer)
            {
                materializer(this);
                deferredTreeMaterializer = null;
            }
        }
        finally
        {
            materializationGate.Release();
        }
    }

    public void ReleaseRetainedState()
    {
        deferredTreePayload      = null;
        deferredTreeOffset       = 0;
        deferredTreeLength       = 0;
        deferredTreeMaterializer = null;
        pool = new OctreeNodePool();
        pool.EnsureRoot();
    }

    public void CompactRetainedState()
    {
        if (HasDeferredTree)
            return;

        pool.Trim();
    }

    private (ulong Voxel, bool Empty, int Depth) LocateLeaf
    (
        int gx,
        int gy,
        int gz
    )
    {
        if (pool.States[0] == NODE_EMPTY)
            return (0, true, 0);
        if (pool.States[0] == NODE_SOLID_LEAF)
            return (0, false, 0);

        ulong voxel = 0;
        var   node  = 0;

        for (var d = 1; d <= MaxDepth; ++d)
        {
            var shift = MaxDepth - d;
            var child = (((gz >> shift) & 1) << 2) |
                        (((gy >> shift) & 1) << 1) |
                        ((gx >> shift) & 1);
            voxel |= (ulong)child << (3 * (d - 1));
            var slot = pool.GetChild(node, child);

            if (slot == 0)
                return (voxel | ((ulong)d << DEPTH_SHIFT), true, d);

            node = slot - 1;

            if (pool.States[node] == NODE_SOLID_LEAF)
                return (voxel | ((ulong)d << DEPTH_SHIFT), false, d);
        }

        return (voxel | ((ulong)MaxDepth << DEPTH_SHIFT), false, MaxDepth);
    }

    private bool TryGetLeafCoord
    (
        Vector3 p,
        out int gx,
        out int gy,
        out int gz
    )
    {
        var rel = p - BoundsMin;
        gx = (int)(rel.X / LeafSize);
        gy = (int)(rel.Y / LeafSize);
        gz = (int)(rel.Z / LeafSize);
        var cells = 1 << MaxDepth;
        return (uint)gx < (uint)cells && (uint)gy < (uint)cells && (uint)gz < (uint)cells;
    }

    private (Vector3 min, Vector3 max) BoundsOfCoord
    (
        int    depth,
        int    x,
        int    y,
        int    z,
        float  eps
    )
    {
        var size = LeafSize * (1 << (MaxDepth - depth));
        var min  = BoundsMin + new Vector3(x * size, y * size, z * size);
        var eps3 = new Vector3(eps);
        return (min + eps3, min + new Vector3(size) - eps3);
    }

    private static bool Intersects
    (
        Vector3 bmin,
        Vector3 bmax,
        Vector3 cmin,
        Vector3 cmax
    ) => bmin.X < cmax.X && bmax.X > cmin.X &&
         bmin.Y < cmax.Y && bmax.Y > cmin.Y &&
         bmin.Z < cmax.Z && bmax.Z > cmin.Z;

    private IEnumerable<(ulong voxel, bool empty)> EnumerateNode
    (
        int     node,
        int     depth,
        int     x,
        int     y,
        int     z,
        Vector3 bmin,
        Vector3 bmax
    )
    {
        var (min, max) = BoundsOfCoord(depth, x, y, z, 0);

        if (!Intersects(bmin, bmax, min, max))
            yield break;

        if (pool.States[node] == NODE_SOLID_LEAF)
        {
            yield return (EncodeCoord(depth, x, y, z), false);
            yield break;
        }

        for (var c = 0; c < 8; ++c)
        {
            var cx = (x << 1) | (c & 1);
            var cy = (y << 1) | ((c >> 1) & 1);
            var cz = (z << 1) | ((c >> 2) & 1);
            var slot = pool.GetChild(node, c);

            if (slot == 0)
            {
                var (cmin, cmax) = BoundsOfCoord(depth + 1, cx, cy, cz, 0);

                if (Intersects(bmin, bmax, cmin, cmax))
                    yield return (EncodeCoord(depth + 1, cx, cy, cz), true);
                continue;
            }

            var child = slot - 1;

            if (pool.States[child] == NODE_SOLID_LEAF)
            {
                var (cmin, cmax) = BoundsOfCoord(depth + 1, cx, cy, cz, 0);

                if (Intersects(bmin, bmax, cmin, cmax))
                    yield return (EncodeCoord(depth + 1, cx, cy, cz), false);
                continue;
            }

            foreach (var item in EnumerateNode(child, depth + 1, cx, cy, cz, bmin, bmax))
                yield return item;
        }
    }

    private static void FastForward
    (
        int     shift,
        ref int gx,
        ref int gy,
        ref int gz,
        int     goalGx,
        int     goalGy,
        int     goalGz,
        int     stepX,
        int     stepY,
        int     stepZ,
        ref float tMaxX,
        ref float tMaxY,
        ref float tMaxZ,
        float     tDeltaX,
        float     tDeltaY,
        float     tDeltaZ
    )
    {
        var tExit = float.MaxValue;

        if (stepX > 0)
        {
            var cellEnd = ((gx >> shift) + 1) << shift;
            var nCross  = cellEnd - gx;
            tExit = tMaxX + ((nCross - 1) * tDeltaX);
        }
        else if (stepX < 0)
        {
            var cellStart = (gx >> shift) << shift;
            var nCross    = gx - cellStart + 1;
            tExit = tMaxX + ((nCross - 1) * tDeltaX);
        }

        if (stepY > 0)
        {
            var cellEnd = ((gy >> shift) + 1) << shift;
            var nCross  = cellEnd - gy;
            var tExitY  = tMaxY + ((nCross - 1) * tDeltaY);

            if (tExitY < tExit)
                tExit = tExitY;
        }
        else if (stepY < 0)
        {
            var cellStart = (gy >> shift) << shift;
            var nCross    = gy - cellStart + 1;
            var tExitY    = tMaxY + ((nCross - 1) * tDeltaY);

            if (tExitY < tExit)
                tExit = tExitY;
        }

        if (stepZ > 0)
        {
            var cellEnd = ((gz >> shift) + 1) << shift;
            var nCross  = cellEnd - gz;
            var tExitZ  = tMaxZ + ((nCross - 1) * tDeltaZ);

            if (tExitZ < tExit)
                tExit = tExitZ;
        }
        else if (stepZ < 0)
        {
            var cellStart = (gz >> shift) << shift;
            var nCross    = gz - cellStart + 1;
            var tExitZ    = tMaxZ + ((nCross - 1) * tDeltaZ);

            if (tExitZ < tExit)
                tExit = tExitZ;
        }

        if (stepX != 0 && tExit >= tMaxX)
        {
            var remaining = stepX > 0 ? goalGx - gx : gx - goalGx;
            var n = Math.Min((int)(((tExit - tMaxX) / tDeltaX) + 1e-6f) + 1, remaining);

            if (n > 0)
            {
                gx += stepX * n;
                tMaxX += n * tDeltaX;
            }
        }

        if (stepY != 0 && tExit >= tMaxY)
        {
            var remaining = stepY > 0 ? goalGy - gy : gy - goalGy;
            var n = Math.Min((int)(((tExit - tMaxY) / tDeltaY) + 1e-6f) + 1, remaining);

            if (n > 0)
            {
                gy += stepY * n;
                tMaxY += n * tDeltaY;
            }
        }

        if (stepZ != 0 && tExit >= tMaxZ)
        {
            var remaining = stepZ > 0 ? goalGz - gz : gz - goalGz;
            var n = Math.Min((int)(((tExit - tMaxZ) / tDeltaZ) + 1e-6f) + 1, remaining);

            if (n > 0)
            {
                gz += stepZ * n;
                tMaxZ += n * tDeltaZ;
            }
        }

    }

    private static int BuildCube
    (
        Voxelizer      vox,
        OctreeNodePool buildPool,
        int            level,
        int            x0,
        int            y0,
        int            z0,
        int            extent
    )
    {
        var (anySolid, allSolid) = vox.ClassifyBox(x0, y0, z0, extent, extent, extent);

        if (!anySolid)
            return -1;
        if (allSolid || extent == 1)
            return buildPool.AllocSolidLeaf();

        var node = buildPool.AllocMixedNode();
        var half = extent >> 1;

        for (var cz = 0; cz < 2; ++cz)
        for (var cy = 0; cy < 2; ++cy)
        for (var cx = 0; cx < 2; ++cx)
        {
            var child = BuildCube(vox, buildPool, level + 1, x0 + (cx * half), y0 + (cy * half), z0 + (cz * half), half);

            if (child >= 0)
                buildPool.SetChild(node, (cz << 2) | (cy << 1) | cx, child + 1);
        }

        return node;
    }

    private int EnsureAncestor
    (
        int depth,
        int x,
        int y,
        int z
    )
    {
        if (pool.States[0] == NODE_EMPTY)
        {
            pool.States[0] = NODE_MIXED;
            pool.AllocateChildrenForNode(0);
        }
        else if (pool.States[0] != NODE_MIXED)
        {
            throw new InvalidOperationException("八叉树根为实心叶，无法挂载瓦片子树");
        }

        var node = 0;

        for (var d = 1; d < depth; ++d)
        {
            var child = ChildIndexAtDepth(depth, d, x, y, z);
            var slot  = pool.GetChild(node, child);

            if (slot == 0)
            {
                var created = pool.AllocMixedNode();
                pool.SetChild(node, child, created + 1);
                node = created;
            }
            else
            {
                node = slot - 1;

                if (pool.States[node] != NODE_MIXED)
                    throw new InvalidOperationException("八叉树祖先路径包含实心叶节点");
            }
        }

        return node;
    }

    private int CopySubtree
    (
        OctreeNodePool source,
        int            sourceNode
    )
    {
        int node;

        if (source.States[sourceNode] == NODE_SOLID_LEAF)
        {
            node = pool.AllocSolidLeaf();
        }
        else
        {
            node = pool.AllocMixedNode();

            for (var c = 0; c < 8; ++c)
            {
                var slot = source.GetChild(sourceNode, c);

                if (slot != 0)
                    pool.SetChild(node, c, CopySubtree(source, slot - 1) + 1);
            }
        }

        return node;
    }

    private void WriteNode
    (
        BinaryWriter writer,
        int          node
    )
    {
        if (pool.States[node] == NODE_EMPTY)
        {
            writer.Write((ushort)0);
            return;
        }

        if (pool.States[node] == NODE_SOLID_LEAF)
        {
            writer.Write(ushort.MaxValue);
            return;
        }

        ushort packed = 0;

        for (var c = 0; c < 8; ++c)
        {
            var slot  = pool.GetChild(node, c);
            ushort state = 0;

            if (slot != 0)
                state = pool.States[slot - 1] == NODE_SOLID_LEAF ?
                            (ushort)1 :
                            (ushort)2;
            packed |= (ushort)(state << (c * 2));
        }

        writer.Write(packed);

        for (var c = 0; c < 8; ++c)
        {
            var slot = pool.GetChild(node, c);

            if (slot != 0 && pool.States[slot - 1] == NODE_MIXED)
                WriteNode(writer, slot - 1);
        }
    }

    private void ReadNodeChildren
    (
        BinaryReader reader,
        int          node,
        ushort       packed
    )
    {
        for (var c = 0; c < 8; ++c)
        {
            var state = (packed >> (c * 2)) & 0x3;

            if (state == 3)
                throw new Exception($"体积八叉树节点状态非法: 0x{packed:X4}");
            if (state == 0)
                continue;
            if (state == 1)
            {
                pool.SetChild(node, c, pool.AllocSolidLeaf() + 1);
                continue;
            }

            var childPacked = reader.ReadUInt16();

            if (childPacked == 0)
                throw new Exception($"体积八叉树子树节点不能为空: node={node}, child={c}");

            if (childPacked == ushort.MaxValue)
            {
                pool.SetChild(node, c, pool.AllocSolidLeaf() + 1);
                continue;
            }

            var child = pool.AllocMixedNode();
            pool.SetChild(node, c, child + 1);
            ReadNodeChildren(reader, child, childPacked);
        }
    }

    private static int ChildIndexAtDepth
    (
        int depth,
        int level,
        int x,
        int y,
        int z
    )
    {
        var shift = depth - level;
        return (((z >> shift) & 1) << 2) |
               (((y >> shift) & 1) << 1) |
               ((x >> shift) & 1);
    }
}
