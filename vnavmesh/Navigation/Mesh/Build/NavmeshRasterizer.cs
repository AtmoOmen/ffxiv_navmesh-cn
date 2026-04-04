using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using DotRecast.Core;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Build;

// utility to rasterize various meshes into a heightfield
public class NavmeshRasterizer
{
    public sealed class ScratchBuffers
    {
        private Vector3[]        _worldVertices = [];
        private OutFlags[]       _outFlags      = [];
        private uint[]           _solidSort     = [];
        private int[]            _solidVoxel    = [];
        private IntersectionSet? _intersectionSet;

        internal Span<Vector3> WorldVertices(int count)
        {
            if (_worldVertices.Length < count)
                _worldVertices = GC.AllocateUninitializedArray<Vector3>(count);
            return _worldVertices.AsSpan(0, count);
        }

        internal Span<OutFlags> OutFlags(int count)
        {
            if (_outFlags.Length < count)
                _outFlags = GC.AllocateUninitializedArray<OutFlags>(count);
            return _outFlags.AsSpan(0, count);
        }

        internal void EnsureSolidBuffers(int count)
        {
            if (_solidSort.Length < count)
                _solidSort = GC.AllocateUninitializedArray<uint>(count);
            if (_solidVoxel.Length < count)
                _solidVoxel = GC.AllocateUninitializedArray<int>(count);
        }

        internal Span<uint> SolidSort(int count) => _solidSort.AsSpan(0, count);

        internal Span<int> SolidVoxel(int count) => _solidVoxel.AsSpan(0, count);

        internal IntersectionSet? AcquireIntersectionSet(bool enabled, int numCellsX, int numCellsZ)
        {
            if (!enabled)
                return null;

            if (_intersectionSet == null || _intersectionSet.NumCellsX != numCellsX || _intersectionSet.NumCellsZ != numCellsZ)
                _intersectionSet = new(numCellsX, numCellsZ);
            else
                _intersectionSet.Clear();
            return _intersectionSet;
        }
    }

    // cheap triangle-bbox test: if all 3 vertices are on the same side of the bbox plane, the triangle can be discarded
    [Flags]
    internal enum OutFlags : byte
    {
        None = 0,
        NegX = 1 << 0,
        PosX = 1 << 1,
        NegY = 1 << 2,
        PosY = 1 << 3,
        NegZ = 1 << 4,
        PosZ = 1 << 5
    }

    // a set of per-cell intersections with vertical ray
    // we build a sort key with high 31 bits as the remapped Y coordinate (0 = -1024, 0x7fffffff = 1024-eps) and low bit as normal sign (1 if points up)
    // we need the extra precision to disambiguate faces that map to a single voxel
    internal sealed class IntersectionSet
    {
        public const int PageShift  = 20;
        public const int PageSize   = 1 << PageShift;
        public const int ValueScale = 1 << 20; // 2048 = 2^11, we have 32 bits => 20 bits of scale; note: this is a bit extreme, mantissa is only 24 bits...

        public struct Entry
        {
            public int  Next; // index of next entry in the same cell in page storage
            public uint SortKey;

            public int
                VoxelY; // if normal points up - this is inclusive upper limit of area 'below' triangle (>0), otherwise it's negative inclusive lower limit of area 'above' triangle (<=0)

            public Entry(int next, int voxelY, float preciseY, bool normalUp)
            {
                Next    = next;
                SortKey = (uint)((preciseY + 1024) * ValueScale) << 1 | (normalUp ? 1u : 0);
                VoxelY  = normalUp ? voxelY : -voxelY;
            }
        }

        private readonly int[]         _firstIndices; // x then z
        private readonly List<Entry[]> _pages        = new();
        private          int           _firstFree    = 1;
        private readonly List<int>     _touchedCells = new();

        public int NumCellsX { get; }

        public int NumCellsZ { get; }

        public IReadOnlyList<int> TouchedCells => _touchedCells;

        public IntersectionSet(int numCellsX, int numCellsZ)
        {
            NumCellsX     = numCellsX;
            NumCellsZ     = numCellsZ;
            _firstIndices = new int[numCellsX * numCellsZ];
            _pages.Add(new Entry[PageSize]);
        }

        public void Add(int x, int y, int z, float value, bool normalUp)
        {
            if (value <= -1024 || value >= 1024)
                return;
            var pageIndex = _firstFree >> PageShift;
            if (pageIndex == _pages.Count)
                _pages.Add(new Entry[PageSize]);
            var     indexInPage = _firstFree & PageSize - 1;
            var     cellIndex   = z * NumCellsX + x;
            ref var first       = ref _firstIndices[cellIndex];
            if (first == 0)
                _touchedCells.Add(cellIndex);
            _pages[pageIndex][indexInPage] = new(first, y, value, normalUp);
            first                          = _firstFree++;
        }

        public int FetchSorted(int cellIndex, ScratchBuffers scratch)
        {
            var idx = _firstIndices[cellIndex];
            if (idx == 0)
                return 0;

            var cnt = 0;

            while (idx != 0)
            {
                ++cnt;
                idx = _pages[idx >> PageShift][idx & PageSize - 1].Next;
            }

            scratch.EnsureSolidBuffers(cnt);
            var bufferSort  = scratch.SolidSort(cnt);
            var bufferVoxel = scratch.SolidVoxel(cnt);
            idx = _firstIndices[cellIndex];
            var bufferIndex = 0;

            do
            {
                var entry = _pages[idx >> PageShift][idx & PageSize - 1];
                bufferSort[bufferIndex]  = entry.SortKey;
                bufferVoxel[bufferIndex] = entry.VoxelY;
                idx                      = entry.Next;
                ++bufferIndex;
            }
            while (idx != 0);

            bufferSort.Slice(0, cnt).Sort(bufferVoxel.Slice(0, cnt));
            return cnt;
        }

        public void Clear()
        {
            for (var i = 0; i < _touchedCells.Count; ++i)
                _firstIndices[_touchedCells[i]] = 0;
            _firstFree = 1;
            _touchedCells.Clear();
        }
    }

    private          RcHeightfield _heightfield;
    private          RcContext _telemetry;
    private          Voxelizer? _voxelizer;
    private          IntersectionSet? _iset;
    private          float _invCellXZ;
    private          float _invCellY;
    private          int _maxY;
    private          int _minSpanGap;
    private          int _walkableClimbThreshold; // if two spans have maximums within this number of voxels, their area is 'merged' (higher is selected)
    private          float _walkableNormalThreshold; // triangle is considered 'walkable' if it's world-space normal's Y coordinate is >= this
    private          int _voxShiftX;
    private          int _voxShiftY;
    private          int _voxShiftZ;
    private readonly ScratchBuffers _scratch;

    public NavmeshRasterizer
    (
        RcHeightfield   heightfield,
        float           walkableNormalThreshold,
        int             walkableMaxClimb,
        int             minGap,
        bool            fillInteriors,
        Voxelizer?      voxelizer,
        RcContext       telemetry,
        ScratchBuffers? scratch = null
    )
    {
        _heightfield             = heightfield;
        _telemetry               = telemetry;
        _voxelizer               = voxelizer;
        _scratch                 = scratch ?? new();
        _iset                    = _scratch.AcquireIntersectionSet(fillInteriors, heightfield.width, heightfield.height);
        _invCellXZ               = 1.0f / _heightfield.cs;
        _invCellY                = 1.0f / _heightfield.ch;
        _maxY                    = (int)((_heightfield.bmax.Y - _heightfield.bmin.Y) * _invCellY);
        _minSpanGap              = minGap;
        _walkableClimbThreshold  = walkableMaxClimb;
        _walkableNormalThreshold = walkableNormalThreshold;

        if (voxelizer != null)
        {
            var dx = (heightfield.width - 2 * heightfield.borderSize)  / voxelizer.NumX;
            var dy = _maxY                                             / voxelizer.NumY;
            var dz = (heightfield.height - 2 * heightfield.borderSize) / voxelizer.NumZ;
            if (!BitOperations.IsPow2(dx) || !BitOperations.IsPow2(dy) || !BitOperations.IsPow2(dz))
                throw new Exception($"Cell size mismatch: {dx}x{dy}x{dz}");
            _voxShiftX = BitOperations.Log2((uint)dx);
            _voxShiftY = BitOperations.Log2((uint)dy);
            _voxShiftZ = BitOperations.Log2((uint)dz);
        }
    }

    public void Rasterize(SceneExtractor geom, SceneExtractor.MeshType types, bool perMeshInteriors, bool solidBelowNonManifold)
    {
        foreach (var (name, mesh) in geom.Meshes)
        {
            if ((mesh.MeshType & types) == SceneExtractor.MeshType.None)
                continue;

            foreach (var instance in mesh.Instances)
            {
                if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
                {
                    var z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                    var z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                    var x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                    var x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                    FillInterior(z0, z1, x0, x1, solidBelowNonManifold ? minY : _maxY);
                }
            }
        }

        if (!perMeshInteriors) FillInterior(0, _heightfield.height - 1, 0, _heightfield.width - 1, solidBelowNonManifold ? 0 : _maxY);
    }

    public void Rasterize
    (
        ReadOnlySpan<(SceneExtractor.Mesh mesh, SceneExtractor.MeshInstance instance)> instances,
        SceneExtractor.MeshType                                                        types,
        bool                                                                           perMeshInteriors,
        bool                                                                           solidBelowNonManifold
    )
    {
        foreach (var (mesh, instance) in instances)
        {
            if ((mesh.MeshType & types) == SceneExtractor.MeshType.None)
                continue;

            if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
            {
                var z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                var x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                FillInterior(z0, z1, x0, x1, solidBelowNonManifold ? minY : _maxY);
            }
        }

        if (!perMeshInteriors) FillInterior(0, _heightfield.height - 1, 0, _heightfield.width - 1, solidBelowNonManifold ? 0 : _maxY);
    }

    public void Rasterize
    (
        IEnumerable<(SceneExtractor.Mesh mesh, SceneExtractor.MeshInstance instance)> instances,
        SceneExtractor.MeshType                                                       types,
        bool                                                                          perMeshInteriors,
        bool                                                                          solidBelowNonManifold
    )
    {
        foreach (var (mesh, instance) in instances)
        {
            if ((mesh.MeshType & types) == SceneExtractor.MeshType.None)
                continue;

            if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
            {
                var z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                var x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                FillInterior(z0, z1, x0, x1, solidBelowNonManifold ? minY : _maxY);
            }
        }

        if (!perMeshInteriors) FillInterior(0, _heightfield.height - 1, 0, _heightfield.width - 1, solidBelowNonManifold ? 0 : _maxY);
    }

    // if it returns true, the mesh borders were rasterized, so intersection set could be modified
    public bool RasterizeMesh(SceneExtractor.Mesh mesh, SceneExtractor.MeshInstance instance, out int minimalY)
    {
        minimalY = _maxY;
        if (!IntersectsHeightfield(instance.WorldBounds))
            return false;

        Span<Vector3>  stackWorldVertices = stackalloc Vector3[256];
        Span<OutFlags> stackOutFlags      = stackalloc OutFlags[256];
        var            parts              = mesh.PartSpan;
        var            terrainLike        = (mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0;

        for (var partIndex = 0; partIndex < parts.Length; ++partIndex)
        {
            var part = parts[partIndex];
            if (!IntersectsHeightfield(instance.WorldTransform, part.LocalBounds))
                continue;

            var vertexCount   = part.Vertices.Count;
            var worldVertices = vertexCount <= 256 ? stackWorldVertices : _scratch.WorldVertices(vertexCount);
            var outFlags      = vertexCount <= 256 ? stackOutFlags : _scratch.OutFlags(vertexCount);

            // fill vertex buffer
            TransformVertices(instance, part.VertexSpan, worldVertices, outFlags);

            if (terrainLike)
                RasterizeTerrainLikePart(part.PrimitiveSpan, instance, worldVertices, outFlags, ref minimalY);
            else
                RasterizeGeneralPart(part.PrimitiveSpan, instance, worldVertices, outFlags, ref minimalY);
        }

        return true;
    }

    private void RasterizeTerrainLikePart
    (
        ReadOnlySpan<SceneExtractor.Primitive> primitives,
        SceneExtractor.MeshInstance            instance,
        Span<Vector3>                          worldVertices,
        Span<OutFlags>                         outFlags,
        ref int                                minimalY
    )
        => RasterizePrimitiveSpan(primitives, instance, worldVertices, outFlags, ref minimalY);

    private void RasterizeGeneralPart
    (
        ReadOnlySpan<SceneExtractor.Primitive> primitives,
        SceneExtractor.MeshInstance            instance,
        Span<Vector3>                          worldVertices,
        Span<OutFlags>                         outFlags,
        ref int                                minimalY
    )
        => RasterizePrimitiveSpan(primitives, instance, worldVertices, outFlags, ref minimalY);

    private void RasterizePrimitiveSpan
    (
        ReadOnlySpan<SceneExtractor.Primitive> primitives,
        SceneExtractor.MeshInstance            instance,
        Span<Vector3>                          worldVertices,
        Span<OutFlags>                         outFlags,
        ref int                                minimalY
    )
    {
        Span<Vector3> clipRemainingZ = stackalloc Vector3[7];
        Span<Vector3> clipRemainingX = stackalloc Vector3[7];
        Span<Vector3> clipScratch    = stackalloc Vector3[7];
        Span<Vector3> clipCell       = stackalloc Vector3[7];
        Span<float>   clipAxisDelta  = stackalloc float[7];
        ref var       primitiveRef   = ref MemoryMarshal.GetReference(primitives);

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            ref readonly var p = ref Unsafe.Add(ref primitiveRef, primitiveIndex);
            if ((outFlags[p.V1] & outFlags[p.V2] & outFlags[p.V3]) != OutFlags.None)
                continue; // vertex is fully outside bounds, on one side of some plane

            var v1         = worldVertices[p.V1];
            var v2         = worldVertices[p.V2];
            var v3         = worldVertices[p.V3];
            var v12        = v2 - v1;
            var v13        = v3 - v1;
            var v12cross13 = Vector3.Cross(v12, v13);
            var lenSq      = v12cross13.LengthSquared();
            if (lenSq == 0)
                continue;
            var crossY   = v12cross13.Y;
            var invDiv   = _iset != null && crossY != 0 ? -1.0f / crossY : 0; // see below
            var normalUp = crossY > 0;

            var flags           = p.Flags & ~instance.ForceClearPrimFlags | instance.ForceSetPrimFlags;
            var realSolid       = !flags.HasFlag(SceneExtractor.PrimitiveFlags.FlyThrough);
            var unwalkableSlope = crossY <= 0 || crossY * crossY < _walkableNormalThreshold * _walkableNormalThreshold * lenSq;
            var unwalkable = flags.HasFlag
                                 (SceneExtractor.PrimitiveFlags.ForceUnwalkable) ||
                             unwalkableSlope                                     ||
                             _voxelizer != null                                      &&
                             flags.HasFlag(SceneExtractor.PrimitiveFlags.Unlandable) &&
                             !flags.HasFlag(SceneExtractor.PrimitiveFlags.ForceWalkable);
            var areaId = unwalkable ? 0 : RcConstants.RC_WALKABLE_AREA;

            var numRemainingZ = 0;
            clipRemainingZ[numRemainingZ++] = v1;
            clipRemainingZ[numRemainingZ++] = v2;
            clipRemainingZ[numRemainingZ++] = v3;

            var (minZ, maxZ) = MinMaxZ(clipRemainingZ, numRemainingZ);
            var z0 = (int)((minZ - _heightfield.bmin.Z) * _invCellXZ);
            var z1 = (int)((maxZ - _heightfield.bmin.Z) * _invCellXZ);
            z0 = Math.Clamp(z0, -1, _heightfield.height - 1);
            z1 = Math.Clamp(z1, 0,  _heightfield.height - 1);

            for (var z = z0; z <= z1; ++z)
            {
                if (numRemainingZ < 3)
                    break;

                var cellZMax      = _heightfield.bmin.Z + (z + 1) * _heightfield.cs;
                var numRemainingX = SplitConvexPolyZ(clipRemainingZ, clipRemainingX, clipScratch, clipAxisDelta, ref numRemainingZ, cellZMax);

                var swapZ = clipRemainingZ;
                clipRemainingZ = clipScratch;
                clipScratch    = swapZ;

                if (numRemainingX < 3 || z < 0)
                    continue;

                var (minX, maxX) = MinMaxX(clipRemainingX, numRemainingX);
                var x0 = (int)((minX - _heightfield.bmin.X) * _invCellXZ);
                var x1 = (int)((maxX - _heightfield.bmin.X) * _invCellXZ);
                if (x1 < 0 || x0 >= _heightfield.width)
                    continue;
                x0 = Math.Clamp(x0, -1, _heightfield.width - 1);
                x1 = Math.Clamp(x1, 0,  _heightfield.width - 1);

                var cellZMid = _heightfield.bmin.Z + (z + 0.5f) * _heightfield.cs;

                for (var x = x0; x <= x1; ++x)
                {
                    if (numRemainingX < 3)
                        break;

                    var cellXMax = _heightfield.bmin.X + (x + 1) * _heightfield.cs;
                    var numCell  = SplitConvexPolyX(clipRemainingX, clipCell, clipScratch, clipAxisDelta, ref numRemainingX, cellXMax);

                    var swapX = clipRemainingX;
                    clipRemainingX = clipScratch;
                    clipScratch    = swapX;

                    if (numCell < 3 || x < 0)
                        continue;

                    var (minY, maxY) = MinMaxY(clipCell, numCell);
                    var y0 = (int)MathF.Floor((minY   - _heightfield.bmin.Y) * _invCellY);
                    var y1 = (int)MathF.Ceiling((maxY - _heightfield.bmin.Y) * _invCellY);
                    if (y1 < 0 || y0 >= _maxY)
                        continue;
                    y0 = Math.Clamp(y0, 0,  _maxY - 1);
                    y1 = Math.Clamp(y1, y0, _maxY - 1);

                    AddSpan(x, z, y0, y1, areaId, realSolid);

                    if (realSolid && _iset != null && invDiv != 0)
                    {
                        minimalY = Math.Min(minimalY, y0);
                        var cellXMid = _heightfield.bmin.X + (x + 0.5f) * _heightfield.cs;
                        var apx      = cellXMid            - v1.X;
                        var apz      = cellZMid            - v1.Z;
                        var c        = (apz * v12.X - apx * v12.Z) * invDiv;
                        var b        = (apx * v13.Z - apz * v13.X) * invDiv;

                        if (c >= 0 && b >= 0 && c + b <= 1)
                        {
                            var intersectY = v1.Y + b * v12.Y + c * v13.Y;
                            if (normalUp && y0 > 0)
                                _iset.Add(x, y0 - 1, z, intersectY, true);
                            else if (!normalUp && y1 < _maxY - 1)
                                _iset.Add(x, y1 + 1, z, intersectY, false);
                        }
                    }
                }
            }
        }
    }

    private void AddSpan(int x, int z, int y0, int y1, int areaId, bool includeInVolume, bool mergeBelow = true)
    {
        var     yOrig    = (y0, y1);
        ref var cellHead = ref _heightfield.spans[z * _heightfield.width + x];

        // find insert position for new span: skip any existing spans that end before new span start
        var  prevMaxY      = mergeBelow ? y0 - _minSpanGap - 1 : y1; // any spans that have smax >= prevMaxY are merged
        var  nextMinY      = y1 + _minSpanGap + 1;                   // any spans that have smin <= nextMinY are merged
        uint prevSpanIndex = 0;
        var  currSpanIndex = cellHead;

        while (currSpanIndex != 0)
        {
            ref var currSpan = ref _heightfield.Span(currSpanIndex);

            if (currSpan.smin > nextMinY)
            {
                // new span should be inserted before current one
                break;
            }

            if (currSpan.smax < prevMaxY)
            {
                // new span is fully above current one - continue...
                prevSpanIndex = currSpanIndex;
                currSpanIndex = currSpan.next;
                continue;
            }

            // new span overlaps current one - merge and remove old one
            // the trickiest part is how to merge area ids
            // idea is: if one of the spans is significantly 'above', take area from it; if they are of similar height, take higher area value (assuming it's more permissive)
            var heightDiff = currSpan.smax - y1;
            if (heightDiff > _walkableClimbThreshold || heightDiff >= -_walkableClimbThreshold && currSpan.area > areaId)
                areaId = currSpan.area;
            y0 = mergeBelow ? Math.Min(y0, currSpan.smin) : Math.Max(y0, currSpan.smax);
            y1 = Math.Max(y1, currSpan.smax);

            // free merged span; note that prev would still point to it, we'll fix it later
            var nextSpanIndex = currSpan.next;
            _heightfield.spanPool.Free(currSpanIndex);
            currSpanIndex = nextSpanIndex;
        }

        // insert new span
        var newSpanIndex = _heightfield.spanPool.Alloc();
        _heightfield.Span(newSpanIndex) = new() { smin = y0, smax = y1, area = areaId, next = currSpanIndex };
        if (prevSpanIndex == 0)
            cellHead = newSpanIndex;
        else
            _heightfield.Span(prevSpanIndex).next = newSpanIndex;

        // mark overlapping voxels as solid; use unmodified y coords since it's possible for a realSolid span to be merged with a fly-through span
        // TODO: figure out if we can preserve flythrough-ability using areaId - not sure if nonzero area id always indicates a walkable surface, recast docs are very unclear on this front
        if (includeInVolume && _voxelizer != null)
        {
            x -= _heightfield.borderSize;
            z -= _heightfield.borderSize;

            if (x >= 0 && z >= 0)
            {
                x >>= _voxShiftX;
                z >>= _voxShiftZ;

                if (x < _voxelizer.NumX && z < _voxelizer.NumZ)
                {
                    // block pixels beneath the span for a distance roughly equal to agent height, otherwise volume pathfind will try to move the player through doorframes etc
                    _voxelizer.AddSpan(x, z, yOrig.y0 - _minSpanGap >> _voxShiftY, yOrig.y1 >> _voxShiftY);
                }
            }
        }
    }

    // TODO: maintain non-empty cells in intersection set?
    private void FillInterior(int z0, int z1, int x0, int x1, int yBelowNonManifold)
    {
        if (_iset == null)
            return; // interior filling is disabled

        // fill interiors
        var cells = _iset.TouchedCells;

        for (var celli = 0; celli < cells.Count; ++celli)
        {
            var cell = cells[celli];
            var z    = cell / _iset.NumCellsX;
            var x    = cell - z * _iset.NumCellsX;
            if (z < z0 || z > z1 || x < x0 || x > x1)
                continue;

            var cnt = _iset.FetchSorted(cell, _scratch);
            if (cnt == 0)
                continue; // empty

            var solidVoxel = _scratch.SolidVoxel(cnt);

            var idx = 0;

            if (solidVoxel[idx] > yBelowNonManifold)
            {
                // non-manifold mesh, assume everything below is interior
                while (idx + 1 < cnt && solidVoxel[idx + 1] > yBelowNonManifold)
                    ++idx; // well i dunno, some terrain (eg south thanalan) is really _that_ fucked
                AddSpan(x, z, yBelowNonManifold, solidVoxel[idx], 0, true, false);
                ++idx;
            }

            while (true)
            {
                while (idx < cnt && solidVoxel[idx] > 0)
                    ++idx;
                if (idx == cnt)
                    break;
                var minY = -solidVoxel[idx];
                while (idx < cnt && solidVoxel[idx] <= 0)
                    ++idx;
                if (idx == cnt)
                    break;
                var maxY = solidVoxel[idx];
                if (maxY >= minY)
                    AddSpan(x, z, minY, maxY, 0, true);
            }
        }

        _iset.Clear();
    }

    // TODO: remove after i'm confident in my replacement code
    public void RasterizeOld(SceneExtractor geom, SceneExtractor.MeshType types)
    {
        var vertices = new float[3 * 256];

        foreach (var (name, mesh) in geom.Meshes)
        {
            if ((mesh.MeshType & types) == SceneExtractor.MeshType.None)
                continue;

            foreach (var inst in mesh.Instances)
            {
                if (inst.WorldBounds.Max.X <= _heightfield.bmin.X ||
                    inst.WorldBounds.Max.Z <= _heightfield.bmin.Z ||
                    inst.WorldBounds.Min.X >= _heightfield.bmax.X ||
                    inst.WorldBounds.Min.Z >= _heightfield.bmax.Z)
                    continue;

                foreach (var part in mesh.Parts)
                {
                    // fill vertex buffer
                    var iv = 0;

                    foreach (var v in part.Vertices)
                    {
                        var w = inst.WorldTransform.TransformCoordinate(v);
                        vertices[iv++] = w.X;
                        vertices[iv++] = w.Y;
                        vertices[iv++] = w.Z;
                    }

                    // TODO: move area-id calculations to extraction step + store indices in a form that allows using RasterizeTriangles()
                    foreach (var p in part.Primitives)
                    {
                        var flags = p.Flags & ~inst.ForceClearPrimFlags | inst.ForceSetPrimFlags;
                        if (_voxelizer != null && flags.HasFlag(SceneExtractor.PrimitiveFlags.FlyThrough))
                            continue; // TODO: rasterize to normal heightfield, can't do it right now, since we're using same heightfield for both mesh and volume

                        var unwalkable = flags.HasFlag(SceneExtractor.PrimitiveFlags.ForceUnwalkable);
                        unwalkable |= _voxelizer != null &&
                                      flags.HasFlag(SceneExtractor.PrimitiveFlags.Unlandable); // for flyable scenes, assume unlandable == unwalkable

                        if (!unwalkable)
                        {
                            var v1     = CachedVertex(vertices, p.V1);
                            var v2     = CachedVertex(vertices, p.V2);
                            var v3     = CachedVertex(vertices, p.V3);
                            var v12    = v2 - v1;
                            var v13    = v3 - v1;
                            var normal = Vector3.Normalize(Vector3.Cross(v12, v13));
                            unwalkable = normal.Y < _walkableNormalThreshold;
                        }

                        var areaId = unwalkable ? 0 : RcConstants.RC_WALKABLE_AREA;
                        RcRasterizations.RasterizeTriangle(_telemetry, vertices, p.V1, p.V2, p.V3, areaId, _heightfield, _walkableClimbThreshold);
                    }
                }
            }
        }
    }

    private static Vector3 CachedVertex(ReadOnlySpan<float> vertices, int i)
    {
        var offset = 3 * i;
        return new(vertices[offset], vertices[offset + 1], vertices[offset + 2]);
    }

    private bool IntersectsHeightfield(AABB bounds)
        => bounds.Max.X > _heightfield.bmin.X && bounds.Max.Z > _heightfield.bmin.Z && bounds.Min.X < _heightfield.bmax.X && bounds.Min.Z < _heightfield.bmax.Z;

    private bool IntersectsHeightfield(Matrix4x3 worldTransform, AABB localBounds)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = axisX      * localCenter.X + axisY      * localCenter.Y + axisZ      * localCenter.Z + worldTransform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        var bounds      = new AABB { Min = center - extent, Max = center + extent };
        return IntersectsHeightfield(bounds);
    }

    private static Vector3 Abs(Vector3 value) => new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));

    private void TransformVertices(SceneExtractor.MeshInstance instance, ReadOnlySpan<Vector3> localVertices, Span<Vector3> outWorld, Span<OutFlags> outFlags)
    {
        var wt    = instance.WorldTransform;
        var axisX = wt.Row0;
        var axisY = wt.Row1;
        var axisZ = wt.Row2;
        var trans = wt.Row3;

        var bminX = _heightfield.bmin.X;
        var bminY = _heightfield.bmin.Y;
        var bminZ = _heightfield.bmin.Z;
        var bmaxX = _heightfield.bmax.X;
        var bmaxY = _heightfield.bmax.Y;
        var bmaxZ = _heightfield.bmax.Z;

        ref var srcRef = ref MemoryMarshal.GetReference(localVertices);

        for (var i = 0; i < localVertices.Length; ++i)
        {
            var v = Unsafe.Add(ref srcRef, i);
            var w = axisX * v.X + axisY * v.Y + axisZ * v.Z + trans;

            OutFlags f          = 0;
            if (w.X <= bminX) f |= OutFlags.NegX;
            if (w.X >= bmaxX) f |= OutFlags.PosX;
            if (w.Y <= bminY) f |= OutFlags.NegY;
            if (w.Y >= bmaxY) f |= OutFlags.PosY;
            if (w.Z <= bminZ) f |= OutFlags.NegZ;
            if (w.Z >= bmaxZ) f |= OutFlags.PosZ;

            outWorld[i] = w;
            outFlags[i] = f;
        }
    }

    private static (float min, float max) MinMaxX(Span<Vector3> vertices, int count)
    {
        var min = vertices[0].X;
        var max = min;

        for (var i = 1; i < count; ++i)
        {
            var v = vertices[i].X;
            min = Math.Min(min, v);
            max = Math.Max(max, v);
        }

        return (min, max);
    }

    private static (float min, float max) MinMaxY(Span<Vector3> vertices, int count)
    {
        var min = vertices[0].Y;
        var max = min;

        for (var i = 1; i < count; ++i)
        {
            var v = vertices[i].Y;
            min = Math.Min(min, v);
            max = Math.Max(max, v);
        }

        return (min, max);
    }

    private static (float min, float max) MinMaxZ(Span<Vector3> vertices, int count)
    {
        var min = vertices[0].Z;
        var max = min;

        for (var i = 1; i < count; ++i)
        {
            var v = vertices[i].Z;
            min = Math.Min(min, v);
            max = Math.Max(max, v);
        }

        return (min, max);
    }

    private static int SplitConvexPolyZ(Span<Vector3> src, Span<Vector3> dest, Span<Vector3> remaining, Span<float> axisDelta, ref int count, float axisOffset)
    {
        for (var i = 0; i < count; ++i)
            axisDelta[i] = axisOffset - src[i].Z;

        int cDest = 0, cRem = 0;
        var dPrev = axisDelta[count - 1];
        var vPrev = src[count       - 1];

        for (var i = 0; i < count; ++i)
        {
            var dCurr = axisDelta[i];
            var vCurr = src[i];

            if (dCurr >= 0 != dPrev >= 0)
            {
                var s                             = dPrev / (dPrev - dCurr);
                dest[cDest++] = remaining[cRem++] = vPrev + (vCurr - vPrev) * s;

                if (dCurr > 0)
                    dest[cDest++] = vCurr;
                else if (dCurr < 0)
                    remaining[cRem++] = vCurr;
            }
            else
            {
                if (dCurr      > 0) dest[cDest++]     = vCurr;
                else if (dCurr < 0) remaining[cRem++] = vCurr;
                else
                {
                    dest[cDest++]     = vCurr;
                    remaining[cRem++] = vCurr;
                }
            }

            dPrev = dCurr;
            vPrev = vCurr;
        }

        count = cRem;
        return cDest;
    }

    private static int SplitConvexPolyX(Span<Vector3> src, Span<Vector3> dest, Span<Vector3> remaining, Span<float> axisDelta, ref int count, float axisOffset)
    {
        for (var i = 0; i < count; ++i)
            axisDelta[i] = axisOffset - src[i].X;

        int cDest = 0, cRem = 0;
        var dPrev = axisDelta[count - 1];
        var vPrev = src[count       - 1];

        for (var i = 0; i < count; ++i)
        {
            var dCurr = axisDelta[i];
            var vCurr = src[i];

            if (dCurr >= 0 != dPrev >= 0)
            {
                var s                             = dPrev / (dPrev - dCurr);
                dest[cDest++] = remaining[cRem++] = vPrev + (vCurr - vPrev) * s;

                if (dCurr > 0)
                    dest[cDest++] = vCurr;
                else if (dCurr < 0)
                    remaining[cRem++] = vCurr;
            }
            else
            {
                if (dCurr      > 0) dest[cDest++]     = vCurr;
                else if (dCurr < 0) remaining[cRem++] = vCurr;
                else
                {
                    dest[cDest++]     = vCurr;
                    remaining[cRem++] = vCurr;
                }
            }

            dPrev = dCurr;
            vPrev = vCurr;
        }

        count = cRem;
        return cDest;
    }
}
