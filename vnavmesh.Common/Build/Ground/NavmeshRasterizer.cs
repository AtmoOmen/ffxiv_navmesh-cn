using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using DotRecast.Core;
using DotRecast.Recast;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Common.Build.Models;
using vnavmesh.Common.Models;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;
using SceneExtractor = vnavmesh.Common.Build.BuildScene;

namespace vnavmesh.Common.Build.Ground;

using static RcRecast;

internal sealed class PreparedTerrainGeometry
{
    public readonly record struct PrimitiveInfo
    (
        float V1X,
        float V1Z,
        float V12X,
        float V12Z,
        float V13X,
        float V13Z,
        float MinX,
        float MaxX,
        float MinZ,
        float MaxZ,
        float PlaneGradX,
        float PlaneGradZ,
        float PlaneBias,
        float InverseCrossY,
        int   AreaId,
        byte  Flags
    )
    {
        private const byte FlagRealSolid = 1 << 0;
        private const byte FlagNormalUp  = 1 << 1;
        private const byte FlagProjected = 1 << 2;
        private const byte FlagValid     = 1 << 3;

        public bool RealSolid => (Flags & FlagRealSolid) != 0;
        public bool NormalUp  => (Flags & FlagNormalUp)  != 0;
        public bool Projected => (Flags & FlagProjected) != 0;
        public bool Valid     => (Flags & FlagValid)     != 0;

        public static byte BuildFlags
        (
            bool realSolid,
            bool normalUp,
            bool projected,
            bool valid
        )
        {
            byte flags = 0;
            if (realSolid)
                flags |= FlagRealSolid;
            if (normalUp)
                flags |= FlagNormalUp;
            if (projected)
                flags |= FlagProjected;
            if (valid)
                flags |= FlagValid;
            return flags;
        }
    }

    public required float[]          WorldVertexTriples      { get; init; }
    public          PrimitiveInfo[]? PrimitiveInfos          { get; set; }
    public          int              TileMinX                { get; set; }
    public          int              TileMinZ                { get; set; }
    public          int              TileCountX              { get; set; }
    public          int              TileCountZ              { get; set; }
    public          int[]?           TilePrimitiveOffsets    { get; set; }
    public          int[]?           TilePrimitiveIndices    { get; set; }
    public          bool             HasTilePrimitiveBuckets => TilePrimitiveOffsets != null && TilePrimitiveIndices != null;

    public long MemoryBytes
        => ((long)WorldVertexTriples.Length           * sizeof(float))                  +
           ((long)(PrimitiveInfos?.Length       ?? 0) * Unsafe.SizeOf<PrimitiveInfo>()) +
           ((long)(TilePrimitiveOffsets?.Length ?? 0) * sizeof(int))                    +
           ((long)(TilePrimitiveIndices?.Length ?? 0) * sizeof(int));

    public ReadOnlySpan<int> GetTilePrimitiveIndices
    (
        int tileX,
        int tileZ
    )
    {
        if (!HasTilePrimitiveBuckets)
            return [];

        var localX = tileX - TileMinX;
        var localZ = tileZ - TileMinZ;
        if ((uint)localX >= (uint)TileCountX || (uint)localZ >= (uint)TileCountZ)
            return [];

        var cellIndex = (localZ * TileCountX) + localX;
        var start     = TilePrimitiveOffsets![cellIndex];
        return TilePrimitiveIndices!.AsSpan(start, TilePrimitiveOffsets[cellIndex + 1] - start);
    }
}

internal sealed class RasterJob
{
    public required MeshType      MeshType        { get; init; }
    public required MeshPart      Part            { get; init; }
    public required MeshInstance  Instance        { get; init; }
    public required AABB                     WorldBounds     { get; init; }
    public required int                      MinTileX        { get; init; }
    public required int                      MaxTileX        { get; init; }
    public required int                      MinTileZ        { get; init; }
    public required int                      MaxTileZ        { get; init; }
    public required int                      PrimitiveCount  { get; init; }
    public required int                      VertexCount     { get; init; }
    public required int                      EstimatedWeight { get; init; }
    public required bool                     TerrainLike     { get; init; }
    public          PreparedTerrainGeometry? PreparedTerrain { get; init; }
}

// utility to rasterize various meshes into a heightfield
public class NavmeshRasterizer
{
    public readonly record struct PartInstance
    (
        MeshType     MeshType,
        MeshPart     Part,
        MeshInstance Instance
    );

    public sealed class ScratchBuffers
    {
        internal sealed class TerrainSpanBuffer
        {
            private struct Entry
            {
                public int Next;
                public int Y0;
                public int Y1;
                public int AreaId;
            }

            private          int[]     _cellHeads = [];
            private          Entry[]   _entries   = GC.AllocateUninitializedArray<Entry>(1024);
            private          int       _nextFree  = 1;
            private          int       _freeList;
            private readonly List<int> _touchedCells = new();

            internal void Reset
            (
                int cellCount
            )
            {
                if (_cellHeads.Length != cellCount)
                    _cellHeads = new int[cellCount];
                else
                {
                    for (var i = 0; i < _touchedCells.Count; ++i)
                        _cellHeads[_touchedCells[i]] = 0;
                }

                _nextFree = 1;
                _freeList = 0;
                _touchedCells.Clear();
            }

            internal void Add
            (
                int cellIndex,
                int y0,
                int y1,
                int areaId,
                int walkableClimbThreshold
            )
            {
                ref var head = ref _cellHeads[cellIndex];
                if (head == 0)
                    _touchedCells.Add(cellIndex);

                var prevMaxY = y0;
                var nextMinY = y1;
                var prev     = 0;
                var curr     = head;

                while (curr != 0)
                {
                    ref var current = ref _entries[curr];
                    if (current.Y0 > nextMinY)
                        break;

                    if (current.Y1 < prevMaxY)
                    {
                        prev = curr;
                        curr = current.Next;
                        continue;
                    }

                    var heightDiff = current.Y1 - y1;
                    if (heightDiff > walkableClimbThreshold || (heightDiff >= -walkableClimbThreshold && current.AreaId > areaId))
                        areaId = current.AreaId;
                    y0 = Math.Min(y0, current.Y0);
                    y1 = Math.Max(y1, current.Y1);

                    var next = current.Next;
                    Free(curr);
                    if (prev == 0)
                        head = next;
                    else
                        _entries[prev].Next = next;
                    curr = next;
                }

                var     entryIndex = Alloc();
                ref var entry      = ref _entries[entryIndex];
                entry.Y0     = y0;
                entry.Y1     = y1;
                entry.AreaId = areaId;
                entry.Next   = curr;
                if (prev == 0)
                    head = entryIndex;
                else
                    _entries[prev].Next = entryIndex;
            }

            internal IReadOnlyList<int> TouchedCells => _touchedCells;

            internal int Head
            (
                int cellIndex
            ) => _cellHeads[cellIndex];

            internal (int y0, int y1, int areaId, int next) Read
            (
                int entryIndex
            )
            {
                ref var entry = ref _entries[entryIndex];
                return (entry.Y0, entry.Y1, entry.AreaId, entry.Next);
            }

            private int Alloc()
            {
                if (_freeList != 0)
                {
                    var index = _freeList;
                    _freeList = _entries[index].Next;
                    return index;
                }

                if (_nextFree == _entries.Length)
                    Array.Resize(ref _entries, _entries.Length * 2);

                return _nextFree++;
            }

            private void Free
            (
                int entryIndex
            )
            {
                _entries[entryIndex].Next = _freeList;
                _freeList                 = entryIndex;
            }
        }

        private Vector3[]          _worldVertices      = [];
        private float[]            _worldVertexTriples = [];
        private OutFlags[]         _outFlags           = [];
        private uint[]             _solidSort          = [];
        private int[]              _solidVoxel         = [];
        private IntersectionSet?   _intersectionSet;
        private TerrainSpanBuffer? _terrainSpanBuffer;

        internal Span<Vector3> WorldVertices
        (
            int count
        )
        {
            if (_worldVertices.Length < count)
                _worldVertices = GC.AllocateUninitializedArray<Vector3>(count);
            return _worldVertices.AsSpan(0, count);
        }

        internal Span<float> WorldVertexTriples
        (
            int vertexCount
        )
        {
            var valueCount = vertexCount * 3;
            if (_worldVertexTriples.Length < valueCount)
                _worldVertexTriples = GC.AllocateUninitializedArray<float>(valueCount);
            return _worldVertexTriples.AsSpan(0, valueCount);
        }

        internal Span<OutFlags> OutFlags
        (
            int count
        )
        {
            if (_outFlags.Length < count)
                _outFlags = GC.AllocateUninitializedArray<OutFlags>(count);
            return _outFlags.AsSpan(0, count);
        }

        internal void EnsureSolidBuffers
        (
            int count
        )
        {
            if (_solidSort.Length < count)
                _solidSort = GC.AllocateUninitializedArray<uint>(count);
            if (_solidVoxel.Length < count)
                _solidVoxel = GC.AllocateUninitializedArray<int>(count);
        }

        internal Span<uint> SolidSort
        (
            int count
        ) => _solidSort.AsSpan(0, count);

        internal Span<int> SolidVoxel
        (
            int count
        ) => _solidVoxel.AsSpan(0, count);

        internal IntersectionSet? AcquireIntersectionSet
        (
            bool enabled,
            int  numCellsX,
            int  numCellsZ
        )
        {
            if (!enabled)
                return null;

            if (_intersectionSet == null || _intersectionSet.NumCellsX != numCellsX || _intersectionSet.NumCellsZ != numCellsZ)
                _intersectionSet = new(numCellsX, numCellsZ);
            else
                _intersectionSet.Clear();
            return _intersectionSet;
        }

        internal TerrainSpanBuffer AcquireTerrainSpanBuffer
        (
            int cellCount
        )
        {
            _terrainSpanBuffer ??= new();
            _terrainSpanBuffer.Reset(cellCount);
            return _terrainSpanBuffer;
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

            public Entry
            (
                int   next,
                int   voxelY,
                float preciseY,
                bool  normalUp
            )
            {
                Next = next;
                SortKey = ((uint)((preciseY + 1024) * ValueScale) << 1) |
                          (normalUp ?
                               1u :
                               0);
                VoxelY = normalUp ?
                             voxelY :
                             -voxelY;
            }
        }

        private readonly int[]         _firstIndices; // x then z
        private readonly List<Entry[]> _pages        = new();
        private          int           _firstFree    = 1;
        private readonly List<int>     _touchedCells = new();

        public int NumCellsX { get; }

        public int NumCellsZ { get; }

        public IReadOnlyList<int> TouchedCells => _touchedCells;

        public IntersectionSet
        (
            int numCellsX,
            int numCellsZ
        )
        {
            NumCellsX     = numCellsX;
            NumCellsZ     = numCellsZ;
            _firstIndices = new int[numCellsX * numCellsZ];
            _pages.Add(new Entry[PageSize]);
        }

        public void Add
        (
            int   x,
            int   y,
            int   z,
            float value,
            bool  normalUp
        )
        {
            if (value <= -1024 || value >= 1024)
                return;
            var pageIndex = _firstFree >> PageShift;
            if (pageIndex == _pages.Count)
                _pages.Add(new Entry[PageSize]);
            var     indexInPage = _firstFree & (PageSize - 1);
            var     cellIndex   = (z * NumCellsX) + x;
            ref var first       = ref _firstIndices[cellIndex];
            if (first == 0)
                _touchedCells.Add(cellIndex);
            _pages[pageIndex][indexInPage] = new(first, y, value, normalUp);
            first                          = _firstFree++;
        }

        public int FetchSorted
        (
            int            cellIndex,
            ScratchBuffers scratch
        )
        {
            var idx = _firstIndices[cellIndex];
            if (idx == 0)
                return 0;

            var cnt = 0;

            while (idx != 0)
            {
                ++cnt;
                idx = _pages[idx >> PageShift][idx & (PageSize - 1)].Next;
            }

            scratch.EnsureSolidBuffers(cnt);
            var bufferSort  = scratch.SolidSort(cnt);
            var bufferVoxel = scratch.SolidVoxel(cnt);
            idx = _firstIndices[cellIndex];
            var bufferIndex = 0;

            do
            {
                var entry = _pages[idx >> PageShift][idx & (PageSize - 1)];
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
    private          ScratchBuffers.TerrainSpanBuffer? _terrainSpans;
    private          float _invCellXZ;
    private          float _invCellY;
    private          int _maxY;
    private          int _minSpanGap;
    private          int _walkableClimbThreshold; // if two spans have maximums within this number of voxels, their area is 'merged' (higher is selected)
    private          float _walkableNormalThreshold; // triangle is considered 'walkable' if it's world-space normal's Y coordinate is >= this
    private          int _voxSourceX;
    private          int _voxSourceY;
    private          int _voxSourceZ;
    private          int _voxShiftX = -1;
    private          int _voxShiftY = -1;
    private          int _voxShiftZ = -1;
    private readonly int _tileX;
    private readonly int _tileZ;
    private readonly ScratchBuffers _scratch;
    private readonly float volumeWallThickenNormalYThreshold;
    private readonly int volumeWallThickenHorizontalRadius;
    private readonly float volumeThinWallStripNormalYThreshold;
    private readonly float volumeThinWallStripMaxProjectedThickness;
    private readonly float volumeThinWallStripBaseRadius;
    private readonly float volumeThinWallStripExtraPadding;

    public NavmeshRasterizer
    (
        RcHeightfield   heightfield,
        float           walkableNormalThreshold,
        int             walkableMaxClimb,
        int             minGap,
        bool            fillInteriors,
        Voxelizer?      voxelizer,
        NavmeshSettings settings,
        RcContext       telemetry,
        ScratchBuffers? scratch = null,
        int             tileX   = -1,
        int             tileZ   = -1
    )
    {
        _heightfield = heightfield;
        _telemetry   = telemetry;
        _voxelizer   = voxelizer;
        _scratch     = scratch ?? new();
        _iset        = _scratch.AcquireIntersectionSet(fillInteriors, heightfield.width, heightfield.height);
        _terrainSpans = tileX >= 0 && tileZ >= 0 ?
                            _scratch.AcquireTerrainSpanBuffer(heightfield.width * heightfield.height) :
                            null;
        _invCellXZ               = 1.0f / _heightfield.cs;
        _invCellY                = 1.0f / _heightfield.ch;
        _maxY                    = (int)((_heightfield.bmax.Y - _heightfield.bmin.Y) * _invCellY);
        _minSpanGap              = minGap;
        _walkableClimbThreshold  = walkableMaxClimb;
        _walkableNormalThreshold = walkableNormalThreshold;
        _tileX                   = tileX;
        _tileZ                   = tileZ;
        volumeWallThickenNormalYThreshold        = settings.VolumeWallThickenNormalYThreshold;
        volumeWallThickenHorizontalRadius        = settings.VolumeWallThickenHorizontalRadius;
        volumeThinWallStripNormalYThreshold      = settings.VolumeThinWallStripNormalYThreshold;
        volumeThinWallStripMaxProjectedThickness = settings.VolumeThinWallStripMaxProjectedThickness;
        volumeThinWallStripBaseRadius            = settings.VolumeThinWallStripBaseRadius;
        volumeThinWallStripExtraPadding          = settings.VolumeThinWallStripExtraPadding;

        if (voxelizer != null)
        {
            _voxSourceX = heightfield.width - (2 * heightfield.borderSize);
            _voxSourceY = _maxY;
            _voxSourceZ = heightfield.height - (2 * heightfield.borderSize);

            if (_voxSourceX <= 0 || _voxSourceY <= 0 || _voxSourceZ <= 0)
                throw new Exception($"Invalid volume source size: {_voxSourceX}x{_voxSourceY}x{_voxSourceZ}");

            _voxShiftX = TryGetExactPow2RatioShift(_voxSourceX, voxelizer.SizeX);
            _voxShiftY = TryGetExactPow2RatioShift(_voxSourceY, voxelizer.SizeY);
            _voxShiftZ = TryGetExactPow2RatioShift(_voxSourceZ, voxelizer.SizeZ);
        }
    }

    internal static PreparedTerrainGeometry PrepareTerrainGeometry
    (
        MeshPart     part,
        MeshInstance instance
    )
    {
        var worldVertexTriples = GC.AllocateUninitializedArray<float>(part.Vertices.Count * 3);
        TransformVerticesPacked(instance, part.VertexSpan, worldVertexTriples);
        return new() { WorldVertexTriples = worldVertexTriples };
    }

    public void Rasterize
    (
        SceneExtractor      geom,
        MeshType types,
        bool                perMeshInteriors,
        bool                solidBelowNonManifold
    )
    {
        foreach (var (name, mesh) in geom.Meshes)
        {
            if ((mesh.MeshType & types) == MeshType.None)
                continue;

            foreach (var instance in mesh.Instances)
            {
                if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
                {
                    var z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                    var z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                    var x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                    var x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                    FillInterior
                    (
                        z0,
                        z1,
                        x0,
                        x1,
                        solidBelowNonManifold ?
                            minY :
                            _maxY
                    );
                }
            }
        }

        if (!perMeshInteriors)
        {
            FillInterior
            (
                0,
                _heightfield.height - 1,
                0,
                _heightfield.width - 1,
                solidBelowNonManifold ?
                    0 :
                    _maxY
            );
        }
    }

    public void Rasterize
    (
        ReadOnlySpan<(Mesh mesh, MeshInstance instance)> instances,
        MeshType                                                    types,
        bool                                                                   perMeshInteriors,
        bool                                                                   solidBelowNonManifold
    )
    {
        foreach (var (mesh, instance) in instances)
        {
            if ((mesh.MeshType & types) == MeshType.None)
                continue;

            if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
            {
                var z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                var x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                FillInterior
                (
                    z0,
                    z1,
                    x0,
                    x1,
                    solidBelowNonManifold ?
                        minY :
                        _maxY
                );
            }
        }

        if (!perMeshInteriors)
        {
            FillInterior
            (
                0,
                _heightfield.height - 1,
                0,
                _heightfield.width - 1,
                solidBelowNonManifold ?
                    0 :
                    _maxY
            );
        }
    }

    public void Rasterize
    (
        IEnumerable<(Mesh mesh, MeshInstance instance)> instances,
        MeshType                                                   types,
        bool                                                                  perMeshInteriors,
        bool                                                                  solidBelowNonManifold
    )
    {
        foreach (var (mesh, instance) in instances)
        {
            if ((mesh.MeshType & types) == MeshType.None)
                continue;

            if (RasterizeMesh(mesh, instance, out var minY) && perMeshInteriors)
            {
                var z0 = Math.Clamp((int)((instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var z1 = Math.Clamp((int)((instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var x0 = Math.Clamp((int)((instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                var x1 = Math.Clamp((int)((instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                FillInterior
                (
                    z0,
                    z1,
                    x0,
                    x1,
                    solidBelowNonManifold ?
                        minY :
                        _maxY
                );
            }
        }

        if (!perMeshInteriors)
        {
            FillInterior
            (
                0,
                _heightfield.height - 1,
                0,
                _heightfield.width - 1,
                solidBelowNonManifold ?
                    0 :
                    _maxY
            );
        }
    }

    public void Rasterize
    (
        ReadOnlySpan<PartInstance> parts,
        MeshType        types,
        bool                       perMeshInteriors,
        bool                       solidBelowNonManifold
    )
    {
        foreach (var partInstance in parts)
        {
            if ((partInstance.MeshType & types) == MeshType.None)
                continue;

            if (RasterizePart(partInstance.MeshType, partInstance.Part, partInstance.Instance, out var minY) && perMeshInteriors)
            {
                var z0 = Math.Clamp((int)((partInstance.Instance.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var z1 = Math.Clamp((int)((partInstance.Instance.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var x0 = Math.Clamp((int)((partInstance.Instance.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                var x1 = Math.Clamp((int)((partInstance.Instance.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                FillInterior
                (
                    z0,
                    z1,
                    x0,
                    x1,
                    solidBelowNonManifold ?
                        minY :
                        _maxY
                );
            }
        }

        if (!perMeshInteriors)
        {
            FillInterior
            (
                0,
                _heightfield.height - 1,
                0,
                _heightfield.width - 1,
                solidBelowNonManifold ?
                    0 :
                    _maxY
            );
        }
    }

    internal void Rasterize
    (
        ReadOnlySpan<RasterJob> jobs,
        bool                    perMeshInteriors,
        bool                    solidBelowNonManifold,
        Action<RasterJob>?      onJobFinished = null
    )
    {
        foreach (ref readonly var job in jobs)
        {
            if (RasterizeJob(job, out var minY) && perMeshInteriors)
            {
                var z0 = Math.Clamp((int)((job.WorldBounds.Min.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var z1 = Math.Clamp((int)((job.WorldBounds.Max.Z - _heightfield.bmin.Z) * _invCellXZ), 0, _heightfield.height - 1);
                var x0 = Math.Clamp((int)((job.WorldBounds.Min.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                var x1 = Math.Clamp((int)((job.WorldBounds.Max.X - _heightfield.bmin.X) * _invCellXZ), 0, _heightfield.width  - 1);
                FillInterior
                (
                    z0,
                    z1,
                    x0,
                    x1,
                    solidBelowNonManifold ?
                        minY :
                        _maxY
                );
            }

            onJobFinished?.Invoke(job);
        }

        FlushTerrainSpans();

        if (!perMeshInteriors)
        {
            FillInterior
            (
                0,
                _heightfield.height - 1,
                0,
                _heightfield.width - 1,
                solidBelowNonManifold ?
                    0 :
                    _maxY
            );
        }
    }

    internal bool RasterizeJob
    (
        RasterJob job,
        out int   minimalY
    )
    {
        minimalY = _maxY;
        if (!IntersectsHeightfield(job.WorldBounds))
            return false;

        if (job.TerrainLike && job.PreparedTerrain != null)
            return RasterizePreparedTerrainPart(job, job.PreparedTerrain, out minimalY);

        return RasterizePart(job.MeshType, job.Part, job.Instance, out minimalY);
    }

    // if it returns true, the mesh borders were rasterized, so intersection set could be modified
    public bool RasterizeMesh
    (
        Mesh         mesh,
        MeshInstance instance,
        out int                 minimalY
    )
    {
        minimalY = _maxY;
        if (!IntersectsHeightfield(instance.WorldBounds))
            return false;

        var parts = mesh.PartSpan;

        for (var partIndex = 0; partIndex < parts.Length; ++partIndex)
            if (RasterizePart(mesh.MeshType, parts[partIndex], instance, out var partMinY))
                minimalY = Math.Min(minimalY, partMinY);

        return true;
    }

    private bool RasterizePreparedTerrainPart
    (
        RasterJob               job,
        PreparedTerrainGeometry prepared,
        out int                 minimalY
    )
    {
        minimalY = _maxY;

        if (prepared.HasTilePrimitiveBuckets && prepared.PrimitiveInfos != null)
        {
            var primitiveIndices = prepared.GetTilePrimitiveIndices(_tileX, _tileZ);
            if (primitiveIndices.IsEmpty)
                return false;

            RasterizePreparedTerrainLikePart(job.Part.PrimitiveSpan, primitiveIndices, prepared.WorldVertexTriples, prepared.PrimitiveInfos, ref minimalY);
            return true;
        }

        var            vertexCount   = job.VertexCount;
        Span<OutFlags> stackOutFlags = stackalloc OutFlags[256];
        var outFlags = vertexCount <= 256 ?
                           stackOutFlags[..vertexCount] :
                           _scratch.OutFlags(vertexCount);
        ComputeOutFlags(prepared.WorldVertexTriples, outFlags);
        RasterizeTerrainLikePart(job.Part.PrimitiveSpan, default, job.Instance, prepared.WorldVertexTriples, outFlags, ref minimalY);
        return true;
    }

    public bool RasterizePart
    (
        MeshType     meshType,
        MeshPart     part,
        MeshInstance instance,
        out int                 minimalY
    )
    {
        minimalY = _maxY;
        if (!IntersectsHeightfield(instance.WorldTransform, part.LocalBounds))
            return false;

        Span<Vector3>  stackWorldVertices = stackalloc Vector3[256];
        Span<float>    stackWorldTriples  = stackalloc float[256 * 3];
        Span<OutFlags> stackOutFlags      = stackalloc OutFlags[256];
        var            vertexCount        = part.Vertices.Count;
        var outFlags = vertexCount <= 256 ?
                           stackOutFlags[..vertexCount] :
                           _scratch.OutFlags(vertexCount);
        var terrainLike = (meshType & (MeshType.Terrain | MeshType.AnalyticPlane)) != 0;

        if (terrainLike)
        {
            var worldTriples = vertexCount <= 256 ?
                                   stackWorldTriples[..(vertexCount * 3)] :
                                   _scratch.WorldVertexTriples(vertexCount);
            TransformVerticesPacked(instance, part.VertexSpan, worldTriples, outFlags);
            RasterizeTerrainLikePart(part.PrimitiveSpan, default, instance, worldTriples, outFlags, ref minimalY);
        }
        else
        {
            var worldVertices = vertexCount <= 256 ?
                                    stackWorldVertices[..vertexCount] :
                                    _scratch.WorldVertices(vertexCount);
            TransformVertices(instance, part.VertexSpan, worldVertices, outFlags);
            RasterizeGeneralPart(part.PrimitiveSpan, instance, worldVertices, outFlags, ref minimalY);
        }

        return true;
    }

    private void RasterizeTerrainLikePart
    (
        ReadOnlySpan<Primitive> primitives,
        ReadOnlySpan<int>                  primitiveIndices,
        MeshInstance            instance,
        ReadOnlySpan<float>                worldVertices,
        Span<OutFlags>                     outFlags,
        ref int                            minimalY
    )
    {
        Span<float> clipBuffer   = stackalloc float[7 * 3 * 4];
        var         cellZSize    = _heightfield.cs;
        var         cellXSize    = _heightfield.cs;
        var         inverseCellY = _invCellY;
        var         heightfieldY = _heightfield.bmin.Y;
        var         heightfieldX = _heightfield.bmin.X;
        var         heightfieldZ = _heightfield.bmin.Z;
        ref var     primitiveRef = ref MemoryMarshal.GetReference(primitives);
        var         useBuckets   = !primitiveIndices.IsEmpty;
        var primitiveCount = useBuckets ?
                                 primitiveIndices.Length :
                                 primitives.Length;

        for (var primitiveIndex = 0; primitiveIndex < primitiveCount; ++primitiveIndex)
        {
            var primitiveSourceIndex = useBuckets ?
                                           primitiveIndices[primitiveIndex] :
                                           primitiveIndex;
            ref readonly var p = ref Unsafe.Add(ref primitiveRef, primitiveSourceIndex);
            if (!useBuckets && (outFlags[p.V1] & outFlags[p.V2] & outFlags[p.V3]) != OutFlags.None)
                continue;

            var offset1 = p.V1 * 3;
            var offset2 = p.V2 * 3;
            var offset3 = p.V3 * 3;
            var v1x     = worldVertices[offset1];
            var v1y     = worldVertices[offset1 + 1];
            var v1z     = worldVertices[offset1 + 2];
            var v2x     = worldVertices[offset2];
            var v2y     = worldVertices[offset2 + 1];
            var v2z     = worldVertices[offset2 + 2];
            var v3x     = worldVertices[offset3];
            var v3y     = worldVertices[offset3 + 1];
            var v3z     = worldVertices[offset3 + 2];
            var v12x    = v2x               - v1x;
            var v12y    = v2y               - v1y;
            var v12z    = v2z               - v1z;
            var v13x    = v3x               - v1x;
            var v13y    = v3y               - v1y;
            var v13z    = v3z               - v1z;
            var crossX  = (v12y   * v13z)   - (v12z   * v13y);
            var crossY  = (v12z   * v13x)   - (v12x   * v13z);
            var crossZ  = (v12x   * v13y)   - (v12y   * v13x);
            var lenSq   = (crossX * crossX) + (crossY * crossY) + (crossZ * crossZ);
            if (lenSq == 0)
                continue;

            var flags           = (p.Flags & ~instance.ForceClearPrimFlags) | instance.ForceSetPrimFlags;
            var realSolid       = !flags.HasFlag(PrimitiveFlags.FlyThrough);
            var forceWalkable   = flags.HasFlag(PrimitiveFlags.ForceWalkable);
            var unwalkableSlope = crossY <= 0 || crossY * crossY < _walkableNormalThreshold * _walkableNormalThreshold * lenSq;
            var unwalkable = flags.HasFlag(PrimitiveFlags.ForceUnwalkable) ||
                             (!forceWalkable && unwalkableSlope);
            var areaId = unwalkable ?
                             0 :
                             RC_WALKABLE_AREA;
            var inverseCrossY = _iset != null && crossY != 0 ?
                                    -1.0f / crossY :
                                    0;
            var normalUp = crossY > 0;

            if (_voxelizer != null && realSolid)
            {
                RasterizeVolumeThinWallStrip
                (
                    new(v1x, v1y, v1z),
                    new(v2x, v2y, v2z),
                    new(v3x, v3y, v3z),
                    crossX,
                    crossY,
                    crossZ
                );
            }

            var minZ         = Math.Min(v1z, Math.Min(v2z, v3z));
            var maxZ         = Math.Max(v1z, Math.Max(v2z, v3z));
            var z0           = Math.Clamp((int)((minZ - heightfieldZ) * _invCellXZ), -1, _heightfield.height - 1);
            var z1           = Math.Clamp((int)((maxZ - heightfieldZ) * _invCellXZ), 0,  _heightfield.height - 1);
            var sourceOffset = 0;
            var rowOffset    = 7 * 3;
            var tmpOffset    = rowOffset + (7 * 3);
            var remainOffset = tmpOffset + (7 * 3);
            var numInput     = 3;

            CopyVertexTriplet(clipBuffer, sourceOffset,     worldVertices, offset1);
            CopyVertexTriplet(clipBuffer, sourceOffset + 3, worldVertices, offset2);
            CopyVertexTriplet(clipBuffer, sourceOffset + 6, worldVertices, offset3);

            for (var z = z0; z <= z1; ++z)
            {
                DividePoly(clipBuffer, sourceOffset, numInput, rowOffset, out var numRow, tmpOffset, out numInput, heightfieldZ + ((z + 1) * cellZSize), 2);
                (sourceOffset, tmpOffset) = (tmpOffset, sourceOffset);

                if (numRow < 3 || z < 0)
                    continue;

                FindAxisBounds(clipBuffer, rowOffset, numRow, 0, out var minX, out var maxX);
                var x0 = (int)((minX - heightfieldX) * _invCellXZ);
                var x1 = (int)((maxX - heightfieldX) * _invCellXZ);
                if (x1 < 0 || x0 >= _heightfield.width)
                    continue;

                x0 = Math.Clamp(x0, -1, _heightfield.width - 1);
                x1 = Math.Clamp(x1, 0,  _heightfield.width - 1);

                var numRemaining = numRow;
                var cellZMid     = heightfieldZ + ((z + 0.5f) * cellZSize);

                for (var x = x0; x <= x1; ++x)
                {
                    DividePoly
                        (clipBuffer, rowOffset, numRemaining, tmpOffset, out var numCell, remainOffset, out numRemaining, heightfieldX + ((x + 1) * cellXSize), 0);
                    (rowOffset, remainOffset) = (remainOffset, rowOffset);

                    if (numCell < 3 || x < 0)
                        continue;

                    FindAxisBounds(clipBuffer, tmpOffset, numCell, 1, out var minY, out var maxY);
                    var y0 = (int)MathF.Floor((minY   - heightfieldY) * inverseCellY);
                    var y1 = (int)MathF.Ceiling((maxY - heightfieldY) * inverseCellY);
                    if (y1 < 0 || y0 >= _maxY)
                        continue;

                    y0 = Math.Clamp(y0, 0,  _maxY - 1);
                    y1 = Math.Clamp(y1, y0, _maxY - 1);
                    AddSpan(x, z, y0, y1, areaId, realSolid);
                    if (_voxelizer != null && realSolid)
                        WriteVolumeWallThickness(x, z, y0, y1, crossX, crossY, crossZ);

                    if (realSolid && _iset != null && inverseCrossY != 0)
                    {
                        minimalY = Math.Min(minimalY, y0);
                        var cellXMid = heightfieldX + ((x + 0.5f) * cellXSize);
                        var apx      = cellXMid     - v1x;
                        var apz      = cellZMid     - v1z;
                        var c        = ((apz * v12x) - (apx * v12z)) * inverseCrossY;
                        var b        = ((apx * v13z) - (apz * v13x)) * inverseCrossY;

                        if (c >= 0 && b >= 0 && c + b <= 1)
                        {
                            var intersectY = ComputeTriangleIntersectY(v1y, v12y, v13y, b, c);
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

    private void RasterizePreparedTerrainLikePart
    (
        ReadOnlySpan<Primitive>                  primitives,
        ReadOnlySpan<int>                                   primitiveIndices,
        ReadOnlySpan<float>                                 worldVertices,
        ReadOnlySpan<PreparedTerrainGeometry.PrimitiveInfo> primitiveInfos,
        ref int                                             minimalY
    )
    {
        ref var primitiveRef = ref MemoryMarshal.GetReference(primitives);
        ref var infoRef      = ref MemoryMarshal.GetReference(primitiveInfos);

        for (var bucketIndex = 0; bucketIndex < primitiveIndices.Length; ++bucketIndex)
        {
            var              primitiveIndex = primitiveIndices[bucketIndex];
            ref readonly var info           = ref Unsafe.Add(ref infoRef, primitiveIndex);
            if (!info.Valid)
                continue;

            ref readonly var primitive = ref Unsafe.Add(ref primitiveRef, primitiveIndex);

            if (_voxelizer != null && info.RealSolid)
            {
                var offset1 = primitive.V1 * 3;
                var offset2 = primitive.V2 * 3;
                var offset3 = primitive.V3 * 3;
                var v1x     = worldVertices[offset1];
                var v1y     = worldVertices[offset1 + 1];
                var v1z     = worldVertices[offset1 + 2];
                var v2x     = worldVertices[offset2];
                var v2y     = worldVertices[offset2 + 1];
                var v2z     = worldVertices[offset2 + 2];
                var v3x     = worldVertices[offset3];
                var v3y     = worldVertices[offset3 + 1];
                var v3z     = worldVertices[offset3 + 2];
                var v12x    = v2x           - v1x;
                var v12y    = v2y           - v1y;
                var v12z    = v2z           - v1z;
                var v13x    = v3x           - v1x;
                var v13y    = v3y           - v1y;
                var v13z    = v3z           - v1z;
                var crossX  = (v12y * v13z) - (v12z * v13y);
                var crossY  = (v12z * v13x) - (v12x * v13z);
                var crossZ  = (v12x * v13y) - (v12y * v13x);
                RasterizeVolumeThinWallStrip(new(v1x, v1y, v1z), new(v2x, v2y, v2z), new(v3x, v3y, v3z), crossX, crossY, crossZ);
            }

            if (info.Projected)
                RasterizePreparedTerrainPrimitiveProjected(info, ref minimalY);
            else
                RasterizePreparedTerrainPrimitiveFallback(primitive, worldVertices, info, ref minimalY);
        }
    }

    private void RasterizePreparedTerrainPrimitiveProjected
    (
        PreparedTerrainGeometry.PrimitiveInfo info,
        ref int                               minimalY
    )
    {
        Span<float> clipBuffer   = stackalloc float[7 * 2 * 4];
        var         cellZSize    = _heightfield.cs;
        var         cellXSize    = _heightfield.cs;
        var         inverseCellY = _invCellY;
        var         heightfieldY = _heightfield.bmin.Y;
        var         heightfieldX = _heightfield.bmin.X;
        var         heightfieldZ = _heightfield.bmin.Z;
        var         z0           = Math.Clamp((int)((info.MinZ - heightfieldZ) * _invCellXZ), -1, _heightfield.height - 1);
        var         z1           = Math.Clamp((int)((info.MaxZ - heightfieldZ) * _invCellXZ), 0,  _heightfield.height - 1);
        var         sourceOffset = 0;
        var         rowOffset    = 7 * 2;
        var         tmpOffset    = rowOffset + (7 * 2);
        var         remainOffset = tmpOffset + (7 * 2);
        var         numInput     = 3;

        CopyVertexPair(clipBuffer, sourceOffset,     info.V1X,             info.V1Z);
        CopyVertexPair(clipBuffer, sourceOffset + 2, info.V1X + info.V12X, info.V1Z + info.V12Z);
        CopyVertexPair(clipBuffer, sourceOffset + 4, info.V1X + info.V13X, info.V1Z + info.V13Z);

        for (var z = z0; z <= z1; ++z)
        {
            DividePoly2D(clipBuffer, sourceOffset, numInput, rowOffset, out var numRow, tmpOffset, out numInput, heightfieldZ + ((z + 1) * cellZSize), 1);
            (sourceOffset, tmpOffset) = (tmpOffset, sourceOffset);

            if (numRow < 3 || z < 0)
                continue;

            FindAxisBounds2D(clipBuffer, rowOffset, numRow, 0, out var minX, out var maxX);
            var x0 = (int)((minX - heightfieldX) * _invCellXZ);
            var x1 = (int)((maxX - heightfieldX) * _invCellXZ);
            if (x1 < 0 || x0 >= _heightfield.width)
                continue;

            x0 = Math.Clamp(x0, -1, _heightfield.width - 1);
            x1 = Math.Clamp(x1, 0,  _heightfield.width - 1);

            var numRemaining = numRow;
            var cellZMid     = heightfieldZ + ((z + 0.5f) * cellZSize);

            for (var x = x0; x <= x1; ++x)
            {
                DividePoly2D
                    (clipBuffer, rowOffset, numRemaining, tmpOffset, out var numCell, remainOffset, out numRemaining, heightfieldX + ((x + 1) * cellXSize), 0);
                (rowOffset, remainOffset) = (remainOffset, rowOffset);

                if (numCell < 3 || x < 0)
                    continue;

                var vertexOffset = tmpOffset;
                var minY         = (info.PlaneGradX * clipBuffer[vertexOffset]) + (info.PlaneGradZ * clipBuffer[vertexOffset + 1]) + info.PlaneBias;
                var maxY         = minY;

                for (var i = 1; i < numCell; ++i)
                {
                    vertexOffset = tmpOffset + (i * 2);
                    var y = (info.PlaneGradX * clipBuffer[vertexOffset]) + (info.PlaneGradZ * clipBuffer[vertexOffset + 1]) + info.PlaneBias;
                    minY = Math.Min(minY, y);
                    maxY = Math.Max(maxY, y);
                }

                var y0 = (int)MathF.Floor((minY   - heightfieldY) * inverseCellY);
                var y1 = (int)MathF.Ceiling((maxY - heightfieldY) * inverseCellY);
                if (y1 < 0 || y0 >= _maxY)
                    continue;

                y0 = Math.Clamp(y0, 0,  _maxY - 1);
                y1 = Math.Clamp(y1, y0, _maxY - 1);
                AddTerrainSpan(x, z, y0, y1, info.AreaId, info.RealSolid);
                if (_voxelizer != null && info.RealSolid)
                    WriteVolumeWallThicknessFromProjectedPlane(x, z, y0, y1, info.PlaneGradX, info.PlaneGradZ);

                if (info.RealSolid && _iset != null)
                {
                    minimalY = Math.Min(minimalY, y0);
                    var cellXMid = heightfieldX + ((x + 0.5f) * cellXSize);
                    var apx      = cellXMid     - info.V1X;
                    var apz      = cellZMid     - info.V1Z;
                    var c        = ((apz * info.V12X) - (apx * info.V12Z)) * info.InverseCrossY;
                    var b        = ((apx * info.V13Z) - (apz * info.V13X)) * info.InverseCrossY;

                    if (c >= 0 && b >= 0 && c + b <= 1)
                    {
                        var intersectY = (info.PlaneGradX * cellXMid) + (info.PlaneGradZ * cellZMid) + info.PlaneBias;
                        if (info.NormalUp && y0 > 0)
                            _iset.Add(x, y0 - 1, z, intersectY, true);
                        else if (!info.NormalUp && y1 < _maxY - 1)
                            _iset.Add(x, y1 + 1, z, intersectY, false);
                    }
                }
            }
        }
    }

    private void RasterizePreparedTerrainPrimitiveFallback
    (
        Primitive                  primitive,
        ReadOnlySpan<float>                   worldVertices,
        PreparedTerrainGeometry.PrimitiveInfo info,
        ref int                               minimalY
    )
    {
        Span<float> clipBuffer   = stackalloc float[7 * 3 * 4];
        var         cellZSize    = _heightfield.cs;
        var         cellXSize    = _heightfield.cs;
        var         inverseCellY = _invCellY;
        var         heightfieldY = _heightfield.bmin.Y;
        var         heightfieldX = _heightfield.bmin.X;
        var         heightfieldZ = _heightfield.bmin.Z;
        var         offset1      = primitive.V1 * 3;
        var         offset2      = primitive.V2 * 3;
        var         offset3      = primitive.V3 * 3;
        var         z0           = Math.Clamp((int)((info.MinZ - heightfieldZ) * _invCellXZ), -1, _heightfield.height - 1);
        var         z1           = Math.Clamp((int)((info.MaxZ - heightfieldZ) * _invCellXZ), 0,  _heightfield.height - 1);
        var         sourceOffset = 0;
        var         rowOffset    = 7 * 3;
        var         tmpOffset    = rowOffset + (7 * 3);
        var         remainOffset = tmpOffset + (7 * 3);
        var         numInput     = 3;

        CopyVertexTriplet(clipBuffer, sourceOffset,     worldVertices, offset1);
        CopyVertexTriplet(clipBuffer, sourceOffset + 3, worldVertices, offset2);
        CopyVertexTriplet(clipBuffer, sourceOffset + 6, worldVertices, offset3);

        for (var z = z0; z <= z1; ++z)
        {
            DividePoly(clipBuffer, sourceOffset, numInput, rowOffset, out var numRow, tmpOffset, out numInput, heightfieldZ + ((z + 1) * cellZSize), 2);
            (sourceOffset, tmpOffset) = (tmpOffset, sourceOffset);

            if (numRow < 3 || z < 0)
                continue;

            FindAxisBounds(clipBuffer, rowOffset, numRow, 0, out var minX, out var maxX);
            var x0 = (int)((minX - heightfieldX) * _invCellXZ);
            var x1 = (int)((maxX - heightfieldX) * _invCellXZ);
            if (x1 < 0 || x0 >= _heightfield.width)
                continue;

            x0 = Math.Clamp(x0, -1, _heightfield.width - 1);
            x1 = Math.Clamp(x1, 0,  _heightfield.width - 1);

            var numRemaining = numRow;
            var cellZMid     = heightfieldZ + ((z + 0.5f) * cellZSize);

            for (var x = x0; x <= x1; ++x)
            {
                DividePoly(clipBuffer, rowOffset, numRemaining, tmpOffset, out var numCell, remainOffset, out numRemaining, heightfieldX + ((x + 1) * cellXSize), 0);
                (rowOffset, remainOffset) = (remainOffset, rowOffset);

                if (numCell < 3 || x < 0)
                    continue;

                FindAxisBounds(clipBuffer, tmpOffset, numCell, 1, out var minY, out var maxY);
                var y0 = (int)MathF.Floor((minY   - heightfieldY) * inverseCellY);
                var y1 = (int)MathF.Ceiling((maxY - heightfieldY) * inverseCellY);
                if (y1 < 0 || y0 >= _maxY)
                    continue;

                y0 = Math.Clamp(y0, 0,  _maxY - 1);
                y1 = Math.Clamp(y1, y0, _maxY - 1);
                AddTerrainSpan(x, z, y0, y1, info.AreaId, info.RealSolid);
                if (_voxelizer != null && info.RealSolid)
                    WriteVolumeWallThicknessFromTriangle(x, z, y0, y1, worldVertices, offset1, offset2, offset3);

                if (info.RealSolid && _iset != null && info.InverseCrossY != 0)
                {
                    minimalY = Math.Min(minimalY, y0);
                    var cellXMid = heightfieldX + ((x + 0.5f) * cellXSize);
                    var apx      = cellXMid     - info.V1X;
                    var apz      = cellZMid     - info.V1Z;
                    var c        = ((apz * info.V12X) - (apx * info.V12Z)) * info.InverseCrossY;
                    var b        = ((apx * info.V13Z) - (apz * info.V13X)) * info.InverseCrossY;

                    if (c >= 0 && b >= 0 && c + b <= 1)
                    {
                        var intersectY = ComputeTriangleIntersectY
                        (
                            worldVertices[offset1 + 1],
                            worldVertices[offset2 + 1] - worldVertices[offset1 + 1],
                            worldVertices[offset3 + 1] - worldVertices[offset1 + 1],
                            b,
                            c
                        );
                        if (info.NormalUp && y0 > 0)
                            _iset.Add(x, y0 - 1, z, intersectY, true);
                        else if (!info.NormalUp && y1 < _maxY - 1)
                            _iset.Add(x, y1 + 1, z, intersectY, false);
                    }
                }
            }
        }
    }

    private static float ComputeTriangleIntersectY
    (
        float v1y,
        float v12y,
        float v13y,
        float b,
        float c
    ) => v1y + (b * v12y) + (c * v13y);

    private void RasterizeGeneralPart
    (
        ReadOnlySpan<Primitive> primitives,
        MeshInstance            instance,
        Span<Vector3>                      worldVertices,
        Span<OutFlags>                     outFlags,
        ref int                            minimalY
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
            var crossY = v12cross13.Y;
            var invDiv = _iset != null && crossY != 0 ?
                             -1.0f / crossY :
                             0; // see below
            var normalUp = crossY > 0;

            var flags           = (p.Flags & ~instance.ForceClearPrimFlags) | instance.ForceSetPrimFlags;
            var realSolid       = !flags.HasFlag(PrimitiveFlags.FlyThrough);
            var forceWalkable   = flags.HasFlag(PrimitiveFlags.ForceWalkable);
            var unwalkableSlope = crossY <= 0 || crossY * crossY < _walkableNormalThreshold * _walkableNormalThreshold * lenSq;
            var unwalkable = flags.HasFlag(PrimitiveFlags.ForceUnwalkable) ||
                             (!forceWalkable && unwalkableSlope);
            var areaId = unwalkable ?
                             0 :
                             RC_WALKABLE_AREA;
            if (_voxelizer != null && realSolid)
                RasterizeVolumeThinWallStrip(v1, v2, v3, v12cross13.X, crossY, v12cross13.Z);

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

                var cellZMax      = _heightfield.bmin.Z + ((z + 1) * _heightfield.cs);
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

                var cellZMid = _heightfield.bmin.Z + ((z + 0.5f) * _heightfield.cs);

                for (var x = x0; x <= x1; ++x)
                {
                    if (numRemainingX < 3)
                        break;

                    var cellXMax = _heightfield.bmin.X + ((x + 1) * _heightfield.cs);
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
                    if (_voxelizer != null && realSolid)
                        WriteVolumeWallThickness(x, z, y0, y1, v12cross13.X, crossY, v12cross13.Z);

                    if (realSolid && _iset != null && invDiv != 0)
                    {
                        minimalY = Math.Min(minimalY, y0);
                        var cellXMid = _heightfield.bmin.X + ((x + 0.5f) * _heightfield.cs);
                        var apx      = cellXMid            - v1.X;
                        var apz      = cellZMid            - v1.Z;
                        var c        = ((apz * v12.X) - (apx * v12.Z)) * invDiv;
                        var b        = ((apx * v13.Z) - (apz * v13.X)) * invDiv;

                        if (c >= 0 && b >= 0 && c + b <= 1)
                        {
                            var intersectY = v1.Y + (b * v12.Y) + (c * v13.Y);
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

    private void AddSpan
    (
        int  x,
        int  z,
        int  y0,
        int  y1,
        int  areaId,
        bool includeInVolume,
        bool mergeFromLowerBound = true
    )
    {
        var     yOrig    = (y0, y1);
        ref var cellHead = ref _heightfield.spans[(z * _heightfield.width) + x];

        // find insert position for new span: skip any existing spans that end before new span start
        var  prevMaxY      = y0;
        var  nextMinY      = y1;
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
            if (heightDiff > _walkableClimbThreshold || (heightDiff >= -_walkableClimbThreshold && currSpan.area > areaId))
                areaId = currSpan.area;
            y0 = mergeFromLowerBound ?
                     Math.Min(y0, currSpan.smin) :
                     Math.Max(y0, currSpan.smax);
            y1 = Math.Max(y1, currSpan.smax);

            var nextSpanIndex = currSpan.next;
            if (prevSpanIndex == 0)
                cellHead = nextSpanIndex;
            else
                _heightfield.Span(prevSpanIndex).next = nextSpanIndex;
            FreeSpan(currSpanIndex);
            currSpanIndex = nextSpanIndex;
        }

        // insert new span
        var     newSpanIndex = AllocSpan();
        ref var newSpan      = ref _heightfield.Span(newSpanIndex);
        newSpan.smin = y0;
        newSpan.smax = y1;
        newSpan.area = areaId;
        newSpan.next = currSpanIndex;
        if (prevSpanIndex == 0)
            cellHead = newSpanIndex;
        else
            _heightfield.Span(prevSpanIndex).next = newSpanIndex;

        // mark overlapping voxels as solid; use unmodified y coords since it's possible for a realSolid span to be merged with a fly-through span
        // TODO: figure out if we can preserve flythrough-ability using areaId - not sure if nonzero area id always indicates a walkable surface, recast docs are very unclear on this front
        WriteVolumeSpan(x, z, yOrig.y0, yOrig.y1, includeInVolume);
    }

    private void AddTerrainSpan
    (
        int  x,
        int  z,
        int  y0,
        int  y1,
        int  areaId,
        bool includeInVolume
    )
    {
        WriteVolumeSpan(x, z, y0, y1, includeInVolume);

        if (_terrainSpans != null)
        {
            var cellIndex = (z * _heightfield.width) + x;
            _terrainSpans.Add(cellIndex, y0, y1, areaId, _walkableClimbThreshold);
            return;
        }

        AddSpan(x, z, y0, y1, areaId, false);
    }

    private void FlushTerrainSpans()
    {
        if (_terrainSpans == null)
            return;

        var touchedCells = _terrainSpans.TouchedCells;

        for (var i = 0; i < touchedCells.Count; ++i)
        {
            var cellIndex = touchedCells[i];
            var z         = cellIndex / _heightfield.width;
            var x         = cellIndex - (z * _heightfield.width);
            var entry     = _terrainSpans.Head(cellIndex);

            while (entry != 0)
            {
                var (y0, y1, areaId, next) = _terrainSpans.Read(entry);
                AddSpan(x, z, y0, y1, areaId, false);
                entry = next;
            }
        }
    }

    private static int TryGetExactPow2RatioShift
    (
        int sourceCells,
        int targetCells
    )
    {
        if (sourceCells <= 0 || targetCells <= 0 || sourceCells % targetCells != 0)
            return -1;

        var ratio = sourceCells / targetCells;
        return BitOperations.IsPow2((uint)ratio) ?
                   BitOperations.Log2((uint)ratio) :
                   -1;
    }

    private static int MapVoxelIndex
    (
        int sourceIndex,
        int sourceCells,
        int targetCells,
        int exactShift
    )
        => exactShift >= 0 ?
               sourceIndex >> exactShift :
               (int)((long)sourceIndex * targetCells / sourceCells);

    private static int MapVoxelSpanMaxInclusive
    (
        int sourceIndex,
        int sourceCells,
        int targetCells,
        int exactShift
    )
        => exactShift >= 0 ?
               sourceIndex >> exactShift :
               (int)((((long)(sourceIndex + 1) * targetCells) - 1) / sourceCells);

    private void WriteVolumeSpan
    (
        int  x,
        int  z,
        int  y0,
        int  y1,
        bool includeInVolume
    )
    {
        if (!includeInVolume || !TryMapVolumeSpan(x, z, y0, y1, out var volumeX, out var volumeZ, out var volumeY0, out var volumeY1))
            return;

        _voxelizer!.AddSpan(volumeX, volumeZ, volumeY0, volumeY1);
    }

    private bool TryMapVolumeSpan
    (
        int     x,
        int     z,
        int     y0,
        int     y1,
        out int volumeX,
        out int volumeZ,
        out int volumeY0,
        out int volumeY1
    )
    {
        volumeX  = 0;
        volumeZ  = 0;
        volumeY0 = 0;
        volumeY1 = 0;

        if (_voxelizer == null)
            return false;

        x -= _heightfield.borderSize;
        z -= _heightfield.borderSize;

        if ((uint)x >= (uint)_voxSourceX || (uint)z >= (uint)_voxSourceZ)
            return false;

        var yMin = y0 - _minSpanGap;
        if (y1 < 0 || yMin >= _voxSourceY)
            return false;

        yMin = Math.Clamp(yMin, 0,    _voxSourceY - 1);
        y1   = Math.Clamp(y1,   yMin, _voxSourceY - 1);

        volumeX  = MapVoxelIndex(x,    _voxSourceX, _voxelizer.SizeX, _voxShiftX);
        volumeZ  = MapVoxelIndex(z,    _voxSourceZ, _voxelizer.SizeZ, _voxShiftZ);
        volumeY0 = MapVoxelIndex(yMin, _voxSourceY, _voxelizer.SizeY, _voxShiftY);
        volumeY1 = MapVoxelSpanMaxInclusive(y1, _voxSourceY, _voxelizer.SizeY, _voxShiftY);
        return true;
    }

    private bool TryMapVolumeYSpan
    (
        int     y0,
        int     y1,
        out int volumeY0,
        out int volumeY1
    )
    {
        volumeY0 = 0;
        volumeY1 = 0;

        if (_voxelizer == null)
            return false;

        var yMin = y0 - _minSpanGap;
        if (y1 < 0 || yMin >= _voxSourceY)
            return false;

        yMin = Math.Clamp(yMin, 0,    _voxSourceY - 1);
        y1   = Math.Clamp(y1,   yMin, _voxSourceY - 1);

        volumeY0 = MapVoxelIndex(yMin, _voxSourceY, _voxelizer.SizeY, _voxShiftY);
        volumeY1 = MapVoxelSpanMaxInclusive(y1, _voxSourceY, _voxelizer.SizeY, _voxShiftY);
        return true;
    }

    private void AddVolumeSpanDirect
    (
        int x,
        int z,
        int y0,
        int y1
    )
    {
        if (_voxelizer == null)
            return;

        if ((uint)x >= (uint)_voxelizer.SizeX || (uint)z >= (uint)_voxelizer.SizeZ)
            return;

        if (y1 < 0 || y0 >= _voxelizer.SizeY)
            return;

        y0 = Math.Clamp(y0, 0,  _voxelizer.SizeY - 1);
        y1 = Math.Clamp(y1, y0, _voxelizer.SizeY - 1);
        _voxelizer.AddSpan(x, z, y0, y1);
    }

    private void WriteVolumeWallThickness
    (
        int   x,
        int   z,
        int   y0,
        int   y1,
        float normalX,
        float normalY,
        float normalZ
    )
    {
        if (_voxelizer == null || !TryMapVolumeSpan(x, z, y0, y1, out var volumeX, out var volumeZ, out var volumeY0, out var volumeY1))
            return;

        var horizontalLengthSq = (normalX * normalX) + (normalZ * normalZ);
        if (horizontalLengthSq <= 0.000001f)
            return;

        var normalLength = MathF.Sqrt(horizontalLengthSq + (normalY * normalY));
        if (normalLength <= 0.000001f)
            return;

        var absNormalY = MathF.Abs(normalY) / normalLength;
        if (absNormalY > volumeWallThickenNormalYThreshold)
            return;

        var horizontalLength = MathF.Sqrt(horizontalLengthSq);
        var dirX             = normalX / horizontalLength;
        var dirZ             = normalZ / horizontalLength;
        var stepX = Math.Abs(dirX) >= 0.35f ?
                        Math.Sign(dirX) :
                        0;
        var stepZ = Math.Abs(dirZ) >= 0.35f ?
                        Math.Sign(dirZ) :
                        0;

        if (stepX == 0 && stepZ == 0)
            return;

        for (var radius = 1; radius <= volumeWallThickenHorizontalRadius; ++radius)
        {
            AddVolumeSpanDirect(volumeX + (stepX * radius), volumeZ + (stepZ * radius), volumeY0, volumeY1);
            AddVolumeSpanDirect(volumeX - (stepX * radius), volumeZ - (stepZ * radius), volumeY0, volumeY1);

            if (stepX != 0 && stepZ != 0)
            {
                AddVolumeSpanDirect(volumeX + (stepX * radius), volumeZ,                    volumeY0, volumeY1);
                AddVolumeSpanDirect(volumeX - (stepX * radius), volumeZ,                    volumeY0, volumeY1);
                AddVolumeSpanDirect(volumeX,                    volumeZ + (stepZ * radius), volumeY0, volumeY1);
                AddVolumeSpanDirect(volumeX,                    volumeZ - (stepZ * radius), volumeY0, volumeY1);
            }
        }
    }

    private void RasterizeVolumeThinWallStrip
    (
        Vector3 v1,
        Vector3 v2,
        Vector3 v3,
        float   normalX,
        float   normalY,
        float   normalZ
    )
    {
        if (_voxelizer == null)
            return;

        var horizontalLengthSq = (normalX * normalX) + (normalZ * normalZ);
        if (horizontalLengthSq <= 0.000001f)
            return;

        var normalLength = MathF.Sqrt(horizontalLengthSq + (normalY * normalY));
        if (normalLength <= 0.000001f)
            return;

        var absNormalY = MathF.Abs(normalY) / normalLength;
        if (absNormalY > volumeThinWallStripNormalYThreshold)
            return;

        var minY = Math.Min(v1.Y, Math.Min(v2.Y, v3.Y));
        var maxY = Math.Max(v1.Y, Math.Max(v2.Y, v3.Y));
        var y0   = (int)MathF.Floor((minY   - _heightfield.bmin.Y) * _invCellY);
        var y1   = (int)MathF.Ceiling((maxY - _heightfield.bmin.Y) * _invCellY);
        if (!TryMapVolumeYSpan(y0, y1, out var volumeY0, out var volumeY1))
            return;

        var p1 = WorldToVolumePlane(v1);
        var p2 = WorldToVolumePlane(v2);
        var p3 = WorldToVolumePlane(v3);

        var segA  = p1;
        var segB  = p2;
        var other = p3;
        var len12 = Vector2.DistanceSquared(p1, p2);
        var len23 = Vector2.DistanceSquared(p2, p3);
        var len31 = Vector2.DistanceSquared(p3, p1);

        if (len23 > len12 && len23 >= len31)
        {
            segA  = p2;
            segB  = p3;
            other = p1;
        }
        else if (len31 > len12 && len31 > len23)
        {
            segA  = p3;
            segB  = p1;
            other = p2;
        }

        if (Vector2.DistanceSquared(segA, segB) <= 0.0001f)
            return;

        var projectedThickness = MathF.Sqrt(DistanceToSegmentSquared(other, segA, segB));
        if (projectedThickness > volumeThinWallStripMaxProjectedThickness)
            return;

        var radius   = MathF.Max(volumeThinWallStripBaseRadius, projectedThickness + volumeThinWallStripExtraPadding);
        var minX     = Math.Max(0, (int)MathF.Floor(Math.Min(segA.X, segB.X)                      - radius));
        var maxX     = Math.Min(_voxelizer.SizeX - 1, (int)MathF.Ceiling(Math.Max(segA.X, segB.X) + radius));
        var minZ     = Math.Max(0, (int)MathF.Floor(Math.Min(segA.Y, segB.Y)                      - radius));
        var maxZ     = Math.Min(_voxelizer.SizeZ - 1, (int)MathF.Ceiling(Math.Max(segA.Y, segB.Y) + radius));
        var radiusSq = radius * radius;

        for (var z = minZ; z <= maxZ; ++z)
        for (var x = minX; x <= maxX; ++x)
        {
            var cellCenter = new Vector2(x + 0.5f, z + 0.5f);
            if (DistanceToSegmentSquared(cellCenter, segA, segB) <= radiusSq)
                AddVolumeSpanDirect(x, z, volumeY0, volumeY1);
        }
    }

    private Vector2 WorldToVolumePlane
    (
        Vector3 world
    )
    {
        var sourceX = ((world.X - _heightfield.bmin.X) * _invCellXZ) - _heightfield.borderSize;
        var sourceZ = ((world.Z - _heightfield.bmin.Z) * _invCellXZ) - _heightfield.borderSize;
        var volumeX = sourceX * _voxelizer!.SizeX / _voxSourceX;
        var volumeZ = sourceZ * _voxelizer.SizeZ  / _voxSourceZ;
        return new(volumeX, volumeZ);
    }

    private static float DistanceToSegmentSquared
    (
        Vector2 point,
        Vector2 a,
        Vector2 b
    )
    {
        var ab      = b - a;
        var abLenSq = ab.LengthSquared();
        if (abLenSq <= 0.000001f)
            return Vector2.DistanceSquared(point, a);

        var t = Vector2.Dot(point - a, ab) / abLenSq;
        t = Math.Clamp(t, 0f, 1f);
        var projection = a + (ab * t);
        return Vector2.DistanceSquared(point, projection);
    }

    private void WriteVolumeWallThicknessFromProjectedPlane
    (
        int   x,
        int   z,
        int   y0,
        int   y1,
        float planeGradX,
        float planeGradZ
    )
    {
        var normalX = -planeGradX;
        var normalY = 1f;
        var normalZ = -planeGradZ;
        WriteVolumeWallThickness(x, z, y0, y1, normalX, normalY, normalZ);
    }

    private void WriteVolumeWallThicknessFromTriangle
    (
        int                 x,
        int                 z,
        int                 y0,
        int                 y1,
        ReadOnlySpan<float> worldVertices,
        int                 offset1,
        int                 offset2,
        int                 offset3
    )
    {
        var v1x    = worldVertices[offset1];
        var v1y    = worldVertices[offset1 + 1];
        var v1z    = worldVertices[offset1 + 2];
        var v2x    = worldVertices[offset2];
        var v2y    = worldVertices[offset2 + 1];
        var v2z    = worldVertices[offset2 + 2];
        var v3x    = worldVertices[offset3];
        var v3y    = worldVertices[offset3 + 1];
        var v3z    = worldVertices[offset3 + 2];
        var v12x   = v2x           - v1x;
        var v12y   = v2y           - v1y;
        var v12z   = v2z           - v1z;
        var v13x   = v3x           - v1x;
        var v13y   = v3y           - v1y;
        var v13z   = v3z           - v1z;
        var crossX = (v12y * v13z) - (v12z * v13y);
        var crossY = (v12z * v13x) - (v12x * v13z);
        var crossZ = (v12x * v13y) - (v12y * v13x);
        WriteVolumeWallThickness(x, z, y0, y1, crossX, crossY, crossZ);
    }

    private uint AllocSpan() =>
        _heightfield.spanPool.Alloc();

    private void FreeSpan
    (
        uint spanIndex
    ) =>
        _heightfield.spanPool.Free(spanIndex);

    // TODO: maintain non-empty cells in intersection set?
    private void FillInterior
    (
        int z0,
        int z1,
        int x0,
        int x1,
        int yBelowNonManifold
    )
    {
        if (_iset == null)
            return; // interior filling is disabled

        // fill interiors
        var cells = _iset.TouchedCells;

        for (var celli = 0; celli < cells.Count; ++celli)
        {
            var cell = cells[celli];
            var z    = cell / _iset.NumCellsX;
            var x    = cell - (z * _iset.NumCellsX);
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
    public void RasterizeOld
    (
        SceneExtractor      geom,
        MeshType types
    )
    {
        var vertices = new float[3 * 256];

        foreach (var (name, mesh) in geom.Meshes)
        {
            if ((mesh.MeshType & types) == MeshType.None)
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
                        var flags = (p.Flags & ~inst.ForceClearPrimFlags) | inst.ForceSetPrimFlags;
                        if (_voxelizer != null && flags.HasFlag(PrimitiveFlags.FlyThrough))
                            continue; // TODO: rasterize to normal heightfield, can't do it right now, since we're using same heightfield for both mesh and volume

                        var unwalkable = flags.HasFlag(PrimitiveFlags.ForceUnwalkable);

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

                        var areaId = unwalkable ?
                                         0 :
                                         RC_WALKABLE_AREA;
                        RcRasterizations.RasterizeTriangle(_telemetry, vertices, p.V1, p.V2, p.V3, areaId, _heightfield, _walkableClimbThreshold);
                    }
                }
            }
        }
    }

    private static Vector3 CachedVertex
    (
        ReadOnlySpan<float> vertices,
        int                 i
    )
    {
        var offset = 3 * i;
        return new(vertices[offset], vertices[offset + 1], vertices[offset + 2]);
    }

    private bool IntersectsHeightfield
    (
        AABB bounds
    )
        => bounds.Max.X > _heightfield.bmin.X && bounds.Max.Z > _heightfield.bmin.Z && bounds.Min.X < _heightfield.bmax.X && bounds.Min.Z < _heightfield.bmax.Z;

    private bool IntersectsHeightfield
    (
        Matrix4x3 worldTransform,
        AABB      localBounds
    )
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = (axisX      * localCenter.X) + (axisY      * localCenter.Y) + (axisZ      * localCenter.Z) + worldTransform.Row3;
        var extent      = (Abs(axisX) * localExtent.X) + (Abs(axisY) * localExtent.Y) + (Abs(axisZ) * localExtent.Z);
        var bounds      = new AABB { Min = center - extent, Max = center + extent };
        return IntersectsHeightfield(bounds);
    }

    private static Vector3 Abs
    (
        Vector3 value
    ) => new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));

    private void TransformVertices
    (
        MeshInstance instance,
        ReadOnlySpan<Vector3>   localVertices,
        Span<Vector3>           outWorld,
        Span<OutFlags>          outFlags
    )
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
            var w = (axisX * v.X) + (axisY * v.Y) + (axisZ * v.Z) + trans;

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

    private void TransformVerticesPacked
    (
        MeshInstance instance,
        ReadOnlySpan<Vector3>   localVertices,
        Span<float>             outWorld,
        Span<OutFlags>          outFlags
    )
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
            var w = (axisX * v.X) + (axisY * v.Y) + (axisZ * v.Z) + trans;

            OutFlags f          = 0;
            if (w.X <= bminX) f |= OutFlags.NegX;
            if (w.X >= bmaxX) f |= OutFlags.PosX;
            if (w.Y <= bminY) f |= OutFlags.NegY;
            if (w.Y >= bmaxY) f |= OutFlags.PosY;
            if (w.Z <= bminZ) f |= OutFlags.NegZ;
            if (w.Z >= bmaxZ) f |= OutFlags.PosZ;

            var offset = i * 3;
            outWorld[offset]     = w.X;
            outWorld[offset + 1] = w.Y;
            outWorld[offset + 2] = w.Z;
            outFlags[i]          = f;
        }
    }

    private static void TransformVerticesPacked
    (
        MeshInstance instance,
        ReadOnlySpan<Vector3>   localVertices,
        Span<float>             outWorld
    )
    {
        var     wt     = instance.WorldTransform;
        var     axisX  = wt.Row0;
        var     axisY  = wt.Row1;
        var     axisZ  = wt.Row2;
        var     trans  = wt.Row3;
        ref var srcRef = ref MemoryMarshal.GetReference(localVertices);

        for (var i = 0; i < localVertices.Length; ++i)
        {
            var v      = Unsafe.Add(ref srcRef, i);
            var w      = (axisX * v.X) + (axisY * v.Y) + (axisZ * v.Z) + trans;
            var offset = i * 3;
            outWorld[offset]     = w.X;
            outWorld[offset + 1] = w.Y;
            outWorld[offset + 2] = w.Z;
        }
    }

    private void ComputeOutFlags
    (
        ReadOnlySpan<float> worldVertices,
        Span<OutFlags>      outFlags
    )
    {
        if (worldVertices.Length < outFlags.Length * 3)
            throw new ArgumentException($"顶点缓存长度不足, 顶点数 = {outFlags.Length}, float 数 = {worldVertices.Length}", nameof(worldVertices));

        var bminX = _heightfield.bmin.X;
        var bminY = _heightfield.bmin.Y;
        var bminZ = _heightfield.bmin.Z;
        var bmaxX = _heightfield.bmax.X;
        var bmaxY = _heightfield.bmax.Y;
        var bmaxZ = _heightfield.bmax.Z;

        for (var i = 0; i < outFlags.Length; ++i)
        {
            var offset = i * 3;
            var x      = worldVertices[offset];
            var y      = worldVertices[offset + 1];
            var z      = worldVertices[offset + 2];

            OutFlags f        = 0;
            if (x <= bminX) f |= OutFlags.NegX;
            if (x >= bmaxX) f |= OutFlags.PosX;
            if (y <= bminY) f |= OutFlags.NegY;
            if (y >= bmaxY) f |= OutFlags.PosY;
            if (z <= bminZ) f |= OutFlags.NegZ;
            if (z >= bmaxZ) f |= OutFlags.PosZ;

            outFlags[i] = f;
        }
    }

    private static void CopyVertexTriplet
    (
        Span<float>         destination,
        int                 destinationOffset,
        ReadOnlySpan<float> source,
        int                 sourceOffset
    )
    {
        destination[destinationOffset]     = source[sourceOffset];
        destination[destinationOffset + 1] = source[sourceOffset + 1];
        destination[destinationOffset + 2] = source[sourceOffset + 2];
    }

    private static void CopyVertexPair
    (
        Span<float> destination,
        int         destinationOffset,
        float       x,
        float       z
    )
    {
        destination[destinationOffset]     = x;
        destination[destinationOffset + 1] = z;
    }

    private static void FindAxisBounds
    (
        Span<float> vertices,
        int         offset,
        int         count,
        int         axis,
        out float   min,
        out float   max
    )
    {
        min = vertices[offset + axis];
        max = min;

        for (var i = 1; i < count; ++i)
        {
            var value = vertices[offset + (i * 3) + axis];
            min = Math.Min(min, value);
            max = Math.Max(max, value);
        }
    }

    private static void FindAxisBounds2D
    (
        Span<float> vertices,
        int         offset,
        int         count,
        int         axis,
        out float   min,
        out float   max
    )
    {
        min = vertices[offset + axis];
        max = min;

        for (var i = 1; i < count; ++i)
        {
            var value = vertices[offset + (i * 2) + axis];
            min = Math.Min(min, value);
            max = Math.Max(max, value);
        }
    }

    private static void DividePoly
    (
        Span<float> vertices,
        int         inputOffset,
        int         inputCount,
        int         outputOffset1,
        out int     outputCount1,
        int         outputOffset2,
        out int     outputCount2,
        float       axisOffset,
        int         axis
    )
    {
        Span<float> axisDelta = stackalloc float[12];
        for (var i = 0; i < inputCount; ++i)
            axisDelta[i] = axisOffset - vertices[inputOffset + (i * 3) + axis];

        var count1 = 0;
        var count2 = 0;

        var previousIndex = inputCount - 1;

        for (var currentIndex = 0; currentIndex < inputCount; previousIndex = currentIndex, ++currentIndex)
        {
            var currentDelta  = axisDelta[currentIndex];
            var previousDelta = axisDelta[previousIndex];
            var sameSide      = (currentDelta >= 0) == (previousDelta >= 0);

            if (!sameSide)
            {
                var previousVertexOffset = inputOffset   + (previousIndex * 3);
                var currentVertexOffset  = inputOffset   + (currentIndex  * 3);
                var outputVertex1Offset  = outputOffset1 + (count1        * 3);
                var outputVertex2Offset  = outputOffset2 + (count2        * 3);
                var interpolation        = previousDelta / (previousDelta - currentDelta);

                for (var axisIndex = 0; axisIndex < 3; ++axisIndex)
                {
                    var value = vertices[previousVertexOffset               + axisIndex] +
                                ((vertices[currentVertexOffset + axisIndex] - vertices[previousVertexOffset + axisIndex]) * interpolation);
                    vertices[outputVertex1Offset + axisIndex] = value;
                    vertices[outputVertex2Offset + axisIndex] = value;
                }

                ++count1;
                ++count2;

                if (currentDelta > 0)
                {
                    CopyVertexTriplet(vertices, outputOffset1 + (count1 * 3), vertices, currentVertexOffset);
                    ++count1;
                }
                else if (currentDelta < 0)
                {
                    CopyVertexTriplet(vertices, outputOffset2 + (count2 * 3), vertices, currentVertexOffset);
                    ++count2;
                }
            }
            else
            {
                var currentVertexOffset = inputOffset + (currentIndex * 3);

                if (currentDelta >= 0)
                {
                    CopyVertexTriplet(vertices, outputOffset1 + (count1 * 3), vertices, currentVertexOffset);
                    ++count1;
                    if (currentDelta != 0)
                        continue;
                }

                CopyVertexTriplet(vertices, outputOffset2 + (count2 * 3), vertices, currentVertexOffset);
                ++count2;
            }
        }

        outputCount1 = count1;
        outputCount2 = count2;
    }

    private static void DividePoly2D
    (
        Span<float> vertices,
        int         inputOffset,
        int         inputCount,
        int         outputOffset1,
        out int     outputCount1,
        int         outputOffset2,
        out int     outputCount2,
        float       axisOffset,
        int         axis
    )
    {
        Span<float> axisDelta = stackalloc float[12];
        for (var i = 0; i < inputCount; ++i)
            axisDelta[i] = axisOffset - vertices[inputOffset + (i * 2) + axis];

        var count1 = 0;
        var count2 = 0;

        var previousIndex = inputCount - 1;

        for (var currentIndex = 0; currentIndex < inputCount; previousIndex = currentIndex, ++currentIndex)
        {
            var currentDelta  = axisDelta[currentIndex];
            var previousDelta = axisDelta[previousIndex];
            var sameSide      = (currentDelta >= 0) == (previousDelta >= 0);

            if (!sameSide)
            {
                var previousVertexOffset = inputOffset   + (previousIndex * 2);
                var currentVertexOffset  = inputOffset   + (currentIndex  * 2);
                var outputVertex1Offset  = outputOffset1 + (count1        * 2);
                var outputVertex2Offset  = outputOffset2 + (count2        * 2);
                var interpolation        = previousDelta / (previousDelta - currentDelta);

                for (var axisIndex = 0; axisIndex < 2; ++axisIndex)
                {
                    var value = vertices[previousVertexOffset               + axisIndex] +
                                ((vertices[currentVertexOffset + axisIndex] - vertices[previousVertexOffset + axisIndex]) * interpolation);
                    vertices[outputVertex1Offset + axisIndex] = value;
                    vertices[outputVertex2Offset + axisIndex] = value;
                }

                ++count1;
                ++count2;

                if (currentDelta > 0)
                {
                    CopyVertexPair(vertices, outputOffset1 + (count1 * 2), vertices[currentVertexOffset], vertices[currentVertexOffset + 1]);
                    ++count1;
                }
                else if (currentDelta < 0)
                {
                    CopyVertexPair(vertices, outputOffset2 + (count2 * 2), vertices[currentVertexOffset], vertices[currentVertexOffset + 1]);
                    ++count2;
                }
            }
            else
            {
                var currentVertexOffset = inputOffset + (currentIndex * 2);

                if (currentDelta >= 0)
                {
                    CopyVertexPair(vertices, outputOffset1 + (count1 * 2), vertices[currentVertexOffset], vertices[currentVertexOffset + 1]);
                    ++count1;
                    if (currentDelta != 0)
                        continue;
                }

                CopyVertexPair(vertices, outputOffset2 + (count2 * 2), vertices[currentVertexOffset], vertices[currentVertexOffset + 1]);
                ++count2;
            }
        }

        outputCount1 = count1;
        outputCount2 = count2;
    }

    private static (float min, float max) MinMaxX
    (
        Span<Vector3> vertices,
        int           count
    )
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

    private static (float min, float max) MinMaxY
    (
        Span<Vector3> vertices,
        int           count
    )
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

    private static (float min, float max) MinMaxZ
    (
        Span<Vector3> vertices,
        int           count
    )
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

    private static int SplitConvexPolyZ
    (
        Span<Vector3> src,
        Span<Vector3> dest,
        Span<Vector3> remaining,
        Span<float>   axisDelta,
        ref int       count,
        float         axisOffset
    )
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

            if ((dCurr >= 0) != (dPrev >= 0))
            {
                var s                             = dPrev / (dPrev - dCurr);
                dest[cDest++] = remaining[cRem++] = vPrev + ((vCurr - vPrev) * s);

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

    private static int SplitConvexPolyX
    (
        Span<Vector3> src,
        Span<Vector3> dest,
        Span<Vector3> remaining,
        Span<float>   axisDelta,
        ref int       count,
        float         axisOffset
    )
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

            if ((dCurr >= 0) != (dPrev >= 0))
            {
                var s                             = dPrev / (dPrev - dCurr);
                dest[cDest++] = remaining[cRem++] = vPrev + ((vCurr - vPrev) * s);

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
