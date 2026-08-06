using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Common.Extensions;
using vnavmesh.Internal;
using vnavmesh.Query.Flight;
using vnavmesh.Query.Ground;
using vnavmesh.Query.Models;

namespace vnavmesh.Query;

using static DtDetour;

public class NavmeshQuery
{
    internal readonly record struct SurfaceAwareVolumeVoxelResolution
    (
        ulong   Voxel,
        Vector3 SearchPoint,
        Vector3 SafePoint,
        float?  MinCandidateY,
        bool    UsedSurfaceAnchor
    );

    internal Navmesh NavmeshData { get; }

    internal PluginConfig ConfigData { get; }

    internal IDtQueryFilter AllTraversableFilter { get; } = new DtQueryDefaultFilter();

    internal NavmeshGroundQuery.GroundAreaCostFilter GroundAreaFilter { get; }

    internal NavmeshGroundQuery GroundQuery { get; }

    internal NavmeshFlightQuery FlightQuery { get; }

    internal DtNavMeshQuery MeshQuery
    {
        get
        {
            if (released)
                throw new ObjectDisposedException(nameof(NavmeshQuery));

            var existing = Volatile.Read(ref meshQuery);
            if (existing != null)
                return existing;

            var created = new DtNavMeshQuery(NavmeshData.Mesh);
            return Interlocked.CompareExchange(ref meshQuery, created, null) ?? created;
        }
    }

    internal VoxelPathfind? VolumeQuery =>
        released ?
            null :
            volumeQuery ??= NavmeshData.Volume != null ?
                                new(NavmeshData.Volume) :
                                null;

    internal List<long> LastPath { get; } = [];

    internal static bool SegmentEntersAvoid
    (
        Vector3 from,
        Vector3 to,
        Vector3 center,
        float   radius
    )
    {
        var abx = to.X - from.X;
        var abz = to.Z - from.Z;
        var lenSq = (abx * abx) + (abz * abz);
        float t;

        if (lenSq < 1e-6f)
            t = 0;
        else
        {
            t = ((center.X - from.X) * abx + (center.Z - from.Z) * abz) / lenSq;
            t = Math.Clamp(t, 0f, 1f);
        }

        var dx = from.X + (abx * t) - center.X;
        var dz = from.Z + (abz * t) - center.Z;
        var fromDx = from.X - center.X;
        var fromDz = from.Z - center.Z;
        var minAllowedSq = MathF.Min(radius * radius, (fromDx * fromDx) + (fromDz * fromDz));
        return (dx * dx) + (dz * dz) + 1e-3f < minAllowedSq;
    }

    private readonly PathPostprocessor postprocessor;

    private DtNavMeshQuery? meshQuery;
    private VoxelPathfind?  volumeQuery;
    private bool            released;

    public NavmeshQuery
    (
        Navmesh      navmesh,
        PluginConfig config
    )
    {
        NavmeshData      = navmesh;
        ConfigData       = config;
        GroundAreaFilter = new(navmesh);
        postprocessor    = new(() => MeshQuery, () => GroundAreaFilter);
        GroundQuery      = new(this);
        FlightQuery      = new(this, GroundQuery);
    }

    internal void ReleaseRetainedState()
    {
        LastPath.Clear();
        LastPath.TrimExcess();
        meshQuery = null;
        volumeQuery?.ReleaseRetainedState();
        volumeQuery = null;
        released    = true;
    }

    internal PostprocessedPath Postprocess
    (
        PlannerResult     result,
        CancellationToken cancel
    ) =>
        postprocessor.Process(result, cancel);

    internal PostprocessedPath PostprocessStraightPath
    (
        PlannerResult     result,
        CancellationToken cancel,
        int               straightPathOptions = 0
    ) =>
        postprocessor.ProcessStraightPath(result, cancel, straightPathOptions);

    internal NavmeshGroundQuery.GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        GroundQuery.GetGroundDiagnostics();

    internal PlannerResult PlanMeshPathDetailed
    (
        Vector3           from,
        Vector3           to,
        float             range,
        CancellationToken cancel,
        Vector3?          avoidCenter = null,
        float             avoidRadius = 0
    ) =>
        GroundQuery.PlanMeshPathDetailed(from, to, range, cancel, avoidCenter, avoidRadius);

    internal PlannerResult PlanVolumePathDetailed
    (
        Vector3           from,
        Vector3           to,
        CancellationToken cancel,
        Vector3?          avoidCenter = null,
        float             avoidRadius = 0
    ) =>
        FlightQuery.PlanVolumePathDetailed(from, to, cancel, avoidCenter, avoidRadius);

    internal (int X, int Z) FindMeshTile
    (
        Vector3 position
    )
    {
        MeshQuery.GetAttachedNavMesh().CalcTileLoc(position.ToRecast(), out var tileX, out var tileZ);
        return (tileX, tileZ);
    }

    internal (Vector3 Min, Vector3 Max) GetMeshTileBounds
    (
        int tileX,
        int tileZ
    )
    {
        ref readonly var param = ref MeshQuery.GetAttachedNavMesh().GetParams();
        var              min   = new Vector3(param.orig.X + (tileX * param.tileWidth), param.orig.Y, param.orig.Z + (tileZ * param.tileHeight));
        var              max   = new Vector3(min.X        + param.tileWidth,           min.Y,        min.Z        + param.tileHeight);
        return (min, max);
    }

    public bool IsPointOnMesh
    (
        Vector3 p,
        float   halfExtentY      = 5,
        bool    allowUnreachable = true
    )
    {
        MeshQuery.FindNearestPoly
        (
            p.ToRecast(),
            new(0, halfExtentY, 0),
            allowUnreachable ?
                AllTraversableFilter :
                GroundAreaFilter,
            out _,
            out _,
            out var isOverPoly
        );
        return isOverPoly;
    }

    internal long FindNearestMeshPoly
    (
        Vector3 p,
        float   halfExtentXZ     = 5,
        float   halfExtentY      = 5,
        bool    allowUnreachable = true
    )
    {
        MeshQuery.FindNearestPoly
        (
            p.ToRecast(),
            new(halfExtentXZ, halfExtentY, halfExtentXZ),
            allowUnreachable ?
                AllTraversableFilter :
                GroundAreaFilter,
            out var nearestRef,
            out _,
            out _
        );
        return nearestRef;
    }

    internal List<long> FindIntersectingMeshPolys
    (
        Vector3 p,
        Vector3 halfExtent,
        bool    allowUnreachable = true
    )
    {
        var capacity = 256;

        while (true)
        {
            var refs  = new long[capacity];
            var query = new DtCollectPolysQuery(refs, refs.Length);
            MeshQuery.QueryPolygons
            (
                p.ToRecast(),
                halfExtent.ToRecast(),
                allowUnreachable ?
                    AllTraversableFilter :
                    GroundAreaFilter,
                query
            );
            if (!query.Overflowed())
                return [.. refs.AsSpan(0, query.NumCollected()).ToArray()];

            capacity *= 2;
        }
    }

    private bool TryClosestPointOnPolyWithFlags
    (
        Vector3     point,
        long        poly,
        out Vector3 closestPoint,
        out bool    isOverPoly
    )
    {
        if (MeshQuery.ClosestPointOnPoly(poly, point.ToRecast(), out var closest, out isOverPoly).Succeeded())
        {
            closestPoint = closest.ToSystem();
            return true;
        }

        closestPoint = default;
        isOverPoly   = false;
        return false;
    }

    internal Vector3? FindNearestPointOnMeshPoly
    (
        Vector3 p,
        long    poly
    ) =>
        TryClosestPointOnPolyWithFlags(p, poly, out var closest, out _) ?
            closest :
            null;

    internal Vector3? FindNearestPointOnMesh
    (
        Vector3 p,
        float   halfExtentXZ     = 5,
        float   halfExtentY      = 5,
        bool    allowUnreachable = true
    ) =>
        FindNearestPointOnMeshPoly
            (p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

    internal Vector3? FindRandomPointOnMeshAroundCircle
    (
        Vector3 center,
        float   maxRadius,
        bool    allowUnreachable = true
    )
    {
        if (maxRadius <= 0)
            return null;

        var filter = allowUnreachable ?
                         AllTraversableFilter :
                         GroundAreaFilter;
        var startRef = FindNearestMeshPoly(center, 8, 8, allowUnreachable);
        if (startRef == 0)
            return null;

        var status = MeshQuery.FindRandomPointWithinCircle
            (startRef, center.ToRecast(), maxRadius, filter, new RcRand(Random.Shared.NextInt64()), out _, out var point);
        return status.Succeeded() ?
                   point.ToSystem() :
                   null;
    }

    internal Vector3? FindPointOnFloor
    (
        Vector3 p,
        float   halfExtentXZ     = 5,
        bool    allowUnreachable = true
    )
    {
        var floor = FindPointOnFloorStrict(p, halfExtentXZ, allowUnreachable);
        if (floor is { } strictFloor)
            return strictFloor;

        var         searchRadius = MathF.Max(halfExtentXZ, MathF.Max(ConfigData.PathTolerance, 0.5f));
        Span<float> extraRadii   = stackalloc float[3];
        extraRadii[0] = searchRadius * 1.5f;
        extraRadii[1] = searchRadius * 2.0f;
        extraRadii[2] = searchRadius + MathF.Max(4f, searchRadius);

        foreach (var t in extraRadii)
        {
            var radius = MathF.Max(searchRadius, t);
            floor = FindPointOnFloorStrict(p, radius, allowUnreachable);
            if (floor is not { } expandedFloor)
                continue;

            return expandedFloor;
        }

        var fallbackRadius = MathF.Max(searchRadius * 2.0f, searchRadius + 4f);
        var nearestPoint   = FindNearestPointOnMesh(p, fallbackRadius, 2048, allowUnreachable);
        if (nearestPoint is not { } fallbackFloor || fallbackFloor.Y > p.Y + float.Epsilon)
            return null;

        var horizontalDistance = HorizontalDistanceXZ(p, fallbackFloor);
        if (horizontalDistance > fallbackRadius + MathF.Max(ConfigData.PathTolerance, 0.25f))
            return null;

        return fallbackFloor;
    }

    internal SurfaceAwareVolumeVoxelResolution FindNearestVolumeVoxelSurfaceAware
    (
        Vector3 point,
        float   halfExtentXZ = 5,
        float   halfExtentY  = 5
    )
    {
        if (VolumeQuery == null)
            return new(VoxelMap.INVALID_VOXEL, point, point, null, false);

        var    volume            = VolumeQuery.Volume;
        var    leafCellSize      = volume.Levels[^1].CellSize;
        var    surfaceSnapHeight = MathF.Max(leafCellSize.Y * 2f, 0.75f);
        var    probeLift         = MathF.Max(surfaceSnapHeight,   MathF.Min(halfExtentY, leafCellSize.Y * 4f));
        var    floorProbe        = point + (Vector3.UnitY * probeLift);
        var    floor             = FindPointOnFloor(floorProbe, halfExtentXZ);
        var    searchPoint       = point;
        float? minCandidateY     = null;
        var    usedSurfaceAnchor = false;

        if (floor is { } floorPoint                     &&
            point.Y >= floorPoint.Y - surfaceSnapHeight &&
            point.Y <= floorPoint.Y + surfaceSnapHeight)
        {
            minCandidateY = floorPoint.Y + MathF.Max(leafCellSize.Y * 0.35f, 0.2f);

            var requestedLeaf = volume.FindLeafVoxel(point);

            if (!requestedLeaf.empty                          ||
                requestedLeaf.voxel == VoxelMap.INVALID_VOXEL ||
                IsVoxelLikelyBelowSurface(requestedLeaf.voxel, minCandidateY.Value))
            {
                var searchLift = MathF.Max(MathF.Max(leafCellSize.Y * 0.85f, 0.45f), minCandidateY.Value - floorPoint.Y);
                searchPoint       = point with { Y = MathF.Max(point.Y,              floorPoint.Y        + searchLift) };
                usedSurfaceAnchor = true;
            }
        }

        var voxel = FindNearestVolumeVoxel(searchPoint, halfExtentXZ, halfExtentY, true, minCandidateY);
        var safePoint = voxel != VoxelMap.INVALID_VOXEL ?
                            VoxelSearch.FindClosestVoxelPoint(volume, voxel, searchPoint) :
                            searchPoint;
        return new(voxel, searchPoint, safePoint, minCandidateY, usedSurfaceAnchor);
    }

    internal ulong FindNearestVolumeVoxel
    (
        Vector3 p,
        float   halfExtentXZ   = 5,
        float   halfExtentY    = 5,
        bool    preferNonBelow = false,
        float?  minCandidateY  = null
    )
    {
        if (VolumeQuery == null)
            return VoxelMap.INVALID_VOXEL;

        var volume       = VolumeQuery.Volume;
        var halfExtent   = new Vector3(halfExtentXZ, halfExtentY, halfExtentXZ);
        var searchCenter = p;
        var voxel        = VoxelSearch.FindNearestEmptyVoxel(volume, p, halfExtent, preferNonBelow, minCandidateY);
        if (voxel != VoxelMap.INVALID_VOXEL)
            return voxel;

        var leafCellSize = volume.Levels[^1].CellSize;
        var clampPadding = new Vector3
        (
            MathF.Max
            (
                MathF.Min(leafCellSize.X, MathF.Min(leafCellSize.Y, leafCellSize.Z)) * 0.5f,
                MathF.Max(ConfigData.PathTolerance, float.Epsilon)
            )
        );
        var boundsMin = volume.RootTile.BoundsMin + clampPadding;
        var boundsMax = volume.RootTile.BoundsMax - clampPadding;
        var clamped   = Vector3.Clamp(p, boundsMin, boundsMax);
        var usedClamp = Vector3.DistanceSquared(clamped, p) > 0.000001f;

        if (usedClamp)
        {
            searchCenter = clamped;
            voxel        = VoxelSearch.FindNearestEmptyVoxel(volume, clamped, halfExtent, preferNonBelow, minCandidateY);

            if (voxel != VoxelMap.INVALID_VOXEL)
            {
                Service.Log.Debug($"[算路] 体素定位改用边界贴靠点：原始位置 = {p:f3}，贴靠后 = {clamped:f3}，搜索范围 = {halfExtent:f3}");
                return voxel;
            }
        }

        var searchLimit = MathF.Max
        (
            volume.RootTile.BoundsMax.X - volume.RootTile.BoundsMin.X,
            MathF.Max(volume.RootTile.BoundsMax.Y - volume.RootTile.BoundsMin.Y, volume.RootTile.BoundsMax.Z - volume.RootTile.BoundsMin.Z)
        );
        var expandedHalfExtent = halfExtent;
        var expansionStep = new Vector3
        (
            MathF.Max(halfExtent.X, MathF.Max(ConfigData.PathTolerance, float.Epsilon)),
            MathF.Max(halfExtent.Y, MathF.Max(ConfigData.PathTolerance, float.Epsilon)),
            MathF.Max(halfExtent.Z, MathF.Max(ConfigData.PathTolerance, float.Epsilon))
        );
        var previousSearchRadius = expandedHalfExtent.Length();

        while (previousSearchRadius < searchLimit)
        {
            expandedHalfExtent += expansionStep;
            voxel              =  VoxelSearch.FindNearestEmptyVoxel(volume, searchCenter, expandedHalfExtent, preferNonBelow, minCandidateY);

            if (voxel == VoxelMap.INVALID_VOXEL)
            {
                previousSearchRadius = expandedHalfExtent.Length();
                continue;
            }

            Service.Log.Debug
            (
                $"[算路] 体素定位触发扩搜：原始位置 = {p:f3}，搜索中心 = {searchCenter:f3}，搜索范围 = {expandedHalfExtent:f3}"
            );
            return voxel;
        }

        return VoxelMap.INVALID_VOXEL;
    }

    private bool IsVoxelLikelyBelowSurface
    (
        ulong voxel,
        float minCandidateY
    )
    {
        if (VolumeQuery == null || voxel == VoxelMap.INVALID_VOXEL)
            return true;

        var bounds     = VolumeQuery.Volume.VoxelBounds(voxel, 0);
        var leafCellY  = VolumeQuery.Volume.Levels[^1].CellSize.Y;
        var floorSlack = MathF.Max(leafCellY * 0.5f, 0.25f);
        return bounds.min.Y + floorSlack < minCandidateY;
    }

    private Vector3? FindPointOnFloorStrict
    (
        Vector3 p,
        float   halfExtentXZ,
        bool    allowUnreachable
    )
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ), allowUnreachable);
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    private static float HorizontalDistanceXZ
    (
        Vector3 left,
        Vector3 right
    )
    {
        var dx = left.X             - right.X;
        var dz = left.Z             - right.Z;
        return MathF.Sqrt((dx * dx) + (dz * dz));
    }

    internal HashSet<long> FindReachableMeshPolys
    (
        params long[] starting
    )
    {
        HashSet<long> result = [];

        List<long> queue = [.. starting];
        queue.RemoveAll(s => s == 0);

        while (queue.Count > 0)
        {
            var next = queue[^1];
            queue.RemoveAt(queue.Count - 1);

            if (!result.Add(next))
                continue;

            MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(next, out var nextTile, out var nextPoly);

            for (var i = nextPoly.firstLink; i != DT_NULL_LINK; i = nextTile.links[i].next)
            {
                var neighbourRef = nextTile.links[i].refs;
                if (neighbourRef != 0)
                    queue.Add(neighbourRef);
            }
        }

        return result;
    }
}
