using System.Numerics;
using DotRecast.Core;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Common.Utilities;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume.Pathfinding;

namespace vnavmesh.Navigation.Mesh.Query;

using static DtDetour;

public class NavmeshQuery
{
    internal Navmesh NavmeshData { get; }

    internal Config ConfigData { get; }

    internal IDtQueryFilter AllTraversableFilter { get; } = new DtQueryDefaultFilter();

    internal NavmeshGroundQuery.GroundAreaCostFilter GroundAreaFilter { get; } = new();

    internal NavmeshGroundQuery.GroundAreaCostFilter GroundAreaFilterIgnoringUnreachable { get; } = new(false);

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
        released ? null : volumeQuery ??= NavmeshData.Volume != null ? new(NavmeshData.Volume, ConfigData) : null;

    internal List<long> LastPath { get; } = [];

    private readonly PathPostprocessor postprocessor;

    private DtNavMeshQuery? meshQuery;
    private VoxelPathfind?  volumeQuery;
    private bool            released;

    public NavmeshQuery(Navmesh navmesh, Config config)
    {
        this.NavmeshData = navmesh;
        this.ConfigData  = config;
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

    internal List<Vector3> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel) =>
        Postprocess(PlanMeshPathDetailed(from, to, useRaycast, range, cancel), useStringPulling, cancel).Waypoints;

    internal List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel) =>
        Postprocess(PlanVolumePathDetailed(from, to, useRaycast, cancel), useStringPulling, cancel).Waypoints;

    internal PostprocessedPath Postprocess(PlannerResult result, bool useStringPulling, CancellationToken cancel) =>
        postprocessor.Process(result, useStringPulling, cancel);

    internal PostprocessedPath PostprocessStraightPath(PlannerResult result, CancellationToken cancel, int straightPathOptions = 0) =>
        postprocessor.ProcessStraightPath(result, cancel, straightPathOptions);

    internal NavmeshGroundQuery.GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        GroundQuery.GetGroundDiagnostics();

    internal PlannerResult PlanMeshPathDetailed(Vector3 from, Vector3 to, bool useRaycast, float range, CancellationToken cancel) =>
        GroundQuery.PlanMeshPathDetailed(from, to, useRaycast, range, cancel);

    internal PlannerResult PlanVolumePathDetailed(Vector3 from, Vector3 to, bool useRaycast, CancellationToken cancel) =>
        FlightQuery.PlanVolumePathDetailed(from, to, useRaycast, cancel);

    internal (int X, int Z) FindMeshTile(Vector3 position)
    {
        MeshQuery.GetAttachedNavMesh().CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        return (tileX, tileZ);
    }

    internal (Vector3 Min, Vector3 Max) GetMeshTileBounds(int tileX, int tileZ)
    {
        ref readonly var param = ref MeshQuery.GetAttachedNavMesh().GetParams();
        var              min   = new Vector3(param.orig.X + tileX * param.tileWidth, param.orig.Y, param.orig.Z + tileZ * param.tileHeight);
        var              max   = new Vector3(min.X        + param.tileWidth,         min.Y,        min.Z        + param.tileHeight);
        return (min, max);
    }

    internal long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true)
    {
        MeshQuery.FindNearestPoly
        (
            p.SystemToRecast(),
            new(halfExtentXZ, halfExtentY, halfExtentXZ),
            allowUnreachable ? AllTraversableFilter : GroundAreaFilter,
            out var nearestRef,
            out _,
            out _
        );
        return nearestRef;
    }

    internal List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent, bool allowUnreachable = true)
    {
        var capacity = 256;

        while (true)
        {
            var refs  = new long[capacity];
            var query = new DtCollectPolysQuery(refs, refs.Length);
            MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), allowUnreachable ? AllTraversableFilter : GroundAreaFilter, query);
            if (!query.Overflowed())
                return [.. refs.AsSpan(0, query.NumCollected()).ToArray()];

            capacity *= 2;
        }
    }

    private bool TryClosestPointOnPolyWithFlags(Vector3 point, long poly, out Vector3 closestPoint, out bool isOverPoly)
    {
        if (MeshQuery.ClosestPointOnPoly(poly, point.SystemToRecast(), out var closest, out isOverPoly).Succeeded())
        {
            closestPoint = closest.RecastToSystem();
            return true;
        }

        closestPoint = default;
        isOverPoly   = false;
        return false;
    }

    internal Vector3? FindNearestPointOnMeshPoly(Vector3 p, long poly) =>
        TryClosestPointOnPolyWithFlags(p, poly, out var closest, out _) ? closest : null;

    internal Vector3? FindNearestPointOnMesh
    (
        Vector3 p,
        float   halfExtentXZ     = 5,
        float   halfExtentY      = 5,
        bool    allowUnreachable = true
    ) =>
        FindNearestPointOnMeshPoly
            (p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

    internal Vector3? FindRandomPointOnMeshAroundCircle(Vector3 center, float maxRadius, bool allowUnreachable = true)
    {
        if (maxRadius <= 0)
            return null;

        var filter   = allowUnreachable ? this.AllTraversableFilter : GroundAreaFilter;
        var startRef = FindNearestMeshPoly(center, 8, 8, allowUnreachable);
        if (startRef == 0)
            return null;

        var status = MeshQuery.FindRandomPointWithinCircle
            (startRef, center.SystemToRecast(), maxRadius, filter, new RcRand(Random.Shared.NextInt64()), out _, out var point);
        return status.Succeeded() ? point.RecastToSystem() : null;
    }

    internal Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5, bool allowUnreachable = true)
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ), allowUnreachable);
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    internal ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5)
    {
        if (VolumeQuery == null)
            return VoxelMap.INVALID_VOXEL;

        var volume       = VolumeQuery.Volume;
        var halfExtent   = new Vector3(halfExtentXZ, halfExtentY, halfExtentXZ);
        var searchCenter = p;
        var voxel        = VoxelSearch.FindNearestEmptyVoxel(volume, p, halfExtent);
        if (voxel != VoxelMap.INVALID_VOXEL)
            return voxel;

        var leafCellSize = volume.Levels[^1].CellSize;
        var clampPadding = new Vector3
            (MathF.Max(MathF.Min(leafCellSize.X, MathF.Min(leafCellSize.Y, leafCellSize.Z)) * 0.5f, MathF.Max(ConfigData.PathTolerance, float.Epsilon)));
        var boundsMin = volume.RootTile.BoundsMin + clampPadding;
        var boundsMax = volume.RootTile.BoundsMax - clampPadding;
        var clamped   = Vector3.Clamp(p, boundsMin, boundsMax);
        var usedClamp = Vector3.DistanceSquared(clamped, p) > 0.000001f;

        if (usedClamp)
        {
            searchCenter = clamped;
            voxel        = VoxelSearch.FindNearestEmptyVoxel(volume, clamped, halfExtent);

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
            voxel              =  VoxelSearch.FindNearestEmptyVoxel(volume, searchCenter, expandedHalfExtent);

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

    internal HashSet<long> FindReachableMeshPolys(params long[] starting)
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
