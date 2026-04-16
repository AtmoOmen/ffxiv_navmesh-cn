using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Query;

public partial class NavmeshQuery
{
    private readonly Config _config;
    private const    float  StartPolyCandidateHalfExtentXZ          = 5.0f;
    private const    float  StartPolyCandidateHalfExtentY           = 6.0f;
    private const    float  StartPolyCandidateMaxHorizontalDistance = 4.0f;
    private const    float  StartPolyCandidateMaxVerticalDistance   = 3.0f;
    private const    float  StartPolyCandidateAboveTolerance        = 0.75f;
    private const    float  StartSupportProbeRadius                 = 0.35f;
    private const    int    StartSupportProbeCount                  = 8;
    private const    float  StartSupportMatchDistance               = 0.20f;
    private const    int    MaxStartPolyCandidatesToEvaluate        = 8;
    private const    float  EndPolyCandidateHalfExtentXZ            = 5.0f;
    private const    float  EndPolyCandidateHalfExtentY             = 8.0f;
    private const    float  EndPolyCandidateMaxHorizontalDistance   = 4.0f;
    private const    float  EndPolyCandidateMaxVerticalDistance     = 8.0f;
    private const    float  EndPolyCandidateAboveTolerance          = 0.25f;
    private const    float  ShortGapRepairSearchHalfExtentXZ        = 4.0f;
    private const    float  ShortGapRepairSearchHalfExtentY         = 2.5f;
    private const    float  ShortGapRepairMaxBridgeDistance         = 3.5f;
    private const    float  ShortGapRepairMaxVerticalDelta          = 1.0f;
    private const    float  SuspectedTileSeamGapMaxDistance         = 3.0f;
    private const    float  SuspectedTileSeamBoundaryMaxDistance    = 1.0f;
    private const    float  VolumeBoundsClampEpsilon                = 0.1f;

    private readonly record struct TileCoord
    (
        int X,
        int Z
    )
    {
        public override string ToString() => $"{X}x{Z}";
    }

    private readonly record struct SeamDiagnostic
    (
        TileCoord StartTile,
        TileCoord RequestedTile,
        TileCoord FinalTile,
        float     DistanceToNearestBoundary,
        bool      IsNearTileBoundary,
        bool      IsNearbyTile,
        bool      IsShortGap
    )
    {
        public bool IsSuspectedTileSeamCutoff => IsNearTileBoundary && IsNearbyTile && IsShortGap;
    }

    private readonly record struct MeshPolyCandidate
    (
        long    PolyRef,
        Vector3 ProjectedPoint,
        float   HorizontalDistanceSq,
        float   VerticalDelta,
        bool    IsRequestedStart,
        bool    IsPointOverPoly,
        int     SupportProbeHits
    )
    {
        public float VerticalDistanceAbs => MathF.Abs(VerticalDelta);
        public bool  IsTooFarAbove       => VerticalDelta > StartPolyCandidateAboveTolerance;
    }

    private readonly record struct MeshPathCandidate
    (
        MeshPolyCandidate StartCandidate,
        DtStatus          QueryStatus,
        PathfindStatus    ResultStatus,
        Vector3           FinalDestination,
        List<long>        Corridor
    )
    {
        public long    StartRef   => StartCandidate.PolyRef;
        public Vector3 StartPoint => StartCandidate.ProjectedPoint;
        public long    LastPoly   => Corridor.Count > 0 ? Corridor[^1] : 0;
        public bool    IsRequestedStart => StartCandidate.IsRequestedStart;
        public bool    IsPointOverPoly  => StartCandidate.IsPointOverPoly;
        public int     SupportProbeHits => StartCandidate.SupportProbeHits;

        public float DistanceToRequestedTargetSq(Vector3 requestedTarget) => Vector3.DistanceSquared(FinalDestination, requestedTarget);
    }

    private readonly record struct MeshEndCandidate
    (
        long    PolyRef,
        Vector3 ProjectedPoint,
        float   HorizontalDistanceSq,
        float   VerticalDelta,
        bool    IsRequestedEnd,
        bool    IsPointOverPoly
    )
    {
        public float VerticalDistanceAbs => MathF.Abs(VerticalDelta);
        public bool  IsAboveTarget       => VerticalDelta > EndPolyCandidateAboveTolerance;
    }

    private class IntersectQuery : IDtPolyQuery
    {
        public readonly List<long> Result = [];

        public void Process(DtMeshTile tile, DtPoly poly, long refs) => Result.Add(refs);
    }

    private sealed class RandomnessFilter
    (
        IDtQueryFilter inner
    ) : IDtQueryFilter
    {
        public float RandomnessMultiplier;
        public ulong RandomSeed;

        public float GetCost
        (
            RcVec3f    pa,
            RcVec3f    pb,
            long       prevRef,
            DtMeshTile prevTile,
            DtPoly     prevPoly,
            long       curRef,
            DtMeshTile curTile,
            DtPoly     curPoly,
            long       nextRef,
            DtMeshTile nextTile,
            DtPoly     nextPoly
        )
        {
            var cost = inner.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);
            var mult = RandomnessMultiplier;
            if (mult <= 0 || nextPoly == null)
                return cost;

            if (curPoly.GetArea() == Navmesh.AREAID_TELEPORT && nextPoly.GetArea() == Navmesh.AREAID_TELEPORT)
                return cost;

            var a     = (ulong)Math.Min(curRef, nextRef);
            var b     = (ulong)Math.Max(curRef, nextRef);
            var noise = HashToUnitFloat(RandomSeed, a, b);
            return cost + noise * mult;
        }

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => inner.PassFilter(refs, tile, poly);

        private static float HashToUnitFloat(ulong seed, ulong a, ulong b)
        {
            var x = seed ^ a + 0x9E3779B97F4A7C15UL ^ b * 0xBF58476D1CE4E5B9UL;
            x += 0x9E3779B97F4A7C15UL;
            x =  (x ^ x >> 30) * 0xBF58476D1CE4E5B9UL;
            x =  (x ^ x >> 27) * 0x94D049BB133111EBUL;
            x ^= x >> 31;
            return (x >> 40 & 0xFFFFFF) * (1.0f / 16777216.0f);
        }
    }

    public class GoalRadiusHeuristic
    (
        float tolerance
    ) : IDtQueryHeuristic
    {
        float IDtQueryHeuristic.GetCost(RcVec3f neighbourPos, RcVec3f endPos)
        {
            var dist = RcVec3f.Distance(neighbourPos, endPos) * DtDefaultQueryHeuristic.H_SCALE;
            return dist < tolerance ? -1 : dist;
        }
    }

    public class TeleportAwareFilter : IDtQueryFilter
    {
        private readonly DtQueryDefaultFilter _f = new();

        public float GetCost
        (
            RcVec3f    pa,
            RcVec3f    pb,
            long       prevRef,
            DtMeshTile prevTile,
            DtPoly     prevPoly,
            long       curRef,
            DtMeshTile curTile,
            DtPoly     curPoly,
            long       nextRef,
            DtMeshTile nextTile,
            DtPoly     nextPoly
        )
        {
            var cst = _f.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);
            // increase cost of regular connections instead of reducing cost of off-mesh connections, since lowering cost interferes with heuristic
            if (!(curPoly.GetArea() == Navmesh.AREAID_TELEPORT && nextPoly?.GetArea() == Navmesh.AREAID_TELEPORT))
                cst *= 3;
            return cst;
        }


        public virtual bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => true;
    }

    public class FloodFillAwareFilter : TeleportAwareFilter
    {
        public override bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) =>
            (poly.flags & Navmesh.FLAG_UNREACHABLE) == 0;
    }

    public           DtNavMeshQuery      MeshQuery;
    public           VoxelPathfind?      VolumeQuery;
    private readonly PathPostprocessor   _postprocessor;
    private readonly IDtQueryFilter      _filter         = new DtQueryDefaultFilter();
    private readonly TeleportAwareFilter _teleportFilter = new();
    private readonly RandomnessFilter    _randomnessFilter;
    private readonly IDtQueryFilter      _reachableFilter = new FloodFillAwareFilter();

    public List<long> LastPath { get; } = [];

    public NavmeshQuery(Navmesh navmesh, Config config)
    {
        _config   = config;
        MeshQuery = new(navmesh.Mesh /*, s => Service.Log.Debug(s)*/);
        if (navmesh.Volume != null)
            VolumeQuery = new(navmesh.Volume, _config);
        _postprocessor    = new(MeshQuery);
        _randomnessFilter = new(_teleportFilter);
    }

    public List<Vector3> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel)
        => Postprocess(PlanMeshPathDetailed(from, to, useRaycast, range, cancel), useStringPulling, cancel).Waypoints;

    public List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
        => Postprocess(PlanVolumePathDetailed(from, to, useRaycast, cancel), useStringPulling, cancel).Waypoints;

    internal PostprocessedPath Postprocess(PlannerResult result, bool useStringPulling, CancellationToken cancel) =>
        _postprocessor.Process(result, useStringPulling, cancel);


    internal (int x, int z) FindMeshTile(Vector3 position)
    {
        MeshQuery.GetAttachedNavMesh().CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        return (tileX, tileZ);
    }

    internal (Vector3 min, Vector3 max) GetMeshTileBounds(int tileX, int tileZ)
    {
        ref readonly var param = ref MeshQuery.GetAttachedNavMesh().GetParams();
        var              min   = new Vector3(param.orig.X + tileX * param.tileWidth, param.orig.Y, param.orig.Z + tileZ * param.tileHeight);
        var              max   = new Vector3(min.X        + param.tileWidth,         min.Y,        min.Z        + param.tileHeight);
        return (min, max);
    }

    // returns 0 if not found, otherwise polygon ref
    public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true)
    {
        MeshQuery.FindNearestPoly
            (p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), allowUnreachable ? _filter : _reachableFilter, out var nearestRef, out _, out _);
        return nearestRef;
    }

    public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent, bool allowUnreachable = true)
    {
        IntersectQuery query = new();
        MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), allowUnreachable ? _filter : _reachableFilter, query);
        return query.Result;
    }

    public Vector3? FindNearestPointOnMeshPoly
        (Vector3 p, long poly) => TryClosestPointOnPolyWithFlags(p, poly, out var closest, out _) ? closest : null;

    public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true) => FindNearestPointOnMeshPoly
        (p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

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

    // finds the point on the mesh within specified x/z tolerance and with largest Y that is still smaller than p.Y
    public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5, bool allowUnreachable = true)
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ), allowUnreachable);
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    // returns VoxelMap.InvalidVoxel if not found, otherwise voxel index
    public ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5)
    {
        if (VolumeQuery == null)
            return VoxelMap.InvalidVoxel;

        var volume     = VolumeQuery.Volume;
        var halfExtent = new Vector3(halfExtentXZ, halfExtentY, halfExtentXZ);
        var voxel      = VoxelSearch.FindNearestEmptyVoxel(volume, p, halfExtent);
        if (voxel != VoxelMap.InvalidVoxel)
            return voxel;

        var boundsMin = volume.RootTile.BoundsMin + new Vector3(VolumeBoundsClampEpsilon);
        var boundsMax = volume.RootTile.BoundsMax - new Vector3(VolumeBoundsClampEpsilon);
        var clamped   = Vector3.Clamp(p, boundsMin, boundsMax);
        var usedClamp = Vector3.DistanceSquared(clamped, p) > 0.000001f;

        if (usedClamp)
        {
            voxel = VoxelSearch.FindNearestEmptyVoxel(volume, clamped, halfExtent);
            if (voxel != VoxelMap.InvalidVoxel)
            {
                Service.Log.Debug($"[算路] 体素定位改用边界贴靠点：原始位置 = {p:f3}，贴靠后 = {clamped:f3}，搜索范围 = {halfExtent:f3}");
                return voxel;
            }
        }

        ReadOnlySpan<float> fallbackMultipliers = [2f, 4f, 8f, 16f];
        foreach (var multiplier in fallbackMultipliers)
        {
            var expandedHalfExtent = new Vector3(halfExtentXZ * multiplier, halfExtentY * multiplier, halfExtentXZ * multiplier);
            voxel = VoxelSearch.FindNearestEmptyVoxel(volume, usedClamp ? clamped : p, expandedHalfExtent);
            if (voxel == VoxelMap.InvalidVoxel)
                continue;

            Service.Log.Debug
            (
                $"[算路] 体素定位触发扩搜：原始位置 = {p:f3}，搜索中心 = {(usedClamp ? clamped : p):f3}，搜索范围 = {expandedHalfExtent:f3}"
            );
            return voxel;
        }

        return VoxelMap.InvalidVoxel;
    }

    // collect all mesh polygons reachable from specified polygon
    public HashSet<long> FindReachableMeshPolys(params long[] starting)
    {
        HashSet<long> result = [];

        List<long> queue = [.. starting];
        queue.RemoveAll(s => s == 0);

        while (queue.Count > 0)
        {
            var next = queue[^1];
            queue.RemoveAt(queue.Count - 1);

            if (!result.Add(next))
                continue; // already visited

            MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(next, out var nextTile, out var nextPoly);

            for (var i = nextTile.polyLinks[nextPoly.index]; i != DtNavMesh.DT_NULL_LINK; i = nextTile.links[i].next)
            {
                var neighbourRef = nextTile.links[i].refs;
                if (neighbourRef != 0)
                    queue.Add(neighbourRef);
            }
        }

        return result;
    }
}
