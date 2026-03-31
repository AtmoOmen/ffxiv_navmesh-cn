using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Navmesh.NavVolume;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading;

namespace Navmesh;

public class NavmeshQuery
{
    private class IntersectQuery : IDtPolyQuery
    {
        public readonly List<long> Result = [];
        public void Process(DtMeshTile tile, DtPoly poly, long refs) => Result.Add(refs);
    }

    private sealed class RandomnessFilter(IDtQueryFilter inner) : IDtQueryFilter
    {
        public float RandomnessMultiplier;
        public ulong RandomSeed;

        public float GetCost(RcVec3f pa, RcVec3f pb, long prevRef, DtMeshTile prevTile, DtPoly prevPoly, long curRef, DtMeshTile curTile, DtPoly curPoly, long nextRef, DtMeshTile nextTile, DtPoly nextPoly)
        {
            var cost = inner.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);
            var mult = RandomnessMultiplier;
            if (mult <= 0 || nextPoly == null)
                return cost;

            if (curPoly.GetArea() == Navmesh.AREAID_TELEPORT && nextPoly.GetArea() == Navmesh.AREAID_TELEPORT)
                return cost;

            var a = (ulong)System.Math.Min(curRef, nextRef);
            var b = (ulong)System.Math.Max(curRef, nextRef);
            var noise = HashToUnitFloat(RandomSeed, a, b);
            return cost + noise * mult;
        }

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => inner.PassFilter(refs, tile, poly);

        private static float HashToUnitFloat(ulong seed, ulong a, ulong b)
        {
            var x = seed ^ (a + 0x9E3779B97F4A7C15UL) ^ (b * 0xBF58476D1CE4E5B9UL);
            x += 0x9E3779B97F4A7C15UL;
            x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9UL;
            x = (x ^ (x >> 27)) * 0x94D049BB133111EBUL;
            x ^= x >> 31;
            return ((x >> 40) & 0xFFFFFF) * (1.0f / 16777216.0f);
        }
    }

    public class GoalRadiusHeuristic(float tolerance) : IDtQueryHeuristic
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

        public float GetCost(RcVec3f pa, RcVec3f pb, long prevRef, DtMeshTile prevTile, DtPoly prevPoly, long curRef, DtMeshTile curTile, DtPoly curPoly, long nextRef, DtMeshTile nextTile, DtPoly nextPoly)
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
        public override bool PassFilter(long refs, DtMeshTile tile, DtPoly poly)
        {
            return (poly.flags & Navmesh.FLAG_UNREACHABLE) == 0;
        }
    }

    public GroundDetourQuery MeshQuery;
    public VoxelPathfind? VolumeQuery;
    private readonly IDtQueryFilter _filter = new DtQueryDefaultFilter();
    private readonly TeleportAwareFilter _teleportFilter = new();
    private readonly RandomnessFilter _randomnessFilter;
    private readonly IDtQueryFilter _reachableFilter = new FloodFillAwareFilter();

    public List<long> LastPath => _lastPath;
    private List<long> _lastPath = [];
    public GroundPathDebugInfo? LastGroundPath { get; private set; }

    public NavmeshQuery(Navmesh navmesh)
    {
        MeshQuery = new(navmesh.Mesh/*, s => Service.Log.Debug(s)*/);
        if (navmesh.Volume != null)
            VolumeQuery = new(navmesh.Volume);
        _randomnessFilter = new(_teleportFilter);
    }

    public List<Vector3> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel)
    {
        var startRef = FindNearestMeshPoly(from);
        var endRef = FindNearestMeshPoly(to);
        var requestedEnd = to.SystemToRecast();
        Service.Log.Debug($"[pathfind] poly {startRef:X} -> {endRef:X}");
        if (startRef == 0 || endRef == 0)
        {
            Service.Log.Error($"Failed to find a path from {from} ({startRef:X}) to {to} ({endRef:X}): failed to find polygon on a mesh");
            return [];
        }

        var timer = Timer.Create();
        _lastPath.Clear();
        LastGroundPath = null;
        var opt = new DtFindPathOption(range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default, 0, 0);
        var randomness = Service.Config.RandomnessMultiplier;
        IDtQueryFilter filter = randomness > 0 ? _randomnessFilter : _teleportFilter;
        if (randomness > 0)
        {
            _randomnessFilter.RandomnessMultiplier = randomness;
            _randomnessFilter.RandomSeed = (ulong)System.Random.Shared.NextInt64();
        }
        var status = MeshQuery.FindPath(startRef, endRef, from.SystemToRecast(), requestedEnd, filter, ref _lastPath, opt);
        if (_lastPath.Count == 0)
        {
            Service.Log.Error($"Failed to find a path from {from} ({startRef:X}) to {to} ({endRef:X}): failed to find path on mesh");
            return [];
        }
        var reachedEndRef = _lastPath[^1];
        var isPartial = status.IsPartial() || reachedEndRef != endRef;
        var resolvedEnd = ResolvePathEndPosition(reachedEndRef, requestedEnd);
        Service.Log.Debug($"Pathfind took {timer.Value().TotalSeconds:f3}s: {string.Join(", ", _lastPath.Select(r => r.ToString("X")))}");
        Service.Log.Debug($"[pathfind] status={status}, partial={isPartial}, reached={reachedEndRef:X}, target={endRef:X}, resolvedEnd={resolvedEnd}");

        cancel.ThrowIfCancellationRequested();

        var settings = GroundPathSettings.FromConfig(Service.Config);
        if (MeshQuery.TryBuildCorridor(_lastPath, from.SystemToRecast(), resolvedEnd, out var corridor) && corridor != null)
        {
            List<Vector3> result;
            GroundPathDebugInfo debug;
            if (useStringPulling)
                result = GroundPathSmoother.BuildCenteredPath(MeshQuery, corridor, settings, useRaycast, out debug);
            else
                result = GroundPathSmoother.BuildPortalMidpointPath(MeshQuery, corridor, settings, useRaycast, out debug);

            var terminal = isPartial ? resolvedEnd.RecastToSystem() : to;
            EnsureTerminalPoint(result, terminal);
            EnsureTerminalPoint(debug.FinalPath, terminal);
            PopulateGroundPathDebug(debug, isPartial, to, resolvedEnd.RecastToSystem(), status, reachedEndRef);
            LastGroundPath = debug;
            return result;
        }

        var res = _lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).ToList();
        EnsureTerminalPoint(res, isPartial ? resolvedEnd.RecastToSystem() : to);
        LastGroundPath = CreateFallbackGroundPathDebug(res, isPartial, to, resolvedEnd.RecastToSystem(), status, reachedEndRef);
        return res;
    }

    public List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error($"Nav volume was not built");
            return [];
        }

        var startVoxel = FindNearestVolumeVoxel(from);
        var endVoxel = FindNearestVolumeVoxel(to);
        Service.Log.Debug($"[pathfind] voxel {startVoxel:X} -> {endVoxel:X}");
        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"Failed to find a path from {from} ({startVoxel:X}) to {to} ({endVoxel:X}): failed to find empty voxel");
            return [];
        }

        var timer = Timer.Create();
        var voxelPath = VolumeQuery.FindPath(startVoxel, endVoxel, from, to, useRaycast, false, cancel); // TODO: do we need intermediate points for string-pulling algo?
        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"Failed to find a path from {from} ({startVoxel:X}) to {to} ({endVoxel:X}): failed to find path on volume");
            return [];
        }
        Service.Log.Debug($"Pathfind took {timer.Value().TotalSeconds:f3}s: {string.Join(", ", voxelPath.Select(r => $"{r.p} {r.voxel:X}"))}");

        // TODO: string-pulling support
        var res = voxelPath.Select(r => r.p).ToList();
        res.Add(to);
        return res;
    }

    // returns 0 if not found, otherwise polygon ref
    public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true)
    {
        MeshQuery.FindNearestPoly(p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), allowUnreachable ? _filter : _reachableFilter, out var nearestRef, out _, out _);
        return nearestRef;
    }

    public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent, bool allowUnreachable = true)
    {
        IntersectQuery query = new();
        MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), allowUnreachable ? _filter : _reachableFilter, query);
        return query.Result;
    }

    public Vector3? FindNearestPointOnMeshPoly(Vector3 p, long poly) => MeshQuery.ClosestPointOnPoly(poly, p.SystemToRecast(), out var closest, out _).Succeeded() ? closest.RecastToSystem() : null;

    public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true) => FindNearestPointOnMeshPoly(p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

    // finds the point on the mesh within specified x/z tolerance and with largest Y that is still smaller than p.Y
    public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5, bool allowUnreachable = true)
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ), allowUnreachable);
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    // returns VoxelMap.InvalidVoxel if not found, otherwise voxel index
    public ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) => VolumeQuery != null ? VoxelSearch.FindNearestEmptyVoxel(VolumeQuery.Volume, p, new(halfExtentXZ, halfExtentY, halfExtentXZ)) : VoxelMap.InvalidVoxel;

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
            for (int i = nextTile.polyLinks[nextPoly.index]; i != DtNavMesh.DT_NULL_LINK; i = nextTile.links[i].next)
            {
                long neighbourRef = nextTile.links[i].refs;
                if (neighbourRef != 0)
                    queue.Add(neighbourRef);
            }
        }

        return result;
    }

    private RcVec3f ResolvePathEndPosition(long reachedEndRef, RcVec3f requestedEnd)
    {
        if (MeshQuery.ClosestPointOnPoly(reachedEndRef, requestedEnd, out var closest, out _).Succeeded())
            return closest;
        if (MeshQuery.ClosestPointOnPolyBoundary(reachedEndRef, requestedEnd, out closest).Succeeded())
            return closest;

        return MeshQuery.GetAttachedNavMesh().GetPolyCenter(reachedEndRef);
    }

    private static void EnsureTerminalPoint(List<Vector3> points, Vector3 destination)
    {
        if (points.Count == 0)
        {
            points.Add(destination);
            return;
        }

        if (Vector3.Distance(points[^1], destination) <= 0.01f)
            points[^1] = destination;
        else
            points.Add(destination);
    }

    private static void PopulateGroundPathDebug(GroundPathDebugInfo debug, bool isPartial, Vector3 requestedEnd, Vector3 resolvedEnd, DtStatus status, long reachedEndRef)
    {
        debug.IsPartial = isPartial;
        debug.RequestedEnd = requestedEnd;
        debug.ResolvedEnd = resolvedEnd;
        debug.PathStatusText = status.ToString();
        debug.ReachedEndRef = reachedEndRef;
    }

    private GroundPathDebugInfo CreateFallbackGroundPathDebug(List<Vector3> finalPath, bool isPartial, Vector3 requestedEnd, Vector3 resolvedEnd, DtStatus status, long reachedEndRef)
    {
        var debug = new GroundPathDebugInfo
        {
            CorridorCenters = [.. _lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem())],
            Centerline = [.. finalPath],
            FinalPath = [.. finalPath]
        };
        PopulateGroundPathDebug(debug, isPartial, requestedEnd, resolvedEnd, status, reachedEndRef);
        return debug;
    }
}
