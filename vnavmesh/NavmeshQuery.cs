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

    public DtNavMeshQuery MeshQuery;
    public VoxelPathfind? VolumeQuery;
    private readonly IDtQueryFilter _filter = new DtQueryDefaultFilter();
    private readonly TeleportAwareFilter _teleportFilter = new();
    private readonly RandomnessFilter _randomnessFilter;
    private readonly IDtQueryFilter _reachableFilter = new FloodFillAwareFilter();

    public List<long> LastPath => _lastPath;
    private List<long> _lastPath = [];

    public NavmeshQuery(Navmesh navmesh)
    {
        MeshQuery = new(navmesh.Mesh/*, s => Service.Log.Debug(s)*/);
        if (navmesh.Volume != null)
            VolumeQuery = new(navmesh.Volume);
        _randomnessFilter = new(_teleportFilter);
    }

    public List<Vector3> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel)
        => PathfindMeshDetailed(from, to, useRaycast, useStringPulling, range, cancel).Waypoints;

    internal PathfindResult PathfindMeshDetailed(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, float range, CancellationToken cancel)
    {
        var startRef = FindNearestMeshPoly(from);
        var endRef = FindNearestMeshPoly(to);
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}");
        if (startRef == 0 || endRef == 0)
        {
            return LogMeshFailure(from, to, startRef, endRef, 0, range, "无法在导航网格上找到起点或终点多边形");
        }

        var timer = Timer.Create();
        _lastPath.Clear();
        var opt = new DtFindPathOption(range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default, useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0, useRaycast ? 5 : 0);
        var randomness = Service.Config.RandomnessMultiplier;
        IDtQueryFilter filter = randomness > 0 ? _randomnessFilter : _teleportFilter;
        if (randomness > 0)
        {
            _randomnessFilter.RandomnessMultiplier = randomness;
            _randomnessFilter.RandomSeed = (ulong)System.Random.Shared.NextInt64();
        }
        var startPos = from.SystemToRecast();
        var requestedEndPos = to.SystemToRecast();
        var pathStatus = MeshQuery.FindPath(startRef, endRef, startPos, requestedEndPos, filter, ref _lastPath, opt);
        if (pathStatus.Failed() || _lastPath.Count == 0)
        {
            return LogMeshFailure(from, to, startRef, endRef, 0, range, $"地面路径查询失败，状态 = {pathStatus}");
        }
        Service.Log.Debug($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {pathStatus}，路径 = {string.Join(", ", _lastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();

        var lastPoly = _lastPath[^1];
        var resultStatus = PathfindStatus.Complete;
        var finalEndPos = requestedEndPos;
        if (pathStatus.IsPartial() || lastPoly != endRef)
        {
            var closestStatus = MeshQuery.ClosestPointOnPoly(lastPoly, requestedEndPos, out finalEndPos, out _);
            if (closestStatus.Failed())
                return LogMeshFailure(from, to, startRef, endRef, lastPoly, range, $"无法将终点投影到最后可达多边形，状态 = {closestStatus}");

            var projectedDestination = finalEndPos.RecastToSystem();
            resultStatus = range > 0 && Vector3.Distance(projectedDestination, to) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }

        List<Vector3> waypoints;
        if (useStringPulling)
        {
            var straightPath = new List<DtStraightPath>();
            var straightStatus = MeshQuery.FindStraightPath(startPos, finalEndPos, _lastPath, ref straightPath, 1024, 0);
            if (straightStatus.Failed())
                return LogMeshFailure(from, to, startRef, endRef, lastPoly, range, $"直线路径生成失败，状态 = {straightStatus}");

            waypoints = DeduplicateWaypoints(straightPath.Select(p => p.pos.RecastToSystem()));
        }
        else
        {
            var finalDestination = finalEndPos.RecastToSystem();
            waypoints = DeduplicateWaypoints(_lastPath.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(finalDestination));
        }

        if (waypoints.Count == 0)
            return LogMeshFailure(from, to, startRef, endRef, lastPoly, range, "路径点生成结果为空");

        var result = new PathfindResult(resultStatus, waypoints, to, waypoints[^1]);
        LogMeshResult(result, from, startRef, endRef, lastPoly, range, timer.Value());
        return result;
    }

    public List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
        => PathfindVolumeDetailed(from, to, useRaycast, useStringPulling, cancel).Waypoints;

    internal PathfindResult PathfindVolumeDetailed(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("体素导航体未构建，无法执行飞行算路");
            return new(PathfindStatus.Failed, [], to, to);
        }

        var startVoxel = FindNearestVolumeVoxel(from);
        var endVoxel = FindNearestVolumeVoxel(to);
        Service.Log.Debug($"[算路] 飞行体素 {startVoxel:X} -> {endVoxel:X}");
        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 无法定位空体素");
            return new(PathfindStatus.Failed, [], to, to);
        }

        var timer = Timer.Create();
        var voxelPath = VolumeQuery.FindPath(startVoxel, endVoxel, from, to, useRaycast, false, cancel); // TODO: do we need intermediate points for string-pulling algo?
        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 体素路径为空");
            return new(PathfindStatus.Failed, [], to, to);
        }
        Service.Log.Debug($"[算路] 飞行路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，路径 = {string.Join(", ", voxelPath.Select(r => $"{r.p} {r.voxel:X}"))}");

        // TODO: string-pulling support
        var waypoints = DeduplicateWaypoints(voxelPath.Select(r => r.p).Append(to));
        return new(PathfindStatus.Complete, waypoints, to, waypoints[^1]);
    }

    private PathfindResult LogMeshFailure(Vector3 from, Vector3 to, long startRef, long endRef, long lastPoly, float range, string reason)
    {
        var lastPolyText = lastPoly != 0 ? lastPoly.ToString("X") : "<none>";
        Service.Log.Error($"地面算路失败：起点 = {from:f3}，请求终点 = {to:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPolyText}，容差 = {range:f3}，原因 = {reason}");
        return new(PathfindStatus.Failed, [], to, to);
    }

    private void LogMeshResult(PathfindResult result, Vector3 from, long startRef, long endRef, long lastPoly, float range, System.TimeSpan duration)
    {
        var message = $"地面算路完成：状态 = {result.Status}，起点 = {from:f3}，请求终点 = {result.RequestedDestination:f3}，实际终点 = {result.FinalDestination:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPoly:X}，容差 = {range:f3}，耗时 = {duration.TotalSeconds:f3} 秒，路径点 = {result.Waypoints.Count}";
        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Service.Log.Warning(message);
                break;
            case PathfindStatus.Failed:
                Service.Log.Error(message);
                break;
            default:
                Service.Log.Debug(message);
                break;
        }
    }

    private static List<Vector3> DeduplicateWaypoints(IEnumerable<Vector3> points)
    {
        List<Vector3> result = [];
        foreach (var point in points)
        {
            if (result.Count == 0 || Vector3.DistanceSquared(result[^1], point) > 0.000001f)
                result.Add(point);
        }
        return result;
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
}
