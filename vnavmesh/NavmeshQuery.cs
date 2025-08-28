using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Navmesh.NavVolume;

namespace Navmesh;

public class NavmeshQuery
{
    private class IntersectQuery : IDtPolyQuery
    {
        public readonly List<long> Result = [];

        public void Process(DtMeshTile tile, DtPoly poly, long refs) => Result.Add(refs);
    }

    private class ToleranceHeuristic(float tolerance) : IDtQueryHeuristic
    {
        float IDtQueryHeuristic.GetCost(RcVec3f neighbourPos, RcVec3f endPos)
        {
            var dist = RcVec3f.Distance(neighbourPos, endPos) * DtDefaultQueryHeuristic.H_SCALE;
            return dist < tolerance ? -1 : dist;
        }
    }

    private class ObstacleAvoidanceFilter(DtNavMesh navMesh) : IDtQueryFilter
    {
        private readonly DtQueryDefaultFilter baseFilter = new();
        
        private const float obstacleProximityWeight = 2.0f;
        private const float maxPenaltyDistance      = 5.0f;

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => 
            baseFilter.PassFilter(refs, tile, poly);

        public float GetCost(
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
            DtPoly     nextPoly)
        {
            var baseCost = baseFilter.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);

            // 计算障碍物邻近惩罚
            var obstaclePenalty = CalculateObstacleProximityPenalty(curRef, RcVec3f.Lerp(pa, pb, 0.5f));

            return baseCost + obstaclePenalty;
        }

        private float CalculateObstacleProximityPenalty(long polyRef, RcVec3f position)
        {
            if (navMesh == null || polyRef == 0)
                return 0;

            // 获取当前多边形周围的区域进行搜索
            var halfExtents = new RcVec3f(maxPenaltyDistance * 2, maxPenaltyDistance, maxPenaltyDistance * 2);
            var query       = new ObstacleDistanceQuery(position);

            var navQuery = new DtNavMeshQuery(navMesh);
            navQuery.QueryPolygons(position, halfExtents, this, query);

            return query.MinDistance < maxPenaltyDistance ? (maxPenaltyDistance - query.MinDistance) * obstacleProximityWeight : 0;
        }
    }

    private class ObstacleDistanceQuery(RcVec3f center) : IDtPolyQuery
    {
        public float MinDistance = float.MaxValue;

        public void Process(DtMeshTile tile, DtPoly poly, long refs)
        {
            if (poly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
                return;
            
            for (var i = 0; i < poly.vertCount; i++)
            {
                var idx = poly.verts[i];
                var v = new RcVec3f(
                    tile.data.verts[idx * 3],
                    tile.data.verts[(idx * 3) + 1],
                    tile.data.verts[(idx * 3) + 2]
                );
                var dist = RcVec3f.Distance(v, center);
                if (dist < MinDistance)
                    MinDistance = dist;
                
                if (MinDistance < 0.1f) // 如果已经非常接近，提前返回
                    return;
            }
        }
    }

    public readonly DtNavMeshQuery MeshQuery;
    public readonly VoxelPathfind? VolumeQuery;
    private readonly IDtQueryFilter filter;

    public  List<long> LastPath => lastPath;
    private List<long> lastPath = [];

    public NavmeshQuery(Navmesh navmesh)
    {
        MeshQuery = new(navmesh.Mesh);
        filter = new ObstacleAvoidanceFilter(navmesh.Mesh);
        if (navmesh.Volume != null)
            VolumeQuery = new(navmesh.Volume);
    }

    public List<Vector3> PathfindMesh(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, CancellationToken cancel, float range = 0)
    {
        var startRef = FindNearestMeshPoly(from);
        var endRef   = FindNearestMeshPoly(to);
        Service.Log.Debug($"[寻路] 多边形 {startRef:X} -> {endRef:X}");
        if (startRef == 0 || endRef == 0)
        {
            Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法在网格上找到多边形");
            return [];
        }

        var timer = Timer.Create();
        lastPath.Clear();

        IDtQueryHeuristic heuristic    = range > 0 ? new ToleranceHeuristic(range) : DtDefaultQueryHeuristic.Default;
        var               options      = useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0;
        var               raycastLimit = useRaycast ? 10 : 0;
        
        var opt = new DtFindPathOption(heuristic, options, raycastLimit);
        MeshQuery.FindPath(startRef, endRef, from.SystemToRecast(), to.SystemToRecast(), filter, ref lastPath, opt);

        if (lastPath.Count == 0)
        {
            Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法在网格上找到路径");
            return [];
        }

        Service.Log.Debug($"寻路耗时 {timer.Value().TotalSeconds:f3} 秒: {string.Join(", ", lastPath.Select(r => r.ToString("X")))}");

        var endPos = to.SystemToRecast();

        if (useStringPulling)
        {
            var pathPoints = new List<Vector3>(lastPath.Count);
            foreach (var t in lastPath)
            {
                var  polyCenter = MeshQuery.GetAttachedNavMesh().GetPolyCenter(t);
                pathPoints.Add(polyCenter.RecastToSystem());
            }
            
            if (pathPoints.Count > 0)
            {
                // 确保起点和终点正确
                pathPoints[0] = from;
                if (pathPoints.Count > 1)
                    pathPoints[^1] = to;
                else
                    pathPoints.Add(to);
                    
                return ApplyMeshStringPulling(pathPoints, to);
            }
        }
        
        // 如果拉绳失败或未启用拉绳，返回基本路径
        var res = new List<Vector3>(lastPath.Count + 1);
        foreach (var t in lastPath)
        {
            var polyCenter = MeshQuery.GetAttachedNavMesh().GetPolyCenter(t);
            res.Add(polyCenter.RecastToSystem());
        }

        res.Add(endPos.RecastToSystem());
        return res;
    }

    public List<Vector3> PathfindVolume(Vector3 from, Vector3 to, bool useRaycast, bool useStringPulling, Action<float>? progressCallback, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("导航尚未构建");
            return [];
        }

        var startVoxel = FindNearestVolumeVoxel(from);
        var endVoxel   = FindNearestVolumeVoxel(to);
        Service.Log.Debug($"[寻路] 体素 {startVoxel:X} -> {endVoxel:X}");
        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"无法找到从 {from} ({startVoxel:X}) 到 {to} ({endVoxel:X}) 的路径：未能找到空体素");
            return [];
        }

        // 如果起点和终点足够近，并且有视线，则直接返回一条直线路径
        if ((from - to).LengthSquared() < 100.0f && VoxelSearch.LineOfSight(VolumeQuery.Volume, startVoxel, endVoxel, from, to))
        {
            Service.Log.Debug($"[寻路] 从 {from} 到 {to} 间存在视线，直接返回直线路径");
            var directPath = new List<Vector3> { from, to };

            return directPath;
        }

        var timer = Timer.Create();

        var voxelPath = VolumeQuery.FindPath(startVoxel, endVoxel, from, to, useRaycast, false, progressCallback, cancel);

        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"无法找到从 {from} ({startVoxel:X}) 到 {to} ({endVoxel:X}) 的路径：未能找到路径");
            return [];
        }

        Service.Log.Debug($"寻路耗时 {timer.Value().TotalSeconds:f3}秒: {string.Join(", ", voxelPath.Select(r => $"{r.p} {r.voxel:X}"))}");

        List<Vector3> res;

        // 支持弦拉优化
        if (useStringPulling && voxelPath.Count > 2)
            res = ApplyStringPulling(voxelPath.Select(r => r.p).ToList(), to);
        else
        {
            res = voxelPath.Select(r => r.p).ToList();
            res.Add(to);
        }

        return res;
    }

    private List<Vector3> ApplyStringPulling(List<Vector3> pathPoints, Vector3 destination)
    {
        if (pathPoints.Count <= 2)
            return pathPoints;

        var result       = new List<Vector3> { pathPoints[0] };
        var currentPoint = 0;

        while (currentPoint < pathPoints.Count - 1)
        {
            var farthestVisible = currentPoint + 1;

            // 查找最远的可见点
            for (var i = farthestVisible + 1; i < pathPoints.Count; i++)
            {
                var fromVoxel = FindNearestVolumeVoxel(pathPoints[currentPoint]);
                var toVoxel   = FindNearestVolumeVoxel(pathPoints[i]);

                if (VoxelSearch.LineOfSight(VolumeQuery!.Volume, fromVoxel, toVoxel,
                                            pathPoints[currentPoint], pathPoints[i]))
                    farthestVisible = i;
                else
                    break;
            }

            // 添加最远的可见点
            result.Add(pathPoints[farthestVisible]);
            currentPoint = farthestVisible;
        }

        // 确保终点在路径中
        if ((result[^1] - destination).LengthSquared() > 0.01f) result.Add(destination);

        return result;
    }

    private List<Vector3> ApplyMeshStringPulling(List<Vector3> pathPoints, Vector3 destination)
    {
        if (pathPoints.Count <= 2)
            return pathPoints;

        var result = new List<Vector3> { pathPoints[0] };
        const int segmentSize = 8; // 每段的最大长度
        
        for (var segmentStart = 0; segmentStart < pathPoints.Count - 1; )
        {
            var segmentEnd = Math.Min(segmentStart + segmentSize, pathPoints.Count - 1);
            
            // 对当前段进行拉绳优化
            var segmentOptimized = OptimizePathSegment(pathPoints, segmentStart, segmentEnd);
            
            // 添加优化后的段落点（跳过第一个点以避免重复）
            for (var i = segmentStart == 0 ? 1 : 0; i < segmentOptimized.Count; i++)
                result.Add(segmentOptimized[i]);
            
            segmentStart = segmentEnd;
        }
        
        // 确保终点在路径中
        if ((result[^1] - destination).LengthSquared() > 0.01f)
            result.Add(destination);
        
        // 应用拐角插值以提升平滑度
        result = ApplyCornerInterpolation(result);
            
        return result;
    }
    
    private List<Vector3> OptimizePathSegment(List<Vector3> pathPoints, int startIndex, int endIndex)
    {
        if (endIndex - startIndex <= 1)
            return [pathPoints[startIndex], pathPoints[endIndex]];
            
        var result = new List<Vector3> { pathPoints[startIndex] };
        var currentIndex = startIndex;
        
        while (currentIndex < endIndex)
        {
            var farthestReachable = currentIndex + 1;
            
            // 查找当前点能直接到达的最远点
            for (var i = currentIndex + 2; i <= endIndex; i++)
            {
                if (CanReachDirectly(pathPoints[currentIndex], pathPoints[i]))
                    farthestReachable = i;
                else
                    break;
            }
            
            result.Add(pathPoints[farthestReachable]);
            currentIndex = farthestReachable;
        }
        
        return result;
    }
    
    private bool CanReachDirectly(Vector3 from, Vector3 to)
    {
        // 使用射线检测验证两点间是否可以直接连接
        var fromPoly = FindNearestMeshPoly(from);
        
        if (fromPoly == 0)
            return false;
            
        // 使用DtNavMeshQuery的射线检测
        var path = new List<long>();
        var result = MeshQuery.Raycast(fromPoly, from.SystemToRecast(), to.SystemToRecast(), filter, out var t, out _, ref path);
        
        // 如果射线检测成功且没有碰撞（t >= 1.0表示到达终点），则可以直接到达
        return result.Succeeded() && t >= 1.0f;
    }
    
    private static List<Vector3> ApplyCornerInterpolation(List<Vector3> pathPoints)
    {
        if (pathPoints.Count <= 2)
            return pathPoints;
            
        var result = new List<Vector3> { pathPoints[0] };
        
        for (var i = 1; i < pathPoints.Count - 1; i++)
        {
            var prevPoint = pathPoints[i - 1];
            var currentPoint = pathPoints[i];
            var nextPoint = pathPoints[i + 1];
            
            // 检测是否为拐角
            if (IsCorner(prevPoint, currentPoint, nextPoint))
            {
                // 在拐角处添加贝塞尔曲线插值点
                var interpolatedPoints = GenerateBezierInterpolation(prevPoint, currentPoint, nextPoint);
                result.AddRange(interpolatedPoints);
            }
            else
            {
                result.Add(currentPoint);
            }
        }
        
        result.Add(pathPoints[^1]);
        return result;
    }
    
    private static bool IsCorner(Vector3 prev, Vector3 current, Vector3 next)
    {
        // 计算两个向量的夹角
        var vec1 = Vector3.Normalize(current - prev);
        var vec2 = Vector3.Normalize(next - current);
        
        // 计算点积得到夹角余弦值
        var dotProduct = Vector3.Dot(vec1, vec2);
        
        // 如果夹角小于150度（余弦值大于-0.866），认为是拐角
        const float cornerThreshold = -0.866f; // cos(150°)
        return dotProduct > cornerThreshold;
    }
    
    private static List<Vector3> GenerateBezierInterpolation(Vector3 prev, Vector3 current, Vector3 next)
    {
        var result = new List<Vector3>();
        
        // 计算控制点
        var dir1 = Vector3.Normalize(current - prev);
        var dir2 = Vector3.Normalize(next - current);
        
        // 控制点距离为路径段长度的1/3
        var dist1 = (current - prev).Length() * 0.33f;
        var dist2 = (next - current).Length() * 0.33f;
        
        var controlPoint1 = current - dir1 * dist1;
        var controlPoint2 = current + dir2 * dist2;
        
        // 生成贝塞尔曲线上的点（3个插值点）
        const int interpolationPoints = 3;
        for (var i = 1; i <= interpolationPoints; i++)
        {
            var t = i / (float)(interpolationPoints + 1);
            var bezierPoint = CalculateBezierPoint(prev, controlPoint1, controlPoint2, next, t);
            result.Add(bezierPoint);
        }
        
        return result;
    }
    
    private static Vector3 CalculateBezierPoint(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // 三次贝塞尔曲线公式
        var u = 1 - t;
        var tt = t * t;
        var uu = u * u;
        var uuu = uu * u;
        var ttt = tt * t;
        
        return (uuu * p0) + (3 * uu * t * p1) + (3 * u * tt * p2) + (ttt * p3);
    }

    // returns 0 if not found, otherwise polygon ref
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5)
    {
        MeshQuery.FindNearestPoly(p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), filter, out var nearestRef, out _, out _);
        return nearestRef;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent)
    {
        IntersectQuery query = new();
        MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), filter, query);
        return query.Result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3? FindNearestPointOnMeshPoly(Vector3 p, long poly) =>
        MeshQuery.ClosestPointOnPoly(poly, p.SystemToRecast(), out var closest, out _).Succeeded() ? closest.RecastToSystem() : null;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) =>
        FindNearestPointOnMeshPoly(p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY));

    // finds the point on the mesh within specified x/z tolerance and with largest Y that is still smaller than p.Y
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3? FindPointOnFloor(Vector3 p, float halfExtentXZ = 5)
    {
        IEnumerable<long> polys = FindIntersectingMeshPolys(p, new(halfExtentXZ, 2048, halfExtentXZ));
        return polys.Select(poly => FindNearestPointOnMeshPoly(p, poly)).Where(pt => pt != null && pt.Value.Y <= p.Y).MaxBy(pt => pt!.Value.Y);
    }

    // returns VoxelMap.InvalidVoxel if not found, otherwise voxel index
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ulong FindNearestVolumeVoxel(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5) =>
        VolumeQuery != null ? VoxelSearch.FindNearestEmptyVoxel(VolumeQuery.Volume, p, new(halfExtentXZ, halfExtentY, halfExtentXZ)) : VoxelMap.InvalidVoxel;

    // collect all mesh polygons reachable from specified polygon
    public HashSet<long> FindReachableMeshPolys(long starting)
    {
        HashSet<long> result = [];
        if (starting == 0)
            return result;

        List<long> queue = [starting];
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
