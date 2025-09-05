using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Navmesh.NavVolume;
using Timer = Navmesh.Utilities.Timer;

namespace Navmesh;

public class NavmeshQuery
{
    public readonly  DtNavMeshQuery MeshQuery;
    public readonly  VoxelPathfind? VolumeQuery;
    private readonly IDtQueryFilter obstacleFilter;
    private readonly IDtQueryFilter defaultFilter = new DtQueryDefaultFilter();

    public  List<long> LastPath => lastPath;
    private List<long> lastPath = [];

    public NavmeshQuery(Navmesh navmesh)
    {
        MeshQuery      = new(navmesh.Mesh);
        obstacleFilter = new ObstacleAvoidanceFilter(navmesh.Mesh);
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

        var filter = Service.Config.MeshFilterType switch
        {
            1 => obstacleFilter,
            _ => defaultFilter
        };

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
            switch (Service.Config.PullStringType)
            {
                case 1:
                case 2:
                case 3:
                    var pathPoints = new List<Vector3>(lastPath.Count);
                    foreach (var t in lastPath)
                    {
                        var polyCenter = MeshQuery.GetAttachedNavMesh().GetPolyCenter(t);
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

                        var pulled = Service.Config.PullStringType switch
                        {
                            1 => ApplyMeshStringPulling(pathPoints, to),
                            3 => ApplyImprovedStringPulling(pathPoints, Service.Config.PullStringDefaultImprovedSafeMargin),
                            _ => ApplyAdaptiveStringPulling(pathPoints)
                        };
                        
                        return pulled;
                    }

                    break;
                default:
                    var straightPath = new List<DtStraightPath>();
                    var success      = MeshQuery.FindStraightPath(from.SystemToRecast(), endPos, lastPath, ref straightPath, 1024, 0);
                    if (success.Failed())
                        Service.Log.Error($"从 {from} ({startRef:X}) 到 {to} ({endRef:X}) 的路径查找失败：无法找到直线路径 ({success.Value:X})");
                    else
                    {
                        var processed = straightPath.Select(p => p.pos.RecastToSystem()).ToList();
                        processed.Add(endPos.RecastToSystem());
                        return processed;
                    }

                    break;
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
            res = ApplyVoxelStringPulling(voxelPath.Select(r => r.p).ToList(), to);
        else
        {
            res = voxelPath.Select(r => r.p).ToList();
            res.Add(to);
        }

        return res;
    }

    private List<Vector3> ApplyVoxelStringPulling(List<Vector3> pathPoints, Vector3 destination)
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

        List<Vector3> result = [pathPoints[0]];

        var optimizedPath = OptimizePathSegment(pathPoints, 0, pathPoints.Count - 1);
        for (var i = 1; i < optimizedPath.Count; i++)
            result.Add(optimizedPath[i]);

        if ((result[^1] - destination).LengthSquared() > 0.01f)
            result.Add(destination);

        result = ApplyCornerInterpolation(result);
        return result;
    }

    public List<Vector3> ApplyImprovedStringPulling(List<Vector3> waypoints, float safetyMargin = 0.5f)
    {
        if (waypoints.Count <= 2)
            return waypoints;

        var simplifiedInput  = SimplifyPath(waypoints, 0.1f);
        var funnelPath       = ApplyFunnelAlgorithmWithMargin(simplifiedInput, safetyMargin);
        var simplifiedFunnel = SimplifyPath(funnelPath, 0.05f);
        var smoothedPath     = ApplyLightweightSmoothing(simplifiedFunnel);

        return OptimizeFinalPath(smoothedPath);
    }
    
    public List<Vector3> ApplyAdaptiveStringPulling(List<Vector3> waypoints)
    {
        if (waypoints.Count <= 2)
            return waypoints;

        // 计算路径复杂度（基于转角数量和角度）
        var complexity = CalculatePathComplexity(waypoints);

        // 根据复杂度调整安全边距：复杂路径使用更大的安全边距
        var adaptiveMargin = Service.Config.PullStringDefaultImprovedBasedSafeMargin + ((0.8f - 0.2f) * complexity);

        return ApplyImprovedStringPulling(waypoints, adaptiveMargin);
    }
    
    private List<Vector3> ApplyFunnelAlgorithmWithMargin(List<Vector3> waypoints, float safetyMargin)
    {
        if (waypoints.Count <= 2)
            return waypoints;

        var result      = new List<Vector3> { waypoints[0] };
        var apex        = waypoints[0];
        var leftPortal  = waypoints[0];
        var rightPortal = waypoints[0];
        var leftIndex   = 0;
        var rightIndex  = 0;
        var lastAddedIndex = 0;

        for (var i = 1; i < waypoints.Count; i++)
        {
            // 跳过过于接近的点，减少计算量和中间点数量
            if (i - lastAddedIndex < 3 && i < waypoints.Count - 1 && 
                Vector3.Distance(waypoints[i], waypoints[lastAddedIndex]) < 2.0f)
                continue;

            // 计算当前点到路径的门户点（左右边界）
            var (left, right) = CalculatePortalPoints(waypoints, i, safetyMargin);

            // 检查右侧门户
            var apexIndex   = 0;
            if (TriArea2D(apex, rightPortal, right) <= 0.0f)
            {
                if (Vector3.Distance(apex, rightPortal) < 0.001f || TriArea2D(apex, leftPortal, right) > 0.0f)
                {
                    rightPortal = right;
                    rightIndex  = i;
                }
                else
                {
                    // 添加左侧顶点并重新开始
                    result.Add(leftPortal);
                    apex        = leftPortal;
                    apexIndex   = leftIndex;
                    leftPortal  = apex;
                    rightPortal = apex;
                    leftIndex   = apexIndex;
                    rightIndex  = apexIndex;
                    lastAddedIndex = i;
                    i           = apexIndex;
                    continue;
                }
            }

            // 检查左侧门户
            if (TriArea2D(apex, leftPortal, left) >= 0.0f)
            {
                if (Vector3.Distance(apex, leftPortal) < 0.001f || TriArea2D(apex, rightPortal, left) < 0.0f)
                {
                    leftPortal = left;
                    leftIndex  = i;
                }
                else
                {
                    // 添加右侧顶点并重新开始
                    result.Add(rightPortal);
                    apex        = rightPortal;
                    apexIndex   = rightIndex;
                    leftPortal  = apex;
                    rightPortal = apex;
                    leftIndex   = apexIndex;
                    rightIndex  = apexIndex;
                    lastAddedIndex = i;
                    i           = apexIndex;
                }
            }
        }

        result.Add(waypoints[^1]);
        return result;
    }

    private (Vector3 left, Vector3 right) CalculatePortalPoints(List<Vector3> waypoints, int index, float safetyMargin)
    {
        if (index >= waypoints.Count - 1)
            return (waypoints[index], waypoints[index]);

        var current = waypoints[index];
        var next    = waypoints[index + 1];

        // 检测周围环境复杂度，动态调整边距
        var adaptiveMargin = CalculateAdaptiveMargin(current, safetyMargin);
        
        // 计算垂直于路径方向的向量
        var direction     = Vector3.Normalize(next - current);
        var perpendicular = new Vector3(-direction.Z, direction.Y, direction.X);

        // 应用自适应安全边距
        var left  = current + (perpendicular * adaptiveMargin);
        var right = current - (perpendicular * adaptiveMargin);

        // 确保门户点在可行走区域内，如果不可行走则逐步缩小边距
        var attempts = 0;
        while ((!IsPointWalkable(left) || !IsPointWalkable(right)) && attempts < 3)
        {
            adaptiveMargin *= 0.6f;
            left = current + (perpendicular * adaptiveMargin);
            right = current - (perpendicular * adaptiveMargin);
            attempts++;
        }

        // 最后的安全检查
        if (!IsPointWalkable(left))
            left = current;
        if (!IsPointWalkable(right))
            right = current;

        return (left, right);
    }

    private float CalculateAdaptiveMargin(Vector3 point, float baseMargin)
    {
        // 检测周围8个方向的可行走性
        var walkableDirections = 0;
        var directions = new[]
        {
            new Vector3(1, 0, 0), new Vector3(-1, 0, 0),
            new Vector3(0, 0, 1), new Vector3(0, 0, -1),
            new Vector3(1, 0, 1), new Vector3(-1, 0, -1),
            new Vector3(1, 0, -1), new Vector3(-1, 0, 1)
        };

        foreach (var dir in directions)
        {
            var testPoint = point + (dir * baseMargin);
            if (IsPointWalkable(testPoint))
                walkableDirections++;
        }

        // 根据可行走方向数量调整边距
        var openness = walkableDirections / 8.0f;
        
        // 开阔区域使用较小边距，狭窄区域使用更小边距以避免抖动
        if (openness > 0.75f)
            return baseMargin * 0.6f; // 开阔区域
        else if (openness > 0.5f)
            return baseMargin * 0.4f; // 中等区域
        else
            return baseMargin * 0.2f; // 狭窄区域，最小边距避免抖动
    }

    private static float TriArea2D(Vector3 a, Vector3 b, Vector3 c) => 
        ((b.X - a.X) * (c.Z - a.Z)) - ((c.X - a.X) * (b.Z - a.Z));

    private List<Vector3> SimplifyPath(List<Vector3> waypoints, float tolerance)
    {
        if (waypoints.Count <= 2)
            return waypoints;

        var simplified = new List<Vector3> { waypoints[0] };
        var lastAdded = 0;

        for (var i = 1; i < waypoints.Count - 1; i++)
        {
            var distance = PointToLineDistance(waypoints[i], waypoints[lastAdded], waypoints[^1]);
            if (distance > tolerance || !CanReachDirectly(waypoints[lastAdded], waypoints[i + 1]))
            {
                simplified.Add(waypoints[i]);
                lastAdded = i;
            }
        }

        simplified.Add(waypoints[^1]);
        return simplified;
    }

    private static float PointToLineDistance(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        var line = lineEnd - lineStart;
        var lineLength = line.Length();
        if (lineLength < 0.001f)
            return Vector3.Distance(point, lineStart);

        var t = Math.Max(0, Math.Min(1, Vector3.Dot(point - lineStart, line) / (lineLength * lineLength)));
        var projection = lineStart + (line * t);
        return Vector3.Distance(point, projection);
    }

    private List<Vector3> ApplyLightweightSmoothing(List<Vector3> waypoints)
    {
        if (waypoints.Count <= 2)
            return waypoints;

        var smoothed = new List<Vector3> { waypoints[0] };

        for (var i = 1; i < waypoints.Count - 1; i++)
        {
            var prev = waypoints[i - 1];
            var current = waypoints[i];
            var next = waypoints[i + 1];

            // 检测是否为微小抖动
            if (DetectMicroZigzag(prev, current, next))
            {
                // 对微小抖动使用更强的平滑
                var smoothedPoint = (prev + next) / 2; // 直接连接前后点
                if (CanReachDirectly(prev, next) && IsPointWalkable(smoothedPoint))
                {
                    // 跳过当前点，直接连接
                    continue;
                }

                // 使用加权平滑
                smoothedPoint = (prev * 0.3f + current * 0.4f + next * 0.3f);
                smoothed.Add(CanReachDirectly(smoothed[^1], smoothedPoint) ? smoothedPoint : current);
            }
            else
            {
                // 计算转角角度
                var dir1 = Vector3.Normalize(current - prev);
                var dir2 = Vector3.Normalize(next - current);
                var dot = Vector3.Dot(dir1, dir2);
                var angle = (float)Math.Acos(Math.Clamp(dot, -1f, 1f)) * 180f / (float)Math.PI;

                // 对较大转角进行轻微平滑
                if (angle > 45f)
                {
                    var smoothedPoint = (prev + (current * 2) + next) / 4;
                    smoothed.Add(CanReachDirectly(smoothed[^1], smoothedPoint) ? smoothedPoint : current);
                }
                else
                {
                    smoothed.Add(current);
                }
            }
        }

        smoothed.Add(waypoints[^1]);
        return smoothed;
    }

    private static bool DetectMicroZigzag(Vector3 prev, Vector3 current, Vector3 next)
    {
        var dist1 = Vector3.Distance(prev, current);
        var dist2 = Vector3.Distance(current, next);
        var directDist = Vector3.Distance(prev, next);
        
        // 如果绕行距离比直线距离长很少，且距离很短，则认为是微小抖动
        var detourRatio = (dist1 + dist2) / Math.Max(directDist, 0.001f);
        var isShortSegment = Math.Max(dist1, dist2) < 1.0f;
        
        return detourRatio < 1.3f && isShortSegment;
    }

    private List<Vector3> OptimizeFinalPath(List<Vector3> waypoints)
    {
        if (waypoints.Count <= 2)
            return waypoints;

        var optimized = new List<Vector3> { waypoints[0] };
        var lastIndex = 0;

        for (var i = 1; i < waypoints.Count; i++)
        {
            // 尝试跳过中间点，直接连接到更远的点
            var canSkip = true;
            for (var j = lastIndex + 1; j < i; j++)
            {
                if (!CanReachDirectly(waypoints[lastIndex], waypoints[i]))
                {
                    canSkip = false;
                    break;
                }
            }

            if (!canSkip || i == waypoints.Count - 1)
            {
                if (i > lastIndex + 1)
                    optimized.Add(waypoints[i - 1]);
                optimized.Add(waypoints[i]);
                lastIndex = i;
            }
        }

        return optimized;
    }

    private bool IsPointWalkable(Vector3 point)
    {
        // 使用现有的射线检测方法
        return CanReachDirectly(point, point + new Vector3(0, 0.1f, 0));
    }
    
    private static float CalculatePathComplexity(List<Vector3> waypoints)
    {
        if (waypoints.Count <= 2)
            return 0f;

        var totalAngleChange = 0f;
        var cornerCount      = 0;

        for (var i = 1; i < waypoints.Count - 1; i++)
        {
            var prev    = waypoints[i - 1];
            var current = waypoints[i];
            var next    = waypoints[i + 1];

            var dir1 = Vector3.Normalize(current - prev);
            var dir2 = Vector3.Normalize(next    - current);

            var dot = Vector3.Dot(dir1, dir2);
            dot = Math.Clamp(dot, -1f, 1f);
            var angle = (float)Math.Acos(dot) * 180f / (float)Math.PI;

            if (angle > 15f) // 认为大于15度的转角为有效转角
            {
                totalAngleChange += angle;
                cornerCount++;
            }
        }

        // 归一化复杂度值到 [0, 1] 范围
        var avgAngle = cornerCount > 0 ? totalAngleChange / cornerCount : 0f;
        return Math.Clamp(avgAngle / 90f, 0f, 1f); // 90度作为最大角度参考
    }

    private List<Vector3> OptimizePathSegment(List<Vector3> pathPoints, int startIndex, int endIndex)
    {
        if (endIndex - startIndex <= 1)
            return [pathPoints[startIndex], pathPoints[endIndex]];

        var result       = new List<Vector3> { pathPoints[startIndex] };
        var currentIndex = startIndex;

        while (currentIndex < endIndex)
        {
            var farthestReachable = currentIndex + 1;

            // 查找当前点能直接到达的最远点
            for (var i = currentIndex + 2; i <= endIndex; i++)
                if (CanReachDirectly(pathPoints[currentIndex], pathPoints[i]))
                    farthestReachable = i;
                else
                    break;

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
        var path   = new List<long>();
        var result = MeshQuery.Raycast(fromPoly, from.SystemToRecast(), to.SystemToRecast(), obstacleFilter, out var t, out _, ref path);

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
            var prevPoint    = pathPoints[i - 1];
            var currentPoint = pathPoints[i];
            var nextPoint    = pathPoints[i + 1];

            // 检测是否为拐角
            if (IsCorner(prevPoint, currentPoint, nextPoint))
            {
                // 在拐角处添加贝塞尔曲线插值点
                var interpolatedPoints = GenerateBezierInterpolation(prevPoint, currentPoint, nextPoint);
                result.AddRange(interpolatedPoints);
            }
            else
                result.Add(currentPoint);
        }

        result.Add(pathPoints[^1]);
        return result;
    }

    /// <summary>
    /// 使用线性插值在拐点周围生成平滑点
    /// </summary>
    private static List<Vector3> GenerateLinearInterpolation(Vector3 prev, Vector3 current, Vector3 next, int points)
    {
        var result = new List<Vector3>();
        
        // 计算拐点的入向量和出向量
        var incoming = Vector3.Normalize(current - prev);
        var outgoing = Vector3.Normalize(next - current);
        
        // 计算平滑区域的起点和终点（在当前点前后各一段距离）
        var segmentLength1 = (current - prev).Length();
        var segmentLength2 = (next - current).Length();
        
        var smoothDistance1 = Math.Min(segmentLength1 * 0.3f, 2.0f);
        var smoothDistance2 = Math.Min(segmentLength2 * 0.3f, 2.0f);
        
        var startPoint = current - incoming * smoothDistance1;
        var endPoint = current + outgoing * smoothDistance2;
        
        // 在平滑区域内生成插值点
        for (var i = 1; i <= points; i++)
        {
            var t = i / (float)(points + 1);
            
            // 使用简单的二次插值（类似圆弧）
            var linearPoint = Vector3.Lerp(startPoint, endPoint, t);
            var curveOffset = Vector3.Lerp(Vector3.Zero, current - linearPoint, 0.5f * (1.0f - Math.Abs(2.0f * t - 1.0f)));
            
            var smoothedPoint = linearPoint + curveOffset;
            result.Add(smoothedPoint);
        }
        
        return result;
    }

    private static bool IsCorner(Vector3 prev, Vector3 current, Vector3 next)
    {
        // 计算两个向量的夹角
        var vec1 = Vector3.Normalize(current - prev);
        var vec2 = Vector3.Normalize(next    - current);

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
        var dir2 = Vector3.Normalize(next    - current);

        // 控制点距离为路径段长度的1/3
        var dist1 = (current - prev).Length()    * 0.33f;
        var dist2 = (next    - current).Length() * 0.33f;

        var controlPoint1 = current - (dir1 * dist1);
        var controlPoint2 = current + (dir2 * dist2);

        // 生成贝塞尔曲线上的点（3个插值点）
        const int interpolationPoints = 3;
        for (var i = 1; i <= interpolationPoints; i++)
        {
            var t           = i / (float)(interpolationPoints + 1);
            var bezierPoint = CalculateBezierPoint(prev, controlPoint1, controlPoint2, next, t);
            result.Add(bezierPoint);
        }

        return result;
    }

    private static Vector3 CalculateBezierPoint(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // 三次贝塞尔曲线公式
        var u   = 1 - t;
        var tt  = t  * t;
        var uu  = u  * u;
        var uuu = uu * u;
        var ttt = tt * t;

        return (uuu * p0) + (3 * uu * t * p1) + (3 * u * tt * p2) + (ttt * p3);
    }

    // returns 0 if not found, otherwise polygon ref
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public long FindNearestMeshPoly(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5)
    {
        MeshQuery.FindNearestPoly(p.SystemToRecast(), new(halfExtentXZ, halfExtentY, halfExtentXZ), obstacleFilter, out var nearestRef, out _, out _);
        return nearestRef;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public List<long> FindIntersectingMeshPolys(Vector3 p, Vector3 halfExtent)
    {
        IntersectQuery query = new();
        MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), obstacleFilter, query);
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
}
