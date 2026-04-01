using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Navmesh.NavVolume;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading;
using System;

namespace Navmesh;

public class NavmeshQuery
{
    private const float StartPolyCandidateHalfExtentXZ = 5.0f;
    private const float StartPolyCandidateHalfExtentY = 6.0f;
    private const float StartPolyCandidateMaxHorizontalDistance = 4.0f;
    private const float StartPolyCandidateMaxVerticalDistance = 3.0f;
    private const float StartPolyCandidateAboveTolerance = 0.75f;
    private const int MaxStartPolyCandidatesToEvaluate = 8;
    private const float ShortGapRepairSearchHalfExtentXZ = 4.0f;
    private const float ShortGapRepairSearchHalfExtentY = 2.5f;
    private const float ShortGapRepairMaxBridgeDistance = 3.5f;
    private const float ShortGapRepairMaxVerticalDelta = 1.0f;
    private const float SuspectedTileSeamGapMaxDistance = 3.0f;
    private const float SuspectedTileSeamBoundaryMaxDistance = 1.0f;

    private readonly record struct TileCoord(int X, int Z)
    {
        public override string ToString() => $"{X}x{Z}";
    }

    private readonly record struct SeamDiagnostic(TileCoord StartTile, TileCoord RequestedTile, TileCoord FinalTile, float DistanceToNearestBoundary, bool IsNearTileBoundary, bool IsNearbyTile, bool IsShortGap)
    {
        public bool IsSuspectedTileSeamCutoff => IsNearTileBoundary && IsNearbyTile && IsShortGap;
    }

    private readonly record struct MeshPolyCandidate(long PolyRef, Vector3 ProjectedPoint, float HorizontalDistanceSq, float VerticalDelta)
    {
        public float VerticalDistanceAbs => MathF.Abs(VerticalDelta);
        public bool IsTooFarAbove => VerticalDelta > StartPolyCandidateAboveTolerance;
    }

    private readonly record struct MeshPathCandidate(MeshPolyCandidate StartCandidate, DtStatus QueryStatus, PathfindStatus ResultStatus, Vector3 FinalDestination, List<long> Corridor)
    {
        public long StartRef => StartCandidate.PolyRef;
        public Vector3 StartPoint => StartCandidate.ProjectedPoint;
        public long LastPoly => Corridor.Count > 0 ? Corridor[^1] : 0;
        public float DistanceToRequestedTargetSq(Vector3 requestedTarget) => Vector3.DistanceSquared(FinalDestination, requestedTarget);
    }

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
        var requestedStartRef = FindNearestMeshPoly(from);
        var endRef = FindNearestMeshPoly(to);
        if (requestedStartRef == 0 || endRef == 0)
        {
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法在导航网格上找到起点或终点多边形");
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
        var requestedEndPos = to.SystemToRecast();
        var bestStartCandidate = ResolveBestStartPathCandidate(from, to, requestedStartRef, endRef, requestedEndPos, filter, opt, range, cancel);
        if (bestStartCandidate == null)
        {
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");
        }

        var pathCandidate = bestStartCandidate.Value;
        var startRef = pathCandidate.StartRef;
        var startPos = pathCandidate.StartPoint.SystemToRecast();
        _lastPath.AddRange(pathCandidate.Corridor);
        var pathStatus = pathCandidate.QueryStatus;
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}（原始起点候选 = {requestedStartRef:X}）");
        Service.Log.Debug($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {pathStatus}，路径 = {string.Join(", ", _lastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();

        var lastPoly = pathCandidate.LastPoly;
        var resultStatus = pathCandidate.ResultStatus;
        var finalEndPos = pathCandidate.FinalDestination.SystemToRecast();

        if (resultStatus == PathfindStatus.Partial && TryRepairShortGroundGap(pathCandidate, to, endRef, requestedEndPos, filter, opt, range, useStringPulling, cancel, out var repairedResult))
        {
            LogMeshResult(repairedResult, from, startRef, endRef, lastPoly, range, timer.Value());
            return repairedResult;
        }

        var waypoints = BuildMeshWaypoints(startPos, _lastPath, finalEndPos, useStringPulling);
        if (waypoints == null)
            return LogMeshFailure(from, to, startRef, endRef, lastPoly, range, "路径点生成失败");

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

    private void LogMeshResult(PathfindResult result, Vector3 from, long startRef, long endRef, long lastPoly, float range, TimeSpan duration)
    {
        var diagnostic = BuildSeamDiagnostic(from, result.RequestedDestination, result.FinalDestination);
        var message = $"地面算路完成：状态 = {result.Status}，起点 = {from:f3}，请求终点 = {result.RequestedDestination:f3}，实际终点 = {result.FinalDestination:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPoly:X}，容差 = {range:f3}，耗时 = {duration.TotalSeconds:f3} 秒，路径点 = {result.Waypoints.Count}";
        if (result.Status == PathfindStatus.Partial)
            message += $"，起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}";

        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Service.Log.Warning(message);
                if (diagnostic.IsSuspectedTileSeamCutoff)
                    Service.Log.Warning($"[SuspectedTileSeamCutoff] 疑似区块接缝截断：起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}");
                break;
            case PathfindStatus.Failed:
                Service.Log.Error(message);
                break;
            default:
                Service.Log.Debug(message);
                break;
        }
    }

    private MeshPathCandidate? ResolveBestStartPathCandidate(Vector3 from, Vector3 to, long requestedStartRef, long endRef, RcVec3f requestedEndPos, IDtQueryFilter filter, DtFindPathOption opt, float range, CancellationToken cancel)
    {
        var candidates = CollectStartPolyCandidates(from, requestedStartRef);
        if (candidates.Count == 0)
            return null;

        MeshPathCandidate? best = null;
        List<string> candidateLogs = [];

        foreach (var candidate in candidates.Take(MaxStartPolyCandidatesToEvaluate))
        {
            cancel.ThrowIfCancellationRequested();

            var corridor = new List<long>();
            var status = MeshQuery.FindPath(candidate.PolyRef, endRef, candidate.ProjectedPoint.SystemToRecast(), requestedEndPos, filter, ref corridor, opt);
            if (status.Failed() || corridor.Count == 0)
            {
                candidateLogs.Add($"{candidate.PolyRef:X}: 失败 ({status})");
                continue;
            }

            var pathCandidate = BuildMeshPathCandidate(candidate, corridor, status, requestedEndPos, to, endRef, range);
            if (pathCandidate.ResultStatus == PathfindStatus.Failed)
            {
                candidateLogs.Add($"{candidate.PolyRef:X}: 失败（无法投影最终可达点）");
                continue;
            }
            candidateLogs.Add($"{candidate.PolyRef:X}: {pathCandidate.ResultStatus}，距目标 {MathF.Sqrt(pathCandidate.DistanceToRequestedTargetSq(to)):f3}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}");

            if (best == null || IsBetterStartPathCandidate(pathCandidate, best.Value, to))
                best = pathCandidate;

            if (pathCandidate.ResultStatus == PathfindStatus.Complete)
                break;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] 起点候选评估：{string.Join(" | ", candidateLogs)}");

        if (best is { } selected)
        {
            Service.Log.Debug($"[算路] 已选起点多边形 {selected.StartRef:X}，投影点 = {selected.StartPoint:f3}，结果 = {selected.ResultStatus}");
            if (selected.StartRef != requestedStartRef)
                Service.Log.Warning($"[算路] 已修正起点多边形：原始候选 = {requestedStartRef:X}，选中 = {selected.StartRef:X}");
        }

        return best;
    }

    private List<MeshPolyCandidate> CollectStartPolyCandidates(Vector3 from, long requestedStartRef)
    {
        HashSet<long> seen = [];
        List<MeshPolyCandidate> candidates = [];

        foreach (var poly in FindIntersectingMeshPolys(from, new(StartPolyCandidateHalfExtentXZ, StartPolyCandidateHalfExtentY, StartPolyCandidateHalfExtentXZ)))
            TryAddCandidate(poly, false);

        TryAddCandidate(requestedStartRef, true);

        candidates.Sort(static (a, b) =>
        {
            var bucketA = a.IsTooFarAbove ? 1 : 0;
            var bucketB = b.IsTooFarAbove ? 1 : 0;
            var cmp = bucketA.CompareTo(bucketB);
            if (cmp != 0)
                return cmp;

            cmp = a.VerticalDistanceAbs.CompareTo(b.VerticalDistanceAbs);
            if (cmp != 0)
                return cmp;

            cmp = a.HorizontalDistanceSq.CompareTo(b.HorizontalDistanceSq);
            if (cmp != 0)
                return cmp;

            return a.PolyRef.CompareTo(b.PolyRef);
        });

        return candidates;

        void TryAddCandidate(long poly, bool forceInclude)
        {
            if (poly == 0 || !seen.Add(poly))
                return;

            var projected = FindNearestPointOnMeshPoly(from, poly);
            if (projected == null)
                return;

            var point = projected.Value;
            var dx = point.X - from.X;
            var dz = point.Z - from.Z;
            var horizontalDistanceSq = dx * dx + dz * dz;
            var verticalDistance = point.Y - from.Y;
            if (!forceInclude)
            {
                if (horizontalDistanceSq > StartPolyCandidateMaxHorizontalDistance * StartPolyCandidateMaxHorizontalDistance)
                    return;
                if (MathF.Abs(verticalDistance) > StartPolyCandidateMaxVerticalDistance)
                    return;
            }

            candidates.Add(new(poly, point, horizontalDistanceSq, verticalDistance));
        }
    }

    private MeshPathCandidate BuildMeshPathCandidate(MeshPolyCandidate startCandidate, List<long> corridor, DtStatus status, RcVec3f requestedEndPos, Vector3 requestedTarget, long endRef, float range)
    {
        var lastPoly = corridor[^1];
        var resultStatus = PathfindStatus.Complete;
        Vector3 finalDestination;
        if (status.IsPartial() || lastPoly != endRef)
        {
            var closestStatus = MeshQuery.ClosestPointOnPoly(lastPoly, requestedEndPos, out var projectedEndPos, out _);
            if (closestStatus.Failed())
                return new(startCandidate, status, PathfindStatus.Failed, startCandidate.ProjectedPoint, corridor);

            finalDestination = projectedEndPos.RecastToSystem();

            resultStatus = range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }
        else
        {
            finalDestination = requestedTarget;
        }

        return new(startCandidate, status, resultStatus, finalDestination, corridor);
    }

    private static bool IsBetterStartPathCandidate(MeshPathCandidate candidate, MeshPathCandidate currentBest, Vector3 requestedTarget)
    {
        var rankCandidate = ResultStatusRank(candidate.ResultStatus);
        var rankCurrent = ResultStatusRank(currentBest.ResultStatus);
        if (rankCandidate != rankCurrent)
            return rankCandidate < rankCurrent;

        var candidateDistance = candidate.DistanceToRequestedTargetSq(requestedTarget);
        var currentDistance = currentBest.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(candidateDistance, currentDistance))
            return candidateDistance < currentDistance;

        if (!NearlyEqual(candidate.StartCandidate.VerticalDistanceAbs, currentBest.StartCandidate.VerticalDistanceAbs))
            return candidate.StartCandidate.VerticalDistanceAbs < currentBest.StartCandidate.VerticalDistanceAbs;

        if (!NearlyEqual(candidate.StartCandidate.HorizontalDistanceSq, currentBest.StartCandidate.HorizontalDistanceSq))
            return candidate.StartCandidate.HorizontalDistanceSq < currentBest.StartCandidate.HorizontalDistanceSq;

        return candidate.StartRef < currentBest.StartRef;
    }

    internal (int x, int z) FindMeshTile(Vector3 position)
    {
        MeshQuery.GetAttachedNavMesh().CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        return (tileX, tileZ);
    }

    internal (Vector3 min, Vector3 max) GetMeshTileBounds(int tileX, int tileZ)
    {
        ref readonly var param = ref MeshQuery.GetAttachedNavMesh().GetParams();
        var min = new Vector3(param.orig.X + tileX * param.tileWidth, param.orig.Y, param.orig.Z + tileZ * param.tileHeight);
        var max = new Vector3(min.X + param.tileWidth, min.Y, min.Z + param.tileHeight);
        return (min, max);
    }

    private List<Vector3>? BuildMeshWaypoints(RcVec3f startPos, List<long> corridor, RcVec3f finalEndPos, bool useStringPulling)
    {
        if (useStringPulling)
        {
            var straightPath = new List<DtStraightPath>();
            var straightStatus = MeshQuery.FindStraightPath(startPos, finalEndPos, corridor, ref straightPath, 1024, 0);
            if (straightStatus.Failed())
                return null;

            return DeduplicateWaypoints(straightPath.Select(p => p.pos.RecastToSystem()));
        }

        return DeduplicateWaypoints(corridor.Select(r => MeshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(finalEndPos.RecastToSystem()));
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

    private bool TryRepairShortGroundGap(MeshPathCandidate partialCandidate, Vector3 requestedTarget, long endRef, RcVec3f requestedEndPos, IDtQueryFilter filter, DtFindPathOption opt, float range, bool useStringPulling, CancellationToken cancel, out PathfindResult repairedResult)
    {
        repairedResult = default;

        var partialEnd = partialCandidate.FinalDestination;
        var repairCandidates = FindIntersectingMeshPolys(partialEnd, new(ShortGapRepairSearchHalfExtentXZ, ShortGapRepairSearchHalfExtentY, ShortGapRepairSearchHalfExtentXZ));
        MeshPathCandidate? bestResumeCandidate = null;
        List<string> candidateLogs = [];

        foreach (var poly in repairCandidates)
        {
            if (poly == 0 || poly == partialCandidate.LastPoly || partialCandidate.Corridor.Contains(poly))
                continue;

            cancel.ThrowIfCancellationRequested();

            var projectedPoint = FindNearestPointOnMeshPoly(partialEnd, poly);
            if (projectedPoint == null)
                continue;

            var bridgePoint = projectedPoint.Value;
            var bridgeDelta = bridgePoint - partialEnd;
            var bridgeHorizontalDistance = new Vector2(bridgeDelta.X, bridgeDelta.Z).Length();
            var bridgeVerticalDistance = MathF.Abs(bridgeDelta.Y);
            if (bridgeHorizontalDistance > ShortGapRepairMaxBridgeDistance || bridgeVerticalDistance > ShortGapRepairMaxVerticalDelta)
                continue;

            var corridor = new List<long>();
            var status = MeshQuery.FindPath(poly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, ref corridor, opt);
            if (status.Failed() || corridor.Count == 0)
            {
                candidateLogs.Add($"{poly:X}: 失败 ({status})");
                continue;
            }

            var resumeCandidate = BuildMeshPathCandidate(new(poly, bridgePoint, bridgeHorizontalDistance * bridgeHorizontalDistance, bridgePoint.Y - partialEnd.Y), corridor, status, requestedEndPos, requestedTarget, endRef, range);
            if (resumeCandidate.ResultStatus == PathfindStatus.Failed)
            {
                candidateLogs.Add($"{poly:X}: 失败（无法投影终点）");
                continue;
            }

            candidateLogs.Add($"{poly:X}: {resumeCandidate.ResultStatus}，桥接距离 {bridgeHorizontalDistance:f3}，高差 {bridgeVerticalDistance:f3}");
            if (bestResumeCandidate == null || IsBetterStartPathCandidate(resumeCandidate, bestResumeCandidate.Value, requestedTarget))
                bestResumeCandidate = resumeCandidate;

            if (resumeCandidate.ResultStatus == PathfindStatus.Complete)
                break;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] Partial 终点附近候选评估：{string.Join(" | ", candidateLogs)}");

        if (bestResumeCandidate == null)
            return false;

        var partialWaypoints = BuildMeshWaypoints(partialCandidate.StartPoint.SystemToRecast(), partialCandidate.Corridor, partialCandidate.FinalDestination.SystemToRecast(), useStringPulling);
        var resumedWaypoints = BuildMeshWaypoints(bestResumeCandidate.Value.StartPoint.SystemToRecast(), bestResumeCandidate.Value.Corridor, bestResumeCandidate.Value.FinalDestination.SystemToRecast(), useStringPulling);
        if (partialWaypoints == null || resumedWaypoints == null)
            return false;

        var bridgePointToAdd = bestResumeCandidate.Value.StartPoint;
        var mergedWaypoints = DeduplicateWaypoints(partialWaypoints.Concat([bridgePointToAdd]).Concat(resumedWaypoints));
        if (mergedWaypoints.Count == 0)
            return false;

        Service.Log.Warning($"[算路] 已触发短距补桥：partial 终点 = {partialCandidate.FinalDestination:f3}，桥接点 = {bridgePointToAdd:f3}，桥接后结果 = {bestResumeCandidate.Value.ResultStatus}");
        repairedResult = new(bestResumeCandidate.Value.ResultStatus, mergedWaypoints, requestedTarget, mergedWaypoints[^1]);
        return true;
    }

    private SeamDiagnostic BuildSeamDiagnostic(Vector3 from, Vector3 requested, Vector3 actual)
    {
        var startTile = ToTileCoord(FindMeshTile(from));
        var requestedTile = ToTileCoord(FindMeshTile(requested));
        var actualTile = ToTileCoord(FindMeshTile(actual));
        var gap = Vector3.Distance(requested, actual);
        var boundaryDistance = DistanceToNearestTileBoundary(actual);
        var isNearbyTile = Math.Abs(requestedTile.X - actualTile.X) <= 1 && Math.Abs(requestedTile.Z - actualTile.Z) <= 1;
        return new(startTile, requestedTile, actualTile, boundaryDistance, boundaryDistance <= SuspectedTileSeamBoundaryMaxDistance, isNearbyTile, gap <= SuspectedTileSeamGapMaxDistance);
    }

    private float DistanceToNearestTileBoundary(Vector3 position)
    {
        var (tileX, tileZ) = FindMeshTile(position);
        var (tileMin, tileMax) = GetMeshTileBounds(tileX, tileZ);
        var distMinX = MathF.Abs(position.X - tileMin.X);
        var distMaxX = MathF.Abs(tileMax.X - position.X);
        var distMinZ = MathF.Abs(position.Z - tileMin.Z);
        var distMaxZ = MathF.Abs(tileMax.Z - position.Z);
        return MathF.Min(MathF.Min(distMinX, distMaxX), MathF.Min(distMinZ, distMaxZ));
    }

    private static int ResultStatusRank(PathfindStatus status) => status switch
    {
        PathfindStatus.Complete => 0,
        PathfindStatus.ReachedWithinRange => 1,
        PathfindStatus.Partial => 2,
        _ => 3
    };

    private static bool NearlyEqual(float left, float right) => MathF.Abs(left - right) <= 0.0001f;

    private static TileCoord ToTileCoord((int x, int z) tile) => new(tile.x, tile.z);

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
