using System.Numerics;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Common.Utilities;
using vnavmesh.Configuration;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume.Pathfinding;

namespace vnavmesh.Navigation.Mesh.Query;

using static DtDetour;

public class NavmeshQuery
{
    private readonly Config _config;
    private const    float  StartPolyCandidateHalfExtentXZ                  = 5.0f;
    private const    float  StartPolyCandidateHalfExtentY                   = 6.0f;
    private const    float  StartPolyCandidateMaxHorizontalDistance         = 4.0f;
    private const    float  StartPolyCandidateMaxVerticalDistance           = 3.0f;
    private const    float  StartPolyCandidateAboveTolerance                = 0.75f;
    private const    float  StartSupportProbeRadius                         = 0.35f;
    private const    int    StartSupportProbeCount                          = 8;
    private const    float  StartSupportMatchDistance                       = 0.20f;
    private const    int    MaxStartPolyCandidatesToEvaluate                = 8;
    private const    float  ExpandedStartPolyCandidateHalfExtentXZ          = 8.0f;
    private const    float  ExpandedStartPolyCandidateHalfExtentY           = 16.0f;
    private const    float  ExpandedStartPolyCandidateMaxHorizontalDistance = 8.0f;
    private const    float  ExpandedStartPolyCandidateMaxVerticalDistance   = 12.0f;
    private const    int    ExpandedMaxStartPolyCandidatesToEvaluate        = 24;
    private const    float  EndPolyCandidateHalfExtentXZ                    = 5.0f;
    private const    float  EndPolyCandidateHalfExtentY                     = 8.0f;
    private const    float  EndPolyCandidateMaxHorizontalDistance           = 4.0f;
    private const    float  EndPolyCandidateMaxVerticalDistance             = 8.0f;
    private const    float  EndPolyCandidateAboveTolerance                  = 0.25f;
    private const    float  ShortGapRepairSearchHalfExtentXZ                = 4.0f;
    private const    float  ShortGapRepairSearchHalfExtentY                 = 2.5f;
    private const    float  ShortGapRepairMaxBridgeDistance                 = 3.5f;
    private const    float  ShortGapRepairMaxVerticalDelta                  = 1.0f;
    private const    float  SuspectedTileSeamGapMaxDistance                 = 3.0f;
    private const    float  SuspectedTileSeamBoundaryMaxDistance            = 1.0f;
    private const    float  VolumeBoundsClampEpsilon                        = 0.1f;
    private const    int    MaxPathPolys                                    = 4096;

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
        List<long>        Corridor,
        GroundQueryMode   QueryMode,
        float             PathLength,
        int               WeightedLinkPenalty,
        int               OffMeshTransitionCount,
        int               AreaCrossingCount
    )
    {
        public long    StartRef         => StartCandidate.PolyRef;
        public Vector3 StartPoint       => StartCandidate.ProjectedPoint;
        public long    LastPoly         => Corridor.Count > 0 ? Corridor[^1] : 0;
        public bool    IsRequestedStart => StartCandidate.IsRequestedStart;
        public bool    IsPointOverPoly  => StartCandidate.IsPointOverPoly;
        public int     SupportProbeHits => StartCandidate.SupportProbeHits;

        public float DistanceToRequestedTargetSq(Vector3 requestedTarget) => Vector3.DistanceSquared(FinalDestination, requestedTarget);
    }

    private enum GroundQueryMode
    {
        AnyAngle,
        Classic
    }

    public sealed class GroundPathDiagnosticsSnapshot
    {
        public required long GroundQueries               { get; init; }
        public required long PartialQueries              { get; init; }
        public required long SuspectedTileSeamCutoffs    { get; init; }
        public required long AnyAnglePreferred           { get; init; }
        public required long ClassicFallbacks            { get; init; }
        public required long StartReplacements           { get; init; }
        public required long EndReplacements             { get; init; }
        public required long GeneratedClimbLinksAccepted { get; init; }
        public required long GeneratedJumpLinksAccepted  { get; init; }
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

    public class GroundAreaCostFilter
    (
        bool excludeUnreachable = true
    ) : IDtQueryFilter
    {
        private readonly DtQueryDefaultFilter _filter = new
            ((int)NavmeshPolyFlags.AllTraversable, excludeUnreachable ? (int)NavmeshPolyFlags.Unreachable : 0, CreateAreaCosts());

        private static float[] CreateAreaCosts()
        {
            var costs = new float[DT_MAX_AREAS];
            Array.Fill(costs, 1f);
            costs[(int)NavmeshArea.Null]               = float.MaxValue;
            costs[(int)NavmeshArea.Ground]             = 1.0f;
            costs[(int)NavmeshArea.GeneratedClimbDown] = 1.8f;
            costs[(int)NavmeshArea.GeneratedEdgeJump]  = 2.6f;
            costs[(int)NavmeshArea.ManualOffMesh]      = 1.35f;
            costs[(int)NavmeshArea.Teleport]           = 1.15f;
            costs[(int)NavmeshArea.ClientPath]         = 3.0f;
            return costs;
        }

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
            => _filter.GetCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef, nextTile, nextPoly);

        public bool PassFilter(long refs, DtMeshTile tile, DtPoly poly) => _filter.PassFilter(refs, tile, poly);
    }

    private readonly PathPostprocessor    _postprocessor;
    private readonly Navmesh              _navmesh;
    private readonly IDtQueryFilter       _filter                          = new DtQueryDefaultFilter();
    private readonly GroundAreaCostFilter _groundFilter                    = new();
    private readonly GroundAreaCostFilter _groundFilterIgnoringUnreachable = new(false);
    private readonly IDtQueryFilter       _reachableFilter;
    private          DtNavMeshQuery?      _meshQuery;
    private          VoxelPathfind?       _volumeQuery;
    private          bool                 _released;

    private long _groundQueryCount;
    private long _partialGroundQueryCount;
    private long _suspectedTileSeamCutoffCount;
    private long _anyAnglePreferredCount;
    private long _classicFallbackCount;
    private long _startReplacementCount;
    private long _endReplacementCount;

    internal IDtQueryFilter GroundFilter => _groundFilter;

    public DtNavMeshQuery MeshQuery
    {
        get
        {
            if (_released)
                throw new ObjectDisposedException(nameof(NavmeshQuery));

            var existing = Volatile.Read(ref _meshQuery);
            if (existing != null)
                return existing;

            var created = new DtNavMeshQuery(_navmesh.Mesh);
            return Interlocked.CompareExchange(ref _meshQuery, created, null) ?? created;
        }
    }

    public VoxelPathfind? VolumeQuery => _released ? null : _volumeQuery ??= _navmesh.Volume != null ? new(_navmesh.Volume, _config) : null;

    public List<long> LastPath { get; } = [];

    public NavmeshQuery(Navmesh navmesh, Config config)
    {
        _navmesh         = navmesh;
        _config          = config;
        _postprocessor   = new(() => MeshQuery);
        _reachableFilter = _groundFilter;
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
        var capacity = 256;

        while (true)
        {
            var refs  = new long[capacity];
            var query = new DtCollectPolysQuery(refs, refs.Length);
            MeshQuery.QueryPolygons(p.SystemToRecast(), halfExtent.SystemToRecast(), allowUnreachable ? _filter : _reachableFilter, query);
            if (!query.Overflowed())
                return [.. refs.AsSpan(0, query.NumCollected()).ToArray()];

            capacity *= 2;
        }
    }

    public Vector3? FindNearestPointOnMeshPoly
        (Vector3 p, long poly) => TryClosestPointOnPolyWithFlags(p, poly, out var closest, out _) ? closest : null;

    public Vector3? FindNearestPointOnMesh(Vector3 p, float halfExtentXZ = 5, float halfExtentY = 5, bool allowUnreachable = true) => FindNearestPointOnMeshPoly
        (p, FindNearestMeshPoly(p, halfExtentXZ, halfExtentY, allowUnreachable));

    public Vector3? FindRandomPointOnMeshAroundCircle(Vector3 center, float maxRadius, bool allowUnreachable = true)
    {
        if (maxRadius <= 0)
            return null;

        var filter   = allowUnreachable ? _filter : _reachableFilter;
        var startRef = FindNearestMeshPoly(center, 8, 8, allowUnreachable);
        if (startRef == 0)
            return null;

        var status = MeshQuery.FindRandomPointWithinCircle
            (startRef, center.SystemToRecast(), maxRadius, filter, new RcRand(Random.Shared.NextInt64()), out _, out var point);
        return status.Succeeded() ? point.RecastToSystem() : null;
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
            return VoxelMap.INVALID_VOXEL;

        var volume     = VolumeQuery.Volume;
        var halfExtent = new Vector3(halfExtentXZ, halfExtentY, halfExtentXZ);
        var voxel      = VoxelSearch.FindNearestEmptyVoxel(volume, p, halfExtent);
        if (voxel != VoxelMap.INVALID_VOXEL)
            return voxel;

        var boundsMin = volume.RootTile.BoundsMin + new Vector3(VolumeBoundsClampEpsilon);
        var boundsMax = volume.RootTile.BoundsMax - new Vector3(VolumeBoundsClampEpsilon);
        var clamped   = Vector3.Clamp(p, boundsMin, boundsMax);
        var usedClamp = Vector3.DistanceSquared(clamped, p) > 0.000001f;

        if (usedClamp)
        {
            voxel = VoxelSearch.FindNearestEmptyVoxel(volume, clamped, halfExtent);

            if (voxel != VoxelMap.INVALID_VOXEL)
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
            if (voxel == VoxelMap.INVALID_VOXEL)
                continue;

            Service.Log.Debug
            (
                $"[算路] 体素定位触发扩搜：原始位置 = {p:f3}，搜索中心 = {(usedClamp ? clamped : p):f3}，搜索范围 = {expandedHalfExtent:f3}"
            );
            return voxel;
        }

        return VoxelMap.INVALID_VOXEL;
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

            for (var i = nextPoly.firstLink; i != DT_NULL_LINK; i = nextTile.links[i].next)
            {
                var neighbourRef = nextTile.links[i].refs;
                if (neighbourRef != 0)
                    queue.Add(neighbourRef);
            }
        }

        return result;
    }

    public GroundPathDiagnosticsSnapshot GetGroundDiagnostics() =>
        new()
        {
            GroundQueries               = Interlocked.Read(ref _groundQueryCount),
            PartialQueries              = Interlocked.Read(ref _partialGroundQueryCount),
            SuspectedTileSeamCutoffs    = Interlocked.Read(ref _suspectedTileSeamCutoffCount),
            AnyAnglePreferred           = Interlocked.Read(ref _anyAnglePreferredCount),
            ClassicFallbacks            = Interlocked.Read(ref _classicFallbackCount),
            StartReplacements           = Interlocked.Read(ref _startReplacementCount),
            EndReplacements             = Interlocked.Read(ref _endReplacementCount),
            GeneratedClimbLinksAccepted = _navmesh.GeneratedClimbDownLinkCount,
            GeneratedJumpLinksAccepted  = _navmesh.GeneratedEdgeJumpLinkCount
        };

    internal void ReleaseRetainedState()
    {
        LastPath.Clear();
        LastPath.TrimExcess();
        _meshQuery = null;
        _volumeQuery?.ReleaseRetainedState();
        _volumeQuery = null;
        _released    = true;
    }

    private PlannerResult LogMeshFailure(Vector3 from, Vector3 to, long startRef, long endRef, long lastPoly, float range, string reason)
    {
        var lastPolyText = lastPoly != 0 ? lastPoly.ToString("X") : "<none>";
        Service.Log.Error($"地面算路失败：起点 = {from:f3}，请求终点 = {to:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPolyText}，容差 = {range:f3}，原因 = {reason}");
        return new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = to,
            FinalDestination     = to,
            DestinationTolerance = range
        };
    }

    private void LogMeshResult(PlannerResult result, Vector3 from, long startRef, long endRef, long lastPoly, float range, TimeSpan duration)
    {
        var startTile     = new TileCoord(FindMeshTile(from).x,                        FindMeshTile(from).z);
        var requestedTile = new TileCoord(FindMeshTile(result.RequestedDestination).x, FindMeshTile(result.RequestedDestination).z);
        var actualTile    = new TileCoord(FindMeshTile(result.FinalDestination).x,     FindMeshTile(result.FinalDestination).z);
        var (tileX, tileZ)     = FindMeshTile(result.FinalDestination);
        var (tileMin, tileMax) = GetMeshTileBounds(tileX, tileZ);
        var distanceToNearestBoundary = MathF.Min
        (
            MathF.Min(MathF.Abs(result.FinalDestination.X - tileMin.X), MathF.Abs(tileMax.X - result.FinalDestination.X)),
            MathF.Min(MathF.Abs(result.FinalDestination.Z - tileMin.Z), MathF.Abs(tileMax.Z - result.FinalDestination.Z))
        );
        var diagnostic = new SeamDiagnostic
        (
            startTile,
            requestedTile,
            actualTile,
            distanceToNearestBoundary,
            distanceToNearestBoundary <= SuspectedTileSeamBoundaryMaxDistance,
            Math.Abs(requestedTile.X - actualTile.X) <= 1 && Math.Abs(requestedTile.Z - actualTile.Z) <= 1,
            Vector3.Distance(result.RequestedDestination, result.FinalDestination) <= SuspectedTileSeamGapMaxDistance
        );
        var message =
            $"地面算路完成：状态 = {result.Status}，起点 = {from:f3}，请求终点 = {result.RequestedDestination:f3}，实际终点 = {result.FinalDestination:f3}，多边形 = {startRef:X} -> {endRef:X}，最后可达 = {lastPoly:X}，容差 = {range:f3}，耗时 = {duration.TotalSeconds:f3} 秒，粗路径段 = {result.Segments.Count}";

        if (result.Status == PathfindStatus.Partial)
        {
            message +=
                $"，起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}";
        }

        switch (result.Status)
        {
            case PathfindStatus.Partial:
                Interlocked.Increment(ref _partialGroundQueryCount);
                Service.Log.Warning(message);

                if (diagnostic.IsSuspectedTileSeamCutoff)
                {
                    Interlocked.Increment(ref _suspectedTileSeamCutoffCount);
                    Service.Log.Warning
                    (
                        $"[SuspectedTileSeamCutoff] 疑似区块接缝截断：起点区块 = {diagnostic.StartTile}，目标区块 = {diagnostic.RequestedTile}，终点区块 = {diagnostic.FinalTile}，最近边界距离 = {diagnostic.DistanceToNearestBoundary:f3}"
                    );
                }

                break;
            case PathfindStatus.Failed:
                Service.Log.Error(message);
                break;
            default:
                Service.Log.Debug(message);
                break;
        }
    }

    private void LogStartCandidateDecision
    (
        MeshPathCandidate  selected,
        Vector3            requestedTarget,
        long               requestedStartRef,
        MeshPathCandidate? requestedSuccessful,
        bool               requestedFailed,
        bool               lockedByRequested
    )
    {
        Service.Log.Debug
        (
            $"[算路] 已选起点多边形 {selected.StartRef:X}，投影点 = {selected.StartPoint:f3}，结果 = {selected.ResultStatus}，requested = {(selected.IsRequestedStart ? 1 : 0)}，overPoly = {(selected.IsPointOverPoly ? 1 : 0)}，supportHits = {selected.SupportProbeHits}"
        );

        if (lockedByRequested)
        {
            Service.Log.Information($"[算路] 起点锁定：原始候选 = {requestedStartRef:X}，原因 = 原始候选在 over-poly 内且算路成功。");
            return;
        }

        if (selected.StartRef == requestedStartRef)
        {
            var keepReason = requestedFailed
                                 ? "原始候选已恢复可用且综合评分最优"
                                 : requestedSuccessful == null
                                     ? "原始候选通过筛选并最终胜出"
                                     : selected.IsPointOverPoly
                                         ? "原始候选在 over-poly 内并保持最优"
                                         : "原始候选在候选竞争中综合评分最优";
            Service.Log.Debug
            (
                $"[算路] 起点保持：原始候选 = {requestedStartRef:X}，原因 = {keepReason}。"
            );
            return;
        }

        Service.Log.Warning
        (
            $"[算路] 起点替换：原始候选 = {requestedStartRef:X}，选中 = {selected.StartRef:X}，原因 = {BuildStartReplacementReason(selected, requestedTarget, requestedSuccessful, requestedFailed)}。"
        );
        Interlocked.Increment(ref _startReplacementCount);
    }

    private static string BuildStartReplacementReason
        (MeshPathCandidate selected, Vector3 requestedTarget, MeshPathCandidate? requestedSuccessful, bool requestedFailed)
    {
        if (requestedSuccessful == null)
            return requestedFailed ? "原始候选算路失败" : "原始候选未通过有效路径评估";

        var requested = requestedSuccessful.Value;
        if (!requested.IsPointOverPoly && selected.IsPointOverPoly)
            return "原始候选不在 over-poly，且选中候选在 over-poly";

        if (selected.SupportProbeHits > requested.SupportProbeHits)
            return "原始候选的 support 命中更低";

        var selectedRank  = ResultStatusRank(selected.ResultStatus);
        var requestedRank = ResultStatusRank(requested.ResultStatus);
        if (selectedRank < requestedRank)
            return $"选中候选路径状态更优（{selected.ResultStatus} > {requested.ResultStatus}）";

        var selectedDistance  = selected.DistanceToRequestedTargetSq(requestedTarget);
        var requestedDistance = requested.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(selectedDistance, requestedDistance) && selectedDistance < requestedDistance)
            return "选中候选更接近最终目的地";

        if (!NearlyEqual(selected.StartCandidate.VerticalDistanceAbs, requested.StartCandidate.VerticalDistanceAbs) &&
            selected.StartCandidate.VerticalDistanceAbs < requested.StartCandidate.VerticalDistanceAbs)
            return "选中候选的垂直偏移更小";

        if (!NearlyEqual(selected.StartCandidate.HorizontalDistanceSq, requested.StartCandidate.HorizontalDistanceSq) &&
            selected.StartCandidate.HorizontalDistanceSq < requested.StartCandidate.HorizontalDistanceSq)
            return "选中候选的水平偏移更小";

        return "选中候选综合评分更优";
    }

    private bool TryRepairGroundGap
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        CancellationToken cancel,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        if (TryRepairGroundGapByRaycast(partialCandidate, requestedTarget, requestedEndPos, filter, range, out repairedResult, out repairedLastPoly))
            return true;

        if (TryRepairGroundGapByMoveAlongSurface
                (partialCandidate, requestedTarget, endRef, requestedEndPos, filter, opt, range, out repairedResult, out repairedLastPoly))
            return true;

        return TryRepairGroundGapByNearbyContinuation
            (partialCandidate, requestedTarget, endRef, requestedEndPos, filter, opt, range, cancel, out repairedResult, out repairedLastPoly);
    }

    private bool TryRepairGroundGapByRaycast
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        float             range,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        Span<long> visited = stackalloc long[MaxPathPolys];
        var status = MeshQuery.Raycast
        (
            partialCandidate.LastPoly,
            partialCandidate.FinalDestination.SystemToRecast(),
            requestedEndPos,
            filter,
            out var hitT,
            out _,
            visited,
            out var visitedCount,
            visited.Length
        );
        if (status.Failed() || hitT < 0.99f)
            return false;

        repairedLastPoly = visitedCount > 0 ? visited[visitedCount - 1] : partialCandidate.LastPoly;
        repairedResult = BuildGroundPlannerResult
        (
            requestedTarget,
            range,
            range > 0 && Vector3.Distance(partialCandidate.FinalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Complete,
            requestedTarget,
            [BuildGroundMeshCorridorSegment(partialCandidate), BuildGroundDiscreteSegment(partialCandidate.FinalDestination, requestedTarget)]
        );
        Service.Log.Warning($"[算路] Partial 修复命中 Raycast：起点 = {partialCandidate.FinalDestination:f3}，终点 = {requestedTarget:f3}，最后多边形 = {repairedLastPoly:X}");
        return true;
    }

    private bool TryRepairGroundGapByMoveAlongSurface
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        Span<long> visited = stackalloc long[32];
        var status = MeshQuery.MoveAlongSurface
        (
            partialCandidate.LastPoly,
            partialCandidate.FinalDestination.SystemToRecast(),
            requestedEndPos,
            filter,
            out var moved,
            visited,
            out var visitedCount,
            visited.Length
        );
        if (status.Failed() || visitedCount == 0)
            return false;

        var movedPoint   = moved.RecastToSystem();
        var movedDelta   = movedPoint - partialCandidate.FinalDestination;
        var movedPoly    = visited[visitedCount - 1];
        var movedDistSq  = movedDelta.X * movedDelta.X + movedDelta.Z * movedDelta.Z;
        var verticalDist = MathF.Abs(movedDelta.Y);
        if (movedDistSq <= 0.000001f || verticalDist > ShortGapRepairMaxVerticalDelta)
            return false;

        if (Vector3.Distance(movedPoint, requestedTarget) <= MathF.Max(range, 0.15f))
        {
            repairedLastPoly = movedPoly;
            repairedResult = BuildGroundPlannerResult
            (
                requestedTarget,
                range,
                range > 0 ? PathfindStatus.ReachedWithinRange : PathfindStatus.Complete,
                movedPoint,
                [BuildGroundMeshCorridorSegment(partialCandidate), BuildGroundDiscreteSegment(partialCandidate.FinalDestination, movedPoint)]
            );
            Service.Log.Warning($"[算路] Partial 修复命中 MoveAlongSurface：终点已贴近目标，桥接点 = {movedPoint:f3}，最后多边形 = {repairedLastPoly:X}");
            return true;
        }

        if (!TryClosestPointOnPolyWithFlags(movedPoint, movedPoly, out var bridgePoint, out var isPointOverPoly))
            return false;

        var resumeStatus = FindPath(MeshQuery, movedPoly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);
        if (resumeStatus.Failed() || corridor.Count == 0)
            return false;

        var resumeCandidate = BuildMeshPathCandidate
        (
            new
            (
                movedPoly,
                bridgePoint,
                Vector3.DistanceSquared(bridgePoint, partialCandidate.FinalDestination),
                bridgePoint.Y - partialCandidate.FinalDestination.Y,
                false,
                isPointOverPoly,
                CountStartSupportProbeHits(partialCandidate.FinalDestination, movedPoly)
            ),
            corridor,
            resumeStatus,
            requestedEndPos,
            requestedTarget,
            endRef,
            GroundQueryMode.Classic,
            range
        );
        if (resumeCandidate.ResultStatus == PathfindStatus.Failed)
            return false;

        repairedLastPoly = resumeCandidate.LastPoly;
        List<PlannerPathSegment> segments = [BuildGroundMeshCorridorSegment(partialCandidate)];
        if (Vector3.DistanceSquared(partialCandidate.FinalDestination, bridgePoint) > 0.000001f)
            segments.Add(BuildGroundDiscreteSegment(partialCandidate.FinalDestination, bridgePoint));
        segments.Add(BuildGroundMeshCorridorSegment(resumeCandidate));
        repairedResult = BuildGroundPlannerResult(requestedTarget, range, resumeCandidate.ResultStatus, resumeCandidate.FinalDestination, segments);
        Service.Log.Warning($"[算路] Partial 修复命中 MoveAlongSurface + 续算：桥接点 = {bridgePoint:f3}，结果 = {resumeCandidate.ResultStatus}，最后多边形 = {repairedLastPoly:X}");
        return true;
    }

    private bool TryRepairGroundGapByNearbyContinuation
    (
        MeshPathCandidate partialCandidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        float             range,
        CancellationToken cancel,
        out PlannerResult repairedResult,
        out long          repairedLastPoly
    )
    {
        repairedResult   = null!;
        repairedLastPoly = partialCandidate.LastPoly;

        var partialEnd = partialCandidate.FinalDestination;
        var repairCandidates = FindIntersectingMeshPolys
            (partialEnd, new(ShortGapRepairSearchHalfExtentXZ, ShortGapRepairSearchHalfExtentY, ShortGapRepairSearchHalfExtentXZ));
        MeshPathCandidate? bestResumeCandidate = null;
        List<string>       candidateLogs       = [];

        foreach (var poly in repairCandidates)
        {
            if (poly == 0 || poly == partialCandidate.LastPoly || partialCandidate.Corridor.Contains(poly))
                continue;

            cancel.ThrowIfCancellationRequested();

            if (!TryClosestPointOnPolyWithFlags(partialEnd, poly, out var bridgePoint, out var isPointOverPoly))
                continue;

            var bridgeDelta              = bridgePoint - partialEnd;
            var bridgeHorizontalDistance = new Vector2(bridgeDelta.X, bridgeDelta.Z).Length();
            var bridgeVerticalDistance   = MathF.Abs(bridgeDelta.Y);
            if (bridgeHorizontalDistance > ShortGapRepairMaxBridgeDistance || bridgeVerticalDistance > ShortGapRepairMaxVerticalDelta)
                continue;

            var status = FindPath(MeshQuery, poly, endRef, bridgePoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);

            if (status.Failed() || corridor.Count == 0)
            {
                candidateLogs.Add($"{poly:X}: 失败 ({status})");
                continue;
            }

            var resumeCandidate = BuildMeshPathCandidate
            (
                new
                (
                    poly,
                    bridgePoint,
                    bridgeHorizontalDistance * bridgeHorizontalDistance,
                    bridgePoint.Y - partialEnd.Y,
                    false,
                    isPointOverPoly,
                    CountStartSupportProbeHits(partialEnd, poly)
                ),
                corridor,
                status,
                requestedEndPos,
                requestedTarget,
                endRef,
                GroundQueryMode.Classic,
                range
            );

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

        var bridgePointToAdd = bestResumeCandidate.Value.StartPoint;
        repairedLastPoly = bestResumeCandidate.Value.LastPoly;
        List<PlannerPathSegment> segments = [BuildGroundMeshCorridorSegment(partialCandidate)];
        if (Vector3.DistanceSquared(partialCandidate.FinalDestination, bridgePointToAdd) > 0.000001f)
            segments.Add(BuildGroundDiscreteSegment(partialCandidate.FinalDestination, bridgePointToAdd));
        segments.Add(BuildGroundMeshCorridorSegment(bestResumeCandidate.Value));

        Service.Log.Warning
        (
            $"[算路] 已触发短距补桥：partial 终点 = {partialCandidate.FinalDestination:f3}，桥接点 = {bridgePointToAdd:f3}，桥接后结果 = {bestResumeCandidate.Value.ResultStatus}，段数 = {segments.Count}，结果来源 = 短距补桥 + 续算，最后可达 = {repairedLastPoly:X}"
        );
        repairedResult = BuildGroundPlannerResult
        (
            requestedTarget,
            range,
            bestResumeCandidate.Value.ResultStatus,
            bestResumeCandidate.Value.FinalDestination,
            segments
        );
        return true;
    }

    private static int ResultStatusRank(PathfindStatus status) => status switch
    {
        PathfindStatus.Complete           => 0,
        PathfindStatus.ReachedWithinRange => 1,
        PathfindStatus.Partial            => 2,
        _                                 => 3
    };

    private static bool NearlyEqual(float left, float right) => MathF.Abs(left - right) <= 0.0001f;


    internal PlannerResult PlanMeshPathDetailed(Vector3 from, Vector3 to, bool useRaycast, float range, CancellationToken cancel)
    {
        Interlocked.Increment(ref _groundQueryCount);
        var                    requestedStartRef   = FindNearestMeshPoly(from);
        var                    requestedEndRef     = FindNearestMeshPoly(to);
        Dictionary<long, int>  endCandidateIndices = [];
        List<MeshEndCandidate> endCandidates       = [];

        foreach (var poly in FindIntersectingMeshPolys(to, new(EndPolyCandidateHalfExtentXZ, EndPolyCandidateHalfExtentY, EndPolyCandidateHalfExtentXZ)))
            TryAddEndCandidate(poly, false);

        TryAddEndCandidate(requestedEndRef, true);
        endCandidates.Sort
        (static (a, b) =>
            {
                if (a.IsPointOverPoly != b.IsPointOverPoly)
                    return b.IsPointOverPoly.CompareTo(a.IsPointOverPoly);

                if (a.IsAboveTarget != b.IsAboveTarget)
                    return a.IsAboveTarget.CompareTo(b.IsAboveTarget);

                var cmp = a.VerticalDistanceAbs.CompareTo(b.VerticalDistanceAbs);
                if (cmp != 0)
                    return cmp;

                cmp = a.HorizontalDistanceSq.CompareTo(b.HorizontalDistanceSq);
                if (cmp != 0)
                    return cmp;

                if (a.IsRequestedEnd != b.IsRequestedEnd)
                    return b.IsRequestedEnd.CompareTo(a.IsRequestedEnd);

                return a.PolyRef.CompareTo(b.PolyRef);
            }
        );

        MeshEndCandidate? endCandidate     = null;
        List<string>      endCandidateLogs = [];

        foreach (var candidate in endCandidates)
        {
            endCandidateLogs.Add
            (
                $"{candidate.PolyRef:X}: requested = {(candidate.IsRequestedEnd ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}"
            );

            if (endCandidate == null)
            {
                endCandidate = candidate;
                continue;
            }

            var currentBest = endCandidate.Value;
            var isBetterCandidate =
                candidate.IsPointOverPoly != currentBest.IsPointOverPoly                       ? candidate.IsPointOverPoly :
                candidate.IsAboveTarget   != currentBest.IsAboveTarget                         ? !candidate.IsAboveTarget :
                !NearlyEqual(candidate.VerticalDistanceAbs,  currentBest.VerticalDistanceAbs)  ? candidate.VerticalDistanceAbs < currentBest.VerticalDistanceAbs :
                !NearlyEqual(candidate.HorizontalDistanceSq, currentBest.HorizontalDistanceSq) ? candidate.HorizontalDistanceSq < currentBest.HorizontalDistanceSq :
                candidate.IsRequestedEnd != currentBest.IsRequestedEnd ? candidate.IsRequestedEnd :
                                                                         candidate.PolyRef < currentBest.PolyRef;

            if (isBetterCandidate)
                endCandidate = candidate;
        }

        if (endCandidateLogs.Count > 0)
            Service.Log.Debug($"[算路] 终点候选评估：{string.Join(" | ", endCandidateLogs)}");

        if (endCandidate is { } selectedEndCandidate && selectedEndCandidate.PolyRef != requestedEndRef)
            Interlocked.Increment(ref _endReplacementCount);

        if (endCandidate == null)
            return LogMeshFailure(from, to, requestedStartRef, requestedEndRef, 0, range, "无法为终点选择可用的导航多边形");

        var resolvedDestination = endCandidate.Value.ProjectedPoint;
        var endRef              = endCandidate.Value.PolyRef;

        var timer           = StopWatchTimer.Create();
        var requestedEndPos = resolvedDestination.SystemToRecast();
        var prunedAttempt   = ExecuteGroundPathAttempt(from, to, requestedStartRef, endRef, requestedEndPos, _groundFilter, useRaycast, range, cancel);
        var selectedAttempt = prunedAttempt;

        if (prunedAttempt == null || prunedAttempt.Value.Result.Status == PathfindStatus.Partial)
        {
            var unprunedAttempt = ExecuteGroundPathAttempt
            (
                from,
                to,
                requestedStartRef,
                endRef,
                requestedEndPos,
                _groundFilterIgnoringUnreachable,
                useRaycast,
                range,
                cancel
            );

            var shouldUseFallback = false;

            if (unprunedAttempt is { } fallbackAttempt)
            {
                shouldUseFallback = selectedAttempt == null;

                if (!shouldUseFallback && selectedAttempt is { } currentAttempt)
                {
                    var fallbackRank = ResultStatusRank(fallbackAttempt.Result.Status);
                    var currentRank  = ResultStatusRank(currentAttempt.Result.Status);
                    shouldUseFallback = fallbackRank       != currentRank
                                            ? fallbackRank < currentRank
                                            : IsBetterStartPathCandidate(fallbackAttempt.Candidate, currentAttempt.Candidate, to);
                }
            }

            if (unprunedAttempt is { } fallbackAttemptToApply && shouldUseFallback)
            {
                var initialAttempt = selectedAttempt is { } currentAttempt
                                         ? $"{currentAttempt.Candidate.StartRef:X}/{currentAttempt.Candidate.ResultStatus}/requested={(currentAttempt.Candidate.IsRequestedStart ? 1 : 0)}/overPoly={(currentAttempt.Candidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(currentAttempt.Candidate.DistanceToRequestedTargetSq(to)):f3}/结果={currentAttempt.Result.Status}/最后={currentAttempt.LastPoly:X}"
                                         : "<none>";
                var fallbackAttemptText =
                    $"{fallbackAttemptToApply.Candidate.StartRef:X}/{fallbackAttemptToApply.Candidate.ResultStatus}/requested={(fallbackAttemptToApply.Candidate.IsRequestedStart ? 1 : 0)}/overPoly={(fallbackAttemptToApply.Candidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(fallbackAttemptToApply.Candidate.DistanceToRequestedTargetSq(to)):f3}/结果={fallbackAttemptToApply.Result.Status}/最后={fallbackAttemptToApply.LastPoly:X}";
                Service.Log.Warning
                (
                    $"[算路] 已改用忽略裁剪标记的回退结果：首轮 = {initialAttempt}，回退 = {fallbackAttemptText}"
                );
                selectedAttempt = fallbackAttemptToApply;
            }
        }

        if (selectedAttempt is not { } attempt)
            return LogMeshFailure(from, to, requestedStartRef, endRef, 0, range, "无法为起点选择可用的导航多边形");

        LastPath.Clear();
        var pathCandidate = attempt.Candidate;
        var startRef      = attempt.StartRef;
        if (pathCandidate.QueryMode == GroundQueryMode.AnyAngle)
            Interlocked.Increment(ref _anyAnglePreferredCount);
        LastPath.AddRange(pathCandidate.Corridor);
        Service.Log.Debug($"[算路] 地面多边形 {startRef:X} -> {endRef:X}（原始起点候选 = {requestedStartRef:X}，查询模式 = {pathCandidate.QueryMode}）");
        Service.Log.Debug($"[算路] 地面终点解析：原始终点候选 = {requestedEndRef:X}，选中 = {endRef:X}，终点投影 = {resolvedDestination:f3}");
        Service.Log.Debug
            ($"[算路] 地面路径查询耗时 {timer.Value().TotalSeconds:f3} 秒，状态 = {attempt.QueryStatus}，路径 = {string.Join(", ", LastPath.Select(r => r.ToString("X")))}");

        cancel.ThrowIfCancellationRequested();
        LogMeshResult(attempt.Result, from, startRef, endRef, attempt.LastPoly, range, timer.Value());
        return attempt.Result;

        void TryAddEndCandidate(long poly, bool forceInclude)
        {
            if (poly == 0)
                return;

            if (endCandidateIndices.TryGetValue(poly, out var existingIndex))
            {
                if (forceInclude && !endCandidates[existingIndex].IsRequestedEnd)
                    endCandidates[existingIndex] = endCandidates[existingIndex] with { IsRequestedEnd = true };
                return;
            }

            if (!TryClosestPointOnPolyWithFlags(to, poly, out var point, out var isPointOverPoly))
                return;

            var dx                   = point.X - to.X;
            var dz                   = point.Z - to.Z;
            var horizontalDistanceSq = dx * dx + dz * dz;
            var verticalDistance     = point.Y - to.Y;

            if (!forceInclude)
            {
                if (horizontalDistanceSq > EndPolyCandidateMaxHorizontalDistance * EndPolyCandidateMaxHorizontalDistance)
                    return;
                if (MathF.Abs(verticalDistance) > EndPolyCandidateMaxVerticalDistance)
                    return;
            }

            endCandidateIndices.Add(poly, endCandidates.Count);
            endCandidates.Add(new(poly, point, horizontalDistanceSq, verticalDistance, forceInclude, isPointOverPoly));
        }
    }

    private GroundPathAttempt? ExecuteGroundPathAttempt
    (
        Vector3           from,
        Vector3           to,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        bool              useRaycast,
        float             range,
        CancellationToken cancel
    )
    {
        var anyAngleOption = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            useRaycast ? DtFindPathOptions.DT_FINDPATH_ANY_ANGLE : 0,
            useRaycast ? 5 : 0
        );
        var classicOption = new DtFindPathOption
        (
            range > 0 ? new GoalRadiusHeuristic(range) : DtDefaultQueryHeuristic.Default,
            0,
            0
        );
        var anyAngleCandidate = ResolveBestStartPathCandidate
        (
            from,
            to,
            requestedStartRef,
            endRef,
            requestedEndPos,
            filter,
            anyAngleOption,
            GroundQueryMode.AnyAngle,
            range,
            cancel
        );
        MeshPathCandidate? classicCandidate = null;

        var fallbackDistanceThreshold = MathF.Max(range, 0.35f);

        if (useRaycast                                                                                                      ||
            anyAngleCandidate                                       == null                                                 ||
            anyAngleCandidate.Value.ResultStatus                    != PathfindStatus.Complete                              ||
            anyAngleCandidate.Value.DistanceToRequestedTargetSq(to) > fallbackDistanceThreshold * fallbackDistanceThreshold ||
            anyAngleCandidate.Value.WeightedLinkPenalty             >= 4)
        {
            Interlocked.Increment(ref _classicFallbackCount);
            classicCandidate = ResolveBestStartPathCandidate
            (
                from,
                to,
                requestedStartRef,
                endRef,
                requestedEndPos,
                filter,
                classicOption,
                GroundQueryMode.Classic,
                range,
                cancel
            );
        }

        MeshPathCandidate? bestStartCandidate;
        if (anyAngleCandidate     == null) bestStartCandidate = classicCandidate;
        else if (classicCandidate == null) bestStartCandidate = anyAngleCandidate;
        else bestStartCandidate = IsBetterStartPathCandidate(anyAngleCandidate.Value, classicCandidate.Value, to) ? anyAngleCandidate : classicCandidate;
        if (bestStartCandidate is not { } pathCandidate)
            return null;

        var lastPoly     = pathCandidate.LastPoly;
        var resultStatus = pathCandidate.ResultStatus;
        var result       = BuildGroundPlannerResult(to, range, resultStatus, pathCandidate.FinalDestination, [BuildGroundMeshCorridorSegment(pathCandidate)]);

        if (resultStatus == PathfindStatus.Partial &&
            TryRepairGroundGap(pathCandidate, to, endRef, requestedEndPos, filter, classicOption, range, cancel, out var repairedResult, out var repairedLastPoly))
        {
            result   = repairedResult;
            lastPoly = repairedLastPoly;
        }

        return new(pathCandidate, result, pathCandidate.StartRef, lastPoly, pathCandidate.QueryStatus);
    }

    private MeshPathCandidate? ResolveBestStartPathCandidate
    (
        Vector3           from,
        Vector3           to,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        GroundQueryMode   queryMode,
        float             range,
        CancellationToken cancel
    )
    {
        var selection = EvaluateBestStartPathCandidate
        (
            from,
            to,
            requestedStartRef,
            endRef,
            requestedEndPos,
            filter,
            opt,
            queryMode,
            range,
            cancel,
            false,
            true
        );

        if (selection.Selected == null || selection.Selected.Value.ResultStatus == PathfindStatus.Partial)
        {
            var expandedSelection = EvaluateBestStartPathCandidate
            (
                from,
                to,
                requestedStartRef,
                endRef,
                requestedEndPos,
                filter,
                opt,
                queryMode,
                range,
                cancel,
                true,
                false
            );

            if (expandedSelection.Selected is { } expandedCandidate &&
                (selection.Selected == null || IsBetterStartPathCandidate(expandedCandidate, selection.Selected.Value, to)))
            {
                var firstAttempt = selection.Selected is { } currentSelection
                                       ? $"{currentSelection.StartRef:X}/{currentSelection.ResultStatus}/requested={(currentSelection.IsRequestedStart ? 1 : 0)}/overPoly={(currentSelection.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(currentSelection.DistanceToRequestedTargetSq(to)):f3}"
                                       : "<none>";
                var expandedAttempt =
                    $"{expandedCandidate.StartRef:X}/{expandedCandidate.ResultStatus}/requested={(expandedCandidate.IsRequestedStart ? 1 : 0)}/overPoly={(expandedCandidate.IsPointOverPoly ? 1 : 0)}/距目标={MathF.Sqrt(expandedCandidate.DistanceToRequestedTargetSq(to)):f3}";
                Service.Log.Warning
                (
                    $"[算路] 起点扩搜改判：查询模式 = {queryMode}，首轮 = {firstAttempt}，扩搜 = {expandedAttempt}，扩搜候选 = {expandedSelection.EvaluatedCandidates}/{expandedSelection.TotalCandidates}"
                );
                selection = expandedSelection;
            }
        }

        if (selection.Selected is { } selected)
            LogStartCandidateDecision(selected, to, requestedStartRef, selection.RequestedSuccessful, selection.RequestedFailed, selection.LockedByRequested);

        return selection.Selected;
    }

    private StartCandidateSelection EvaluateBestStartPathCandidate
    (
        Vector3           from,
        Vector3           requestedTarget,
        long              requestedStartRef,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        GroundQueryMode   queryMode,
        float             range,
        CancellationToken cancel,
        bool              expandedSearch,
        bool              allowRequestedLock
    )
    {
        var candidates = CollectStartPolyCandidates(from, requestedStartRef, expandedSearch);
        if (candidates.Count == 0)
            return new(null, null, false, false, 0, 0);

        var evaluationLimit = expandedSearch ? ExpandedMaxStartPolyCandidatesToEvaluate : MaxStartPolyCandidatesToEvaluate;
        var candidateBatch  = candidates.Take(evaluationLimit).ToArray();

        MeshPathCandidate? best                = null;
        MeshPathCandidate? requestedSuccessful = null;
        MeshPathCandidate? requestedLocked     = null;
        List<string>       candidateLogs       = [];
        var                requestedFailed     = false;

        var evaluations = EvaluateStartPathCandidates(candidateBatch, requestedTarget, endRef, requestedEndPos, filter, opt, queryMode, range, cancel);

        foreach (var evaluation in evaluations)
        {
            if (!string.IsNullOrEmpty(evaluation.Log))
                candidateLogs.Add(evaluation.Log);

            if (evaluation.RequestedFailed)
                requestedFailed = true;

            if (evaluation.PathCandidate is not { } pathCandidate)
                continue;

            if (pathCandidate.IsRequestedStart)
                requestedSuccessful = pathCandidate;

            if (allowRequestedLock &&
                pathCandidate is { IsRequestedStart: true, IsPointOverPoly: true, ResultStatus: PathfindStatus.Complete or PathfindStatus.ReachedWithinRange })
                requestedLocked = pathCandidate;

            if (best == null || IsBetterStartPathCandidate(pathCandidate, best.Value, requestedTarget))
                best = pathCandidate;
        }

        if (candidateLogs.Count > 0)
            Service.Log.Debug($"[算路] {(expandedSearch ? "起点扩搜候选评估" : "起点候选评估")}：{string.Join(" | ", candidateLogs)}");

        var selected = allowRequestedLock && requestedLocked is { } locked ? locked : best;
        return new(selected, requestedSuccessful, requestedFailed, allowRequestedLock && requestedLocked != null, candidates.Count, candidateBatch.Length);
    }

    private StartCandidateEvaluation[] EvaluateStartPathCandidates
    (
        IReadOnlyList<MeshPolyCandidate> candidates,
        Vector3                          requestedTarget,
        long                             endRef,
        RcVec3f                          requestedEndPos,
        IDtQueryFilter                   filter,
        DtFindPathOption                 opt,
        GroundQueryMode                  queryMode,
        float                            range,
        CancellationToken                cancel
    )
    {
        var result = new StartCandidateEvaluation[candidates.Count];
        if (candidates.Count == 0)
            return result;

        if (candidates.Count > 1 && Environment.ProcessorCount > 1)
        {
            Parallel.For
            (
                0,
                candidates.Count,
                new ParallelOptions { CancellationToken = cancel, MaxDegreeOfParallelism = Math.Min(Environment.ProcessorCount, candidates.Count) },
                i => result[i] = EvaluateStartPathCandidate(candidates[i], requestedTarget, endRef, requestedEndPos, filter, opt, queryMode, range)
            );
        }
        else
        {
            for (var i = 0; i < candidates.Count; ++i)
            {
                cancel.ThrowIfCancellationRequested();
                result[i] = EvaluateStartPathCandidate(candidates[i], requestedTarget, endRef, requestedEndPos, filter, opt, queryMode, range);
            }
        }

        return result;
    }

    private StartCandidateEvaluation EvaluateStartPathCandidate
    (
        MeshPolyCandidate candidate,
        Vector3           requestedTarget,
        long              endRef,
        RcVec3f           requestedEndPos,
        IDtQueryFilter    filter,
        DtFindPathOption  opt,
        GroundQueryMode   queryMode,
        float             range
    )
    {
        var query  = new DtNavMeshQuery(MeshQuery.GetAttachedNavMesh());
        var status = FindPath(query, candidate.PolyRef, endRef, candidate.ProjectedPoint.SystemToRecast(), requestedEndPos, filter, opt, out var corridor);

        if (status.Failed() || corridor.Count == 0)
        {
            return new
            (
                null,
                $"{candidate.PolyRef:X}: 失败 ({status})，requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}",
                candidate.IsRequestedStart
            );
        }

        var pathCandidate = BuildMeshPathCandidate(query, candidate, corridor, status, requestedEndPos, requestedTarget, endRef, queryMode, range);

        if (pathCandidate.ResultStatus == PathfindStatus.Failed)
        {
            return new
            (
                null,
                $"{candidate.PolyRef:X}: 失败（无法投影最终可达点），requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}",
                candidate.IsRequestedStart
            );
        }

        return new
        (
            pathCandidate,
            $"{candidate.PolyRef:X}: {pathCandidate.ResultStatus}，requested = {(candidate.IsRequestedStart ? 1 : 0)}，overPoly = {(candidate.IsPointOverPoly ? 1 : 0)}，supportHits = {candidate.SupportProbeHits}，路径长 {pathCandidate.PathLength:f3}，距目标 {MathF.Sqrt(pathCandidate.DistanceToRequestedTargetSq(requestedTarget)):f3}，水平偏移 {MathF.Sqrt(candidate.HorizontalDistanceSq):f3}，高差 {candidate.VerticalDelta:f3}",
            false
        );
    }

    private List<MeshPolyCandidate> CollectStartPolyCandidates(Vector3 from, long requestedStartRef, bool expandedSearch)
    {
        Dictionary<long, int>   candidateIndices      = [];
        List<MeshPolyCandidate> candidates            = [];
        var                     halfExtentXZ          = expandedSearch ? ExpandedStartPolyCandidateHalfExtentXZ : StartPolyCandidateHalfExtentXZ;
        var                     halfExtentY           = expandedSearch ? ExpandedStartPolyCandidateHalfExtentY : StartPolyCandidateHalfExtentY;
        var                     maxHorizontalDistance = expandedSearch ? ExpandedStartPolyCandidateMaxHorizontalDistance : StartPolyCandidateMaxHorizontalDistance;
        var                     maxVerticalDistance   = expandedSearch ? ExpandedStartPolyCandidateMaxVerticalDistance : StartPolyCandidateMaxVerticalDistance;

        foreach (var poly in FindIntersectingMeshPolys(from, new(halfExtentXZ, halfExtentY, halfExtentXZ)))
            TryAddCandidate(poly, false);

        TryAddCandidate(requestedStartRef, true);

        candidates.Sort
        (static (a, b) =>
            {
                if (a.IsRequestedStart != b.IsRequestedStart)
                    return b.IsRequestedStart.CompareTo(a.IsRequestedStart);

                if (a.IsPointOverPoly != b.IsPointOverPoly)
                    return b.IsPointOverPoly.CompareTo(a.IsPointOverPoly);

                var supportCmp = b.SupportProbeHits.CompareTo(a.SupportProbeHits);
                if (supportCmp != 0)
                    return supportCmp;

                var bucketA = a.IsTooFarAbove ? 1 : 0;
                var bucketB = b.IsTooFarAbove ? 1 : 0;
                var cmp     = bucketA.CompareTo(bucketB);
                if (cmp != 0)
                    return cmp;

                cmp = a.VerticalDistanceAbs.CompareTo(b.VerticalDistanceAbs);
                if (cmp != 0)
                    return cmp;

                cmp = a.HorizontalDistanceSq.CompareTo(b.HorizontalDistanceSq);
                if (cmp != 0)
                    return cmp;

                return a.PolyRef.CompareTo(b.PolyRef);
            }
        );

        return candidates;

        void TryAddCandidate(long poly, bool forceInclude)
        {
            if (poly == 0)
                return;

            if (candidateIndices.TryGetValue(poly, out var existingIndex))
            {
                if (forceInclude && !candidates[existingIndex].IsRequestedStart)
                    candidates[existingIndex] = candidates[existingIndex] with { IsRequestedStart = true };
                return;
            }

            if (!TryClosestPointOnPolyWithFlags(from, poly, out var point, out var isPointOverPoly))
                return;

            var dx                   = point.X - from.X;
            var dz                   = point.Z - from.Z;
            var horizontalDistanceSq = dx * dx + dz * dz;
            var verticalDistance     = point.Y - from.Y;

            if (!forceInclude)
            {
                if (horizontalDistanceSq > maxHorizontalDistance * maxHorizontalDistance)
                    return;
                if (MathF.Abs(verticalDistance) > maxVerticalDistance)
                    return;
            }

            var supportHits = CountStartSupportProbeHits(from, poly);
            var candidate   = new MeshPolyCandidate(poly, point, horizontalDistanceSq, verticalDistance, forceInclude, isPointOverPoly, supportHits);
            candidateIndices.Add(poly, candidates.Count);
            candidates.Add(candidate);
        }
    }

    private int CountStartSupportProbeHits(Vector3 from, long poly)
    {
        var matchDistanceSq = StartSupportMatchDistance * StartSupportMatchDistance;
        var step            = MathF.Tau                 / StartSupportProbeCount;
        var hits            = 0;

        for (var i = 0; i < StartSupportProbeCount; i++)
        {
            var angle = step * i;
            var probe = new Vector3(from.X + MathF.Cos(angle) * StartSupportProbeRadius, from.Y, from.Z + MathF.Sin(angle) * StartSupportProbeRadius);
            if (!TryClosestPointOnPolyWithFlags(probe, poly, out var probeClosest, out var probeOverPoly))
                continue;

            if (probeOverPoly)
            {
                hits++;
                continue;
            }

            var dx = probeClosest.X - probe.X;
            var dz = probeClosest.Z - probe.Z;
            if (dx * dx + dz * dz <= matchDistanceSq)
                hits++;
        }

        return hits;
    }

    private readonly record struct StartCandidateEvaluation
    (
        MeshPathCandidate? PathCandidate,
        string             Log,
        bool               RequestedFailed
    );

    private readonly record struct StartCandidateSelection
    (
        MeshPathCandidate? Selected,
        MeshPathCandidate? RequestedSuccessful,
        bool               RequestedFailed,
        bool               LockedByRequested,
        int                TotalCandidates,
        int                EvaluatedCandidates
    );

    private readonly record struct GroundPathAttempt
    (
        MeshPathCandidate Candidate,
        PlannerResult     Result,
        long              StartRef,
        long              LastPoly,
        DtStatus          QueryStatus
    );

    private MeshPathCandidate BuildMeshPathCandidate
    (
        MeshPolyCandidate startCandidate,
        List<long>        corridor,
        DtStatus          status,
        RcVec3f           requestedEndPos,
        Vector3           requestedTarget,
        long              endRef,
        GroundQueryMode   queryMode,
        float             range
    )
        => BuildMeshPathCandidate(MeshQuery, startCandidate, corridor, status, requestedEndPos, requestedTarget, endRef, queryMode, range);

    private static MeshPathCandidate BuildMeshPathCandidate
    (
        DtNavMeshQuery    query,
        MeshPolyCandidate startCandidate,
        List<long>        corridor,
        DtStatus          status,
        RcVec3f           requestedEndPos,
        Vector3           requestedTarget,
        long              endRef,
        GroundQueryMode   queryMode,
        float             range
    )
    {
        var     lastPoly     = corridor[^1];
        var     resultStatus = PathfindStatus.Complete;
        Vector3 finalDestination;

        if (status.IsPartial() || lastPoly != endRef)
        {
            var closestStatus = query.ClosestPointOnPoly(lastPoly, requestedEndPos, out var projectedEndPos, out _);
            if (closestStatus.Failed())
                return new(startCandidate, status, PathfindStatus.Failed, startCandidate.ProjectedPoint, corridor, queryMode, float.MaxValue, int.MaxValue, 0, 0);

            finalDestination = projectedEndPos.RecastToSystem();
            resultStatus = range > 0 && Vector3.Distance(finalDestination, requestedTarget) <= range ? PathfindStatus.ReachedWithinRange : PathfindStatus.Partial;
        }
        else finalDestination = requestedEndPos.RecastToSystem();

        var pathLength = EstimatePathLength(query, startCandidate.ProjectedPoint.SystemToRecast(), finalDestination.SystemToRecast(), corridor);
        CountPathSemantics(query.GetAttachedNavMesh(), corridor, out var weightedLinkPenalty, out var offMeshTransitionCount, out var areaCrossingCount);
        return new
        (
            startCandidate,
            status,
            resultStatus,
            finalDestination,
            corridor,
            queryMode,
            pathLength,
            weightedLinkPenalty,
            offMeshTransitionCount,
            areaCrossingCount
        );
    }

    private static float EstimatePathLength(DtNavMeshQuery query, RcVec3f startPos, RcVec3f endPos, IReadOnlyList<long> corridor)
    {
        if (corridor.Count == 0)
            return float.MaxValue;

        var straightPath = new DtStraightPath[256];
        var status       = query.FindStraightPath(startPos, endPos, [.. corridor], corridor.Count, straightPath, out var count, straightPath.Length, 0);

        if (status.Succeeded() && count > 0)
        {
            var total    = 0f;
            var previous = startPos;

            for (var i = 0; i < count; i++)
            {
                total    += RcVec3f.Distance(previous, straightPath[i].pos);
                previous =  straightPath[i].pos;
            }

            return total;
        }

        var      navmesh        = query.GetAttachedNavMesh();
        var      fallback       = RcVec3f.Distance(startPos, endPos);
        RcVec3f? previousCenter = null;

        foreach (var polyRef in corridor)
        {
            var center = navmesh.GetPolyCenter(polyRef);
            if (previousCenter is { } prev)
                fallback += RcVec3f.Distance(prev, center);
            previousCenter = center;
        }

        return fallback;
    }

    private static void CountPathSemantics
    (
        DtNavMesh           navmesh,
        IReadOnlyList<long> corridor,
        out int             weightedLinkPenalty,
        out int             offMeshTransitionCount,
        out int             areaCrossingCount
    )
    {
        weightedLinkPenalty    = 0;
        offMeshTransitionCount = 0;
        areaCrossingCount      = 0;
        NavmeshArea? previousArea = null;

        foreach (var polyRef in corridor)
        {
            navmesh.GetTileAndPolyByRefUnsafe(polyRef, out _, out var poly);
            var area = (NavmeshArea)poly.GetArea();
            if (previousArea is { } lastArea && lastArea != area)
                areaCrossingCount++;

            switch (area)
            {
                case NavmeshArea.GeneratedClimbDown:
                    weightedLinkPenalty += 2;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.GeneratedEdgeJump:
                    weightedLinkPenalty += 4;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.ManualOffMesh:
                    weightedLinkPenalty += 1;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.Teleport:
                    weightedLinkPenalty += 1;
                    offMeshTransitionCount++;
                    break;
                case NavmeshArea.ClientPath:
                    weightedLinkPenalty += 3;
                    offMeshTransitionCount++;
                    break;
            }

            previousArea = area;
        }
    }

    private static DtStatus FindPath
    (
        DtNavMeshQuery   query,
        long             startRef,
        long             endRef,
        RcVec3f          startPos,
        RcVec3f          endPos,
        IDtQueryFilter   filter,
        DtFindPathOption opt,
        out List<long>   corridor
    )
    {
        var buffer = new long[MaxPathPolys];
        corridor = [];

        if (opt.options != 0)
        {
            var status = query.InitSlicedFindPath(startRef, endRef, startPos, endPos, filter, opt.options);
            if (status.Failed())
                return status;

            do
            {
                status = query.UpdateSlicedFindPath(MaxPathPolys, out _);
            }
            while (status.InProgress());

            if (status.Failed())
                return status;

            status   = query.FinalizeSlicedFindPath(buffer, out var count, buffer.Length);
            corridor = [.. buffer.AsSpan(0, count).ToArray()];
            return status;
        }

        var directStatus = query.FindPath(startRef, endRef, startPos, endPos, filter, buffer, out var directCount, buffer.Length);
        corridor = [.. buffer.AsSpan(0, directCount).ToArray()];
        return directStatus;
    }

    private static bool IsBetterStartPathCandidate(MeshPathCandidate candidate, MeshPathCandidate currentBest, Vector3 requestedTarget)
    {
        var rankCandidate = ResultStatusRank(candidate.ResultStatus);
        var rankCurrent   = ResultStatusRank(currentBest.ResultStatus);
        if (rankCandidate != rankCurrent)
            return rankCandidate < rankCurrent;

        var candidateRequestedOver = candidate is { IsRequestedStart: true, IsPointOverPoly: true };
        var currentRequestedOver   = currentBest is { IsRequestedStart: true, IsPointOverPoly: true };
        if (candidateRequestedOver != currentRequestedOver)
            return candidateRequestedOver;

        if (candidate.IsPointOverPoly != currentBest.IsPointOverPoly)
            return candidate.IsPointOverPoly;

        if (!NearlyEqual(candidate.StartCandidate.VerticalDistanceAbs, currentBest.StartCandidate.VerticalDistanceAbs))
            return candidate.StartCandidate.VerticalDistanceAbs < currentBest.StartCandidate.VerticalDistanceAbs;

        var candidateDistance = candidate.DistanceToRequestedTargetSq(requestedTarget);
        var currentDistance   = currentBest.DistanceToRequestedTargetSq(requestedTarget);
        if (!NearlyEqual(candidateDistance, currentDistance))
            return candidateDistance < currentDistance;

        if (!NearlyEqual(candidate.PathLength, currentBest.PathLength))
            return candidate.PathLength < currentBest.PathLength;

        if (candidate.WeightedLinkPenalty != currentBest.WeightedLinkPenalty)
            return candidate.WeightedLinkPenalty < currentBest.WeightedLinkPenalty;

        if (!NearlyEqual(candidate.StartCandidate.HorizontalDistanceSq, currentBest.StartCandidate.HorizontalDistanceSq))
            return candidate.StartCandidate.HorizontalDistanceSq < currentBest.StartCandidate.HorizontalDistanceSq;

        if (candidate.SupportProbeHits != currentBest.SupportProbeHits)
            return candidate.SupportProbeHits > currentBest.SupportProbeHits;

        if (candidate.QueryMode != currentBest.QueryMode)
            return candidate.QueryMode == GroundQueryMode.AnyAngle;

        return candidate.StartRef < currentBest.StartRef;
    }

    private static PlannerResult BuildGroundPlannerResult
    (
        Vector3                           requestedTarget,
        float                             range,
        PathfindStatus                    status,
        Vector3                           finalDestination,
        IReadOnlyList<PlannerPathSegment> segments
    ) =>
        new()
        {
            Status               = status,
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = requestedTarget,
            FinalDestination     = finalDestination,
            DestinationTolerance = range,
            Segments             = segments
        };

    private static PlannerPathSegment BuildGroundMeshCorridorSegment(MeshPathCandidate candidate) =>
        new()
        {
            MovementMode         = MovementMode.Ground,
            SegmentKind          = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl = false,
            ReachabilitySource   = PathReachabilitySource.Mesh,
            GeometryKind         = PlannerSegmentGeometryKind.MeshCorridor,
            StartPosition        = candidate.StartPoint,
            EndPosition          = candidate.FinalDestination,
            Corridor             = [.. candidate.Corridor]
        };

    private static PlannerPathSegment BuildGroundDiscreteSegment(Vector3 startPosition, params Vector3[] points) =>
        new()
        {
            MovementMode         = MovementMode.Ground,
            SegmentKind          = MovementSegmentKind.GroundTraverse,
            AllowVerticalControl = false,
            ReachabilitySource   = PathReachabilitySource.Mesh,
            GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
            StartPosition        = startPosition,
            EndPosition          = points.Length > 0 ? points[^1] : startPosition,
            Points               = [.. points]
        };


    private const float FlightLandingProbeHalfExtentXZ        = 1f;
    private const float FlightLandingProbeHalfExtentY         = 2f;
    private const float FlightLandingMaxRequestHorizontal     = 1f;
    private const float FlightLandingMaxSafeHorizontal        = 1.5f;
    private const float FlightLandingMaxSafeVerticalDrop      = 8f;
    private const float FlightLandingCompletionSlack          = 0.25f;
    private const float FlightGroundTransitionSlack           = 0.25f;
    private const float FlightGroundApproachTriggerHorizontal = 0.5f;
    private const float FlightGroundApproachMinHorizontal     = 1.5f;
    private const float FlightGroundApproachMaxHorizontal     = 4f;
    private const float FlightGroundApproachHeightRatio       = 0.7f;

    internal PlannerResult PlanVolumePathDetailed(Vector3 from, Vector3 to, bool useRaycast, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("体素导航体未构建，无法执行飞行算路");
            return CreateFlightFailure(to);
        }

        var volume         = VolumeQuery.Volume;
        var locateTimer    = StopWatchTimer.Create();
        var startVoxel     = FindNearestVolumeVoxel(from);
        var endVoxel       = FindNearestVolumeVoxel(to);
        var locateDuration = locateTimer.Value();
        Service.Log.Debug($"[算路] 飞行体素 {startVoxel:X} -> {endVoxel:X}");

        if (startVoxel == VoxelMap.INVALID_VOXEL || endVoxel == VoxelMap.INVALID_VOXEL)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 无法定位空体素");
            return CreateFlightFailure(to);
        }

        var requestedStartLeaf = volume.FindLeafVoxel(from);
        var requestedTargetLeaf = volume.FindLeafVoxel(to);
        var safeStart = requestedStartLeaf.empty && requestedStartLeaf.voxel == startVoxel ? from : VoxelSearch.FindClosestVoxelPoint(volume, startVoxel, from);
        var safeDestination = requestedTargetLeaf.empty && requestedTargetLeaf.voxel == endVoxel ? to : VoxelSearch.FindClosestVoxelPoint(volume, endVoxel, to);
        var safeDestinationAdjusted = Vector3.DistanceSquared(safeDestination, to) > 0.000001f;
        var searchTimer = StopWatchTimer.Create();
        var voxelPath = VolumeQuery.FindPath
            (startVoxel, endVoxel, safeStart, safeDestination, useRaycast, false, cancel);
        var telemetry = VolumeQuery.LastTelemetry;

        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 体素路径为空");
            return CreateFlightFailure(to);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行路径查询完成：空体素定位耗时 = {locateDuration.TotalSeconds:f3} 秒，主体搜索耗时 = {searchTimer.Value().TotalSeconds:f3} 秒，访问节点 = {telemetry.VisitedNodes}，生成节点 = {telemetry.GeneratedNodes}，LoS 检查 = {telemetry.LineOfSightChecks}，LoS 命中 = {telemetry.LineOfSightHits}，开放表峰值 = {telemetry.PeakOpenListSize}，终止 = {DescribeVolumeSearchTermination(telemetry.Termination)}，搜索射线优化 = {(telemetry.SearchRaycastEnabled ? "是" : "否")}，搜索轮次 = {telemetry.SearchAttempts}，启发式权重 = {telemetry.HeuristicWeight:f2}，路径点 = {voxelPath.Count}，起点修正 = {(Vector3.DistanceSquared(safeStart, from) > 0.000001f ? "是" : "否")}，安全终点修正 = {(safeDestinationAdjusted ? "是" : "否")}"
        );

        List<Vector3> rawWaypoints = new(voxelPath.Count);
        foreach (var step in voxelPath)
            rawWaypoints.Add(step.p);

        if (telemetry.Termination != VolumeSearchTermination.ReachedGoal)
        {
            var partialDestination = rawWaypoints[^1];
            Service.Log.Warning
            (
                $"飞行体素搜索未抵达终点：终止 = {DescribeVolumeSearchTermination(telemetry.Termination)}，请求空体素终点 = {safeDestination:f3}，当前终点 = {partialDestination:f3}，后续不再强接终点"
            );

            return new()
            {
                Status               = PathfindStatus.Partial,
                RequestedMode        = MovementMode.Flight,
                RequestedDestination = to,
                FinalDestination     = partialDestination,
                DestinationTolerance = 0,
                Segments =
                [
                    new()
                    {
                        MovementMode         = MovementMode.Flight,
                        SegmentKind          = MovementSegmentKind.FlightTraverse,
                        AllowVerticalControl = true,
                        ReachabilitySource   = PathReachabilitySource.Volume,
                        GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                        StartPosition        = from,
                        EndPosition          = partialDestination,
                        Points               = [.. rawWaypoints]
                    }
                ]
            };
        }

        if (!requestedTargetLeaf.empty &&
            TryBuildFlightGroundTransitionResult(from, to, safeDestination, rawWaypoints, useRaycast, cancel, out var hybridResult))
            return hybridResult;

        var finalDestination    = safeDestination;
        var destinationAdjusted = safeDestinationAdjusted;
        var landingPoint        = !requestedTargetLeaf.empty ? TryResolveFlightLandingPoint(to, safeDestination) : null;
        var completionTolerance = MathF.Max(_config.PathTolerance, FlightLandingCompletionSlack);

        if (landingPoint is { } resolvedLandingPoint)
        {
            finalDestination    = resolvedLandingPoint;
            destinationAdjusted = Vector3.Distance(resolvedLandingPoint, to) > completionTolerance;

            if (rawWaypoints.Count == 0 || Vector3.DistanceSquared(rawWaypoints[^1], resolvedLandingPoint) > 0.000001f)
                rawWaypoints.Add(resolvedLandingPoint);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行终点解析：请求终点 = {to:f3}，空体素终点 = {safeDestination:f3}，落地点 = {(landingPoint is { } lp ? lp.ToString("f3") : "无")}，最终终点 = {finalDestination:f3}，落地吸附 = {(landingPoint != null ? "是" : "否")}"
        );

        var finalDestinationTolerance = landingPoint != null ? completionTolerance : 0;

        return new()
        {
            Status               = destinationAdjusted ? PathfindStatus.Partial : PathfindStatus.Complete,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = to,
            FinalDestination     = finalDestination,
            DestinationTolerance = finalDestinationTolerance,
            Segments =
            [
                new()
                {
                    MovementMode         = MovementMode.Flight,
                    SegmentKind          = MovementSegmentKind.FlightTraverse,
                    AllowVerticalControl = true,
                    ReachabilitySource   = PathReachabilitySource.Volume,
                    GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                    StartPosition        = from,
                    EndPosition          = finalDestination,
                    Points               = [.. rawWaypoints]
                }
            ]
        };
    }

    private Vector3? TryResolveFlightLandingPoint(Vector3 requestedTarget, Vector3 safeDestination)
    {
        var landingPoint = FindPointOnFloor
                               (requestedTarget, FlightLandingProbeHalfExtentXZ) ??
                           FindNearestPointOnMesh(requestedTarget, FlightLandingProbeHalfExtentXZ, FlightLandingProbeHalfExtentY);
        if (landingPoint is not { } resolved)
            return null;

        var requestHorizontalDistance = HorizontalDistanceXZ(resolved, requestedTarget);
        if (requestHorizontalDistance > FlightLandingMaxRequestHorizontal)
            return null;

        var safeHorizontalDistance = HorizontalDistanceXZ(resolved, safeDestination);
        if (safeHorizontalDistance > FlightLandingMaxSafeHorizontal)
            return null;

        var verticalDrop = safeDestination.Y - resolved.Y;
        if (verticalDrop < -_config.PathTolerance || verticalDrop > FlightLandingMaxSafeVerticalDrop)
            return null;

        return resolved;
    }

    private static float HorizontalDistanceXZ(Vector3 left, Vector3 right)
    {
        var dx = left.X           - right.X;
        var dz = left.Z           - right.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
    }

    private Vector3? TryBuildFlightGroundApproachPoint(Vector3 safeFlightDestination, Vector3 transitionPoint, Vector3 groundLeadTarget, Vector3 requestedTarget)
    {
        var horizontalGap = HorizontalDistanceXZ(safeFlightDestination, transitionPoint);
        if (horizontalGap > FlightGroundApproachTriggerHorizontal)
            return null;

        var verticalDrop = safeFlightDestination.Y - transitionPoint.Y;
        if (verticalDrop <= FlightGroundTransitionSlack)
            return null;

        var leadDelta = new Vector2(groundLeadTarget.X - transitionPoint.X, groundLeadTarget.Z - transitionPoint.Z);
        if (leadDelta.LengthSquared() <= 0.000001f)
            leadDelta = new Vector2(requestedTarget.X - transitionPoint.X, requestedTarget.Z - transitionPoint.Z);
        if (leadDelta.LengthSquared() <= 0.000001f)
            return null;

        leadDelta = Vector2.Normalize(leadDelta);
        var approachHorizontal = Math.Clamp(verticalDrop * 0.75f, FlightGroundApproachMinHorizontal, FlightGroundApproachMaxHorizontal);
        var candidate = new Vector3
        (
            transitionPoint.X - leadDelta.X  * approachHorizontal,
            transitionPoint.Y + verticalDrop * FlightGroundApproachHeightRatio,
            transitionPoint.Z - leadDelta.Y  * approachHorizontal
        );

        var approachVoxel = FindNearestVolumeVoxel(candidate, FlightGroundApproachMinHorizontal, MathF.Max(1f, verticalDrop * 0.5f));
        if (approachVoxel == VoxelMap.INVALID_VOXEL)
            return candidate;

        return VoxelSearch.FindClosestVoxelPoint(VolumeQuery!.Volume, approachVoxel, candidate);
    }

    private void TrimFlightWaypointsForGroundTransition(List<Vector3> flightWaypoints, Vector3 approachPoint)
    {
        if (VolumeQuery == null || flightWaypoints.Count < 2)
            return;

        var volume       = VolumeQuery.Volume;
        var approachLeaf = volume.FindLeafVoxel(approachPoint);
        if (!approachLeaf.empty || approachLeaf.voxel == VoxelMap.INVALID_VOXEL)
            return;

        while (flightWaypoints.Count >= 2)
        {
            var previousPoint = flightWaypoints[^2];
            var previousLeaf  = volume.FindLeafVoxel(previousPoint);
            if (!previousLeaf.empty || previousLeaf.voxel == VoxelMap.INVALID_VOXEL)
                break;

            if (!VoxelSearch.LineOfSight(volume, previousLeaf.voxel, approachLeaf.voxel, previousPoint, approachPoint))
                break;

            flightWaypoints.RemoveAt(flightWaypoints.Count - 1);
        }
    }

    private bool TryBuildFlightGroundTransitionResult
    (
        Vector3           requestedStart,
        Vector3           requestedTarget,
        Vector3           safeFlightDestination,
        List<Vector3>     rawFlightWaypoints,
        bool              useRaycast,
        CancellationToken cancel,
        out PlannerResult result
    )
    {
        var groundResult = PlanMeshPathDetailed(safeFlightDestination, requestedTarget, useRaycast, 0, cancel);

        if (!groundResult.Succeeded || groundResult.Segments.Count == 0)
        {
            result = null!;
            return false;
        }

        var transitionPoint = groundResult.Segments[0].StartPosition;
        var approachPoint = TryBuildFlightGroundApproachPoint(safeFlightDestination, transitionPoint, groundResult.Segments[0].EndPosition, requestedTarget);
        List<Vector3> flightWaypoints = [.. rawFlightWaypoints];

        if (approachPoint is { } resolvedApproachPoint)
        {
            TrimFlightWaypointsForGroundTransition(flightWaypoints, resolvedApproachPoint);
            if (flightWaypoints.Count == 0 || Vector3.DistanceSquared(flightWaypoints[^1], resolvedApproachPoint) > 0.000001f)
                flightWaypoints.Add(resolvedApproachPoint);
        }

        if (flightWaypoints.Count == 0 || Vector3.DistanceSquared(flightWaypoints[^1], transitionPoint) > 0.000001f)
            flightWaypoints.Add(transitionPoint);

        List<PlannerPathSegment> segments =
        [
            new()
            {
                MovementMode         = MovementMode.Flight,
                SegmentKind          = MovementSegmentKind.FlightTraverse,
                AllowVerticalControl = true,
                ReachabilitySource   = PathReachabilitySource.Volume,
                GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                StartPosition        = requestedStart,
                EndPosition          = transitionPoint,
                Points               = flightWaypoints
            }
        ];
        foreach (var segment in groundResult.Segments)
            segments.Add(segment);

        var transitionAdjusted = Vector3.Distance(safeFlightDestination, transitionPoint) > FlightGroundTransitionSlack;
        Service.Log.Debug
        (
            $"[算路] 飞行接地面续算：空体素终点 = {safeFlightDestination:f3}，近地点 = {(approachPoint is { } ap ? ap.ToString("f3") : "无")}，桥接点 = {transitionPoint:f3}，桥接修正 = {(transitionAdjusted ? "是" : "否")}，地面结果 = {groundResult.Status}，地面段数 = {groundResult.Segments.Count}"
        );

        var destinationTolerance = MathF.Max(groundResult.DestinationTolerance, MathF.Max(_config.PathTolerance, FlightLandingCompletionSlack));

        result = new()
        {
            Status               = groundResult.Status,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = requestedTarget,
            FinalDestination     = groundResult.FinalDestination,
            DestinationTolerance = destinationTolerance,
            Segments             = segments
        };
        return true;
    }

    private static PlannerResult CreateFlightFailure(Vector3 destination) =>
        new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = destination,
            FinalDestination     = destination,
            DestinationTolerance = 0
        };

    private static string DescribeVolumeSearchTermination(VolumeSearchTermination termination) => termination switch
    {
        VolumeSearchTermination.ReachedGoal       => "达到终点",
        VolumeSearchTermination.SearchExhausted   => "搜索穷尽",
        VolumeSearchTermination.StepBudgetReached => "步数触顶",
        _                                         => "未知"
    };
}
