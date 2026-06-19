using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume.Utils;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private List<(ulong voxel, Vector3 p)> SimplifyPath(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return path;

        List<(ulong voxel, Vector3 p)> simplified = [path[0]];
        var                            anchorIndex = 0;

        while (anchorIndex < path.Count - 1)
        {
            var furthestVisibleIndex = FindFurthestVisibleIndex(path, anchorIndex, cancel);
            simplified.Add(path[furthestVisibleIndex]);
            anchorIndex = furthestVisibleIndex;
        }

        return simplified;
    }

    private List<(ulong voxel, Vector3 p)> RefineSimplifiedPath(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
        {
            LastPathDebug = null;
            return path;
        }

        var refined = SimplifyPath(path, cancel);
        refined.Reverse();
        refined = SimplifyPath(refined, cancel);
        refined.Reverse();

        for (var iteration = 0; iteration < REFINE_RELAX_ITERATION_LIMIT; ++iteration)
        {
            ProjectInteriorWaypoints(refined, cancel);
            RelaxTowardOpenSpace(refined, debugInfos: null, cancel);
            var straightened = SimplifyPath(refined, cancel);
            if (straightened.Count == refined.Count)
            {
                refined = straightened;
                break;
            }

            refined = straightened;
        }

        var debugInfos = new FlightPathWaypointDebug?[refined.Count];
        ProjectInteriorWaypoints(refined, cancel);
        RelaxTowardOpenSpace(refined, debugInfos, cancel);

        var finalPath = SimplifyPath(refined, cancel);
        finalPath     = RestoreSteepDescentWaypoints(refined, finalPath);
        LastPathDebug = BuildFlightPathDebugPayload(refined, debugInfos, finalPath, pendingLongRangeProxyDebug);
        return finalPath;
    }

    private void ProjectInteriorWaypoints(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return;

        for (var i = 1; i < path.Count - 1; ++i)
        {
            if ((i & 0x3f) == 0)
                cancel.ThrowIfCancellationRequested();

            var previous = path[i - 1];
            var current  = path[i];
            var next     = path[i + 1];

            if (current.voxel == goalVoxel)
                continue;

            var segment       = next.p - previous.p;
            var lengthSquared = segment.LengthSquared();
            if (lengthSquared <= SCORE_EPSILON * SCORE_EPSILON)
                continue;

            var progress  = Math.Clamp(Vector3.Dot(current.p - previous.p, segment) / lengthSquared, 0f, 1f);
            var projected = previous.p + (progress * segment);
            var relaxed   = Volume.ClampPointToVoxel(current.voxel, projected);

            if (Vector3.DistanceSquared(relaxed, current.p) <= SCORE_EPSILON * SCORE_EPSILON)
                continue;
            if (!HasLineOfSight(previous, current.voxel, relaxed))
                continue;
            if (!HasLineOfSight((current.voxel, relaxed), next.voxel, next.p))
                continue;

            path[i] = (current.voxel, relaxed);
        }
    }

    private void RelaxTowardOpenSpace(List<(ulong voxel, Vector3 p)> path, FlightPathWaypointDebug?[]? debugInfos, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return;

        for (var i = 1; i < path.Count - 1; ++i)
        {
            if ((i & 0x1f) == 0)
                cancel.ThrowIfCancellationRequested();

            var previous = path[i - 1];
            var current  = path[i];
            var next     = path[i + 1];

            if (TryRelaxWaypoint(previous, current, next, i, out var adjusted, out var debugInfo))
                path[i] = adjusted;

            if (debugInfos != null)
                debugInfos[i] = debugInfo;
        }
    }

    private bool TryRelaxWaypoint
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        int                          pathIndex,
        out (ulong voxel, Vector3 p) adjusted,
        out FlightPathWaypointDebug? debugInfo
    )
    {
        adjusted  = current;
        debugInfo = null;

        var overallDirection = next.p - previous.p;
        if (overallDirection.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
            return false;

        var horizontalForward = new Vector2(overallDirection.X, overallDirection.Z);
        if (!VoxelMathUtil.TryNormalize(horizontalForward, out var normalizedForward))
        {
            horizontalForward = new Vector2(next.p.X - current.p.X, next.p.Z - current.p.Z);
            if (!VoxelMathUtil.TryNormalize(horizontalForward, out normalizedForward))
            {
                horizontalForward = new Vector2(current.p.X - previous.p.X, current.p.Z - previous.p.Z);
                normalizedForward = VoxelMathUtil.TryNormalize(horizontalForward, out var fallback) ? fallback : Vector2.UnitX;
            }
        }

        var forward3       = new Vector3(normalizedForward.X, 0, normalizedForward.Y);
        var right3         = new Vector3(-normalizedForward.Y, 0, normalizedForward.X);
        var forwardRight3  = Vector3.Normalize(forward3 + right3);
        var forwardLeft3   = Vector3.Normalize(forward3 - right3);
        var backwardRight3 = Vector3.Normalize(-forward3 + right3);
        var backwardLeft3  = Vector3.Normalize(-forward3 - right3);

        var voxelSize          = GetVoxelSize(current.voxel);
        var leafHorizontalSize = MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z);
        var leafVerticalSize   = l2Desc.CellSize.Y;
        var voxelHorizontal    = MathF.Max(voxelSize.X, voxelSize.Z);
        var voxelVertical      = voxelSize.Y;

        var horizontalSampleStep = MathF.Max
        (
            FLIGHT_PUSH_MIN_DISTANCE,
            MathF.Max(leafHorizontalSize * FLIGHT_PUSH_SAMPLE_STEP_SCALE, MathF.Min(voxelHorizontal * 0.5f, leafHorizontalSize * FLIGHT_PUSH_SAMPLE_STEP_MAX_SCALE))
        );
        var verticalSampleStep = MathF.Max
        (
            FLIGHT_PUSH_MIN_DISTANCE,
            MathF.Max(leafVerticalSize * FLIGHT_PUSH_SAMPLE_STEP_SCALE, MathF.Min(voxelVertical * 0.5f, leafVerticalSize * FLIGHT_PUSH_SAMPLE_STEP_MAX_SCALE))
        );
        var horizontalScanDistance = MathF.Min
        (
            MathF.Max(voxelHorizontal, leafHorizontalSize * FLIGHT_PUSH_SCAN_DISTANCE_SCALE),
            leafHorizontalSize * FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS
        );
        var verticalScanDistance = MathF.Min
        (
            MathF.Max(voxelVertical, leafVerticalSize * FLIGHT_PUSH_SCAN_DISTANCE_SCALE),
            leafVerticalSize * FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS
        );

        var forwardSample       = MeasureDirectionalClearance(current, forward3,       horizontalScanDistance, horizontalSampleStep);
        var backwardSample      = MeasureDirectionalClearance(current, -forward3,      horizontalScanDistance, horizontalSampleStep);
        var rightSample         = MeasureDirectionalClearance(current, right3,         horizontalScanDistance, horizontalSampleStep);
        var leftSample          = MeasureDirectionalClearance(current, -right3,        horizontalScanDistance, horizontalSampleStep);
        var forwardRightSample  = MeasureDirectionalClearance(current, forwardRight3,  horizontalScanDistance, horizontalSampleStep);
        var forwardLeftSample   = MeasureDirectionalClearance(current, forwardLeft3,   horizontalScanDistance, horizontalSampleStep);
        var backwardRightSample = MeasureDirectionalClearance(current, backwardRight3, horizontalScanDistance, horizontalSampleStep);
        var backwardLeftSample  = MeasureDirectionalClearance(current, backwardLeft3,  horizontalScanDistance, horizontalSampleStep);
        var upSample            = MeasureDirectionalClearance(current, Vector3.UnitY,  verticalScanDistance,   verticalSampleStep);
        var downSample          = MeasureDirectionalClearance(current, -Vector3.UnitY, verticalScanDistance,   verticalSampleStep);

        var forwardClearance       = forwardSample.Clearance;
        var backwardClearance      = backwardSample.Clearance;
        var rightClearance         = rightSample.Clearance;
        var leftClearance          = leftSample.Clearance;
        var forwardRightClearance  = forwardRightSample.Clearance;
        var forwardLeftClearance   = forwardLeftSample.Clearance;
        var backwardRightClearance = backwardRightSample.Clearance;
        var backwardLeftClearance  = backwardLeftSample.Clearance;
        var upClearance            = upSample.Clearance;
        var downClearance          = downSample.Clearance;

        var preferredHorizontal = MathF.Max(voxelHorizontal * FLIGHT_PUSH_PREFERRED_CLEARANCE_VOXEL_SCALE, leafHorizontalSize * FLIGHT_PUSH_PREFERRED_CLEARANCE_LEAF_SCALE);
        var preferredVertical   = MathF.Max(voxelVertical   * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_VOXEL_SCALE, leafVerticalSize   * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_LEAF_SCALE);

        var minHorizontalClearance =
            MathF.Min(MathF.Min(forwardClearance, backwardClearance),
            MathF.Min(MathF.Min(rightClearance, leftClearance),
            MathF.Min(MathF.Min(forwardRightClearance, forwardLeftClearance),
                      MathF.Min(backwardRightClearance, backwardLeftClearance))));
        var maxHorizontalClearance =
            MathF.Max(MathF.Max(forwardClearance, backwardClearance),
            MathF.Max(MathF.Max(rightClearance, leftClearance),
            MathF.Max(MathF.Max(forwardRightClearance, forwardLeftClearance),
                      MathF.Max(backwardRightClearance, backwardLeftClearance))));

        var horizontalCramped = minHorizontalClearance < preferredHorizontal;
        var floorCramped      = downClearance < preferredVertical && upClearance > FLIGHT_PUSH_MIN_DISTANCE;
        var ceilingCramped    = upClearance   < preferredVertical && downClearance > FLIGHT_PUSH_MIN_DISTANCE;

        Vector3 horizontalBias           = Vector3.Zero;
        var     horizontalTotalClearance = 0f;

        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forward3,        forwardClearance,       1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -forward3,       backwardClearance,      1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, right3,          rightClearance,         1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -right3,         leftClearance,          1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardRight3,   forwardRightClearance,  1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardLeft3,    forwardLeftClearance,   1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardRight3,  backwardRightClearance, 1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardLeft3,   backwardLeftClearance,  1f);

        Vector3 horizontalOffset = Vector3.Zero;
        if (horizontalCramped && VoxelMathUtil.TryNormalize(new Vector2(horizontalBias.X, horizontalBias.Z), out var horizontalPushDirection))
        {
            var deficit               = preferredHorizontal - minHorizontalClearance;
            var maxHorizontalPush     = MathF.Min(horizontalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxHorizontalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var horizontalPushDistance = MathF.Min(maxHorizontalPush, deficit * FLIGHT_PUSH_RELIEF_SCALE);
            if (horizontalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
                horizontalOffset = new Vector3(horizontalPushDirection.X * horizontalPushDistance, 0, horizontalPushDirection.Y * horizontalPushDistance);
        }

        Vector3                 verticalOffset = Vector3.Zero;
        FlightPathVerticalMode verticalMode    = FlightPathVerticalMode.None;

        if (floorCramped)
        {
            var deficit             = preferredVertical - downClearance;
            var maxVerticalPush     = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, upClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush, deficit * FLIGHT_PUSH_RELIEF_SCALE);
            if (verticalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
            {
                verticalOffset = Vector3.UnitY * verticalPushDistance;
                verticalMode   = FlightPathVerticalMode.FloorAvoidance;
            }
        }
        else if (ceilingCramped)
        {
            var deficit             = preferredVertical - upClearance;
            var maxVerticalPush     = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, downClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush, deficit * FLIGHT_PUSH_RELIEF_SCALE);
            if (verticalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
            {
                verticalOffset = -Vector3.UnitY * verticalPushDistance;
                verticalMode   = FlightPathVerticalMode.DownwardBias;
            }
        }

        var combinedOffset   = horizontalOffset + verticalOffset;
        var selectedKind     = FlightPathAdjustmentKind.None;
        var accepted         = false;

        if (TryAcceptScaledOffset(previous, current, next, combinedOffset, out adjusted))
        {
            selectedKind = FlightPathAdjustmentKind.NeutralCombined;
            accepted     = true;
        }
        else if (TryAcceptScaledOffset(previous, current, next, horizontalOffset, out adjusted))
        {
            selectedKind = FlightPathAdjustmentKind.NeutralHorizontalOnly;
            accepted     = true;
        }
        else if (TryAcceptScaledOffset(previous, current, next, verticalOffset, out adjusted))
        {
            selectedKind = FlightPathAdjustmentKind.NeutralVerticalOnly;
            accepted     = true;
        }
        else adjusted = current;

        var horizontalImbalance = horizontalTotalClearance > SCORE_EPSILON
                                      ? new Vector2(horizontalBias.X, horizontalBias.Z).Length() / horizontalTotalClearance
                                      : 0f;
        var verticalTotal     = upClearance + downClearance;
        var verticalImbalance = verticalTotal > SCORE_EPSILON ? MathF.Abs(upClearance - downClearance) / verticalTotal : 0f;
        var maxClearance      = MathF.Max
        (
            MathF.Max(horizontalScanDistance, verticalScanDistance),
            MathF.Max
            (
                MathF.Max(MathF.Max(forwardClearance, backwardClearance), MathF.Max(rightClearance, leftClearance)),
                MathF.Max
                (
                    MathF.Max(MathF.Max(forwardRightClearance, forwardLeftClearance), MathF.Max(backwardRightClearance, backwardLeftClearance)),
                    MathF.Max(upClearance, downClearance)
                )
            )
        );

        List<FlightPathDebugSample> samples =
        [
            BuildFlightDebugSample(FlightPathDebugSampleKind.Forward,       current.p, forwardSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Backward,      current.p, backwardSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Right,         current.p, rightSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Left,          current.p, leftSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.ForwardRight,  current.p, forwardRightSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.ForwardLeft,   current.p, forwardLeftSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.BackwardRight, current.p, backwardRightSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.BackwardLeft,  current.p, backwardLeftSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Up,            current.p, upSample),
            BuildFlightDebugSample(FlightPathDebugSampleKind.Down,          current.p, downSample)
        ];

        var goalDescentApproach = next.voxel == goalVoxel && next.p.Y + preferredVertical < current.p.Y;

        debugInfo = new FlightPathWaypointDebug
        (
            pathIndex,
            current.voxel,
            adjusted.voxel,
            current.p,
            adjusted.p,
            current.p + horizontalOffset,
            current.p + verticalOffset,
            current.p + combinedOffset,
            horizontalOffset.Length(),
            MathF.Abs(verticalOffset.Y),
            Vector3.Distance(current.p, adjusted.p),
            horizontalImbalance,
            verticalImbalance,
            forwardClearance,
            backwardClearance,
            leftClearance,
            rightClearance,
            forwardLeftClearance,
            forwardRightClearance,
            backwardLeftClearance,
            backwardRightClearance,
            upClearance,
            downClearance,
            maxClearance,
            verticalMode,
            selectedKind,
            goalDescentApproach,
            DownhillTunnelTrend: false,
            ConstrainedTunnelDescent: false,
            TunnelDescentAssist: false,
            HeightCatchUpRequested: false,
            AllowDownwardPush: ceilingCramped,
            FinalRaiseApplied: false,
            HeightMatchTarget: current.p.Y,
            PreferredMinHeight: current.p.Y,
            BaseAdjustedPosition: adjusted.p,
            samples
        );

        return accepted && Vector3.DistanceSquared(current.p, adjusted.p) > SCORE_EPSILON * SCORE_EPSILON;
    }

    private FlightPushProbeResult MeasureDirectionalClearance
    (
        (ulong voxel, Vector3 p) origin,
        Vector3                  direction,
        float                    maxDistance,
        float                    stepDistance
    )
    {
        if (direction.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON ||
            maxDistance               <= FLIGHT_PUSH_MIN_DISTANCE      ||
            stepDistance              <= FLIGHT_PUSH_MIN_DISTANCE)
            return new(0, origin.p);

        direction = Vector3.Normalize(direction);
        var clearance = 0f;
        var endpoint  = origin.p;

        for (var distance = stepDistance; distance <= maxDistance + SCORE_EPSILON; distance += stepDistance)
        {
            var probePoint = origin.p + (direction * distance);
            if (!TryResolveEmptyWaypoint(probePoint, direction, out var probe))
                break;
            if (!HasLineOfSight(origin, probe.voxel, probe.p))
                break;

            clearance = distance;
            endpoint  = probe.p;
        }

        return new(clearance, endpoint);
    }

    private bool TryAcceptScaledOffset
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        Vector3                      offset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (offset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        Span<float> scales = [1f, 0.85f, 0.70f, 0.55f, 0.40f, 0.25f];

        foreach (var t in scales)
        {
            if (TryAcceptScaledOffset(previous, current, next, offset, t, out adjusted))
                return true;
        }

        adjusted = current;
        return false;
    }

    private bool TryAcceptScaledOffset
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        Vector3                      offset,
        float                        scale,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (scale <= 0)
            return false;

        var candidatePoint = current.p + (offset * scale);
        if (!TryResolveEmptyWaypoint(candidatePoint, offset, out var candidate))
            return false;
        if (Vector3.DistanceSquared(candidate.p, current.p) <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;
        if (!HasLineOfSight(previous, candidate.voxel, candidate.p))
            return false;
        if (!HasLineOfSight(candidate, next.voxel, next.p))
            return false;

        adjusted = candidate;
        return true;
    }

    private bool TryResolveEmptyWaypoint(Vector3 point, Vector3 directionHint, out (ulong voxel, Vector3 p) waypoint)
    {
        var resolved = Volume.FindLeafVoxel(point);

        if (!resolved.empty || resolved.voxel == VoxelMap.INVALID_VOXEL)
        {
            if (!VoxelMathUtil.TryNormalize(directionHint, out var normalizedHint))
            {
                waypoint = default;
                return false;
            }

            var nudgeDistance = MathF.Max(ResolveLeafInset(), FLIGHT_PUSH_MIN_DISTANCE);
            point    -= normalizedHint * nudgeDistance;
            resolved =  Volume.FindLeafVoxel(point);

            if (!resolved.empty || resolved.voxel == VoxelMap.INVALID_VOXEL)
            {
                waypoint = default;
                return false;
            }
        }

        point    = Volume.ClampPointToVoxel(resolved.voxel, point, ResolveVoxelInset(resolved.voxel));
        waypoint = (resolved.voxel, point);
        return true;
    }

    private Vector3 GetVoxelSize(ulong voxel)
    {
        var (min, max) = Volume.VoxelBounds(voxel, 0);
        return max - min;
    }

    private float ResolveVoxelInset(ulong voxel)
    {
        var voxelSize   = GetVoxelSize(voxel);
        var minExtent   = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
        var scaledInset = minExtent * FLIGHT_PUSH_VOXEL_INSET_RATIO;
        return Math.Clamp(scaledInset, FLIGHT_PUSH_VOXEL_INSET_MIN, FLIGHT_PUSH_VOXEL_INSET_MAX);
    }

    private float ResolveLeafInset()
    {
        var minLeafExtent = MathF.Min(l2Desc.CellSize.X, MathF.Min(l2Desc.CellSize.Y, l2Desc.CellSize.Z));
        var scaledInset   = minLeafExtent * FLIGHT_PUSH_VOXEL_INSET_RATIO;
        return Math.Clamp(scaledInset, FLIGHT_PUSH_VOXEL_INSET_MIN, FLIGHT_PUSH_VOXEL_INSET_MAX);
    }

    private static void AccumulateDirectionalBias
    (
        ref Vector3 bias,
        ref float   totalClearance,
        Vector3     direction,
        float       clearance,
        float       weight
    )
    {
        if (clearance <= SCORE_EPSILON || weight <= SCORE_EPSILON)
            return;

        var contribution = clearance * weight;
        bias           += direction * contribution;
        totalClearance += contribution;
    }

    private static FlightPathDebugSample BuildFlightDebugSample(FlightPathDebugSampleKind kind, Vector3 start, FlightPushProbeResult sample)
        => new(kind, start, sample.Endpoint, sample.Clearance);

    private static FlightPathDebugPayload? BuildFlightPathDebugPayload
    (
        IReadOnlyList<(ulong voxel, Vector3 p)> refinedPath,
        IReadOnlyList<FlightPathWaypointDebug?> debugInfos,
        IReadOnlyList<(ulong voxel, Vector3 p)> finalPath,
        FlightLongRangeProxyDebug?              proxyDebug = null
    )
    {
        if (refinedPath.Count == 0 || debugInfos.Count == 0 || finalPath.Count == 0)
            return null;

        List<FlightPathWaypointDebug> remapped    = [];
        var                           searchStart = 0;

        for (var finalIndex = 0; finalIndex < finalPath.Count; ++finalIndex)
        {
            var finalPoint = finalPath[finalIndex];

            for (var refinedIndex = searchStart; refinedIndex < refinedPath.Count; ++refinedIndex)
            {
                var refinedPoint = refinedPath[refinedIndex];
                if (Vector3.DistanceSquared(refinedPoint.p, finalPoint.p) > SCORE_EPSILON * SCORE_EPSILON)
                    continue;

                if (refinedIndex < debugInfos.Count && debugInfos[refinedIndex] is { } debug)
                    remapped.Add(debug with { PathIndex = finalIndex });

                searchStart = refinedIndex + 1;
                break;
            }
        }

        if (remapped.Count == 0 && proxyDebug == null)
            return null;

        return new()
        {
            Waypoints  = remapped,
            CoarsePath = [],
            ProxyDebug = proxyDebug
        };
    }

    private List<(ulong voxel, Vector3 p)> RestoreSteepDescentWaypoints
    (
        IReadOnlyList<(ulong voxel, Vector3 p)> refined,
        IReadOnlyList<(ulong voxel, Vector3 p)> simplified
    )
    {
        if (refined.Count == 0 || simplified.Count <= 1)
            return [.. simplified];

        List<(ulong voxel, Vector3 p)> restored           = [simplified[0]];
        var                            refinedSearchStart = 0;

        for (var simplifiedIndex = 1; simplifiedIndex < simplified.Count; ++simplifiedIndex)
        {
            var restoredStart = restored[^1];
            var segmentEnd    = simplified[simplifiedIndex];
            var startIndex    = VoxelPathUtil.FindPathPointIndex(refined, restoredStart, refinedSearchStart, SCORE_EPSILON);

            if (startIndex < 0)
            {
                VoxelPathUtil.AppendPathPoint(restored, segmentEnd, SCORE_EPSILON);
                continue;
            }

            var endIndex = VoxelPathUtil.FindPathPointIndex(refined, segmentEnd, startIndex + 1, SCORE_EPSILON);

            if (endIndex < 0)
            {
                VoxelPathUtil.AppendPathPoint(restored, segmentEnd, SCORE_EPSILON);
                continue;
            }

            AppendSteepDescentAwareSegment(restored, refined, startIndex, endIndex);
            refinedSearchStart = endIndex;
        }

        return restored;
    }

    private void AppendSteepDescentAwareSegment
    (
        List<(ulong voxel, Vector3 p)>          output,
        IReadOnlyList<(ulong voxel, Vector3 p)> refined,
        int                                     startIndex,
        int                                     endIndex
    )
    {
        var currentIndex = startIndex;

        while (currentIndex < endIndex)
        {
            var nextIndex = endIndex;

            if (NeedsFlightDescentSmoothing(refined[currentIndex].p, refined[endIndex].p))
            {
                nextIndex = currentIndex + 1;

                for (var probeIndex = endIndex - 1; probeIndex > currentIndex; --probeIndex)
                {
                    if (NeedsFlightDescentSmoothing(refined[currentIndex].p, refined[probeIndex].p))
                        continue;

                    nextIndex = probeIndex;
                    break;
                }
            }

            VoxelPathUtil.AppendPathPoint(output, refined[nextIndex], SCORE_EPSILON);
            currentIndex = nextIndex;
        }
    }

    private bool NeedsFlightDescentSmoothing(Vector3 from, Vector3 to)
    {
        var verticalDrop = from.Y - to.Y;
        if (verticalDrop <= ResolveFlightDescentSmoothingMinDrop())
            return false;

        var horizontalDistance = VoxelMathUtil.HorizontalDistanceXZ(from, to);
        if (horizontalDistance <= ResolveFlightDescentNearVerticalDistance())
            return true;

        return verticalDrop / horizontalDistance >= FLIGHT_DESCENT_SMOOTHING_MAX_SLOPE;
    }

    private float ResolveFlightDescentSmoothingMinDrop()
        => MathF.Max(l2Desc.CellSize.Y * FLIGHT_DESCENT_SMOOTHING_MIN_DROP_LEAF_SCALE, FLIGHT_DESCENT_SMOOTHING_MIN_DROP_MIN);

    private float ResolveFlightDescentNearVerticalDistance()
        => MathF.Max
            (MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z) * FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_LEAF_SCALE, FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_MIN);

    private int FindFurthestVisibleIndex(List<(ulong voxel, Vector3 p)> path, int anchorIndex, CancellationToken cancel)
    {
        var pathLastIndex = path.Count  - 1;
        var nextIndex     = anchorIndex + 1;
        if (nextIndex >= pathLastIndex || !HasLineOfSight(path, anchorIndex, nextIndex, cancel))
            return nextIndex;

        var furthestVisibleIndex = nextIndex;
        var step                 = 1;

        while (furthestVisibleIndex < pathLastIndex)
        {
            var probeIndex = Math.Min(furthestVisibleIndex + step, pathLastIndex);
            if (!HasLineOfSight(path, anchorIndex, probeIndex, cancel))
                return FindVisibleBoundary(path, anchorIndex, furthestVisibleIndex, probeIndex - 1, cancel);

            furthestVisibleIndex =   probeIndex;
            step                 <<= 1;
        }

        return furthestVisibleIndex;
    }

    private int FindVisibleBoundary
    (
        List<(ulong voxel, Vector3 p)> path,
        int                            anchorIndex,
        int                            visibleIndex,
        int                            blockedIndex,
        CancellationToken              cancel
    )
    {
        while (visibleIndex < blockedIndex)
        {
            var mid = visibleIndex + ((blockedIndex - visibleIndex + 1) >> 1);
            if (HasLineOfSight(path, anchorIndex, mid, cancel)) visibleIndex = mid;
            else blockedIndex                                                = mid - 1;
        }

        return visibleIndex;
    }

    private bool HasLineOfSight(List<(ulong voxel, Vector3 p)> path, int anchorIndex, int probeIndex, CancellationToken cancel)
    {
        if ((probeIndex & 0x3f) == 0)
            cancel.ThrowIfCancellationRequested();

        var anchor = path[anchorIndex];
        var probe  = path[probeIndex];
        ++lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, anchor.voxel, probe.voxel, anchor.p, probe.p))
            return false;

        ++lineOfSightHits;
        return true;
    }

    private bool HasLineOfSight((ulong voxel, Vector3 p) from, ulong toVoxel, Vector3 toPosition)
    {
        ++lineOfSightChecks;
        if (!VoxelSearch.LineOfSight(Volume, from.voxel, toVoxel, from.p, toPosition))
            return false;

        ++lineOfSightHits;
        return true;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode(int nodeIndex, bool returnIntermediatePoints)
    {
        var result = new List<(ulong voxel, Vector3 p)>();

        if ((uint)nodeIndex >= (uint)nodes.Count)
            return result;

        var nodeSpan = NodeSpan;
        result.Add((nodeSpan[nodeIndex].Voxel, nodeSpan[nodeIndex].Position));

        while (nodeSpan[nodeIndex].ParentIndex != nodeIndex)
        {
            ref var child       = ref nodeSpan[nodeIndex];
            var     parentIndex = child.ParentIndex;
            ref var parent      = ref nodeSpan[parentIndex];

            if (returnIntermediatePoints)
            {
                var delta = parent.Position - child.Position;

                foreach (var step in VoxelSearch.EnumerateVoxelsInLine(Volume, child.Voxel, parent.Voxel, child.Position, parent.Position))
                {
                    if (!step.empty)
                        continue;

                    result.Add((step.voxel, child.Position + (step.t * delta)));
                }
            }
            else result.Add((parent.Voxel, parent.Position));

            nodeIndex = parentIndex;
        }

        result.Reverse();
        return result;
    }
}
