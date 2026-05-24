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

        List<(ulong voxel, Vector3 p)> simplified  = [path[0]];
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
        RelaxInteriorWaypoints(refined, cancel);
        var debugInfos = new FlightPathWaypointDebug?[refined.Count];
        PushInteriorWaypoints(refined, debugInfos, cancel);
        var finalPath = SimplifyPath(refined, cancel);
        finalPath     = RestoreSteepDescentWaypoints(refined, finalPath);
        LastPathDebug = BuildFlightPathDebugPayload(refined, debugInfos, finalPath, pendingLongRangeProxyDebug);
        return finalPath;
    }

    private void RelaxInteriorWaypoints(List<(ulong voxel, Vector3 p)> path, CancellationToken cancel)
    {
        if (path.Count <= 2)
            return;

        for (var i = 1; i < path.Count - 1; ++i)
        {
            if ((i & 0x3f) == 0)
                cancel.ThrowIfCancellationRequested();

            var     previous = path[i - 1];
            var     current  = path[i];
            var     next     = path[i + 1];
            Vector3 relaxed;

            if (current.voxel == goalVoxel) relaxed = goalPos;
            else
            {
                var segment       = next.p - previous.p;
                var lengthSquared = segment.LengthSquared();
                if (lengthSquared <= SCORE_EPSILON * SCORE_EPSILON)
                    continue;

                var progress  = Math.Clamp(Vector3.Dot(current.p - previous.p, segment) / lengthSquared, 0f, 1f);
                var projected = previous.p + (progress * segment);
                relaxed = Volume.ClampPointToVoxel(current.voxel, projected);
            }

            if (Vector3.DistanceSquared(relaxed, current.p) <= SCORE_EPSILON * SCORE_EPSILON)
                continue;
            if (!HasLineOfSight(previous, current.voxel, relaxed))
                continue;
            if (!HasLineOfSight((current.voxel, relaxed), next.voxel, next.p))
                continue;

            path[i] = (current.voxel, relaxed);
        }
    }

    private void PushInteriorWaypoints(List<(ulong voxel, Vector3 p)> path, FlightPathWaypointDebug?[] debugInfos, CancellationToken cancel)
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

            var pushed = TryPushInteriorWaypoint(previous, current, next, i, out var adjusted, out var debugInfo);
            debugInfos[i] = debugInfo;
            if (!pushed)
                continue;

            path[i] = adjusted;
        }
    }

    private bool TryPushInteriorWaypoint
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

        if (horizontalForward.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
        {
            horizontalForward = new Vector2(next.p.X - current.p.X, next.p.Z - current.p.Z);
            if (horizontalForward.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON)
                horizontalForward = new Vector2(current.p.X - previous.p.X, current.p.Z - previous.p.Z);
        }

        horizontalForward = !VoxelMathUtil.TryNormalize(horizontalForward, out var normalizedHorizontalForward) ? Vector2.UnitX : normalizedHorizontalForward;

        var horizontalRight      = new Vector2(-horizontalForward.Y, horizontalForward.X);
        var forward3             = new Vector3(horizontalForward.X, 0, horizontalForward.Y);
        var right3               = new Vector3(horizontalRight.X,   0, horizontalRight.Y);
        var forwardRight3        = Vector3.Normalize(forward3  + right3);
        var forwardLeft3         = Vector3.Normalize(forward3  - right3);
        var backwardRight3       = Vector3.Normalize(-forward3 + right3);
        var backwardLeft3        = Vector3.Normalize(-forward3 - right3);
        var goalAdjacentRearBias = next.voxel == goalVoxel;

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

        var horizontalScanDistance = MathF.Max(voxelHorizontal, leafHorizontalSize * FLIGHT_PUSH_SCAN_DISTANCE_SCALE);
        horizontalScanDistance = MathF.Min(horizontalScanDistance, leafHorizontalSize * FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS);
        var verticalScanDistance = MathF.Max(voxelVertical, leafVerticalSize * FLIGHT_PUSH_SCAN_DISTANCE_SCALE);
        verticalScanDistance = MathF.Min(verticalScanDistance, leafVerticalSize * FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS);

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

        var horizontalBias           = Vector3.Zero;
        var horizontalTotalClearance = 0f;
        var maxHorizontalClearance   = 0f;
        var forwardWeight            = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_WEIGHT : FLIGHT_PUSH_FORWARD_WEIGHT;
        var backwardWeight           = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_BACKWARD_WEIGHT : FLIGHT_PUSH_BACKWARD_WEIGHT;
        var forwardDiagonalWeight    = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_DIAGONAL_WEIGHT : FLIGHT_PUSH_DIAGONAL_WEIGHT;
        var backwardDiagonalWeight   = goalAdjacentRearBias ? FLIGHT_PUSH_GOAL_ADJACENT_BACKWARD_DIAGONAL_WEIGHT : FLIGHT_PUSH_DIAGONAL_WEIGHT;

        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forward3, forwardClearance, forwardWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -forward3, backwardClearance, backwardWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, right3, rightClearance, 1f);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, rightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -right3, leftClearance, 1f);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, leftClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardRight3, forwardRightClearance, forwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardRightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forwardLeft3, forwardLeftClearance, forwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, forwardLeftClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardRight3, backwardRightClearance, backwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardRightClearance);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, backwardLeft3, backwardLeftClearance, backwardDiagonalWeight);
        maxHorizontalClearance = MathF.Max(maxHorizontalClearance, backwardLeftClearance);
        var sweepSamples = BuildHorizontalSweepSamples
        (
            current,
            forward3,
            right3,
            horizontalScanDistance,
            horizontalSampleStep,
            goalAdjacentRearBias,
            ref horizontalBias,
            ref horizontalTotalClearance,
            ref maxHorizontalClearance
        );
        var directionalHorizontalCandidates = BuildDirectionalHorizontalCandidates
        (
            current.p,
            forward3,
            forwardSample,
            backwardSample,
            rightSample,
            leftSample,
            forwardRightSample,
            forwardLeftSample,
            backwardRightSample,
            backwardLeftSample,
            sweepSamples,
            goalAdjacentRearBias
        );

        Vector3 horizontalOffset = default;
        var     maxHorizontalPush = MathF.Min(horizontalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxHorizontalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
        var     horizontalBiasFlat = new Vector2(horizontalBias.X, horizontalBias.Z);
        var     horizontalBiasMagnitude = horizontalBiasFlat.Length();
        var     horizontalImbalance = horizontalTotalClearance > SCORE_EPSILON ? horizontalBiasMagnitude / horizontalTotalClearance : 0f;

        if (horizontalBiasMagnitude >= FLIGHT_PUSH_MIN_DISTANCE             &&
            horizontalImbalance     >= FLIGHT_PUSH_MIN_HORIZONTAL_IMBALANCE &&
            VoxelMathUtil.TryNormalize(horizontalBiasFlat, out var horizontalPushDirection))
        {
            if (goalAdjacentRearBias)
            {
                var forwardComponent = Vector2.Dot(horizontalPushDirection, normalizedHorizontalForward);

                if (forwardComponent > FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT)
                {
                    var rearProjectedDirection = horizontalPushDirection - (normalizedHorizontalForward * forwardComponent);
                    if (!VoxelMathUtil.TryNormalize(rearProjectedDirection, out horizontalPushDirection))
                        horizontalPushDirection = default;
                }
            }

            var desiredHorizontalPush  = horizontalBiasMagnitude * FLIGHT_PUSH_HORIZONTAL_BIAS_SCALE;
            var horizontalPushDistance = MathF.Min(maxHorizontalPush, desiredHorizontalPush);
            if (horizontalPushDistance                  >= FLIGHT_PUSH_MIN_DISTANCE &&
                horizontalPushDirection.LengthSquared() > SCORE_EPSILON * SCORE_EPSILON)
                horizontalOffset = new Vector3(horizontalPushDirection.X * horizontalPushDistance, 0, horizontalPushDirection.Y * horizontalPushDistance);
        }

        Vector3 verticalOffset      = default;
        var     verticalBias        = upClearance - downClearance;
        var     verticalMagnitude   = MathF.Abs(verticalBias);
        var     verticalTotal       = upClearance + downClearance;
        var     verticalImbalance   = verticalTotal > SCORE_EPSILON ? verticalMagnitude / verticalTotal : 0f;
        var     goalDescentApproach = IsGoalDescentApproach(previous, current, next, leafVerticalSize);
        var downhillTunnelTrend = VoxelPathUtil.IsTunnelDescentTrend
            (previous, current, next, leafVerticalSize, FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_LEAF_SCALE, FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_MIN);
        var heightMatchTarget = current.p.Y;
        if (previous.p.Y >= current.p.Y - FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE)
            heightMatchTarget = MathF.Max(heightMatchTarget, previous.p.Y + FLIGHT_PUSH_HEIGHT_MATCH_BIAS);
        var constrainedTunnelDescent = VoxelPathUtil.IsConstrainedTunnelDescent
        (
            previous,
            current,
            next,
            leafVerticalSize,
            upClearance,
            heightMatchTarget,
            FLIGHT_PUSH_MIN_DISTANCE,
            FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_LEAF_SCALE,
            FLIGHT_TUNNEL_DESCENT_TREND_TOLERANCE_MIN,
            FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_LEAF_SCALE,
            FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_MIN
        );
        var preferredMinHeight      = !goalDescentApproach && !constrainedTunnelDescent ? heightMatchTarget : current.p.Y;
        var catchupHeight           = MathF.Max(0f, preferredMinHeight - current.p.Y);
        var catchupHeadroomRequired = catchupHeight + ResolveFlightHeightCatchupHeadroom(leafVerticalSize);
        var shouldCatchUpHeight = !constrainedTunnelDescent                 &&
                                  catchupHeight >= FLIGHT_PUSH_MIN_DISTANCE &&
                                  upClearance   >= catchupHeadroomRequired;

        if (downhillTunnelTrend && next.p.Y + FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE < current.p.Y)
        {
            preferredMinHeight      = current.p.Y;
            catchupHeight           = 0f;
            catchupHeadroomRequired = ResolveFlightHeightCatchupHeadroom(leafVerticalSize);
            shouldCatchUpHeight     = false;
        }

        var preferredFloorClearance = MathF.Max
            (voxelVertical * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_VOXEL_SCALE, leafVerticalSize * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_LEAF_SCALE);
        var downwardHeadroomLimit = MathF.Max
            (voxelVertical * FLIGHT_PUSH_DOWNWARD_UPWARD_BLOCKED_VOXEL_SCALE, leafVerticalSize * FLIGHT_PUSH_DOWNWARD_UPWARD_BLOCKED_LEAF_SCALE);
        var downwardClearanceFloor = MathF.Max
            (voxelVertical * FLIGHT_PUSH_DOWNWARD_MIN_CLEARANCE_VOXEL_SCALE, leafVerticalSize * FLIGHT_PUSH_DOWNWARD_MIN_CLEARANCE_LEAF_SCALE);
        var downwardLeadRequired = MathF.Max(leafVerticalSize * FLIGHT_PUSH_DOWNWARD_MIN_LEAD_LEAF_SCALE, FLIGHT_PUSH_MIN_DISTANCE * 2f);
        var allowDownwardPush = verticalBias                < 0                       &&
                                upClearance                 < downwardHeadroomLimit   &&
                                downClearance               >= downwardClearanceFloor &&
                                downClearance - upClearance >= downwardLeadRequired;
        var verticalMode        = FlightPathVerticalMode.None;
        var tunnelDescentAssist = false;

        if (VoxelPathUtil.TryResolveConstrainedTunnelDescentOffset
            (
                constrainedTunnelDescent,
                downhillTunnelTrend,
                previous,
                current,
                next,
                leafVerticalSize,
                voxelVertical,
                verticalScanDistance,
                upClearance,
                downClearance,
                FLIGHT_PUSH_MIN_DISTANCE,
                FLIGHT_TUNNEL_DESCENT_SOFT_HEADROOM_VOXEL_SCALE,
                FLIGHT_TUNNEL_DESCENT_SOFT_HEADROOM_LEAF_SCALE,
                FLIGHT_TUNNEL_DESCENT_DOWNWARD_CLEARANCE_VOXEL_SCALE,
                FLIGHT_TUNNEL_DESCENT_DOWNWARD_CLEARANCE_LEAF_SCALE,
                FLIGHT_TUNNEL_DESCENT_CLEARANCE_LEAD_LEAF_SCALE,
                FLIGHT_TUNNEL_DESCENT_CLEARANCE_LEAD_MIN,
                FLIGHT_TUNNEL_DESCENT_FOLLOW_NEXT_SCALE,
                FLIGHT_TUNNEL_DESCENT_PREVIOUS_FOLLOW_SCALE,
                FLIGHT_TUNNEL_DESCENT_EXTRA_FOLLOW_LEAF_SCALE,
                FLIGHT_TUNNEL_DESCENT_CLEARANCE_ADVANTAGE_SCALE,
                FLIGHT_PUSH_SCAN_PUSH_FRACTION,
                FLIGHT_PUSH_MAX_CLEARANCE_FRACTION,
                out var tunnelDescentOffset
            ))
        {
            verticalOffset      = tunnelDescentOffset;
            tunnelDescentAssist = true;
            verticalMode        = FlightPathVerticalMode.TunnelDescentAssist;
        }
        else if ((verticalBias      > 0                         &&
                  verticalMagnitude >= FLIGHT_PUSH_MIN_DISTANCE &&
                  verticalImbalance >= FLIGHT_PUSH_MIN_VERTICAL_IMBALANCE) ||
                 shouldCatchUpHeight)
        {
            var maxVerticalClearance = MathF.Max(upClearance, downClearance);
            var desiredVerticalPush = verticalBias > 0
                                          ? verticalMagnitude * FLIGHT_PUSH_VERTICAL_BIAS_SCALE
                                          : 0f;
            if (shouldCatchUpHeight)
                desiredVerticalPush = MathF.Max(desiredVerticalPush, catchupHeight * FLIGHT_PUSH_HEIGHT_CATCHUP_SCALE);
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxVerticalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush,                                       desiredVerticalPush);
            verticalOffset = Vector3.UnitY * verticalPushDistance;
            verticalMode = shouldCatchUpHeight && verticalBias <= 0
                               ? FlightPathVerticalMode.HeightCatchUp
                               : FlightPathVerticalMode.UpwardBias;
        }
        else if (allowDownwardPush                             &&
                 verticalMagnitude >= FLIGHT_PUSH_MIN_DISTANCE &&
                 verticalImbalance >= FLIGHT_PUSH_MIN_VERTICAL_IMBALANCE)
        {
            var desiredVerticalPush  = verticalMagnitude * FLIGHT_PUSH_VERTICAL_BIAS_SCALE * FLIGHT_PUSH_DOWNWARD_SCALE;
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, downClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush,                                       desiredVerticalPush);
            verticalOffset = -Vector3.UnitY * verticalPushDistance;
            verticalMode   = FlightPathVerticalMode.DownwardBias;
        }
        else if (downClearance < preferredFloorClearance && upClearance > FLIGHT_PUSH_MIN_DISTANCE)
        {
            var floorPressure        = preferredFloorClearance - downClearance;
            var desiredVerticalPush  = floorPressure * FLIGHT_PUSH_FLOOR_AVOIDANCE_SCALE;
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, upClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush,                                       desiredVerticalPush);

            if (verticalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
            {
                verticalOffset = Vector3.UnitY * verticalPushDistance;
                verticalMode   = FlightPathVerticalMode.FloorAvoidance;
            }
        }

        var combinedOffset         = horizontalOffset + verticalOffset;
        var adjustedResolved       = current;
        var baseAdjustedResolved   = current;
        var upwardPreferred        = verticalOffset.Y > FLIGHT_PUSH_MIN_DISTANCE;
        var downwardPreferred      = verticalOffset.Y < -FLIGHT_PUSH_MIN_DISTANCE;
        var selectedAdjustmentKind = FlightPathAdjustmentKind.None;

        if (upwardPreferred)
        {
            if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardVerticalOnly;
            }
            else if (TryAcceptAdjustedWaypoint
                         (previous, current, next, verticalOffset + (horizontalOffset * FLIGHT_PUSH_VERTICAL_FIRST_HORIZONTAL_BLEND), out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardVerticalBlend;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.UpwardHorizontalOnly;
            }
            else adjusted = current;
        }
        else if (tunnelDescentAssist)
        {
            var tunnelCombinedStrong = verticalOffset + (horizontalOffset * FLIGHT_TUNNEL_DESCENT_HORIZONTAL_BLEND_STRONG);
            var tunnelCombinedMedium = verticalOffset + (horizontalOffset * FLIGHT_TUNNEL_DESCENT_HORIZONTAL_BLEND_MEDIUM);

            if (TryAcceptAdjustedWaypoint(previous, current, next, tunnelCombinedStrong, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelCombinedStrong;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelVerticalOnly;
            }
            else if (TryAcceptDirectionalHorizontalCandidatesWithFixedVertical
                         (previous, current, next, directionalHorizontalCandidates, maxHorizontalPush, verticalOffset, forward3, goalAdjacentRearBias, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelDirectionalFixedVertical;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, tunnelCombinedMedium, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelCombinedMedium;
            }
            else if (TryAcceptAdjustedWaypointAfterVerticalDrop(previous, current, next, horizontalOffset, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelAfterVerticalDrop;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.TunnelHorizontalOnly;
            }
            else adjusted = current;
        }
        else if (downwardPreferred)
        {
            var downwardStrongOffset = verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG);
            var downwardMediumOffset = verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM);

            if (TryAcceptAdjustedWaypointWithFixedVertical(previous, current, next, horizontalOffset, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardFixedVertical;
            }
            else if (TryAcceptDirectionalHorizontalCandidatesWithFixedVertical
                         (previous, current, next, directionalHorizontalCandidates, maxHorizontalPush, verticalOffset, forward3, goalAdjacentRearBias, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardDirectionalFixedVertical;
            }
            else if (TryAcceptAdjustedWaypointAfterVerticalDrop(previous, current, next, horizontalOffset, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardAfterVerticalDrop;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, downwardStrongOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardStrongBlend;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, downwardMediumOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardMediumBlend;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardHorizontalOnly;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.DownwardVerticalOnly;
            }
            else adjusted = current;
        }
        else
        {
            if (TryAcceptAdjustedWaypoint(previous, current, next, combinedOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.NeutralCombined;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, horizontalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.NeutralHorizontalOnly;
            }
            else if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset, out adjusted))
            {
                adjustedResolved       = adjusted;
                selectedAdjustmentKind = FlightPathAdjustmentKind.NeutralVerticalOnly;
            }
            else adjusted = current;
        }

        baseAdjustedResolved = adjustedResolved;
        var finalRaiseApplied = false;

        if (!goalDescentApproach                                                                           &&
            !constrainedTunnelDescent                                                                      &&
            !(downhillTunnelTrend && next.p.Y + FLIGHT_PUSH_HEIGHT_MATCH_TOLERANCE < adjustedResolved.p.Y) &&
            preferredMinHeight > adjustedResolved.p.Y + FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE                &&
            TryRaiseWaypointToPreferredHeight(previous, adjustedResolved, next, preferredMinHeight, out var lifted))
        {
            adjusted          = lifted;
            adjustedResolved  = lifted;
            finalRaiseApplied = true;
        }

        var maxClearance = MathF.Max
        (
            MathF.Max(horizontalScanDistance, verticalScanDistance),
            MathF.Max
            (
                MathF.Max(forwardClearance, backwardClearance),
                MathF.Max
                (
                    MathF.Max(leftClearance, rightClearance),
                    MathF.Max
                    (
                        MathF.Max(forwardLeftClearance,                                     forwardRightClearance),
                        MathF.Max(MathF.Max(backwardLeftClearance, backwardRightClearance), MathF.Max(upClearance, downClearance))
                    )
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
        samples.AddRange(sweepSamples);

        debugInfo = new FlightPathWaypointDebug
        (
            pathIndex,
            current.voxel,
            adjustedResolved.voxel,
            current.p,
            adjustedResolved.p,
            current.p + horizontalOffset,
            current.p + verticalOffset,
            current.p + combinedOffset,
            horizontalOffset.Length(),
            MathF.Abs(verticalOffset.Y),
            Vector3.Distance(current.p, adjustedResolved.p),
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
            selectedAdjustmentKind,
            goalDescentApproach,
            downhillTunnelTrend,
            constrainedTunnelDescent,
            tunnelDescentAssist,
            shouldCatchUpHeight,
            allowDownwardPush,
            finalRaiseApplied,
            heightMatchTarget,
            preferredMinHeight,
            baseAdjustedResolved.p,
            samples
        );

        return debugInfo.Value.PushApplied;
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

    private bool TryAcceptAdjustedWaypoint
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

        const float SCALE100 = 1.00f;
        const float SCALE085 = 0.85f;
        const float SCALE070 = 0.70f;
        const float SCALE055 = 0.55f;
        const float SCALE040 = 0.40f;
        const float SCALE025 = 0.25f;

        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, SCALE100, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, SCALE085, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, SCALE070, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, SCALE055, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, SCALE040, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, offset, SCALE025, out adjusted))
            return true;

        adjusted = current;
        return false;
    }

    private bool TryAcceptAdjustedWaypointAfterVerticalDrop
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        Vector3                      horizontalOffset,
        Vector3                      verticalOffset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (verticalOffset.LengthSquared()   <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE ||
            horizontalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        if (!TryResolveEmptyWaypoint(current.p + verticalOffset, verticalOffset, out var lowered))
            return false;
        if (Vector3.DistanceSquared(lowered.p, current.p) <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_HIGH, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_LIGHT, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, lowered, next, horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MIN, out adjusted))
            return true;

        return false;
    }

    private bool TryAcceptAdjustedWaypointWithFixedVertical
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        Vector3                      horizontalOffset,
        Vector3                      verticalOffset,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        if (verticalOffset.LengthSquared()   <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE ||
            horizontalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE)
            return false;

        if (TryAcceptAdjustedWaypoint
                (previous, current, next, verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_STRONG), 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint
                (previous, current, next, verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_HIGH), 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint
                (previous, current, next, verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MEDIUM), 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint
                (previous, current, next, verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_LIGHT), 1.00f, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (horizontalOffset * FLIGHT_PUSH_DOWNWARD_HORIZONTAL_BLEND_MIN), 1.00f, out adjusted))
            return true;

        return false;
    }

    private bool TryAcceptDirectionalHorizontalCandidatesWithFixedVertical
    (
        (ulong voxel, Vector3 p)                     previous,
        (ulong voxel, Vector3 p)                     current,
        (ulong voxel, Vector3 p)                     next,
        IReadOnlyList<FlightPushHorizontalCandidate> candidates,
        float                                        maxHorizontalPush,
        Vector3                                      verticalOffset,
        Vector3                                      forward,
        bool                                         goalAdjacentRearBias,
        out (ulong voxel, Vector3 p)                 adjusted
    )
    {
        adjusted = current;
        if (verticalOffset.LengthSquared() <= FLIGHT_PUSH_MIN_DISTANCE * FLIGHT_PUSH_MIN_DISTANCE ||
            maxHorizontalPush              <= FLIGHT_PUSH_MIN_DISTANCE                            ||
            candidates.Count               == 0)
            return false;

        foreach (var candidate in candidates)
        {
            if (goalAdjacentRearBias &&
                Vector3.Dot(candidate.Direction, forward) > FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT)
                continue;

            var horizontalDistance = MathF.Min(maxHorizontalPush, candidate.Clearance * FLIGHT_PUSH_DIRECTIONAL_CLEARANCE_FRACTION);
            if (horizontalDistance <= FLIGHT_PUSH_MIN_DISTANCE)
                continue;

            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (candidate.Direction * horizontalDistance), 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (candidate.Direction * horizontalDistance * 0.85f), 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (candidate.Direction * horizontalDistance * 0.70f), 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (candidate.Direction * horizontalDistance * 0.55f), 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (candidate.Direction * horizontalDistance * 0.40f), 1.00f, out adjusted))
                return true;
            if (TryAcceptAdjustedWaypoint(previous, current, next, verticalOffset + (candidate.Direction * horizontalDistance * 0.25f), 1.00f, out adjusted))
                return true;
        }

        return false;
    }

    private bool TryAcceptAdjustedWaypointFromBase
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

        const float SCALE100 = 1.00f;
        const float SCALE085 = 0.85f;
        const float SCALE070 = 0.70f;
        const float SCALE055 = 0.55f;
        const float SCALE040 = 0.40f;
        const float SCALE025 = 0.25f;

        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, SCALE100, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, SCALE085, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, SCALE070, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, SCALE055, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, SCALE040, out adjusted))
            return true;
        if (TryAcceptAdjustedWaypointFromBase(previous, current, next, offset, SCALE025, out adjusted))
            return true;

        adjusted = current;
        return false;
    }

    private bool TryAcceptAdjustedWaypointFromBase
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

    private bool TryAcceptAdjustedWaypoint
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

    private bool TryRaiseWaypointToPreferredHeight
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        float                        preferredMinHeight,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;
        var requiredLift = preferredMinHeight - current.p.Y;
        if (requiredLift <= FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE)
            return false;

        var horizontal = new Vector3
        (
            current.p.X - previous.p.X,
            0,
            current.p.Z - previous.p.Z
        );
        var directionHint = horizontal.LengthSquared() > SCORE_EPSILON * SCORE_EPSILON
                                ? (Vector3.Normalize(horizontal) * FLIGHT_PUSH_HEIGHT_RAISE_HORIZONTAL_BLEND) + Vector3.UnitY
                                : Vector3.UnitY;
        directionHint = !VoxelMathUtil.TryNormalize(directionHint, out var normalizedDirectionHint) ? Vector3.UnitY : normalizedDirectionHint;

        var         attemptLift = requiredLift + FLIGHT_PUSH_HEIGHT_STRICT_BIAS;
        Span<float> scales      = [1.0f, 0.85f, 0.70f, 0.55f];

        for (var i = 0; i < 4; ++i)
        {
            var candidatePoint = current.p + (Vector3.UnitY * (attemptLift * scales[i]));
            if (!TryResolveEmptyWaypoint(candidatePoint, directionHint, out var candidate))
                continue;
            if (candidate.p.Y + FLIGHT_PUSH_HEIGHT_STRICT_TOLERANCE < preferredMinHeight)
                continue;
            if (!HasLineOfSight(previous, candidate.voxel, candidate.p))
                continue;
            if (!HasLineOfSight(candidate, next.voxel, next.p))
                continue;

            adjusted = candidate;
            return true;
        }

        return false;
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

    private static List<FlightPushHorizontalCandidate> BuildDirectionalHorizontalCandidates
    (
        Vector3                              origin,
        Vector3                              forward,
        FlightPushProbeResult                forwardSample,
        FlightPushProbeResult                backwardSample,
        FlightPushProbeResult                rightSample,
        FlightPushProbeResult                leftSample,
        FlightPushProbeResult                forwardRightSample,
        FlightPushProbeResult                forwardLeftSample,
        FlightPushProbeResult                backwardRightSample,
        FlightPushProbeResult                backwardLeftSample,
        IReadOnlyList<FlightPathDebugSample> sweepSamples,
        bool                                 goalAdjacentRearBias
    )
    {
        List<FlightPushHorizontalCandidate> candidates = new(FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT);

        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardSample,       goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardSample,      goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, rightSample,         goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, leftSample,          goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardRightSample,  goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, forwardLeftSample,   goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardRightSample, goalAdjacentRearBias);
        TryAddDirectionalHorizontalCandidate(candidates, origin, forward, backwardLeftSample,  goalAdjacentRearBias);

        foreach (var sample in sweepSamples)
            TryAddDirectionalHorizontalCandidate(candidates, origin, forward, new(sample.Clearance, sample.Endpoint), goalAdjacentRearBias);

        candidates.Sort(static (left, right) => right.Score.CompareTo(left.Score));
        if (candidates.Count > FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT)
            candidates.RemoveRange(FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT, candidates.Count - FLIGHT_PUSH_DIRECTIONAL_CANDIDATE_LIMIT);

        return candidates;
    }

    private static void TryAddDirectionalHorizontalCandidate
    (
        List<FlightPushHorizontalCandidate> candidates,
        Vector3                             origin,
        Vector3                             forward,
        FlightPushProbeResult               sample,
        bool                                goalAdjacentRearBias
    )
    {
        if (sample.Clearance <= FLIGHT_PUSH_MIN_DISTANCE)
            return;

        var delta = sample.Endpoint - origin;
        delta.Y = 0;
        if (!VoxelMathUtil.TryNormalize(delta, out var direction))
            return;

        var forwardDot = Vector3.Dot(direction, forward);
        var scoreScale = goalAdjacentRearBias
                             ? MathF.Max
                             (
                                 FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_MIN_SCALE,
                                 1f -
                                 (MathF.Max(0f, forwardDot) * FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_FORWARD_PENALTY) +
                                 (MathF.Max(0f, -forwardDot) * FLIGHT_PUSH_GOAL_ADJACENT_DIRECTIONAL_BACKWARD_BONUS)
                             )
                             : 1f + (MathF.Max(0f, forwardDot) * FLIGHT_PUSH_DIRECTIONAL_FORWARD_BONUS);
        var score = sample.Clearance * scoreScale;

        for (var i = 0; i < candidates.Count; ++i)
        {
            if (Vector3.Dot(candidates[i].Direction, direction) < FLIGHT_PUSH_DIRECTIONAL_DUPLICATE_DOT)
                continue;

            if (score > candidates[i].Score)
                candidates[i] = new(direction, sample.Clearance, score);
            return;
        }

        candidates.Add(new(direction, sample.Clearance, score));
    }

    private List<FlightPathDebugSample> BuildHorizontalSweepSamples
    (
        (ulong voxel, Vector3 p) origin,
        Vector3                  forward,
        Vector3                  right,
        float                    maxDistance,
        float                    stepDistance,
        bool                     goalAdjacentRearBias,
        ref Vector3              horizontalBias,
        ref float                horizontalTotalClearance,
        ref float                maxHorizontalClearance
    )
    {
        List<FlightPathDebugSample> samples         = new(FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT);
        const float                 STEP_ANGLE      = 2f * MathF.PI                             / FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT;
        const int                   PRIMARY_DIVISOR = FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT / 8;

        for (var i = 0; i < FLIGHT_PUSH_HORIZONTAL_SWEEP_SAMPLE_COUNT; ++i)
        {
            if (i % PRIMARY_DIVISOR == 0)
                continue;

            var angle     = i * STEP_ANGLE;
            var direction = (forward * MathF.Cos(angle)) + (right * MathF.Sin(angle));
            if (!VoxelMathUtil.TryNormalize(direction, out direction))
                continue;

            var sample = MeasureDirectionalClearance(origin, direction, maxDistance, stepDistance);
            maxHorizontalClearance = MathF.Max(maxHorizontalClearance, sample.Clearance);

            var forwardDot = Vector3.Dot(direction, forward);
            var weight     = FLIGHT_PUSH_HORIZONTAL_SWEEP_WEIGHT;

            if (goalAdjacentRearBias)
            {
                if (forwardDot > FLIGHT_PUSH_GOAL_ADJACENT_FORWARD_ALLOWANCE_DOT)
                    weight *= MathF.Max(FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_FORWARD_MIN_SCALE, 1f - (forwardDot * FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_FORWARD_PENALTY));
                else
                    weight += MathF.Max(0f, -forwardDot) * FLIGHT_PUSH_GOAL_ADJACENT_SWEEP_BACKWARD_BONUS;
            }
            else if (forwardDot > 0f)
                weight += forwardDot * FLIGHT_PUSH_HORIZONTAL_SWEEP_FORWARD_BONUS;

            AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, direction, sample.Clearance, weight);
            samples.Add(BuildFlightDebugSample(FlightPathDebugSampleKind.Sweep, origin.p, sample));
        }

        return samples;
    }

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

    private bool IsGoalDescentApproach
    (
        (ulong voxel, Vector3 p) previous,
        (ulong voxel, Vector3 p) current,
        (ulong voxel, Vector3 p) next,
        float                    leafVerticalSize
    )
    {
        if (next.voxel != goalVoxel)
            return false;

        var tolerance = MathF.Max(leafVerticalSize * FLIGHT_GOAL_DESCENT_HEIGHT_TOLERANCE_LEAF_SCALE, FLIGHT_GOAL_DESCENT_HEIGHT_TOLERANCE_MIN);
        return next.p.Y + tolerance < current.p.Y &&
               next.p.Y + tolerance < previous.p.Y;
    }

    private static float ResolveFlightHeightCatchupHeadroom(float leafVerticalSize)
        => MathF.Max(leafVerticalSize * FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_LEAF_SCALE, FLIGHT_PUSH_HEIGHT_CATCHUP_HEADROOM_MIN);

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
