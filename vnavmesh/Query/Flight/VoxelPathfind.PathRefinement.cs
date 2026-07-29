using System.Numerics;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight.Models;
using vnavmesh.Query.Flight.Utils;

namespace vnavmesh.Query.Flight;

public partial class VoxelPathfind
{
    private List<(ulong voxel, Vector3 p)> SimplifyPath
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken              cancel
    )
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

    private List<(ulong voxel, Vector3 p)> RefineSimplifiedPath
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken              cancel
    )
    {
        if (path.Count <= 2)
            return path;

        pathLoSCache.Clear();
        clearanceCache.Clear();

        var refined = SimplifyPath(path, cancel);
        refined.Reverse();
        refined = SimplifyPath(refined, cancel);
        refined.Reverse();

        if (refined.Count <= 2)
        {
            pathLoSCache.Clear();
            clearanceCache.Clear();
            return refined;
        }

        if (refined.Count == 3)
        {
            ProjectInteriorWaypoints(refined, cancel, null);

            if (TryRelaxWaypoint(refined[0], refined[1], refined[2], out var adjusted))
                refined[1] = adjusted;

            var singleCornerResult = SimplifyPath(refined, cancel);
            pathLoSCache.Clear();
            clearanceCache.Clear();
            return singleCornerResult;
        }

        HashSet<int>? dirtyIndices = null;

        for (var iteration = 0; iteration < REFINE_RELAX_ITERATION_LIMIT; ++iteration)
        {
            var projModified = new HashSet<int>();
            ProjectInteriorWaypoints(refined, cancel, projModified);

            // 合并投影修改到脏集
            if (dirtyIndices != null)
                dirtyIndices.UnionWith(projModified);

            var changed      = RelaxTowardOpenSpace(refined, cancel, dirtyIndices, out var relaxDirty);
            var straightened = SimplifyPath(refined, cancel);

            if (!changed && straightened.Count == refined.Count)
            {
                refined = straightened;
                break;
            }

            // SimplifyPath 改变路径结构时重置脏集
            dirtyIndices = straightened.Count != refined.Count ?
                               null :
                               relaxDirty;
            refined = straightened;
        }

        ProjectInteriorWaypoints(refined, cancel, null);
        RelaxTowardOpenSpace(refined, cancel, null, out _);

        pathLoSCache.Clear();
        clearanceCache.Clear();
        return SimplifyPath(refined, cancel);
    }

    private void ProjectInteriorWaypoints
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken              cancel,
        HashSet<int>?                  modifiedIndices
    )
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
            modifiedIndices?.Add(i);
        }
    }

    private bool RelaxTowardOpenSpace
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken              cancel,
        HashSet<int>?                  dirtyIndices,
        out HashSet<int>               newDirty
    )
    {
        newDirty = new HashSet<int>();

        if (path.Count <= 2)
            return false;

        var changed = false;

        for (var i = 1; i < path.Count - 1; ++i)
        {
            if ((i & 0x1f) == 0)
                cancel.ThrowIfCancellationRequested();

            // null 表示全部需要处理；非 null 则跳过未标记的索引
            if (dirtyIndices != null && !dirtyIndices.Contains(i))
                continue;

            var previous = path[i - 1];
            var current  = path[i];
            var next     = path[i + 1];

            if (TryRelaxWaypoint(previous, current, next, out var adjusted))
            {
                path[i] = adjusted;
                changed = true;
                newDirty.Add(Math.Max(1, i - 1));
                newDirty.Add(i);
                newDirty.Add(Math.Min(path.Count - 2, i + 1));
            }
        }

        return changed;
    }

    private bool TryRelaxWaypoint
    (
        (ulong voxel, Vector3 p)     previous,
        (ulong voxel, Vector3 p)     current,
        (ulong voxel, Vector3 p)     next,
        out (ulong voxel, Vector3 p) adjusted
    )
    {
        adjusted = current;

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
                normalizedForward = VoxelMathUtil.TryNormalize(horizontalForward, out var fallback) ?
                                        fallback :
                                        Vector2.UnitX;
            }
        }

        var forward3 = new Vector3(normalizedForward.X,  0, normalizedForward.Y);
        var right3   = new Vector3(-normalizedForward.Y, 0, normalizedForward.X);

        var voxelSize          = GetVoxelSize(current.voxel);
        var leafHorizontalSize = MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z);
        var leafVerticalSize   = l2Desc.CellSize.Y;
        var voxelHorizontal    = MathF.Max(voxelSize.X, voxelSize.Z);
        var voxelVertical      = voxelSize.Y;

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

        var forwardClearance  = MeasureDirectionalClearance(current, forward3,       horizontalScanDistance);
        var backwardClearance = MeasureDirectionalClearance(current, -forward3,      horizontalScanDistance);
        var rightClearance    = MeasureDirectionalClearance(current, right3,         horizontalScanDistance);
        var leftClearance     = MeasureDirectionalClearance(current, -right3,        horizontalScanDistance);
        var upClearance       = MeasureDirectionalClearance(current, Vector3.UnitY,  verticalScanDistance);
        var downClearance     = MeasureDirectionalClearance(current, -Vector3.UnitY, verticalScanDistance);

        var preferredHorizontal = MathF.Max
        (
            voxelHorizontal    * FLIGHT_PUSH_PREFERRED_CLEARANCE_VOXEL_SCALE,
            leafHorizontalSize * FLIGHT_PUSH_PREFERRED_CLEARANCE_LEAF_SCALE
        );
        var preferredVertical = MathF.Max
        (
            voxelVertical    * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_VOXEL_SCALE,
            leafVerticalSize * FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_LEAF_SCALE
        );

        var minHorizontalClearance = MathF.Min(MathF.Min(forwardClearance, backwardClearance), MathF.Min(rightClearance, leftClearance));
        var maxHorizontalClearance = MathF.Max(MathF.Max(forwardClearance, backwardClearance), MathF.Max(rightClearance, leftClearance));

        var horizontalCramped = minHorizontalClearance < preferredHorizontal;
        var floorCramped      = downClearance < preferredVertical && upClearance   > FLIGHT_PUSH_MIN_DISTANCE;
        var ceilingCramped    = upClearance   < preferredVertical && downClearance > FLIGHT_PUSH_MIN_DISTANCE;

        var horizontalBias           = Vector3.Zero;
        var horizontalTotalClearance = 0f;

        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, forward3,  forwardClearance,  1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -forward3, backwardClearance, 1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, right3,    rightClearance,    1f);
        AccumulateDirectionalBias(ref horizontalBias, ref horizontalTotalClearance, -right3,   leftClearance,     1f);

        var horizontalOffset = Vector3.Zero;

        if (horizontalCramped && VoxelMathUtil.TryNormalize(new Vector2(horizontalBias.X, horizontalBias.Z), out var horizontalPushDirection))
        {
            var deficit = preferredHorizontal - minHorizontalClearance;
            var maxHorizontalPush = MathF.Min(horizontalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, maxHorizontalClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var horizontalPushDistance = MathF.Min(maxHorizontalPush, deficit * FLIGHT_PUSH_RELIEF_SCALE);
            if (horizontalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
                horizontalOffset = new Vector3(horizontalPushDirection.X * horizontalPushDistance, 0, horizontalPushDirection.Y * horizontalPushDistance);
        }

        var verticalOffset = Vector3.Zero;

        if (floorCramped)
        {
            var deficit              = preferredVertical - downClearance;
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, upClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush,                                       deficit     * FLIGHT_PUSH_RELIEF_SCALE);
            if (verticalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
                verticalOffset = Vector3.UnitY * verticalPushDistance;
        }
        else if (ceilingCramped)
        {
            var deficit              = preferredVertical - upClearance;
            var maxVerticalPush      = MathF.Min(verticalScanDistance * FLIGHT_PUSH_SCAN_PUSH_FRACTION, downClearance * FLIGHT_PUSH_MAX_CLEARANCE_FRACTION);
            var verticalPushDistance = MathF.Min(maxVerticalPush,                                       deficit       * FLIGHT_PUSH_RELIEF_SCALE);
            if (verticalPushDistance >= FLIGHT_PUSH_MIN_DISTANCE)
                verticalOffset = -Vector3.UnitY * verticalPushDistance;
        }

        var combinedOffset = horizontalOffset + verticalOffset;

        if (TryAcceptScaledOffset(previous, current, next, combinedOffset, out adjusted))
            return true;
        if (TryAcceptScaledOffset(previous, current, next, horizontalOffset, out adjusted))
            return true;
        if (TryAcceptScaledOffset(previous, current, next, verticalOffset, out adjusted))
            return true;

        adjusted = current;
        return false;
    }

    private float MeasureDirectionalClearance
    (
        (ulong voxel, Vector3 p) origin,
        Vector3                  direction,
        float                    maxDistance
    )
    {
        if (direction.LengthSquared() <= SCORE_EPSILON * SCORE_EPSILON ||
            maxDistance               <= FLIGHT_PUSH_MIN_DISTANCE)
            return 0;

        var dirQuantized = QuantizeDirection(direction);
        var cacheKey     = (origin.voxel, dirQuantized);

        if (clearanceCache.TryGetValue(cacheKey, out var cached))
            return cached;

        var dirNorm = Vector3.Normalize(direction);
        var endPos  = origin.p + (dirNorm * maxDistance);
        var endLeaf = Volume.FindLeafVoxel(endPos);

        if (endLeaf.voxel == VoxelMap.INVALID_VOXEL)
            return 0;

        var clearance = 0f;

        foreach (var (_, t, empty) in VoxelSearch.EnumerateVoxelsInLine(Volume, origin.voxel, endLeaf.voxel, origin.p, endPos))
        {
            if (!empty)
                break;

            clearance = t * maxDistance;
        }

        if (clearanceCache.Count < CLEARANCE_CACHE_MAX_SIZE)
            clearanceCache[cacheKey] = clearance;

        return clearance;
    }

    private static int QuantizeDirection
    (
        Vector3 dir
    )
    {
        // 将方向分量量化到 -4..4 范围，打包为 int 键
        var x = Math.Clamp((int)MathF.Round(dir.X * 4f) + 4, 0, 8);
        var y = Math.Clamp((int)MathF.Round(dir.Y * 4f) + 4, 0, 8);
        var z = Math.Clamp((int)MathF.Round(dir.Z * 4f) + 4, 0, 8);
        return (x << 8) | (y << 4) | z;
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

        Span<float> scales = [1f, 0.5f, 0.25f];

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

        var currentIncoming         = current.p   - previous.p;
        var currentOutgoing         = next.p      - current.p;
        var candidateIncoming       = candidate.p - previous.p;
        var candidateOutgoing       = next.p      - candidate.p;
        var currentIncomingLength   = currentIncoming.Length();
        var currentOutgoingLength   = currentOutgoing.Length();
        var candidateIncomingLength = candidateIncoming.Length();
        var candidateOutgoingLength = candidateOutgoing.Length();

        if (currentIncomingLength   <= SCORE_EPSILON ||
            currentOutgoingLength   <= SCORE_EPSILON ||
            candidateIncomingLength <= SCORE_EPSILON ||
            candidateOutgoingLength <= SCORE_EPSILON)
            return false;

        var currentLocalLength   = currentIncomingLength   + currentOutgoingLength;
        var candidateLocalLength = candidateIncomingLength + candidateOutgoingLength;
        if (candidateLocalLength > currentLocalLength * (1f + FLIGHT_PUSH_MAX_LOCAL_LENGTH_INCREASE_RATIO))
            return false;

        var currentDirectionDot   = Vector3.Dot(currentIncoming,   currentOutgoing)   / (currentIncomingLength   * currentOutgoingLength);
        var candidateDirectionDot = Vector3.Dot(candidateIncoming, candidateOutgoing) / (candidateIncomingLength * candidateOutgoingLength);
        if (candidateDirectionDot + SCORE_EPSILON < currentDirectionDot)
            return false;

        if (!HasLineOfSight(previous, candidate.voxel, candidate.p))
            return false;
        if (!HasLineOfSight(candidate, next.voxel, next.p))
            return false;

        adjusted = candidate;
        return true;
    }

    private bool TryResolveEmptyWaypoint
    (
        Vector3                      point,
        Vector3                      directionHint,
        out (ulong voxel, Vector3 p) waypoint
    )
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

    private Vector3 GetVoxelSize
    (
        ulong voxel
    )
    {
        var (min, max) = Volume.VoxelBounds(voxel, 0);
        return max - min;
    }

    private float ResolveVoxelInset
    (
        ulong voxel
    )
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

    private bool NeedsFlightDescentSmoothing
    (
        Vector3 from,
        Vector3 to
    )
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

    private int FindFurthestVisibleIndex
    (
        List<(ulong voxel, Vector3 p)> path,
        int                            anchorIndex,
        CancellationToken              cancel
    )
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
            {
                furthestVisibleIndex = FindVisibleBoundary(path, anchorIndex, furthestVisibleIndex, probeIndex - 1, cancel);
                break;
            }

            furthestVisibleIndex =   probeIndex;
            step                 <<= 1;
        }

        while (furthestVisibleIndex > anchorIndex + 1 &&
               NeedsFlightDescentSmoothing(path[anchorIndex].p, path[furthestVisibleIndex].p))
            --furthestVisibleIndex;

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

    private bool HasLineOfSight
    (
        List<(ulong voxel, Vector3 p)> path,
        int                            anchorIndex,
        int                            probeIndex,
        CancellationToken              cancel
    )
    {
        if ((probeIndex & 0x3f) == 0)
            cancel.ThrowIfCancellationRequested();

        var anchor   = path[anchorIndex];
        var probe    = path[probeIndex];
        var cacheKey = new VolumeVisibilityKey(anchor.voxel, probe.voxel, anchor.p, probe.p);

        if (pathLoSCache.TryGetValue(cacheKey, out var cached))
            return cached;

        ++lineOfSightChecks;
        var visible = VoxelSearch.LineOfSight(Volume, anchor.voxel, probe.voxel, anchor.p, probe.p);
        if (visible)
            ++lineOfSightHits;

        if (pathLoSCache.Count < PATH_LOS_CACHE_MAX_SIZE)
            pathLoSCache[cacheKey] = visible;
        return visible;
    }

    private bool HasLineOfSight
    (
        (ulong voxel, Vector3 p) from,
        ulong                    toVoxel,
        Vector3                  toPosition
    )
    {
        var cacheKey = new VolumeVisibilityKey(from.voxel, toVoxel, from.p, toPosition);

        if (pathLoSCache.TryGetValue(cacheKey, out var cached))
            return cached;

        ++lineOfSightChecks;
        var visible = VoxelSearch.LineOfSight(Volume, from.voxel, toVoxel, from.p, toPosition);
        if (visible)
            ++lineOfSightHits;

        if (pathLoSCache.Count < PATH_LOS_CACHE_MAX_SIZE)
            pathLoSCache[cacheKey] = visible;
        return visible;
    }

    private List<(ulong voxel, Vector3 p)> BuildPathToVisitedNode
    (
        int  nodeIndex,
        bool returnIntermediatePoints
    )
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
