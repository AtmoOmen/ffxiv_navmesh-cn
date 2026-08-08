using System.Numerics;
using System.Runtime.CompilerServices;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Models;

namespace vnavmesh.Query.Flight;

public sealed partial class VoxelPathfinder
{
    private List<(ulong voxel, Vector3 p)> StringPull
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken               cancel
    )
    {
        if (path.Count <= 2)
            return path;

        var result = new List<(ulong voxel, Vector3 p)>();
        result.Add(path[0]);
        var anchorIndex = 0;

        while (anchorIndex < path.Count - 1)
        {
            var furthestIndex = FindFurthestVisibleIndex(path, anchorIndex, cancel);
            result.Add(path[furthestIndex]);
            anchorIndex = furthestIndex;
        }

        return result;
    }

    private int FindFurthestVisibleIndex
    (
        List<(ulong voxel, Vector3 p)> path,
        int                             anchorIndex,
        CancellationToken               cancel
    )
    {
        var pathLastIndex = path.Count - 1;
        var nextIndex     = anchorIndex + 1;

        if (nextIndex >= pathLastIndex || !TryLineOfSight(path[anchorIndex].voxel, path[anchorIndex].p, path[nextIndex].voxel, path[nextIndex].p))
            return nextIndex;

        var furthestVisibleIndex = nextIndex;
        var step                 = 1;

        while (furthestVisibleIndex < pathLastIndex)
        {
            if ((furthestVisibleIndex & 0x3f) == 0)
                cancel.ThrowIfCancellationRequested();

            var probeIndex = Math.Min(furthestVisibleIndex + step, pathLastIndex);

            if (!TryLineOfSight(path[anchorIndex].voxel, path[anchorIndex].p, path[probeIndex].voxel, path[probeIndex].p))
            {
                furthestVisibleIndex = FindVisibleBoundary(path, anchorIndex, furthestVisibleIndex, probeIndex - 1, cancel);
                break;
            }

            furthestVisibleIndex = probeIndex;
            step                <<= 1;
        }

        return furthestVisibleIndex;
    }

    private int FindVisibleBoundary
    (
        List<(ulong voxel, Vector3 p)> path,
        int                             anchorIndex,
        int                             visibleIndex,
        int                             blockedIndex,
        CancellationToken               cancel
    )
    {
        while (visibleIndex < blockedIndex)
        {
            if ((visibleIndex & 0x3f) == 0)
                cancel.ThrowIfCancellationRequested();

            var mid = visibleIndex + ((blockedIndex - visibleIndex + 1) >> 1);

            if (TryLineOfSight(path[anchorIndex].voxel, path[anchorIndex].p, path[mid].voxel, path[mid].p))
                visibleIndex = mid;
            else
                blockedIndex = mid - 1;
        }

        return visibleIndex;
    }

    private List<(ulong voxel, Vector3 p)> ExpandIntermediatePoints
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken               cancel
    )
    {
        if (path.Count <= 1)
            return path;

        var result = new List<(ulong voxel, Vector3 p)>();
        result.Add(path[0]);

        for (var i = 0; i < path.Count - 1; ++i)
        {
            cancel.ThrowIfCancellationRequested();
            var from = path[i];
            var to   = path[i + 1];
            var delta = to.p - from.p;

            foreach (var step in VoxelSearch.EnumerateVoxelsInLine(Volume, from.voxel, to.voxel, from.p, to.p))
            {
                if (!step.empty)
                    continue;

                var point = from.p + (step.t * delta);

                if (Vector3.DistanceSquared(result[^1].p, point) > VoxelPathfinderConstants.DUPLICATE_WAYPOINT_DISTANCE_SQ)
                    result.Add((step.voxel, point));
            }

            if (Vector3.DistanceSquared(result[^1].p, to.p) > VoxelPathfinderConstants.DUPLICATE_WAYPOINT_DISTANCE_SQ)
                result.Add(to);
        }

        return result;
    }

    private List<(ulong voxel, Vector3 p)> NormalizeWaypoints
    (
        List<(ulong voxel, Vector3 p)> path,
        CancellationToken               cancel
    )
    {
        if (path.Count == 0)
            return path;

        var result = new List<(ulong voxel, Vector3 p)>(path.Count);

        foreach (var (voxel, p) in path)
        {
            cancel.ThrowIfCancellationRequested();
            var leaf = Volume.FindLeafVoxel(p);

            if (leaf.empty)
            {
                result.Add((leaf.voxel, p));
                continue;
            }

            var nearest = VoxelSearch.FindNearestEmptyVoxel(Volume, p, new Vector3(16f));

            if (nearest == SparseVoxelOctree.INVALID_VOXEL)
            {
                result.Add((voxel, p));
                continue;
            }

            var bounds = Volume.VoxelBounds(nearest, 0);
            result.Add((nearest, (bounds.min + bounds.max) * 0.5f));
        }

        return result;
    }

    internal static PostprocessedPathSegment ProcessSegment
    (
        PlannerPathSegment segment,
        CancellationToken  cancel
    )
    {
        cancel.ThrowIfCancellationRequested();
        return BuildSegment(segment, SimplifyWaypoints(segment.Points));
    }

    internal static PostprocessedPathSegment ProcessStraightPathSegment
    (
        PlannerPathSegment segment,
        CancellationToken  cancel
    )
    {
        cancel.ThrowIfCancellationRequested();
        return BuildSegment(segment, [.. segment.Points]);
    }

    private static PostprocessedPathSegment BuildSegment
    (
        PlannerPathSegment     segment,
        IReadOnlyList<Vector3> waypoints
    ) =>
        new()
        {
            MovementMode         = segment.MovementMode,
            SegmentKind          = segment.SegmentKind,
            AllowVerticalControl = segment.AllowVerticalControl,
            StartPosition        = segment.StartPosition,
            CompletionTolerance  = 0,
            Waypoints            = waypoints
        };

    private static List<Vector3> SimplifyWaypoints
    (
        IReadOnlyList<Vector3> points
    )
    {
        if (points.Count <= 2)
            return [.. points];

        List<Vector3> simplified = [points[0]];

        for (var i = 1; i < points.Count - 1; ++i)
        {
            var previous = simplified[^1];
            var current  = points[i];
            var next     = points[i + 1];

            if (Vector3.DistanceSquared(previous, current) > VoxelPathfinderConstants.DUPLICATE_WAYPOINT_DISTANCE_SQ &&
                Vector3.DistanceSquared(current, next)    > VoxelPathfinderConstants.DUPLICATE_WAYPOINT_DISTANCE_SQ &&
                DistanceToLineSegment(current, previous, next) > VoxelPathfinderConstants.COLLINEAR_WAYPOINT_TOLERANCE)
            {
                simplified.Add(current);
            }
        }

        simplified.Add(points[^1]);
        return simplified;
    }

    private static float DistanceToLineSegment
    (
        Vector3 value,
        Vector3 start,
        Vector3 end
    )
    {
        var segment       = end - start;
        var lengthSquared = segment.LengthSquared();

        if (lengthSquared <= VoxelPathfinderConstants.DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return Vector3.Distance(value, start);

        var progress  = Math.Clamp(Vector3.Dot(value - start, segment) / lengthSquared, 0f, 1f);
        var projected = start + (progress * segment);
        return Vector3.Distance(value, projected);
    }
}
