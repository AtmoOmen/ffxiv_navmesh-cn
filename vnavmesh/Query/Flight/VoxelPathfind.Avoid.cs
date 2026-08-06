using System.Numerics;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query.Flight.Utils;

namespace vnavmesh.Query.Flight;

public partial class VoxelPathfind
{
    private List<(ulong voxel, Vector3 p)> FindPathUnavoided
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel
    )
    {
        var savedRadius   = avoidRadius;
        var savedRadiusSq = avoidRadiusSq;
        var savedMinDist  = minAvoidDistSq;
        avoidRadius       = 0;
        avoidRadiusSq     = 0;
        minAvoidDistSq    = 0;

        try
        {
            return FindPathInternal(fromVoxel, toVoxel, fromPos, toPos, returnIntermediatePoints, cancel, true);
        }
        finally
        {
            avoidRadius    = savedRadius;
            avoidRadiusSq  = savedRadiusSq;
            minAvoidDistSq = savedMinDist;
        }
    }

    private List<(ulong voxel, Vector3 p)> FindPathAvoided
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel
    )
    {
        l1PathSet               = null;
        l0PathSet               = null;
        l1CorridorDistance      = null;
        l0CorridorDistance      = null;
        l1DistanceField         = null;
        l0DistanceField         = null;
        previouslyVisitedVoxels = null;

        var path = RunSearchAttempt
        (
            fromVoxel,
            toVoxel,
            fromPos,
            toPos,
            returnIntermediatePoints,
            DEFAULT_MAX_SEARCH_STEPS,
            cancel,
            allowCoarseSteppingOverride: false
        );
        return path.Count > 0 ?
                   RefineSimplifiedPath(path, cancel) :
                   path;
    }

    private List<(ulong voxel, Vector3 p)>? TryFindPathAroundAvoid
    (
        ulong             fromVoxel,
        ulong             toVoxel,
        Vector3           fromPos,
        Vector3           toPos,
        bool              returnIntermediatePoints,
        CancellationToken cancel
    )
    {
        if (BuildAroundAvoidWaypoints(fromPos, toPos) is not { Count: > 1 } corners)
            return null;

        var merged = new List<(ulong voxel, Vector3 p)> { (fromVoxel, fromPos) };
        var curPos   = fromPos;
        var curVoxel = fromVoxel;

        for (var i = 1; i < corners.Count; ++i)
        {
            cancel.ThrowIfCancellationRequested();
            var nextPos   = corners[i];
            var nextVoxel = i == corners.Count - 1 ?
                                toVoxel :
                                VoxelAt(nextPos);
            if (nextVoxel == VoxelMap.INVALID_VOXEL)
                return null;

            List<(ulong voxel, Vector3 p)> leg;

            if (!SegmentViolatesAvoid(curPos, nextPos) &&
                VoxelSearch.LineOfSight(Volume, curVoxel, nextVoxel, curPos, nextPos))
            {
                leg = [(nextVoxel, nextPos)];
            }
            else
            {
                leg = FindPathUnavoided(curVoxel, nextVoxel, curPos, nextPos, returnIntermediatePoints, cancel);
                if (leg.Count == 0 || PathViolatesAvoid(leg))
                    return null;

                leg.RemoveAt(0);
                if (leg.Count == 0 || (leg[^1].p - nextPos).LengthSquared() > 0.01f)
                    leg.Add((nextVoxel, nextPos));
            }

            merged.AddRange(leg);
            curPos   = nextPos;
            curVoxel = nextVoxel;
        }

        if (merged.Count == 0 || (merged[^1].p - toPos).LengthSquared() > 0.01f)
            merged.Add((toVoxel, toPos));
        return PathViolatesAvoid(merged) ?
                   null :
                   merged;
    }

    private bool PathViolatesAvoid
    (
        List<(ulong voxel, Vector3 p)> path
    )
    {
        for (var i = 1; i < path.Count; ++i)
        {
            if (SegmentViolatesAvoid(path[i - 1].p, path[i].p))
                return true;
        }

        return false;
    }

    private List<Vector3>? BuildAroundAvoidWaypoints
    (
        Vector3 from,
        Vector3 to
    )
    {
        var r = MathF.Sqrt(minAvoidDistSq) * 1.02f;
        if (r < 0.1f)
            return null;

        var c     = avoidCenter;
        var start = PushOutOfAvoid(from, r);
        var end   = PushOutOfAvoid(to,   r);

        if (!TryCircleTangentsXZ(start, c, r, out var s1, out var s2))
            return null;
        if (!TryCircleTangentsXZ(end, c, r, out var e1, out var e2))
            return null;

        s1.Y = start.Y;
        s2.Y = start.Y;
        e1.Y = end.Y;
        e2.Y = end.Y;

        List<Vector3>? best    = null;
        var            bestLen = float.MaxValue;

        foreach (var (ts, te) in new[] { (s1, e1), (s1, e2), (s2, e1), (s2, e2) })
        {
            var path = new List<Vector3> { from };
            if ((from - start).LengthSquared() > 0.01f)
                path.Add(start);
            path.Add(ts);
            SampleCircleArc(ts, te, c, r, start.Y, end.Y, path);
            path.Add(te);
            if ((end - to).LengthSquared() > 0.01f)
                path.Add(end);
            path.Add(to);

            var valid = true;

            for (var i = 1; i < path.Count; ++i)
            {
                if (SegmentViolatesAvoid(path[i - 1], path[i]))
                {
                    valid = false;
                    break;
                }
            }

            if (!valid)
                continue;

            var length = 0f;
            for (var i = 1; i < path.Count; ++i)
                length += (path[i] - path[i - 1]).Length();

            if (length < bestLen)
            {
                bestLen = length;
                best    = path;
            }
        }

        return best;
    }

    private Vector3 PushOutOfAvoid
    (
        Vector3 p,
        float   r
    )
    {
        var dx = p.X - avoidCenter.X;
        var dz = p.Z - avoidCenter.Z;
        var dSq = (dx * dx) + (dz * dz);
        var target = r + 0.01f;

        if (dSq >= target * target)
            return p;
        if (dSq < 1e-6f)
        {
            dx  = 1;
            dz  = 0;
            dSq = 1;
        }

        var scale = target / MathF.Sqrt(dSq);
        return new Vector3(avoidCenter.X + (dx * scale), p.Y, avoidCenter.Z + (dz * scale));
    }

    private static bool TryCircleTangentsXZ
    (
        Vector3 p,
        Vector3 c,
        float   r,
        out Vector3 t1,
        out Vector3 t2
    )
    {
        t1 = t2 = default;
        var dx = p.X - c.X;
        var dz = p.Z - c.Z;
        var dSq = (dx * dx) + (dz * dz);
        if (dSq <= (r * r) + 1e-3f)
            return false;

        var inv = r * r / dSq;
        var h   = r * MathF.Sqrt(dSq - (r * r)) / dSq;
        var mx  = c.X + (inv * dx);
        var mz  = c.Z + (inv * dz);
        t1 = new Vector3(mx - (h * dz), p.Y, mz + (h * dx));
        t2 = new Vector3(mx + (h * dz), p.Y, mz - (h * dx));
        return true;
    }

    private static void SampleCircleArc
    (
        Vector3      from,
        Vector3      to,
        Vector3      c,
        float        r,
        float        y0,
        float        y1,
        List<Vector3> path
    )
    {
        var a0 = MathF.Atan2(from.Z - c.Z, from.X - c.X);
        var a1 = MathF.Atan2(to.Z   - c.Z, to.X   - c.X);
        var dccw = a1 - a0;

        while (dccw < 0)
            dccw += MathF.PI * 2;

        var dcw   = dccw - (MathF.PI * 2);
        var delta = MathF.Abs(dcw) < dccw ?
                        dcw :
                        dccw;
        var steps = Math.Max(1, (int)MathF.Ceiling(MathF.Abs(delta) / (MathF.PI / 12)));

        for (var i = 1; i < steps; ++i)
        {
            var t = (float)i / steps;
            var a = a0 + (delta * t);
            path.Add(new Vector3(c.X + (r * MathF.Cos(a)), y0 + ((y1 - y0) * t), c.Z + (r * MathF.Sin(a))));
        }
    }

    private ulong VoxelAt
    (
        Vector3 p
    )
    {
        var leaf = Volume.FindLeafVoxel(p);
        return leaf.empty ?
                   leaf.voxel :
                   VoxelSearch.FindNearestEmptyVoxel(Volume, p, new Vector3(5, 5, 5));
    }

    private bool SegmentViolatesAvoid
    (
        Vector3 a,
        Vector3 b
    ) => avoidRadius > 0 && SegmentMinDistSqXZ(a, b) + 1e-3f < minAvoidDistSq;

    private float FlatDistSq
    (
        Vector3 p
    )
    {
        var dx = p.X - avoidCenter.X;
        var dz = p.Z - avoidCenter.Z;
        return (dx * dx) + (dz * dz);
    }

    private float SegmentMinDistSqXZ
    (
        Vector3 a,
        Vector3 b
    )
    {
        var abx   = b.X - a.X;
        var abz   = b.Z - a.Z;
        var lenSq = (abx * abx) + (abz * abz);
        float t;

        if (lenSq < 1e-6f)
            t = 0;
        else
        {
            t = ((avoidCenter.X - a.X) * abx + (avoidCenter.Z - a.Z) * abz) / lenSq;
            t = Math.Clamp(t, 0f, 1f);
        }

        var dx = a.X + (abx * t) - avoidCenter.X;
        var dz = a.Z + (abz * t) - avoidCenter.Z;
        return (dx * dx) + (dz * dz);
    }
}
