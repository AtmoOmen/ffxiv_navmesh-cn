using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using DotRecast.Core.Numerics;

namespace Navmesh;

public static class GroundPathSmoother
{
    private const float Epsilon = 1e-4f;

    public static List<Vector3> BuildCenteredPath(GroundDetourQuery query, MeshCorridor corridor, GroundPathSettings settings, bool useRaycast, out GroundPathDebugInfo debug)
    {
        var trimmedPortals = corridor.Portals.Select(p => p.Trim(settings.Clearance)).ToArray();
        var centerline = BuildCenterline(corridor.Start, corridor.End, trimmedPortals, settings);
        var protectedPointIndices = GetProtectedPointIndices(trimmedPortals);

        List<RcVec3f> sequence = [corridor.Start, .. centerline, corridor.End];
        var simplified = useRaycast ? Simplify(sequence, trimmedPortals, settings.MaxLookAhead) : sequence;
        var cleaned = Cleanup(simplified, settings, protectedPointIndices);

        debug = new()
        {
            CorridorCenters = [.. corridor.Polygons.Select(r => query.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem())],
            RawPortals = [.. corridor.Portals.Select(p => ToDebugPortal(p, 0))],
            TrimmedPortals = [.. trimmedPortals.Select(p => ToDebugPortal(p, p.EffectiveClearance))],
            Centerline = [corridor.Start.RecastToSystem(), .. centerline.Select(p => p.RecastToSystem()), corridor.End.RecastToSystem()],
            FinalPath = [.. cleaned.Select(p => p.RecastToSystem())],
            ProtectedPointIndices = protectedPointIndices
        };

        return [.. cleaned.Select(p => p.RecastToSystem())];
    }

    public static List<Vector3> BuildPortalMidpointPath(GroundDetourQuery query, MeshCorridor corridor, GroundPathSettings settings, bool useRaycast, out GroundPathDebugInfo debug)
    {
        var trimmedPortals = corridor.Portals.Select(p => p.Trim(settings.Clearance)).ToArray();
        var protectedPointIndices = GetProtectedPointIndices(trimmedPortals);
        List<RcVec3f> sequence = [corridor.Start, .. trimmedPortals.Select(p => p.Mid), corridor.End];
        var simplified = useRaycast ? Simplify(sequence, trimmedPortals, settings.MaxLookAhead) : sequence;
        var cleaned = Cleanup(simplified, settings, protectedPointIndices);

        debug = new()
        {
            CorridorCenters = [.. corridor.Polygons.Select(r => query.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem())],
            RawPortals = [.. corridor.Portals.Select(p => ToDebugPortal(p, 0))],
            TrimmedPortals = [.. trimmedPortals.Select(p => ToDebugPortal(p, p.EffectiveClearance))],
            Centerline = [.. sequence.Select(p => p.RecastToSystem())],
            FinalPath = [.. cleaned.Select(p => p.RecastToSystem())],
            ProtectedPointIndices = protectedPointIndices
        };

        return [.. cleaned.Select(p => p.RecastToSystem())];
    }

    private static List<RcVec3f> BuildCenterline(RcVec3f start, RcVec3f end, IReadOnlyList<MeshPortal> portals, GroundPathSettings settings)
    {
        if (portals.Count == 0)
            return [];

        var points = portals.Select(p => p.Mid).ToArray();
        var scratch = new RcVec3f[points.Length];
        for (var iteration = 0; iteration < settings.RelaxIterations; iteration++)
        {
            Array.Copy(points, scratch, points.Length);
            for (var i = 0; i < scratch.Length; i++)
            {
                var prev = i == 0 ? start : scratch[i - 1];
                var next = i == scratch.Length - 1 ? end : scratch[i + 1];
                var target = ConstrainToPortal(portals[i], prev, next);
                points[i] = RcVec3f.Lerp(target, portals[i].Mid, ResolveCenterBias(portals[i], settings.CenterBias));
            }
        }

        return [.. points];
    }

    private static List<RcVec3f> Simplify(IReadOnlyList<RcVec3f> sequence, IReadOnlyList<MeshPortal> portals, int maxLookAhead)
    {
        if (sequence.Count <= 2)
            return [.. sequence];

        List<RcVec3f> result = [sequence[0]];
        var current = 0;
        while (current < sequence.Count - 1)
        {
            var furthest = current + 1;
            var limit = Math.Min(sequence.Count - 1, current + maxLookAhead + 1);
            for (var candidate = limit; candidate > current + 1; candidate--)
            {
                if (CanSkip(sequence[current], sequence[candidate], portals, current, candidate))
                {
                    furthest = candidate;
                    break;
                }
            }

            result.Add(sequence[furthest]);
            current = furthest;
        }

        return result;
    }

    private static List<RcVec3f> Cleanup(IReadOnlyList<RcVec3f> sequence, GroundPathSettings settings, IReadOnlyCollection<int> protectedPointIndices)
    {
        if (sequence.Count <= 2)
            return [.. sequence];

        List<RcVec3f> deduped = [sequence[0]];
        for (var i = 1; i < sequence.Count; i++)
        {
            var current = sequence[i];
            var isProtected = protectedPointIndices.Contains(i);
            if (!isProtected && RcVec3f.Distance(deduped[^1], current) < settings.MinPointSpacing)
                deduped[^1] = current;
            else
                deduped.Add(current);
        }

        if (deduped.Count <= 2)
            return deduped;

        List<RcVec3f> cleaned = [deduped[0]];
        for (var i = 1; i < deduped.Count - 1; i++)
        {
            var prev = cleaned[^1];
            var current = deduped[i];
            var next = deduped[i + 1];
            if (protectedPointIndices.Contains(i))
            {
                cleaned.Add(current);
                continue;
            }

            if (IsNearlyCollinear(prev, current, next, settings.CollinearSlack))
                continue;

            cleaned.Add(current);
        }

        cleaned.Add(deduped[^1]);
        return cleaned;
    }

    private static RcVec3f ConstrainToPortal(MeshPortal portal, RcVec3f prev, RcVec3f next)
    {
        if (portal.IsPoint)
            return portal.Mid;

        var a = new Vector2(prev.X, prev.Z);
        var b = new Vector2(next.X, next.Z);
        var p = new Vector2(portal.Left.X, portal.Left.Z);
        var q = new Vector2(portal.Right.X, portal.Right.Z);
        var line = b - a;
        if (line.LengthSquared() < Epsilon)
            return ClosestPointOnSegment(portal.Left, portal.Right, portal.Mid);

        var portalDir = q - p;
        var denom = Cross(line, portalDir);
        if (MathF.Abs(denom) < Epsilon)
            return ClosestPointOnSegment(portal.Left, portal.Right, RcVec3f.Lerp(prev, next, 0.5f));

        var u = Cross(p - a, line) / denom;
        return RcVec3f.Lerp(portal.Left, portal.Right, Math.Clamp(u, 0, 1));
    }

    private static bool CanSkip(RcVec3f from, RcVec3f to, IReadOnlyList<MeshPortal> portals, int fromIndex, int toIndex)
    {
        var startPortal = fromIndex;
        var endPortal = toIndex - 2;
        if (startPortal > endPortal)
            return true;

        var previousT = -Epsilon;
        for (var i = startPortal; i <= endPortal; i++)
        {
            if (portals[i].IsProtectedAnchor)
                return false;
            if (!TryIntersectSegmentWithPortal(from, to, portals[i], out var t) || t + Epsilon < previousT)
                return false;
            previousT = t;
        }

        return true;
    }

    private static bool TryIntersectSegmentWithPortal(RcVec3f from, RcVec3f to, MeshPortal portal, out float t)
    {
        t = 0;
        var a = new Vector2(from.X, from.Z);
        var b = new Vector2(to.X, to.Z);
        var ab = b - a;
        var abLenSq = ab.LengthSquared();
        if (abLenSq < Epsilon)
            return false;

        if (portal.IsPoint)
        {
            var point = new Vector2(portal.Mid.X, portal.Mid.Z);
            var projection = Math.Clamp(Vector2.Dot(point - a, ab) / abLenSq, 0, 1);
            var closest = a + ab * projection;
            if (Vector2.DistanceSquared(closest, point) > 0.0004f)
                return false;
            t = projection;
            return true;
        }

        var c = new Vector2(portal.Left.X, portal.Left.Z);
        var d = new Vector2(portal.Right.X, portal.Right.Z);
        var cd = d - c;
        var denom = Cross(ab, cd);
        var ac = c - a;
        if (MathF.Abs(denom) < Epsilon)
        {
            if (MathF.Abs(Cross(ac, ab)) >= 0.001f)
                return false;

            var t0 = Math.Clamp(Vector2.Dot(c - a, ab) / abLenSq, 0, 1);
            var t1 = Math.Clamp(Vector2.Dot(d - a, ab) / abLenSq, 0, 1);
            t = Math.Min(t0, t1);
            return true;
        }

        var hitT = Cross(ac, cd) / denom;
        var hitU = Cross(ac, ab) / denom;
        if (hitT < -Epsilon || hitT > 1 + Epsilon || hitU < -Epsilon || hitU > 1 + Epsilon)
            return false;

        t = Math.Clamp(hitT, 0, 1);
        return true;
    }

    private static bool IsNearlyCollinear(RcVec3f prev, RcVec3f current, RcVec3f next, float slack)
    {
        var prev2 = new Vector2(prev.X, prev.Z);
        var current2 = new Vector2(current.X, current.Z);
        var next2 = new Vector2(next.X, next.Z);
        var a = current2 - prev2;
        var b = next2 - current2;
        if (a.LengthSquared() < Epsilon || b.LengthSquared() < Epsilon)
            return true;

        var cross = MathF.Abs(Cross(a, b));
        if (cross > slack * (a.Length() + b.Length()))
            return false;

        var direction = Vector2.Dot(Vector2.Normalize(a), Vector2.Normalize(b));
        return direction > 0.99f;
    }

    private static RcVec3f ClosestPointOnSegment(RcVec3f left, RcVec3f right, RcVec3f point)
    {
        var ab = right - left;
        var lengthSq = RcVec3f.Dot(ab, ab);
        if (lengthSq < Epsilon)
            return left;

        var t = Math.Clamp(RcVec3f.Dot(point - left, ab) / lengthSq, 0, 1);
        return RcVec3f.Lerp(left, right, t);
    }

    private static float ResolveCenterBias(MeshPortal portal, float defaultBias)
    {
        if (!portal.IsNarrow)
            return defaultBias;
        if (portal.IsProtectedAnchor)
            return MathF.Min(defaultBias, 0.15f);

        return MathF.Min(defaultBias, 0.35f);
    }

    private static List<int> GetProtectedPointIndices(IReadOnlyList<MeshPortal> portals)
    {
        List<int> indices = [];
        for (var i = 0; i < portals.Count; i++)
        {
            if (portals[i].IsProtectedAnchor)
                indices.Add(i + 1);
        }

        return indices;
    }

    private static DebugPortalSegment ToDebugPortal(MeshPortal portal, float effectiveClearance)
        => new(portal.Left.RecastToSystem(), portal.Right.RecastToSystem(), portal.IsNarrow, portal.IsProtectedAnchor, portal.Width, effectiveClearance);

    private static float Cross(Vector2 left, Vector2 right) => left.X * right.Y - left.Y * right.X;
}
