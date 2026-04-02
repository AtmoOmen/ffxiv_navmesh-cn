using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh;

internal sealed class GroundPathPostProcessor
{
    private const float EndpointClearanceCap = 0.2f;
    private const float MinimumClearance = 0.05f;
    private const float PortalClearanceMargin = 0.05f;
    private const float WallSearchPadding = 0.25f;
    private const float SampleHalfExtentY = 2.0f;
    private const float SampleStep = 0.35f;
    private const float CornerAngleThresholdDot = 0.70710677f;
    private const float CornerOffsetMin = 0.2f;
    private const float CornerOffsetMax = 0.75f;
    private const float CornerOffsetScale = 1.25f;
    private const float CornerExpansionMinSegment = 0.9f;
    private const float EndpointCleanupRadius = 2.5f;
    private const float ShortcutClearanceCap = 0.2f;
    private const float SamePointDistanceSq = 0.000001f;

    private readonly DtNavMeshQuery _query;
    private readonly DtNavMesh _navMesh;
    private readonly IDtQueryFilter _filter;

    public GroundPathPostProcessor(DtNavMeshQuery query, IDtQueryFilter filter)
    {
        _query = query;
        _navMesh = query.GetAttachedNavMesh();
        _filter = filter;
    }

    public List<Vector3>? Build(RcVec3f startPos, IReadOnlyList<long> corridor, RcVec3f finalEndPos)
    {
        if (corridor.Count == 0 || corridor[0] == 0)
            return null;

        var anchors = BuildAnchors(startPos.RecastToSystem(), corridor, finalEndPos.RecastToSystem());
        if (anchors == null || anchors.Count == 0)
            return null;

        return ShortcutAnchors(anchors);
    }

    private List<GroundAnchor>? BuildAnchors(Vector3 startPos, IReadOnlyList<long> corridor, Vector3 finalEndPos)
    {
        var portals = ExtractPortals(corridor);
        if (portals == null)
            return null;

        List<GroundAnchor> anchors = [CreateAnchor(startPos, corridor[0], corridor[0], 0, true, true)];
        foreach (var portal in portals)
        {
            var anchorPosition = portal.Width > MinimumClearance ? portal.Midpoint : portal.Left;
            anchors.Add(CreateAnchor(anchorPosition, portal.ToRef, portal.FromRef, portal.Width, !portal.IsHardTransition, portal.IsHardTransition));
        }

        anchors.Add(CreateAnchor(finalEndPos, corridor[^1], corridor[^1], 0, true, true));
        anchors = SimplifyAnchorsByVisibility(anchors);
        anchors = ExpandCornerAnchors(anchors);
        return CleanupEndpointDetours(anchors);
    }

    private List<GroundPortal>? ExtractPortals(IReadOnlyList<long> corridor)
    {
        List<GroundPortal> portals = new(corridor.Count - 1);
        for (var i = 0; i < corridor.Count - 1; ++i)
        {
            var fromRef = corridor[i];
            var toRef = corridor[i + 1];
            if (!TryGetPortal(fromRef, toRef, out var portal))
                return null;

            portals.Add(portal);
        }

        return portals;
    }

    private bool TryGetPortal(long fromRef, long toRef, out GroundPortal portal)
    {
        List<RcSegmentVert> segments = [];
        List<long> refs = [];
        var status = _query.GetPolyWallSegments(fromRef, true, _filter, ref segments, ref refs);
        if (status.Failed())
        {
            portal = default;
            return false;
        }

        var bestIndex = -1;
        var bestWidthSq = float.MinValue;
        for (var i = 0; i < refs.Count; ++i)
        {
            if (refs[i] != toRef)
                continue;

            var widthSq = Vector3.DistanceSquared(segments[i].vmin.RecastToSystem(), segments[i].vmax.RecastToSystem());
            if (widthSq > bestWidthSq)
            {
                bestWidthSq = widthSq;
                bestIndex = i;
            }
        }

        if (bestIndex < 0)
        {
            portal = default;
            return false;
        }

        portal = new(fromRef, toRef, segments[bestIndex].vmin.RecastToSystem(), segments[bestIndex].vmax.RecastToSystem(), IsHardTransition(fromRef, toRef));
        return true;
    }

    private bool IsHardTransition(long fromRef, long toRef)
    {
        return IsHardPoly(fromRef) || IsHardPoly(toRef);
    }

    private bool IsHardPoly(long polyRef)
    {
        _navMesh.GetTileAndPolyByRefUnsafe(polyRef, out _, out var poly);
        return poly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION || poly.GetArea() == Navmesh.AREAID_TELEPORT;
    }

    private List<GroundAnchor> ExpandCornerAnchors(List<GroundAnchor> anchors)
    {
        if (anchors.Count <= 2)
            return anchors;

        List<GroundAnchor> expanded = [anchors[0]];
        for (var i = 1; i < anchors.Count - 1; ++i)
        {
            var previous = anchors[i - 1];
            var current = anchors[i];
            var next = anchors[i + 1];
            if (!current.AllowShortcut)
            {
                AppendAnchor(expanded, current);
                continue;
            }

            var inDirection = NormalizeXZ(current.Position - previous.Position, out var previousLength);
            var outDirection = NormalizeXZ(next.Position - current.Position, out var nextLength);
            if (previousLength < CornerExpansionMinSegment || nextLength < CornerExpansionMinSegment || Vector3.Dot(inDirection, outDirection) >= CornerAngleThresholdDot)
            {
                AppendAnchor(expanded, current);
                continue;
            }

            var offset = MathF.Min(MathF.Min(previousLength, nextLength) * 0.4f, current.Clearance * CornerOffsetScale);
            offset = Math.Clamp(offset, CornerOffsetMin, CornerOffsetMax);

            var preAnchor = CreateDerivedAnchor(current.Position - inDirection * offset, previous.PolyRef, current.PolyRef, MathF.Min(previous.Clearance, current.Clearance), true);
            var centerAnchor = current with { AllowShortcut = false };
            var postAnchor = CreateDerivedAnchor(current.Position + outDirection * offset, current.PolyRef, next.PolyRef, MathF.Min(current.Clearance, next.Clearance), true);
            AppendAnchor(expanded, preAnchor);
            AppendAnchor(expanded, centerAnchor);
            AppendAnchor(expanded, postAnchor);
        }

        AppendAnchor(expanded, anchors[^1]);
        return expanded;
    }

    private List<GroundAnchor> CleanupEndpointDetours(List<GroundAnchor> anchors)
    {
        if (anchors.Count <= 3)
            return anchors;

        var cleanedStart = CleanupLeadingDetours(anchors);
        cleanedStart.Reverse();
        var cleanedEnd = CleanupLeadingDetours(cleanedStart);
        cleanedEnd.Reverse();
        return cleanedEnd;
    }

    private List<GroundAnchor> CleanupLeadingDetours(List<GroundAnchor> anchors)
    {
        if (anchors.Count <= 3)
            return anchors;

        var anchor0 = anchors[0];
        var bestIndex = 0;
        for (var candidate = 1; candidate < anchors.Count - 1; ++candidate)
        {
            if (DistanceXZ(anchor0.Position, anchors[candidate].Position) > EndpointCleanupRadius)
                break;

            if (!CanFlattenEndpointSpan(anchors, 0, candidate))
                continue;

            bestIndex = candidate;
        }

        if (bestIndex <= 1)
            return anchors;

        List<GroundAnchor> result = [anchors[0]];
        for (var i = bestIndex; i < anchors.Count; ++i)
            result.Add(anchors[i]);

        return result;
    }

    private bool CanFlattenEndpointSpan(List<GroundAnchor> anchors, int fromIndex, int toIndex)
    {
        if (!HasLineOfSight(anchors[fromIndex], anchors[toIndex]))
            return false;

        for (var i = fromIndex + 1; i < toIndex; ++i)
        {
            if (IsHardPoly(anchors[i].PolyRef))
                return false;
        }

        return true;
    }

    private GroundAnchor CreateDerivedAnchor(Vector3 position, long preferredRef, long fallbackRef, float clearance, bool allowShortcut)
    {
        var resolved = ResolvePointOnMesh(position, preferredRef, fallbackRef, clearance);
        var resolvedClearance = ComputeClearance(resolved.Position, resolved.PolyRef, 0, false);
        return new(resolved.Position, resolved.PolyRef, MathF.Min(MathF.Max(clearance, MinimumClearance), resolvedClearance), allowShortcut);
    }

    private GroundAnchor CreateAnchor(Vector3 position, long preferredRef, long fallbackRef, float portalWidth, bool allowShortcut, bool relaxClearance)
    {
        var resolved = ResolvePointOnMesh(position, preferredRef, fallbackRef, portalWidth);
        var clearance = ComputeClearance(resolved.Position, resolved.PolyRef, portalWidth, relaxClearance);
        return new(resolved.Position, resolved.PolyRef, clearance, allowShortcut);
    }

    private (Vector3 Position, long PolyRef) ResolvePointOnMesh(Vector3 position, long preferredRef, long fallbackRef, float halfExtentHint)
    {
        if (TryProjectToPoly(preferredRef, position, out var preferredPoint))
            return (preferredPoint, preferredRef);

        if (fallbackRef != preferredRef && TryProjectToPoly(fallbackRef, position, out var fallbackPoint))
            return (fallbackPoint, fallbackRef);

        var halfExtentXZ = MathF.Max(0.25f, halfExtentHint + 0.15f);
        var status = _query.FindNearestPoly(position.SystemToRecast(), new(halfExtentXZ, SampleHalfExtentY, halfExtentXZ), _filter, out var nearestRef, out var nearestPoint, out _);
        if (status.Succeeded() && nearestRef != 0)
            return (nearestPoint.RecastToSystem(), nearestRef);

        var polyRef = preferredRef != 0 ? preferredRef : fallbackRef;
        return (position, polyRef);
    }

    private bool TryProjectToPoly(long polyRef, Vector3 position, out Vector3 projectedPoint)
    {
        projectedPoint = default;
        if (polyRef == 0)
            return false;

        var status = _query.ClosestPointOnPoly(polyRef, position.SystemToRecast(), out var closest, out _);
        if (status.Failed())
            return false;

        projectedPoint = closest.RecastToSystem();
        return true;
    }

    private float ComputeClearance(Vector3 position, long polyRef, float portalWidth, bool relaxClearance)
    {
        var clearance = MathF.Max(GetAgentRadius(polyRef), MinimumClearance);
        if (portalWidth > 0)
            clearance = MathF.Min(clearance, MathF.Max(MinimumClearance, portalWidth * 0.5f - PortalClearanceMargin));

        if (polyRef != 0)
        {
            var searchRadius = clearance + WallSearchPadding;
            var status = _query.FindDistanceToWall(polyRef, position.SystemToRecast(), searchRadius, _filter, out var hitDistance, out _, out _);
            if (status.Succeeded())
                clearance = MathF.Min(clearance, MathF.Max(MinimumClearance, hitDistance - 0.02f));
        }

        if (relaxClearance)
            clearance = MathF.Min(clearance, EndpointClearanceCap);

        return MathF.Max(clearance, MinimumClearance);
    }

    private float GetAgentRadius(long polyRef)
    {
        if (polyRef == 0)
            return MinimumClearance;

        _navMesh.GetTileAndPolyByRefUnsafe(polyRef, out var tile, out _);
        return tile.data.header.walkableRadius;
    }

    private List<Vector3> ShortcutAnchors(List<GroundAnchor> anchors)
    {
        List<Vector3> result = [anchors[0].Position];
        var currentIndex = 0;
        while (currentIndex < anchors.Count - 1)
        {
            var nextIndex = currentIndex + 1;
            for (var candidate = anchors.Count - 1; candidate > currentIndex + 1; --candidate)
            {
                if (!CanSkipAnchors(anchors, currentIndex, candidate))
                    continue;

                if (IsSafeShortcut(anchors, currentIndex, candidate))
                {
                    nextIndex = candidate;
                    break;
                }
            }

            currentIndex = nextIndex;
            AppendPoint(result, anchors[currentIndex].Position);
        }

        return result;
    }

    private List<GroundAnchor> SimplifyAnchorsByVisibility(List<GroundAnchor> anchors)
    {
        if (anchors.Count <= 2)
            return anchors;

        List<GroundAnchor> result = [anchors[0]];
        var currentIndex = 0;
        while (currentIndex < anchors.Count - 1)
        {
            var nextIndex = currentIndex + 1;
            for (var candidate = anchors.Count - 1; candidate > currentIndex + 1; --candidate)
            {
                if (!CanSkipAnchors(anchors, currentIndex, candidate))
                    continue;

                if (HasLineOfSight(anchors[currentIndex], anchors[candidate]))
                {
                    nextIndex = candidate;
                    break;
                }
            }

            currentIndex = nextIndex;
            result.Add(anchors[currentIndex]);
        }

        return result;
    }

    private bool CanSkipAnchors(List<GroundAnchor> anchors, int fromIndex, int toIndex)
    {
        for (var i = fromIndex + 1; i < toIndex; ++i)
        {
            if (!anchors[i].AllowShortcut)
                return false;
        }

        return true;
    }

    private bool IsSafeShortcut(List<GroundAnchor> anchors, int fromIndex, int toIndex)
    {
        var from = anchors[fromIndex];
        var to = anchors[toIndex];
        if (!HasLineOfSight(from, to))
            return false;

        var requiredClearance = float.MaxValue;
        for (var i = fromIndex; i <= toIndex; ++i)
            requiredClearance = MathF.Min(requiredClearance, anchors[i].Clearance);

        requiredClearance = MathF.Min(requiredClearance, ShortcutClearanceCap);

        var distance = Vector2.Distance(new(from.Position.X, from.Position.Z), new(to.Position.X, to.Position.Z));
        var sampleCount = Math.Max(1, (int)MathF.Ceiling(distance / SampleStep));
        for (var i = 1; i < sampleCount; ++i)
        {
            var tSample = i / (float)sampleCount;
            var samplePosition = Vector3.Lerp(from.Position, to.Position, tSample);
            if (!HasEnoughWallDistance(samplePosition, requiredClearance))
                return false;
        }

        return true;
    }

    private bool HasLineOfSight(GroundAnchor from, GroundAnchor to)
    {
        if (from.PolyRef == 0)
            return false;

        List<long> raycastPath = [];
        var raycastStatus = _query.Raycast(from.PolyRef, from.Position.SystemToRecast(), to.Position.SystemToRecast(), _filter, out var t, out _, ref raycastPath);
        return raycastStatus.Succeeded() && (t >= 0.999f || t == float.MaxValue);
    }

    private bool HasEnoughWallDistance(Vector3 position, float requiredClearance)
    {
        var halfExtentXZ = MathF.Max(0.25f, requiredClearance + 0.15f);
        var nearestStatus = _query.FindNearestPoly(position.SystemToRecast(), new(halfExtentXZ, SampleHalfExtentY, halfExtentXZ), _filter, out var sampleRef, out var samplePoint, out _);
        if (nearestStatus.Failed() || sampleRef == 0)
            return false;

        var searchRadius = requiredClearance + WallSearchPadding;
        var wallStatus = _query.FindDistanceToWall(sampleRef, samplePoint, searchRadius, _filter, out var hitDistance, out _, out _);
        return wallStatus.Succeeded() && hitDistance + 0.01f >= requiredClearance;
    }

    private static Vector3 NormalizeXZ(Vector3 vector, out float length)
    {
        vector.Y = 0;
        length = vector.Length();
        return length > 0 ? vector / length : Vector3.Zero;
    }

    private static float DistanceXZ(Vector3 a, Vector3 b)
    {
        return Vector2.Distance(new(a.X, a.Z), new(b.X, b.Z));
    }

    private static void AppendAnchor(List<GroundAnchor> anchors, GroundAnchor anchor)
    {
        if (anchors.Count > 0 && Vector3.DistanceSquared(anchors[^1].Position, anchor.Position) <= SamePointDistanceSq)
        {
            anchors[^1] = anchor;
            return;
        }

        anchors.Add(anchor);
    }

    private static void AppendPoint(List<Vector3> points, Vector3 point)
    {
        if (points.Count == 0 || Vector3.DistanceSquared(points[^1], point) > SamePointDistanceSq)
            points.Add(point);
    }
}
