using System.Collections.Generic;
using DotRecast.Core.Numerics;
using DotRecast.Detour;

namespace Navmesh;

public sealed class GroundDetourQuery(DtNavMesh navmesh) : DtNavMeshQuery(navmesh)
{
    public bool TryBuildCorridor(List<long> polygons, RcVec3f startPos, RcVec3f endPos, out MeshCorridor? corridor)
    {
        corridor = null;
        if (polygons.Count == 0)
            return false;

        if (ClosestPointOnPoly(polygons[0], startPos, out var clampedStart, out _).Failed())
            return false;
        if (ClosestPointOnPoly(polygons[^1], endPos, out var clampedEnd, out _).Failed())
            return false;

        List<MeshPortal> portals = new(polygons.Count - 1);
        for (var i = 0; i < polygons.Count - 1; i++)
        {
            if (!TryGetPortal(polygons[i], polygons[i + 1], out var portal))
                return false;
            portals.Add(portal);
        }

        corridor = new(clampedStart, clampedEnd, polygons.ToArray(), portals);
        return true;
    }

    public bool TryGetPortal(long fromRef, long toRef, out MeshPortal portal)
    {
        portal = default;
        var status = GetPortalPoints(fromRef, toRef, out var left, out var right, out _, out _);
        if (status.Failed())
            return false;

        portal = new(fromRef, toRef, left, right);
        return true;
    }
}
