using System.Numerics;
using DotRecast.Detour;
using DotRecast.Core.Numerics;
using vnavmesh.Navigation.Mesh.Runtime;

namespace vnavmesh.Common.Navigation.Mesh.Build;

using static DtDetour;

internal static class DtNavMeshCreateParamsExtensions
{
    public static void AddOffMeshConnection
    (
        this DtNavMeshCreateParams config,
        Vector3                    ptA,
        Vector3                    ptB,
        float                      radius,
        bool                       bidirectional,
        int                        userID,
        NavmeshArea                area,
        NavmeshPolyFlags           flags,
        NavmeshOffMeshKind         _
    )
    {
        if (!IsInsideTile(ptA))
            return;

        Extend(ref config.offMeshConVerts, 6);
        var verts = config.offMeshConVerts!;
        verts[^6] = ptA.X;
        verts[^5] = ptA.Y;
        verts[^4] = ptA.Z;
        verts[^3] = ptB.X;
        verts[^2] = ptB.Y;
        verts[^1] = ptB.Z;

        Extend(ref config.offMeshConDir, 1);
        var dirs = config.offMeshConDir!;
        dirs[^1] = bidirectional ? DT_OFFMESH_CON_BIDIR : 0;

        Extend(ref config.offMeshConFlags, 1);
        var flagsArr = config.offMeshConFlags!;
        flagsArr[^1] = (int)flags;

        config.offMeshConCount++;

        Extend(ref config.offMeshConRad, 1);
        var radii = config.offMeshConRad!;
        radii[^1] = radius;

        Extend(ref config.offMeshConAreas, 1);
        var areas = config.offMeshConAreas!;
        areas[^1] = (int)area;

        Extend(ref config.offMeshConUserID, 1);
        var userIds = config.offMeshConUserID!;
        userIds[^1] = userID;
        return;

        bool IsInsideTile(Vector3 p)
        {
            RcVec3f rp = new(p.X, p.Y, p.Z);
            return rp.X >= config.bmin.X && rp.Y >= config.bmin.Y && rp.Z >= config.bmin.Z &&
                   rp.X <= config.bmax.X && rp.Y <= config.bmax.Y && rp.Z <= config.bmax.Z;
        }
    }

    private static void Extend<T>(ref T[]? arr, int add)
    {
        arr ??= [];
        Array.Resize(ref arr, arr.Length + add);
    }
}
