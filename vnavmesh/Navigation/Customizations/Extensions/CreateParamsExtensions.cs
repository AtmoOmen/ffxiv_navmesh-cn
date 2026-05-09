using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Customizations.Extensions;

public static class CreateParamsExtensions
{
    extension(DtNavMeshCreateParams config)
    {
        public void AddOffMeshConnection
        (
            Vector3 ptA,
            Vector3 ptB,
            float   radius        = 0.5f,
            bool    bidirectional = false,
            int     userID        = 0
        ) =>
            config.AddOffMeshConnection
                (ptA, ptB, radius, bidirectional, userID, NavmeshArea.ManualOffMesh, NavmeshPolyFlags.ManualOffMesh, NavmeshOffMeshKind.ManualOffMesh);

        public void AddOffMeshConnection
        (
            Vector3            ptA,
            Vector3            ptB,
            float              radius,
            bool               bidirectional,
            int                userID,
            NavmeshArea        area,
            NavmeshPolyFlags   flags,
            NavmeshOffMeshKind kind
        )
        {

            var aInside = IsInsideTile(ptA);
            if (!aInside)
                return;

            Extend(ref config.offMeshConVerts, 6);
            config.offMeshConVerts[^6] = ptA.X;
            config.offMeshConVerts[^5] = ptA.Y;
            config.offMeshConVerts[^4] = ptA.Z;
            config.offMeshConVerts[^3] = ptB.X;
            config.offMeshConVerts[^2] = ptB.Y;
            config.offMeshConVerts[^1] = ptB.Z;

            Extend(ref config.offMeshConDir, 1);
            config.offMeshConDir[^1] = bidirectional ? DtDetour.DT_OFFMESH_CON_BIDIR : 0;

            Extend(ref config.offMeshConFlags, 1);
            config.offMeshConFlags[^1] = (int)flags;

            config.offMeshConCount++;

            Extend(ref config.offMeshConRad, 1);
            config.offMeshConRad[^1] = radius;

            Extend(ref config.offMeshConAreas, 1);
            config.offMeshConAreas[^1] = (int)area;

            Extend(ref config.offMeshConUserID, 1);
            config.offMeshConUserID[^1] = userID;

            Service.Log.Debug($"[NavmeshBuilder] 已加入离网连接: 类型 = {kind}, 区域 = {area}, 标记 = {flags}, 起点 = {ptA:f3}, 终点 = {ptB:f3}, 双向 = {bidirectional}");
            return;

            bool IsInsideTile(Vector3 p)
            {
                return p.X >= config.bmin.X && p.Y >= config.bmin.Y && p.Z >= config.bmin.Z && p.X <= config.bmax.X && p.Y <= config.bmax.Y && p.Z <= config.bmax.Z;
            }
        }
    }

    private static void Extend<T>([NotNull] ref T[]? arr, int add)
    {
        arr ??= [];
        Array.Resize(ref arr, arr.Length + add);
    }
}
