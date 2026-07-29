using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Common.Extensions;

using static DtDetour;

internal static class DtNavMeshCreateParamsExtension
{
    extension
    (
        DtNavMeshCreateParams config
    )
    {
        public void AddOffMeshConnection
        (
            Vector3                      ptA,
            Vector3                      ptB,
            float                        radius,
            bool                         bidirectional,
            int                          userID,
            NavmeshArea                  area,
            NavmeshPolyFlags             flags,
            NavmeshOffMeshKind           kind,
            NavmeshLinkTraversalProfile? _ = null
        )
        {
            if (!DtOffMeshConnectionTileClassifier.ShouldStoreConnection(config, ptA, ptB))
            {
                NavmeshBuildLog.Information
                (
                    $"[NavmeshBuilder] 离网连接被当前瓦片忽略: 类型 = {kind}, 区域 = {area}, 标记 = {flags}, 起点 = {ptA}, 终点 = {ptB}, " +
                    $"区块范围 = ({config.bmin.X}, {config.bmin.Y}, {config.bmin.Z}) -> ({config.bmax.X}, {config.bmax.Y}, {config.bmax.Z})"
                );
                return;
            }

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
            dirs[^1] = bidirectional ?
                           DT_OFFMESH_CON_BIDIR :
                           0;

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
        }
    }

    private static void Extend<T>
    (
        ref T[]? arr,
        int      add
    )
    {
        arr ??= [];
        Array.Resize(ref arr, arr.Length + add);
    }
}
