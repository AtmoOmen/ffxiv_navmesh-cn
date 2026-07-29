using System.Numerics;
using System.Runtime.CompilerServices;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Build.Custom;

internal static class OffMeshConnectionMetadataRegistry
{
    private static readonly ConditionalWeakTable<DtNavMeshCreateParams, List<NavmeshBuildOffMeshConnection>> Entries = new();

    public static void Record
    (
        DtNavMeshCreateParams        config,
        Vector3                      start,
        Vector3                      end,
        float                        radius,
        bool                         bidirectional,
        int                          userId,
        NavmeshArea                  area,
        NavmeshPolyFlags             flags,
        NavmeshOffMeshKind           kind,
        NavmeshLinkTraversalProfile? traversalProfile
    ) =>
        Entries.GetOrCreateValue(config).Add
        (
            new
            (
                start,
                end,
                radius,
                bidirectional,
                userId,
                (int)area,
                (int)flags,
                (int)kind,
                traversalProfile
            )
        );

    public static IReadOnlyList<NavmeshBuildOffMeshConnection> Read
    (
        DtNavMeshCreateParams config
    ) =>
        Entries.TryGetValue(config, out var entries) ?
            entries :
            [];

    public static List<NavmeshBuildOffMeshConnection> Collect
    (
        NavmeshCustomization customization
    )
    {
        var config = new DtNavMeshCreateParams
        {
            bmin = new RcVec3f(-1_000_000f, -1_000_000f, -1_000_000f),
            bmax = new RcVec3f(1_000_000f,  1_000_000f,  1_000_000f)
        };
        customization.CustomizeSettings(config);
        return [.. Read(config)];
    }
}
