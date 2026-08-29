using System.Numerics;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.Town;

[CustomizationTerritory(1186)]
internal class Z1186九号解决方案 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        // 以太之光广场东侧 ↔ 上层西侧
        LinkClientPath(mesh, new(143.61594f, 0.55f, 4.898482f), new(219.98926f, 60.7f, 4.989685f));
        LinkClientPath(mesh, new(221.16905f, 60.7f, -4.981979f), new(146.59338f, 0.55f, -5.0202637f));

        // 以太之光广场北侧 ↔ 上层南侧
        LinkClientPath(mesh, new(4.97122f, 0.5f, -114.9926f), new(4.989685f, 36.7f, -170.0008f));
        LinkClientPath(mesh, new(-5.040139f, 36.69999f, -172.79323f), new(-5.0202637f, 0.5f, -117.021484f));

        // 升降梯
        LinkClientPath(mesh, new(-218.04851f, 1.1641076f, -66.99873f), new(-224.56708f, 36.1f, -73.56378f));
        LinkClientPath(mesh, new(-228.85681f, 36.1f, -70.94159f), new(-222.30872f, 1.0844975f, -64.19476f));
    }
}
