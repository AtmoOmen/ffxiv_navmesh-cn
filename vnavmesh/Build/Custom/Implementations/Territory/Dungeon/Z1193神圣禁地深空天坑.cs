using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(1194)]
internal class Z1193神圣禁地深空天坑 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    ) =>
        config.AddOffMeshConnection
        (
            new(-159.43634f, -51.62309f, -209.07713f),
            new(-169.7469f, -201.3f, -208.59708f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        LinkClientPath(mesh, new(100.06602f, -189.8996f, -231.05582f), new(139.63399f, -184.84326f, -294.38354f));
        LinkClientPath(mesh, new(99.991234f, -194.8996f, -363.95795f), new(99.36945f, -191.99121f, -415.7162f));
    }
}
