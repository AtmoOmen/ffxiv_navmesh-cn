using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(1187)]
public class Z1187奥阔帕恰山 : NavmeshCustomization
{
    public override int Version => 3;

    public Z1187奥阔帕恰山() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    ) =>
        // 瓦丘恩佩洛到休养所
        LinkPoints(mesh, new(431.9f, -143.7f, -302.7f), new(456.9f, -131.7f, -294.4f), true);
}
