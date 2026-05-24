using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Overworld;

[CustomizationTerritory(613)]
internal class Z0613红玉海 : NavmeshCustomization
{
    public override int Version => 3;

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings) =>
        settings.AgentRadius = 1f;

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 洞窟1
        LinkPoints(mesh, new(643.7f, 3.5f, -58.9f), new(636.6f, 3.9611104f, -63.3f), true);
        // 洞窟2
        LinkPoints(mesh, new(544.853f, -61.27358f, -163.34909f), new(545.4251f, -61.478912f, -155.09254f), true);
        // 碧玉水喷泉
        LinkClientPath(mesh, new(565.74817f, -60.55099f, -135.02992f), new(617.6802f, 1.8672085f, -81.71452f));
    }
}
