using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Overworld;

[CustomizationTerritory(613)]
internal class Z0613红玉海 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0613红玉海() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // the tunnel into the island containing tamamizu has some floor that is unlandable
        LinkPoints(mesh, new(643.7f, 3.4f, -58.9f), new(636.6f, 3.9f, -63.3f));
    }
}
