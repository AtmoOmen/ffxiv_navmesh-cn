using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.Territories.Overworld;

[CustomizationTerritory(815)]
public class Z0815安穆艾兰 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 新宫
        LinkPoints(mesh, new(434.5f, -106.0f, 590.0f), new(422.7f, -105.3f, 589.6f));
        
        LinkPoints(mesh, new(425.0f, -105.7f, 589.0f), new(434.7f, -105.5f, 591.1f));
    }
}
