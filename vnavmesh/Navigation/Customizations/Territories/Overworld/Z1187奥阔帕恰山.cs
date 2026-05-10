using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.Territories.Overworld;

[CustomizationTerritory(1187)]
public class Z1187奥阔帕恰山 : NavmeshCustomization
{
    public override int Version => 1;
    
    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 瓦丘恩佩洛到休养所
        LinkPoints(mesh, new(431.9f, -143.7f, -302.7f), new(456.9f, -131.7f, -294.4f));
    }
}
