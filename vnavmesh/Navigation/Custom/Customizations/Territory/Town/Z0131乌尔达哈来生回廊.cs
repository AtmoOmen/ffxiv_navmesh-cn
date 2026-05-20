using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Town;

[CustomizationTerritory(131)]
internal class Z0131乌尔达哈来生回廊 : NavmeshCustomization
{
    public override int Version => 2;
    
    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 很诡异但是得要有
        LinkPoints(mesh, new(-87.3f, 42.0f, 48.3f), new(-83.9f, 42.0f, 35.3f), true);
        
        LinkPoints(mesh, new(-22.7f, 30.0f, -6.6f), new(-68.6f, 37.9f, -18.6f), true);
        
        LinkPoints(mesh, new(62.0f, 29.0f, -36.3f), new(42.1f, 34.0f, -24.3f), true);
    }
}
