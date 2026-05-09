using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.Territories.Alexandar;

[CustomizationTerritory(443)]
internal class Z0443启动之章2 : NavmeshCustomization
{
    public override int Version => 1;
    
    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers) =>
        LinkDrop(mesh, new(9.0f, 12.1f, 36.5f), new(-3.3f, -18.2f, 37.5f));
}
