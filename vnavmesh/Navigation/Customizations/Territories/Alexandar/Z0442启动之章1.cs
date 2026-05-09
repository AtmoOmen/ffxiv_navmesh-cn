using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;

namespace vnavmesh.Navigation.Customizations.Territories.Alexandar;

[CustomizationTerritory(442)]
internal class Z0442启动之章1 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 喷气
        LinkPoints(mesh, new(3.2f, -7.3f, 20.3f), new(2.6f, -4.2f, 3.0f));
        LinkPoints(mesh, new(2.6f, -4.2f, 3.0f),  new(-4.6f, -3.4f, -1.9f));
    }
    
     public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        // 传送带
        config.AddOffMeshConnection(new(0.1f, 12.0f, -106.5f), new(-0.0f, -23.9f, -148.1f));
    }
}
