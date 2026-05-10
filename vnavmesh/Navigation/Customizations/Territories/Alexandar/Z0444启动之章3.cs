using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;

namespace vnavmesh.Navigation.Customizations.Territories.Alexandar;

[CustomizationTerritory(444)]
internal class Z0444启动之章3 : NavmeshCustomization
{
    public override int Version => 2;
    
    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 传送带
        LinkPoints(mesh, new(58.3f, -0.0f, -20.4f), new(58.0f, -9.0f, -41.8f));
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(21.6f, 36.0f, -13.3f),
            new(25.0f, 23.6f, -20.7f),
            2f,
            false,
            0,
            NavmeshArea.GeneratedClimbDown,
            NavmeshPolyFlags.GeneratedClimbDown,
            NavmeshOffMeshKind.GeneratedClimbDown
        );
        
        config.AddOffMeshConnection
        (
            new(17.1f, 18.0f, -17.3f),
            new(18.1f, 10.0f, -24.3f),
            2f,
            false,
            0,
            NavmeshArea.GeneratedClimbDown,
            NavmeshPolyFlags.GeneratedClimbDown,
            NavmeshOffMeshKind.GeneratedClimbDown
        );
    }
}
