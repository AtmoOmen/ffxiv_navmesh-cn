using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Custom.Extensions;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Alexandar;

[CustomizationTerritory(443)]
internal class Z0443启动之章2 : NavmeshCustomization
{
    public override int Version => 2;
    
    public override void CustomizeSettings(DtNavMeshCreateParams config) =>
        config.AddOffMeshConnection
        (
            new(9.0f, 12.1f, 36.5f), 
            new(-3.3f, -18.2f, 37.5f),
            0.5f,
            false,
            0,
            NavmeshArea.GeneratedClimbDown,
            NavmeshPolyFlags.GeneratedClimbDown,
            NavmeshOffMeshKind.GeneratedClimbDown
        );
}
