using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;

namespace vnavmesh.Navigation.Customizations.Territories.Alexandar;

[CustomizationTerritory(443)]
internal class Z0443启动之章2 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0443启动之章2() =>
        Settings.Filtering -= NavmeshFilter.LedgeSpans;

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers) =>
        LinkDrop(mesh, new(9.0f, 12.1f, 36.5f), new(-3.3f, -18.2f, 37.5f));
    
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
