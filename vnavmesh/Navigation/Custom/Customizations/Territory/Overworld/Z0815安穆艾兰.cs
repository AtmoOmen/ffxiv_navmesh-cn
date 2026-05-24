using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Custom.Extensions;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Overworld;

[CustomizationTerritory(815)]
public class Z0815安穆艾兰 : NavmeshCustomization
{
    public override int Version => 2;

    public Z0815安穆艾兰() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 新宫
        LinkPoints(mesh, new(434.5f, -106.0f, 590.0f), new(422.7f, -105.3f, 589.6f), true);

        LinkPoints(mesh, new(425.0f, -105.7f, 589.0f), new(434.7f, -105.5f, 591.1f), true);
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        // 新宫
        config.AddOffMeshConnection
        (
            new(425.0f, -105.7f, 589.0f),
            new(434.7f, -105.5f, 591.1f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.AllTraversable,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }
}
