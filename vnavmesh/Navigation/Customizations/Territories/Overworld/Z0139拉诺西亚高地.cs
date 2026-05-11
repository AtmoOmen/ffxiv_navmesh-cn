using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories.Overworld;

[CustomizationTerritory(139)]
public class Z0139拉诺西亚高地 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0139拉诺西亚高地()
    {
        Settings.AgentMaxClimb = 1.2f;
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        //
        LinkPoints(mesh, new(346.7f, -2.9f, 14.7f), new(346.3f, -3.1f, 161.4f));
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(346.7f, -2.9f, 14.7f),
            new(346.3f, -3.1f, 161.4f),
            3f,
            true,
            0,
            NavmeshArea.Ground,
            NavmeshPolyFlags.Ground,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }
}
