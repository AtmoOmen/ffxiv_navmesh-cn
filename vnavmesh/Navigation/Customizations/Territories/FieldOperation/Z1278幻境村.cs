using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories.FieldOperation;

[CustomizationTerritory(1278)]
internal class Z1278幻境村 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings)
    {
        settings.CellSize = 0.25f;
        settings.CellHeight = 0.125f;
        settings.AgentHeight = 1.5f;
        settings.AgentRadius = 0.5f;
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        LinkPoints(mesh, new(-41.43025f, 0f, -15.052547f), new(-50.33422f, 5.0200005f, -15.016414f));
        LinkPoints(mesh, new(40.14966f, 0f, -0.5766049f), new(-40.960224f, -4.7683716E-07f, -15.025065f));
        LinkPoints(mesh, new(-30.270475f, 0.08126736f, -49.636017f), new(-30.27427f, 14.999999f, -74.9241f));
        LinkPoints(mesh, new(-30.234497f, 15.211092f, -84.85207f), new(-30.152498f, 30f, -110.05676f));
        LinkPoints(mesh, new(-30.255522f, 15f, -74.93256f), new(-30.239004f, 15.194771f, -84.824875f));
    }

}
