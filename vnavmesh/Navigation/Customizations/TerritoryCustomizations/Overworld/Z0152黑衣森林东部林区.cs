using System.Numerics;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Overworld;

[CustomizationTerritory(152)]
internal class Z0152黑衣森林东部林区 : NavmeshCustomization
{
    public override int Version => 2;

    public Z0152黑衣森林东部林区() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeScene(SceneExtractor scene) =>
        scene.InsertCylinderCollider(new Vector3(2, 2, 2), new(-40, -8, 225), SceneExtractor.PrimitiveFlags.Unlandable);
}
