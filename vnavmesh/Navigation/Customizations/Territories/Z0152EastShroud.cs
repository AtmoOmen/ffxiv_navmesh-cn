using System.Numerics;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(152)]
internal class Z0152EastShroud : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene) =>
        scene.InsertCylinderCollider(new Vector3(2, 2, 2), new(-40, -8, 225), SceneExtractor.PrimitiveFlags.Unlandable);
}
