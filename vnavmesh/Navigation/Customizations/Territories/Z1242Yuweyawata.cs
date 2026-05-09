using System.Numerics;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(1242)]
internal class Z1242Yuweyawata : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        // avoid the hole in last boss arena
        scene.InsertCylinderCollider(new Vector3(11, 1, 11), new(34, -87.9f, -710), SceneExtractor.PrimitiveFlags.ForceUnwalkable);
    }
}
