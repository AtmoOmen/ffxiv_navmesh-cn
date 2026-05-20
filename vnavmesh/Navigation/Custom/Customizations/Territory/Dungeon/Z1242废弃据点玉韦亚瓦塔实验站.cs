using System.Numerics;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Custom.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(1242)]
internal class Z1242废弃据点玉韦亚瓦塔实验站 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        // avoid the hole in last boss arena
        scene.InsertCylinderCollider(new Vector3(11, 1, 11), new(34, -87.9f, -710), SceneExtractor.PrimitiveFlags.ForceUnwalkable);
    }
}
