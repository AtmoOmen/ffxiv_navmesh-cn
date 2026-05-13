using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Dungeon;

[CustomizationTerritory(1193)]
internal class Z1193通天绝壁沃刻佐莫山 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        // remove large crystal blocking initial path, which is destroyed after first pack dies
        scene.Meshes.Remove("bg/ex5/02_ykt_y6/dun/y6d2/collision/y6d2_a1_cry03.pcb");
    }
}
