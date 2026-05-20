using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(1038)]
internal class Z1038封锁坑道铜铃铜山 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        //remove entire mesh and all instances
        scene.Meshes.Remove("bg/ffxiv/wil_w1/dun/w1d1/collision/w1d1_a2_wall1.pcb");
    }
}
