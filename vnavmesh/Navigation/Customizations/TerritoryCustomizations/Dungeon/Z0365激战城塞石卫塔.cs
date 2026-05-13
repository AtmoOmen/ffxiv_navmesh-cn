using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Dungeon;

[CustomizationTerritory(365)]
internal class Z0365激战城塞石卫塔 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        //remove entire mesh and all instances
        scene.Meshes.Remove("bg/ffxiv/roc_r1/dun/r1d2/collision/r1d2_x1_rubb1.pcb");
    }
}
