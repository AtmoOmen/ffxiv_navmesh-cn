using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(365)]
internal class Z0365激战城塞石卫塔 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    ) =>
        //remove entire mesh and all instances
        scene.Meshes.Remove("bg/ffxiv/roc_r1/dun/r1d2/collision/r1d2_x1_rubb1.pcb");
}
