using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(387)]
internal class Z0387逆转要害沙斯塔夏溶洞 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    ) =>
        //remove entire mesh and all instances
        scene.Meshes.Remove("bg/ffxiv/sea_s1/dun/s1d7/collision/s1d7_b1_mud2.pcb");
}
