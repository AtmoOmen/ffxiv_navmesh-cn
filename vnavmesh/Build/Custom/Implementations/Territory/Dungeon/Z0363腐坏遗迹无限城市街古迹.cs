using DotRecast.Recast;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(363)]
internal class Z0363腐坏遗迹无限城市街古迹 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    ) =>
        //remove entire mesh and all instances
        scene.Meshes.Remove("bg/ffxiv/fst_f1/dun/f1d5/collision/f1d5_a2_door2.pcb");

    public Z0363腐坏遗迹无限城市街古迹() =>
        Settings.Partitioning = RcPartition.MONOTONE;
}
