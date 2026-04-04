using DotRecast.Recast;
using vnavmesh.Navmesh;

namespace vnavmesh.Customizations;

[CustomizationTerritory(363)]
internal class Z0363TheLostCityofAmdapor : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        //remove entire mesh and all instances
        scene.Meshes.Remove("bg/ffxiv/fst_f1/dun/f1d5/collision/f1d5_a2_door2.pcb");
    }

    public Z0363TheLostCityofAmdapor() =>
        Settings.Partitioning = RcPartition.MONOTONE;
}
