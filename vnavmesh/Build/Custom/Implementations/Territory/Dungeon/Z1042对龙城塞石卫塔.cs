using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(1042)]
internal class Z1042对龙城塞石卫塔 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        //Force all instances of Mesh as unwalkable
        if (scene.Meshes.TryGetValue("bg/ffxiv/roc_r1/dun/r1d1/collision/r1d1_b1_sas03.pcb", out var mesh))
        {
            foreach (var instance in mesh.Instances)
                instance.ForceSetPrimFlags |= SceneExtractor.PrimitiveFlags.ForceUnwalkable;
        }
    }
}
