using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory.Town;

[CustomizationTerritory(130)]
internal class Z0130乌尔达哈现世回廊 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        // Force all instances of Mesh as unwalkable
        if (scene.Meshes.TryGetValue("bg/ffxiv/wil_w1/twn/common/collision/w1t0_f0_kadn1.pcb", out var mesh))
        {
            foreach (var instance in mesh.Instances)
                instance.ForceSetPrimFlags |= PrimitiveFlags.ForceUnwalkable;
        }
    }
}
