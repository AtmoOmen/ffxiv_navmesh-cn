using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(146)]
internal class Z0146南萨纳兰 : NavmeshCustomization
{
    public override int Version => 5;

    public Z0146南萨纳兰() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("<box>", out var mesh))
            mesh.Instances.RemoveAll(i => i.Material == 0x206406);

        // the ground directly in front of the bridge next to the amalj'aa camp has two triangles that cannot be landed on
        if (scene.Meshes.TryGetValue("bg/ffxiv/wil_w1/fld/w1f4/collision/tr1610.pcb", out var mesh2))
        {
            foreach (var inst in mesh2.Instances)
                inst.ForceClearPrimFlags |= PrimitiveFlags.Unlandable;
        }
    }
}
