using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Overworld;

[CustomizationTerritory(1188)]
internal class Z1188克扎玛乌卡湿地 : NavmeshCustomization
{
    public override int Version => 1;

    public Z1188克扎玛乌卡湿地() =>
        ApplyNormalPrecisionSettings();

    public override void CustomizeScene(SceneExtractor scene)
    {
        // contender #3 in the most cursed mesh transformation finalists
        if (scene.Meshes.TryGetValue("bg/ex5/02_ykt_y6/fld/y6f2/collision/y6f2_x0_tst00.pcb", out var mesh))
            mesh.Instances[0].WorldTransform.M42 += 0.05f;
    }
}
