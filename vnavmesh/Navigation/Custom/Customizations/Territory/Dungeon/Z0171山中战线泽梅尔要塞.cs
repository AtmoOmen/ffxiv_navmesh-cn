using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(171)]
internal class Z0171山中战线泽梅尔要塞 : NavmeshCustomization
{
    public override int Version => 2;

    public override void CustomizeScene(SceneExtractor scene)
    {
        foreach (var (key, mesh) in scene.Meshes)
        {
            if (key.StartsWith("bg/ffxiv/roc_r1/rad/r1r1/collision/r1r1_a1_dor"))
                mesh.Instances.Clear();
        }
    }
}
