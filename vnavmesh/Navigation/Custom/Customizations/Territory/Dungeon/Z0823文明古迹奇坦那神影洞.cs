using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(823)]
internal class Z0823文明古迹奇坦那神影洞 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        //remove entire mesh and all instances
        scene.Meshes.Remove("<plane one-sided>");
    }
}
