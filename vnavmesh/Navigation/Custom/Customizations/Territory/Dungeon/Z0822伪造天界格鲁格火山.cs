using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(822)]
internal class Z0822伪造天界格鲁格火山 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        //remove entire mesh and all instances
        scene.Meshes.Remove("<plane one-sided>");
    }
}
