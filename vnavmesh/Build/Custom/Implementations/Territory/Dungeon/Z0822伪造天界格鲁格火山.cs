using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(822)]
internal class Z0822伪造天界格鲁格火山 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    ) =>
        //remove entire mesh and all instances
        scene.Meshes.Remove("<plane one-sided>");
}
