using vnavmesh.Navmesh;

namespace vnavmesh.Customizations;

[CustomizationTerritory(823)]
internal class Z0823TheQitanaRavel : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        //remove entire mesh and all instances
        scene.Meshes.Remove("<plane one-sided>");
    }
}
