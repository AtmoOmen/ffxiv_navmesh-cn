using vnavmesh.Navmesh;

namespace vnavmesh.Customizations;

[CustomizationTerritory(1113)]
internal class Z1113Xelphatol : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene) =>
        scene.Meshes.Remove("<box>");
}
