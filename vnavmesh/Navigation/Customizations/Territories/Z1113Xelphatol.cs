using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(1113)]
internal class Z1113Xelphatol : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene) =>
        scene.Meshes.Remove("<box>");
}
