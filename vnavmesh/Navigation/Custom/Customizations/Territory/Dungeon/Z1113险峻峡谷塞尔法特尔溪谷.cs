using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(1113)]
internal class Z1113险峻峡谷塞尔法特尔溪谷 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene) =>
        scene.Meshes.Remove("<box>");
}
