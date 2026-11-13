using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory;

[CustomizationTerritory(939)]
internal class Z0939云冠群岛 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings) =>
        settings.GenerateEdgeClimbLinks = false;

    public override void CustomizeScene(SceneExtractor scene) =>
        scene.InsertWallCollider
        (
            new(4.5764923f, 4.3499985f),
            new(186.903f, 89.131f, 574.0929f),
            -93.99205f,
            true,
            PrimitiveFlags.ForceUnwalkable
        );
}
