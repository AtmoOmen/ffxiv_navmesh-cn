using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(153)]
internal class Z0153黑衣森林南部林区 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("<plane one-sided>", out var planes))
        {
            planes.Instances.RemoveAll(instance => instance.WorldTransform.Row1.Y is > 3 and < 4);
        }
    }
}
