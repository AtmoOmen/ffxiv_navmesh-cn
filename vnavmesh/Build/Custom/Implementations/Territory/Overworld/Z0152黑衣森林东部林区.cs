using System.Numerics;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(152)]
internal class Z0152黑衣森林东部林区 : NavmeshCustomization
{
    public override int Version => 2;

    public Z0152黑衣森林东部林区() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeScene
    (
        SceneExtractor scene
    ) =>
        scene.InsertCylinderCollider(new Vector3(2, 2, 2), new(-40, -8, 225), PrimitiveFlags.Unlandable);
}
