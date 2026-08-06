using System.Numerics;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.Town;

[CustomizationTerritory(129)]
internal class Z0129利姆萨罗敏萨下层甲板 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        // 船内楼梯
        LinkShortcut(mesh, new(-274.10587f, 11.32725f, 188.9568f), new(-272.5555f, 11.780226f, 188.65962f), true);
    }
}
