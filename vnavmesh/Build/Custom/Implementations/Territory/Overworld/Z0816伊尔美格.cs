using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Common.Build.Ground.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(816)]
internal class Z0816伊尔美格 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeSettings(DtNavMeshCreateParams config) =>
        config.AddOffMeshConnection
        (
            new(503.8994f, 147.78073f, 51.210533f),
            new(520f, 147.2f, 36f),
            2f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.Ground | NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
}
