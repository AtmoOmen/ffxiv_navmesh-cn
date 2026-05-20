using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Custom.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Overworld;

[CustomizationTerritory(958)]
internal class Z0958加雷马 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings) =>
        settings.AgentRadius = 1f;

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(89.50866f, 14.650169f, -342.7801f),
            new(88.948364f, 10.800001f, -338.59482f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(85.01472f, 14.713364f, -344.4145f),
            new(85.06559f, 10.5f, -338.85672f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(75.4983f, 15.141907f, -350.81427f),
            new(74.07337f, 14.004005f, -348.2441f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(69.23156f, 14.011064f, -339.73407f),
            new(69.81415f, 10.799999f, -336.58148f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(73.12434f, 13.394736f, -338.09296f),
            new(73.76251f, 10.6405945f, -336.38434f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(78.68066f, 11.839239f, -339.81516f),
            new(78.3684f, 10.500001f, -337.14087f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(76.769554f, 11.339914f, -338.29926f),
            new(76.55521f, 10.5f, -336.611f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(82.057556f, 12.0234f, -341.24762f),
            new(83.17428f, 10.5f, -339.57858f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(80.49151f, 11.447085f, -339.22403f),
            new(80.65747f, 10.5f, -337.3636f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(92.0544f, 14.591408f, -342.25626f),
            new(91.856544f, 10.799999f, -339.33905f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }
}
