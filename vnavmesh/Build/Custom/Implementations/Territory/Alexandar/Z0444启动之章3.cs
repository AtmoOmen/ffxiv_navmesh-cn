using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Ground;
using AABB = vnavmesh.Common.Models.AABB;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.Build.Custom.Implementations.Territory.Alexandar;

[CustomizationTerritory(444)]
internal class Z0444启动之章3 : NavmeshCustomization
{
    public override int Version => 4;

    public override void CustomizeBuildSettings
    (
        SceneDefinition definition,
        NavmeshSettings settings
    )
    {
        settings.CellSize    = 0.25f;
        settings.CellHeight  = 0.125f;
        settings.AgentHeight = 1.5f;
        settings.AgentRadius = 0.5f;
    }

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-17.823376f, -0.12699986f, 5.392f),
                Max = new(19.035625f, 3.019f, 24.518f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(5.84f, -1.561f, 0.6084976f),
                Max = new(26.05f, 3.361f, 38.433502f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-3.0751038f, 19.204956f, -33.202503f),
                Max = new(12.815897f, 23.276955f, -19.051498f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    ) =>
        // 下落2
        config.AddOffMeshConnection
        (
            new(16.790548f, 18f, -18.193995f),
            new(20.274364f, 8.826821f, -23.925808f),
            2f,
            false,
            0,
            NavmeshArea.GeneratedEdgeJump,
            NavmeshPolyFlags.GeneratedEdgeJump | NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        // 下落1
        LinkPoints(mesh, new(20.97561f, 35.141724f, -15.054151f),     new(24.470856f, 23.111866f, -19.177319f));
        // 门前2
        LinkPoints(mesh, new(38.509922f, -9.536743E-07f, -0.274802f), new(48.962902f, 0f, -0.06897486f), true);
        // 传送带
        LinkPoints(mesh, new(58.524216f, 0.011983752f, -21.014153f),  new(58.992924f, -9f, -42.920166f));
        // 门前1
        LinkPoints(mesh, new(35.091225f, 9.536743E-07f, -6.1722064f), new(38.528137f, 9.536743E-07f, 0.020476937f), true);
    }
}
