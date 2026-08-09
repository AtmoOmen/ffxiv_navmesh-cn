using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Ground;
using AABB = vnavmesh.Common.Models.AABB;

namespace vnavmesh.Build.Custom.Implementations.Territory.Town;

[CustomizationTerritory(131)]
internal class Z0131乌尔达哈来生回廊 : NavmeshCustomization
{
    public override int Version => 3;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-61.9686f, 38f, -12.888067f),
                Max = new(-57.172184f, 40.754723f, -10.959627f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(2.8162823f, 12.217501f, 66.46936f),
                Max = new(10.950735f, 16.4005f, 69.52377f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    )
    {
        config.AddOffMeshConnection
        (
            new(-34.117447f, 34.02561f, -9.0725f),
            new(-37.102074f, 34f, -9.786408f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-40.523804f, 34f, -10.927657f),
            new(-43.825356f, 34.643707f, -11.807218f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-48.102303f, 36.102135f, -12.840749f),
            new(-50.240864f, 36.84081f, -13.4674425f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-86.17728f, 41.999958f, 43.71089f),
            new(-87.14816f, 42f, 47.36496f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-105.324554f, 42f, 115.15375f),
            new(-106.32853f, 41.633476f, 118.31534f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-6.4089565f, 30.264957f, 25.631123f),
            new(-7.106923f, 31.514992f, 28.075687f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-9.621347f, 34.014996f, 36.46084f),
            new(-10.628796f, 34.51499f, 39.98262f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-41.663116f, 30.00001f, 12.848243f),
            new(-44.742863f, 30.000008f, 12.526f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-57.489727f, 18f, 61.649475f),
            new(-59.040245f, 18.504017f, 66.094406f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-46.515633f, 18.056557f, 57.43724f),
            new(-43.76459f, 17.433834f, 58.24122f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-5.2727695f, 14.000001f, 37.151855f),
            new(-5.8655596f, 14.000001f, 40.33429f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(2.9432244f, 14f, 70.57076f),
            new(6.0786934f, 13.659374f, 71.46943f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-24.5217f, 14f, 92.30382f),
            new(-26.01208f, 13.5f, 97.49037f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-71.15359f, 37.992012f, -11.672004f),
            new(-70.29941f, 37.920914f, -14.4295025f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-58.32346f, 38f, -15.61594f),
            new(-54.661385f, 38f, -14.612384f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-57.984703f, 38f, -14.5527725f),
            new(-56.08639f, 38f, -14.078575f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-92.09022f, 41f, 62.65243f),
            new(-90.636795f, 42f, 56.94642f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-92.20647f, 41f, 63.42373f),
            new(-93.42785f, 41f, 68.30922f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(60.14369f, 29.686205f, -33.068527f),
            new(62.200813f, 28.991999f, -33.16764f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }
}
