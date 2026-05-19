using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.PVP;

[CustomizationTerritory(554)]
internal class Z0554荣誉野 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-31.531458f, 21.469496f, 227.44655f),
                Max = new(-19.364525f, 29.898293f, 240.27592f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-210.39667f, 29.937592f, -144.9254f),
                Max = new(-210.06776f, 32.190163f, -135.74413f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-210.32625f, 21.991016f, -144.83792f),
                Max = new(-201.74841f, 31.188917f, -135.01823f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(-1.9312462f, 26.249973f, 246.41324f),
            new(3.5733013f, 18.68111f, 238.49294f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(13.294491f, 26.00002f, 253.48108f),
            new(18.672352f, 19.582153f, 246.94081f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-12.704419f, 26.420273f, 251.35121f),
            new(-12.163348f, 20.523623f, 237.13603f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-213.807f, 30.088242f, -125.52468f),
            new(-208.05402f, 21.900002f, -122.36816f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-226.47066f, 30.052082f, -114.688614f),
            new(-219.90501f, 26.073936f, -109.78713f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-211.71916f, 29.977715f, -134.6167f),
            new(-205.17873f, 22.196548f, -131.86891f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(205.90952f, 26.301464f, -134.611f),
            new(195.54102f, 20.138836f, -129.03394f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(208.88576f, 26.106894f, -121.26159f),
            new(201.97766f, 17.9f, -114.98691f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(220.647f, 25.861292f, -112.09984f),
            new(213.74496f, 21.507626f, -105.33897f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }
}
