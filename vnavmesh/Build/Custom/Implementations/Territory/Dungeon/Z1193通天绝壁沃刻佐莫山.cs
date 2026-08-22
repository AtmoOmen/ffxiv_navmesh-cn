using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Build.Ground.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(1193)]
internal class Z1193通天绝壁沃刻佐莫山 : NavmeshCustomization
{
    public override int Version => 2;

    public override void CustomizeScene
    (
        SceneExtractor scene
    ) =>
        // remove large crystal blocking initial path, which is destroyed after first pack dies
        scene.Meshes.Remove("bg/ex5/02_ykt_y6/dun/y6d2/collision/y6d2_a1_cry03.pcb");

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    )
    {
        config.AddOffMeshConnection
        (
            new(-84.984985f, 83.43838f, 16.815525f),
            new(-87.23949f, 81.304985f, 17.476467f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-92.886f, 80.07646f, 21.805302f),
            new(-93.76564f, 78.168976f, 23.270887f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-95.49261f, 77.89406f, 26.56182f),
            new(-96.23459f, 75.66163f, 27.87869f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-96.299484f, 74.923706f, 29.690712f),
            new(-96.711395f, 71.75397f, 31.935032f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-95.62928f, 71.56621f, 34.3437f),
            new(-97.0606f, 70.000046f, 35.750156f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-101.45528f, 71.65432f, 33.624153f),
            new(-102.373276f, 70.000015f, 35.08779f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-93.90274f, 71.61809f, 35.02472f),
            new(-93.40851f, 70.00004f, 37.18663f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-98.566376f, 75.169525f, 29.490572f),
            new(-99.71539f, 73.12116f, 30.449541f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-94.2036f, 74.68306f, 30.736431f),
            new(-94.37086f, 71.95897f, 32.22297f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-96.92457f, 78.09169f, 26.331053f),
            new(-97.84229f, 76.02344f, 27.226051f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-94.37889f, 77.62675f, 27.035778f),
            new(-94.630646f, 75.495056f, 28.086235f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-94.47819f, 80.21791f, 21.210327f),
            new(-95.51148f, 78.58182f, 22.707567f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-84.36124f, 83.42916f, 17.949215f),
            new(-85.31665f, 81.34958f, 18.760738f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-86.4144f, 83.42893f, 16.008148f),
            new(-87.37458f, 81.41614f, 16.410416f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-96.01609f, 77.96924f, 26.361475f),
            new(-96.6175f, 75.70476f, 27.728762f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-97.099884f, 74.99227f, 29.68504f),
            new(-97.36233f, 71.70395f, 31.629616f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-97.75854f, 71.45653f, 32.651985f),
            new(-98.02482f, 70.15888f, 34.33348f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-98.18328f, 70.00004f, 34.944347f),
            new(-98.54333f, 70.00004f, 36.357277f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-82.510544f, 84.99999f, 13.851453f),
            new(-82.92588f, 83.41178f, 16.216335f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-82.571884f, 84.99999f, 13.896072f),
            new(-84.13736f, 83.45045f, 15.613813f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-79.51264f, 84.99999f, 14.76064f),
            new(-81.54171f, 83.728806f, 16.065157f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-57.41605f, 104.38435f, -20.145775f),
            new(-52.023705f, 99.79068f, -22.82613f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-68.47575f, 100.180214f, -14.06108f),
            new(-66.28407f, 100.50047f, -14.799328f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-66.35719f, 100.38226f, -14.468241f),
            new(-57.309746f, 104.38887f, -18.6654f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        LinkClientPath(mesh, new(-60.596786f, 41.184914f, 52.18263f),  new(-49.127636f, 53.765175f, 38.118057f));
        LinkClientPath(mesh, new(-49.69194f, 55.609795f, 16.1616f),    new(-66.190384f, 85.04915f, 13.934385f));
        LinkClientPath(mesh, new(-106.27985f, 71.03789f, 43.525322f),  new(-145.49803f, 80.13396f, 41.328983f));
        LinkClientPath(mesh, new(-52.031303f, 100.02917f, -22.83974f), new(-62.87685f, 319.26578f, -20.215199f));
        LinkPoints(mesh, new(-67.06293f, 100.41734f, -15.714321f), new(-57.767265f, 104.39245f, -19.918083f));
    }
}
