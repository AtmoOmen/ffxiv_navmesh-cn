using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(933)]
internal class Z0933魔术工房玛托雅工作室 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    )
    {
        config.AddOffMeshConnection
        (
            new(50.527153f, 234.05f, -158.33136f),
            new(50.556087f, 219.99991f, -151.703f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(51.37818f, 219.9999f, -144.02701f),
            new(50.02321f, 202f, -136.73102f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-49.426426f, 202.05f, -130.93295f),
            new(-46.1742f, 188f, -125.87389f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-41.663002f, 188f, -119.04332f),
            new(-37.9675f, 170.09996f, -112.67725f),
            0.5f,
            false,
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
        LinkClientPath(mesh, new(-211.99867f, -211.95f, 29.758022f), new(-210.02719f, -216.00003f, -0.32089043f));
        LinkClientPath(mesh, new(-180.01707f, -219.95f, -78.88551f), new(-178.044f, -220f, -121.21176f));
        LinkClientPath(mesh, new(-19.240154f, 198.3f, -183.30945f),  new(-21.766142f, 202f, -189.57169f));
        LinkClientPath(mesh, new(0.036924675f, 170.1f, -108.12259f), new(1.9833529f, 150.2f, -134.99567f));
    }
}
