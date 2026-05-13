using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Alexandar;

[CustomizationTerritory(444)]
internal class Z0444启动之章3 : NavmeshCustomization
{
    public override int Version => 2;

    public Z0444启动之章3() =>
        ApplyExtremeHighPrecisionSettings();

    public override void CustomizeScene(SceneExtractor scene)
    {
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-17.823376f, -0.12699986f, 5.392f),
                Max = new(19.035625f, 3.019f, 24.518f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(5.84f, -1.561f, 0.6084976f),
                Max = new(26.05f, 3.361f, 38.433502f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-3.0751038f, 19.204956f, -33.202503f),
                Max = new(12.815897f, 23.276955f, -19.051498f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
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
        config.AddOffMeshConnection
        (
            new(35.09059f, -9.536743E-07f, -6.1794167f),
            new(38.55001f, -9.536743E-07f, 0.0067253113f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        LinkDrop(mesh, new(20.97561f, 35.141724f, -15.054151f), new(24.470856f, 23.111866f, -19.177319f));
        LinkPoints(mesh, new(38.509922f, -9.536743E-07f, -0.274802f), new(48.962902f, 0f, -0.06897486f));
        LinkPoints(mesh, new(58.524216f, 0.011983752f, -21.014153f),  new(58.992924f, -9f, -42.920166f));
    }
}
