using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories.Alexandar;

[CustomizationTerritory(444)]
internal class Z0444启动之章3 : NavmeshCustomization
{
    public override int Version => 2;

    public Z0444启动之章3()
    {
        Settings.CellSize    = 0.25f;
        Settings.CellHeight  = 0.125f;
        Settings.AgentRadius = 0.5f;
        Settings.AgentHeight = 2f;
        Settings.FastBuild   = false;
    }

    public override void CustomizeScene(SceneExtractor scene)
    {
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-9.5f, 0.3f, 24.7f),
                Max = new(7.7f, 5f, 20.1f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(9.9f, 18.0f, -16.1f),
                Max = new(-1.2f, 20.0f, -37.1f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(25.3f, 0.1f, 8.9f),
                Max = new(7.3f, -1.0f, 5.4f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(19.8f, -0.6f, -8.6f),
                Max = new(12.2f, 4f, -24.8f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(32.9f, 0.0f, -12.7f),
                Max = new(41.4f, 0.0f, -0.6f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(34.1f, 0.0f, -0.5f),
                Max = new(36.7f, 13.0f, -16.6f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 传送带
        LinkPoints(mesh, new(58.3f, -0.0f, -20.4f), new(58.0f, -9.0f, -41.8f));

        // 进门前
        LinkPoints(mesh, new(35.1f, 0.0f, -0.0f),  new(48.5f, 0.0f, 0.2f));
        LinkPoints(mesh, new(29.9f, 0.0f, -11.1f), new(35.2f, 0.0f, 0.1f));
        LinkPoints(mesh, new(35.3f, 0.0f, -6.0f),  new(40.2f, 0.0f, 0.5f));
        LinkPoints(mesh, new(41.5f, 0.0f, 0.0f),   new(47.3f, 0.0f, 0.2f));

        // 两个下落点
        LinkDrop(mesh, new(21.6f, 36.0f, -13.3f), new(25.0f, 23.6f, -20.7f));
        LinkDrop(mesh, new(17.9f, 18.0f, -17.5f), new(20.0f, 8.8f, -23.6f));
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config) =>
        config.AddOffMeshConnection(new(35.1f, 0.0f, -0.0f), new(57.5f, 0.0f, 0.1f), 1f, true);
}
