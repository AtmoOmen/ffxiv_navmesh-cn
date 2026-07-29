using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(1292)]
internal class Z1292永久幽界中央终端 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("<box>", out var mesh0))
        {
            // 老一后门口
            var index = ResolveInstanceIndex(mesh0, 0xB2CBAC1D000000ul, 12);
            if (index >= 0)
                mesh0.Instances.RemoveAt(index);
        }

        if (scene.Meshes.TryGetValue("bg/ex5/01_xkt_x6/dun/x6d8/collision/x6d8_a3_rub01.pcb", out var mesh1))
        {
            // 老二后门槛
            var index = ResolveInstanceIndex(mesh1, 0xB3B5031C000000ul, 0);
            if (index >= 0)
                mesh1.Instances.RemoveAt(index);
        }
    }

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    )
    {
        // 老二后连接
        config.AddOffMeshConnection
        (
            new(-20.41367f, -484.79788f, -170.51857f),
            new(-20.281109f, -484.9981f, -166.43034f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(270.5428f, -582.79987f, -33.563747f),
            new(270.5667f, -582.9989f, -37.213562f),
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
        // 老一后传送·上
        LinkClientPath(mesh, new(265.03302f, -581.89984f, -45.60021f), new(255.37949f, -498.9004f, -49.545013f));
        // 老一后传送·下
        LinkClientPath(mesh, new(251.81793f, -498.9004f, -45.98494f),  new(261.6262f, -581.89984f, -41.988052f));
        // 老二后传送·上
        LinkClientPath(mesh, new(-71.39022f, -483.9f, -149.2653f),     new(-156.8487f, -398.9f, 113.30165f));
        // 老二后传送·下
        LinkClientPath(mesh, new(-152.33615f, -398.9f, 115.94488f),    new(-68.12604f, -483.9f, -145.6464f));
    }

    private static int ResolveInstanceIndex
    (
        SceneExtractor.Mesh mesh,
        ulong               instanceId,
        int                 instanceIndex
    )
    {
        if (instanceIndex >= 0 && instanceIndex < mesh.Instances.Count && (instanceId == 0 || mesh.Instances[instanceIndex].Id == instanceId))
            return instanceIndex;

        if (instanceId != 0)
        {
            for (var i = 0; i < mesh.Instances.Count; ++i)
                if (mesh.Instances[i].Id == instanceId)
                    return i;
        }

        return -1;
    }
}
