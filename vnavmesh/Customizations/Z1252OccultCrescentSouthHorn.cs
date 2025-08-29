using DotRecast.Detour;
using System.Numerics;
using System.Runtime.InteropServices;
using Navmesh.Utilities;

namespace Navmesh.Customizations;

[CustomizationTerritory(1252)]
internal class Z1252OccultCrescentSouthHorn : NavmeshCustomization
{
    public override int Version => 4;

    public Z1252OccultCrescentSouthHorn()
    {
        Settings.AgentMaxSlopeDeg = 85f;
        Settings.AgentMaxClimb    = 0.7f;
    }
    
    public override void CustomizeScene(SceneExtractor scene)
    {
        if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b1/collision/o6b1_a5_stc02.pcb", out var mesh))
        {
            // bottom stair of second-tier staircase around SW tower is too steep even though it's <55 degrees, probably because of rasterization bs, extend it outward by 1y to make slope more gradual
            var verts = CollectionsMarshal.AsSpan(mesh.Parts[221].Vertices);
            verts[8].X += 1;
            verts[16].X += 1;
        }
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        // eldergrowth to south fates
        config.AddOffMeshConnection(new(295.64f, 101.79f, 322.61f), new(293.91f, 82.02f, 355.45f));

        // eldergrowth to south fates (2)
        config.AddOffMeshConnection(new(307.39f, 102.88f, 311.06f), new(339.73f, 69.75f, 321.51f));

        // eldergrowth to south fates (3)
        config.AddOffMeshConnection(new(309.04f, 102.88f, 314.50f), new(321.17f, 76.74f, 335.64f));
        
        // 古树湿原去潮涌 1
        config.AddOffMeshConnection(new(244.8f, 95.1f, 386.7f), new(242.0f, 77.6f, 405.5f));
        
        // 古树湿原去潮涌 2
        config.AddOffMeshConnection(new(200.3f, 69.9f, 531.4f), new(196.2f, 58.5f, 542.4f));

        // eldergrowth to middle east
        config.AddOffMeshConnection(new(331.43f, 96.00f, 111.11f), new(342.42f, 88.90f, 91.92f));

        // purple chest route
        config.AddOffMeshConnection(new(-337.27f, 47.34f, -419.95f), new(-333.29f, 7.06f, -451.97f));

        // The Wanderer's Haven aetheryte platform to beach
        config.AddOffMeshConnection(new(-175.51f, 6.5f, -607.24f), new(-183.04f, 3.85f, -607.21f));
        
        // 巨大鸟台子
        config.AddOffMeshConnection(new(-505.7f, 2.0f, -606.8f), new(-511.0f, 3.8f, -603.7f), bidirectional: true);
        
        // 巨大鸟路径
        config.AddOffMeshConnection(new(-436.9f, -0.3f, -658.7f), new(-450.9f, 3.3f, -657.0f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-299.3f, 5.0f, -613.8f), new(-317.8f, 5.0f, -614.9f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-437.3f, -0.3f, -661.6f), new(-449.0f, 3.5f, -658.3f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-413.2f, 3.8f, -656.6f), new(-425.1f, -0.3f, -656.5f));
        
        config.AddOffMeshConnection(new(-488.4f, 3.5f, -611.0f), new(-496.2f, -0.3f, -609.0f));
        
        // 南部丘陵到幻境滩跳崖点
        config.AddOffMeshConnection(new(-116.3f, 101.7f, -350.8f), new(-118.7f, 25.0f, -376.1f));
        
        config.AddOffMeshConnection(new(5.23f, 106.65f, -390.92f), new(16.14f, 25.44f, -437.46f));
        
        // 水晶洞窟
        config.AddOffMeshConnection(new(-326.0f, 102.7f, 32.7f), new(-309.0f, 105.4f, 36.5f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-309.0f, 105.4f, 36.5f), new(-296.8f, 107.9f, 39.2f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-285.5f, 108.0f, 39.0f), new(-271.1f, 108.4f, 38.3f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-271.1f, 108.4f, 38.3f), new(-256.5f, 107.0f, 36.8f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-303.1f, 106.2f, 36.9f), new(-289.6f, 107.9f, 38.1f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-317.7f, 104.2f, 33.2f), new(-303.1f, 106.2f, 36.9f), bidirectional: true);
        
        config.AddOffMeshConnection(new(-295.4f, 107.8f, 38.7f), new(-279.6f, 110.6f, 35.1f), bidirectional: true);
    }
}
