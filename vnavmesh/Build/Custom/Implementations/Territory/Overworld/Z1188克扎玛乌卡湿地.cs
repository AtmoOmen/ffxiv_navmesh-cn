using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(1188)]
internal class Z1188克扎玛乌卡湿地 : NavmeshCustomization
{
    public override int Version => 2;

    public Z1188克扎玛乌卡湿地() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        // contender #3 in the most cursed mesh transformation finalists
        if (scene.Meshes.TryGetValue("bg/ex5/02_ykt_y6/fld/y6f2/collision/y6f2_x0_tst00.pcb", out var mesh))
            mesh.Instances[0].WorldTransform.Row3.Y += 0.05f;
    }
    
    public override void CustomizeSettings(DtNavMeshCreateParams config) =>
        // 奸臣的展示台
        config.AddOffMeshConnection
        (
            new(379.16513f, 132.05937f, 686.66486f),
            new(378.89407f, 132.15149f, 693.27045f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
}
