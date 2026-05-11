using System.Numerics;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories.Overworld;

[CustomizationTerritory(139)]
public class Z0139拉诺西亚高地 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0139拉诺西亚高地()
    {
        // 小石头遍地，没办法
        Settings.AgentRadius   = 0.5f;
        Settings.AgentHeight   = 1.5f;
        Settings.CellSize      = 0.25f;
        Settings.CellHeight    = 0.125f;
        Settings.AgentMaxClimb = 0.9f;
    }
    
    public override void CustomizeScene(SceneExtractor scene)
    {
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(561.1f, -3.4f, 158.0f),
                Max = new(618.6f, 0, 182.1f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(654.1f, -3.4f, 147.7f),
                Max = new(560.6f, 0, 131.7f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(352.1f, -3.4f, 19.9f),
                Max = new(381.7f, 2.2f, -16.4f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(323.6f, -3.4f, 13.9f),
                Max = new(343.4f, 3.6f, -9.0f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }
}
