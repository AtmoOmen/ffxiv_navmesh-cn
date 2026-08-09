using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build.Enums;
using AABB = vnavmesh.Common.Models.AABB;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(1146)]
internal class Z1146鏖战红莲阿拉米格 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("bg/ex2/01_gyr_g3/dun/g3d2/collision/tr1117.pcb", out var mesh0))
        {
            if (29 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[29];
                if (9 < part.Primitives.Count)
                    part.Primitives[9] = new(0, 0, 0, PrimitiveFlags.None, 0x4000ul);
            }

            if (29 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[29];
                if (7 < part.Primitives.Count)
                    part.Primitives[7] = new(0, 0, 0, PrimitiveFlags.None, 0x4000ul);
            }
        }

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-260.90683f, 26.5f, 94.2588f),
                Max = new(-255.9119f, 29.5f, 96.54003f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
    }
}
