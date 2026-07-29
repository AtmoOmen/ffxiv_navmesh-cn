using System.Numerics;
using System.Runtime.InteropServices;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.CosmicExploration;

[CustomizationTerritory(1319)]
internal class Z1319奥克塞西亚行星 : NavmeshCustomization
{
    public override int Version => 3;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        string[] doubleLiners =
        [
            "bg/ffxiv/cos_c1/hou/c1w4/collision/c1w4_03_t200a.pcb", "bg/ffxiv/cos_c1/hou/c1w4/collision/c1w4_03_t300a.pcb"
        ];

        // 避免被飞线旁边卡脚
        foreach (var liner in doubleLiners)
        {
            if (scene.Meshes.TryGetValue(liner, out var mesh))
            {
                var departVerts = CollectionsMarshal.AsSpan(mesh.Parts[49].Vertices);
                departVerts[81].Y += 1;
                departVerts[85].Y += 1;

                var box = SceneExtractor.BuildBoxMesh()[0];

                foreach (ref var vert in CollectionsMarshal.AsSpan(box.Vertices))
                {
                    vert *= new Vector3(1.5f, 3.75f, 1.5f);
                    vert += new Vector3(4.5f, 6.25f, -1);
                }

                mesh.Parts.Add(box);
            }
        }

        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/c1w4/collision/c1w4_t0_rck03f.pcb", out var mesh0))
        {
            if (ResolveInstance(mesh0, 0xBF844300000000ul, 297) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(1.0589386f, -0.03719455f, -0.56326336f),
                    Row1 = new(0.06595986f, 2.8984663f, -0.06739247f),
                    Row2 = new(0.56382984f, 0.011797147f, 1.0592246f),
                    Row3 = new(528.0706f, 245.4566f, 468.9046f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh0.LocalBounds);
            }
        }

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(13.586f, 181.09999f, -0.30716705f),
                Max = new(18.786001f, 189.7f, 4.3928328f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertCylinderCollider
        (
            new AABB
            {
                Min = new(81.235f, 157.9646f, -117.259f),
                Max = new(104.634995f, 173.16461f, -93.959f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(543.34247f, 240.5433f, 464.7541f),
                Max = new(568.1495f, 248.57132f, 475.38806f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        scene.InsertCylinderCollider
        (
            new Vector3(1, 10, 1),
            new(-572.3f, 200, -480),
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertCylinderCollider
        (
            new Vector3(3, 10, 3),
            new(-556.5f, 200, -498.4f),
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertCylinderCollider
        (
            new Vector3(3, 10, 3),
            new(-559, 200, -448.5f),
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        LinkClientPath(mesh, new(331.49738f, 208.5f, 341.88217f),      new(600.33795f, 190.25f, 150.99554f));
        LinkClientPath(mesh, new(594.30853f, 190.25f, 144.28609f),     new(325.5636f, 208.5f, 335.11337f));
        LinkClientPath(mesh, new(256.7706f, 208.5f, 335.4078f),        new(-4.5577583f, 186.75f, 21.681448f));
        LinkClientPath(mesh, new(-13.599279f, 186.75f, 22.167122f),    new(250.11954f, 208.5f, 341.47668f));
        LinkClientPath(mesh, new(250.43433f, 208.5f, 410.19766f),      new(15.60578f, 170.25f, 694.09033f));
        LinkClientPath(mesh, new(21.61484f, 170.25f, 700.74384f),      new(256.4777f, 208.5f, 416.856f));
        LinkClientPath(mesh, new(325.20856f, 208.5f, 416.59827f),      new(496.68005f, 187.7232f, 765.73334f));
        LinkClientPath(mesh, new(502.7643f, 187.75f, 759.0984f),       new(331.8394f, 208.5f, 410.4817f));
        LinkClientPath(mesh, new(533.5897f, 187.75f, 779.3441f),       new(680.2081f, 247.75f, 505.1005f));
        LinkClientPath(mesh, new(672.1685f, 247.75f, 509.06213f),      new(534.09735f, 187.75f, 770.3581f));
        LinkClientPath(mesh, new(674.895f, 247.8377f, 472.28967f),     new(631.983f, 190.25f, 144.58705f));
        LinkClientPath(mesh, new(625.2561f, 190.25f, 150.63713f),      new(666.1869f, 247.82056f, 470.07736f));
        LinkClientPath(mesh, new(600.8021f, 190.25f, 113.45303f),      new(10.795033f, 186.75f, -6.4183707f));
        LinkClientPath(mesh, new(11.06642f, 186.75f, 2.6156702f),      new(594.0867f, 190.25f, 119.438126f));
        LinkClientPath(mesh, new(130.87128f, 168.52222f, -131.16873f), new(194.74255f, 485.5f, -370.9881f));
        LinkClientPath(mesh, new(195.18652f, 485.39172f, -389.01224f), new(118.42471f, 168.5f, -144.11234f));
        LinkClientPath(mesh, new(-8.421187f, 186.75f, -22.231117f),    new(-205.7743f, 147.75f, -548.9885f));
        LinkClientPath(mesh, new(-209.7019f, 147.75f, -540.969f),      new(-17.42104f, 186.75f, -21.895042f));
        LinkClientPath(mesh, new(-33.137817f, 186.75f, -2.597745f),    new(-368.64044f, 170.25f, -173.81131f));
        LinkClientPath(mesh, new(-365.93262f, 170.25f, -165.23447f),   new(-32.839935f, 186.75f, 6.4062943f));
        LinkClientPath(mesh, new(-245.33563f, 147.75f, -550.645f),     new(-587.45624f, 208.75f, -380.87448f));
        LinkClientPath(mesh, new(-581.42694f, 208.75f, -374.21948f),   new(-242.9607f, 147.75f, -541.93274f));
        LinkClientPath(mesh, new(-587.76105f, 208.75f, -343.40887f),   new(-399.82083f, 170.25f, -181.38249f));
        LinkClientPath(mesh, new(-391.24637f, 170.25f, -184.06683f),   new(-581.0463f, 208.75f, -349.35733f));
        LinkClientPath(mesh, new(-618.6045f, 208.75f, -349.7416f),     new(-771.0979f, 187.75f, -14.827921f));
        LinkClientPath(mesh, new(-762.197f, 187.87321f, -15.964832f),  new(-612.56836f, 208.75f, -343.0673f));
        LinkClientPath(mesh, new(-751.567f, 187.75f, 19.395021f),      new(-689.18414f, 187.75f, 327.70157f));
        LinkClientPath(mesh, new(-682.75793f, 187.75f, 321.39465f),    new(-745.4804f, 187.75f, 12.756671f));
        LinkClientPath(mesh, new(-652.95435f, 187.75f, 343.23218f),    new(-263.95026f, 170.25f, 297.09314f));
        LinkClientPath(mesh, new(-257.85654f, 170.16397f, 290.54712f), new(-652.08527f, 187.7734f, 334.2183f));
        LinkClientPath(mesh, new(-261.49246f, 170.16547f, 321.84518f), new(-672.46265f, 117.75f, 612.5007f));
        LinkClientPath(mesh, new(-666.0572f, 117.75f, 618.8206f),      new(-254.92146f, 170.25f, 327.96594f));
        LinkClientPath(mesh, new(-226.5091f, 170.25f, 294.1832f),      new(-407.45706f, 170.25f, -150.12848f));
        LinkClientPath(mesh, new(-410.0551f, 170.25f, -158.77704f),    new(-233.14682f, 170.25f, 288.07742f));
        LinkClientPath(mesh, new(-230.23788f, 170.25f, 325.4608f),     new(-9.567081f, 170.25f, 731.89075f));
        LinkClientPath(mesh, new(-15.609868f, 170.25f, 725.2171f),     new(-224.10603f, 170.25f, 318.8574f));
        LinkClientPath(mesh, new(-9.228521f, 170.25f, 694.3741f),      new(-15.323056f, 187.75f, 238.82265f));
        LinkClientPath(mesh, new(-22.550953f, 187.75f, 244.21445f),    new(-15.88903f, 170.25f, 700.43884f));
        LinkClientPath(mesh, new(-49.36186f, 187.75f, 218.94516f),     new(-376.13156f, 170.25f, -142.6389f));
        LinkClientPath(mesh, new(-384.81085f, 170.25f, -140.02068f),   new(-51.380566f, 187.81459f, 227.71413f));
    }

    private static SceneExtractor.MeshInstance? ResolveInstance
    (
        SceneExtractor.Mesh mesh,
        ulong               instanceId,
        int                 instanceIndex
    )
    {
        var index = ResolveInstanceIndex(mesh, instanceId, instanceIndex);
        return index >= 0 ?
                   mesh.Instances[index] :
                   null;
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

    private static AABB TransformBounds
    (
        Matrix4x3 worldTransform,
        AABB      localBounds
    )
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = (axisX      * localCenter.X) + (axisY      * localCenter.Y) + (axisZ      * localCenter.Z) + worldTransform.Row3;
        var extent      = (Abs(axisX) * localExtent.X) + (Abs(axisY) * localExtent.Y) + (Abs(axisZ) * localExtent.Z);
        return new() { Min = center                    - extent, Max = center         + extent };
    }

    private static Vector3 Abs
    (
        Vector3 value
    ) => new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));
}
