using System.Numerics;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Models;
using AABB = vnavmesh.Common.Models.AABB;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.Build.Custom.Implementations.Territory.CosmicExploration;

[CustomizationTerritory(1310)]
internal class Z1310俄匊斯行星 : NavmeshCustomization
{
    public override int Version => 4;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/c1w3/collision/c1w3_03_t200a.pcb", out var mesh0))
        {
            if (29 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[29];

                if (129 < part.Vertices.Count)
                {
                    part.Vertices[129] = new(7.4296174f, 1.6966519f, 5.3007555f);
                    part.LocalBounds   = CalculateLocalBounds(part.Vertices);
                    RecalculateMeshBounds(mesh0);
                }
            }

            if (29 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[29];

                if (130 < part.Vertices.Count)
                {
                    part.Vertices[130] = new(7.4453645f, 1.7093903f, 5.7623405f);
                    part.LocalBounds   = CalculateLocalBounds(part.Vertices);
                    RecalculateMeshBounds(mesh0);
                }
            }

            if (29 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[29];

                if (132 < part.Vertices.Count)
                {
                    part.Vertices[132] = new(7.4882116f, 1.4999946f, 7.285237f);
                    part.LocalBounds   = CalculateLocalBounds(part.Vertices);
                    RecalculateMeshBounds(mesh0);
                }
            }

            if (29 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[29];

                if (133 < part.Vertices.Count)
                {
                    part.Vertices[133] = new(7.4882116f, 1.4999946f, 4.714626f);
                    part.LocalBounds   = CalculateLocalBounds(part.Vertices);
                    RecalculateMeshBounds(mesh0);
                }
            }
        }

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-131.259f, -195.082f, -840.71716f),
                Max = new(-128.37198f, -188.6f, -831.78186f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-152.32198f, -193.773f, -933.42145f),
                Max = new(-142.62202f, -190.537f, -923.9085f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        LinkPoints(mesh, new(148.5f, -91.78484f, -540f), new(150.25f, -92.63461f, -536f));
        LinkClientPath(mesh, new(-175.5f, 3.25f, 52.8f),              new(-145.5f, -20.25f, -214.8f));
        LinkClientPath(mesh, new(-154.53415f, -20.25f, -214.1073f),   new(-184.5f, 3.25f, 53.8f));
        LinkClientPath(mesh, new(-298.2f, 3.25f, 133.5f),             new(-519.8f, 25.75f, 115.5f));
        LinkClientPath(mesh, new(-519.12f, 25.75f, 124.54f),          new(-297.2f, 3.25f, 142.5f));
        LinkClientPath(mesh, new(-61.8f, 3.25f, 142.5f),              new(157.8f, 2.25f, -3.5f));
        LinkClientPath(mesh, new(157.25f, 2.25f, -12.5f),             new(-62.8f, 3.25f, 133.5f));
        LinkClientPath(mesh, new(184.5f, 2.25f, -30.75f),             new(196.5f, -51.75f, -377.8f));
        LinkClientPath(mesh, new(187.46585f, -51.75f, -377.1073f),    new(175.5f, 2.25f, -30.2f));
        LinkClientPath(mesh, new(202.88f, 2.25f, -3.4600003f),        new(500.5f, -49.75f, -290.8f));
        LinkClientPath(mesh, new(491.46585f, -49.75f, -290.1073f),    new(202.2f, 2.25f, -12.5f));
        LinkClientPath(mesh, new(214.88f, -51.75f, -395.46002f),      new(473.8f, -49.75f, -308.5f));
        LinkClientPath(mesh, new(473.25f, -49.75f, -317.5f),          new(214.2f, -51.75f, -404.5f));
        LinkClientPath(mesh, new(196.5f, -51.75f, -422.75f),          new(314.5f, -151.25f, -603.8f));
        LinkClientPath(mesh, new(305.46585f, -151.25f, -603.1073f),   new(187.5f, -51.75f, -422.2f));
        LinkClientPath(mesh, new(332.88f, -151.25f, -621.45996f),     new(491.5f, -49.75f, -335.2f));
        LinkClientPath(mesh, new(500.5f, -49.75f, -335.75f),          new(332.2f, -151.25f, -630.5f));
        LinkClientPath(mesh, new(287.25f, -151.25f, -630.5f),         new(92.692f, -190.25f, -780.797f));
        LinkClientPath(mesh, new(88.77321f, -190.25f, -772.66565f),   new(287.8f, -151.25f, -621.5f));
        LinkClientPath(mesh, new(1.75f, -188.75f, -799.4f),           new(-89.8f, -188.75f, -799.4f));
        LinkClientPath(mesh, new(-126.6f, -188.75f, -840.75f),        new(-126.6f, -188.8f, -893.8f));
        LinkClientPath(mesh, new(-182.75f, -188.75f, -949.4f),        new(-290.8f, -188.75f, -949.4f));
        LinkClientPath(mesh, new(-325.43768f, -188.75f, -914.10156f), new(-325.4f, -188.75f, -835.2f));
        LinkClientPath(mesh, new(-306.6f, -188.75f, -835.75f),        new(-306.6f, -188.75f, -914.8f));
        LinkClientPath(mesh, new(-290.10156f, -188.75f, -930.5623f),  new(-182.2f, -188.75f, -930.6f));
        LinkClientPath(mesh, new(-145.4377f, -188.75f, -893.10156f),  new(-145.4f, -188.75f, -840.2f));
        LinkClientPath(mesh, new(-89.10154f, -188.75f, -780.5623f),   new(2.4f, -188.75f, -780.6f));
        LinkClientPath(mesh, new(728.5f, 221f, -123.75f),             new(518.2f, -49.75f, -317.5f));
        LinkClientPath(mesh, new(519f, -49.75f, -308.5f),             new(719.5f, 221f, -123.2f));
        LinkClientPath(mesh, new(674.5f, 136.25f, 267.25f),           new(728.5f, 221f, -78.8f));
        LinkClientPath(mesh, new(719.4658f, 221f, -78.107315f),       new(665.5f, 136.25f, 267.8f));
        LinkClientPath(mesh, new(647.25f, 136.25f, 285.5f),           new(356.976f, 102.75f, 380.004f));
        LinkClientPath(mesh, new(361.99622f, 102.75f, 387.54468f),    new(647.8f, 136.25f, 294.5f));
        LinkClientPath(mesh, new(96.5f, 100.25f, 330.25f),            new(184.5f, 2.25f, 14.2f));
        LinkClientPath(mesh, new(175.46585f, 2.25f, 14.892683f),      new(87.5f, 100.25f, 330.8f));
        LinkClientPath(mesh, new(114.88f, 100.25f, 357.53998f),       new(323.024f, 102.75f, 409.996f));
        LinkClientPath(mesh, new(318.0126f, 102.75f, 402.5252f),      new(114.2f, 100.25f, 348.5f));
        LinkClientPath(mesh, new(-440.5f, 104.25f, 747.25f),          new(-383.5f, 47.25f, 423.2f));
        LinkClientPath(mesh, new(-392.5f, 47.25f, 424f),              new(-449.5f, 104.25f, 747.8f));
        LinkClientPath(mesh, new(-467.75f, 104.25f, 765.5f),          new(-656.5f, 30.63261f, 453.2f));
        LinkClientPath(mesh, new(-665.5342f, 30.75f, 453.8927f),      new(-467.2f, 104.25f, 774.5f));
        LinkClientPath(mesh, new(-656.5f, 30.75f, 408.25f),           new(-564.2f, 25.75f, 124.5f));
        LinkClientPath(mesh, new(-564.75f, 25.75f, 115.5f),           new(-665.5f, 30.75f, 408.8f));
        LinkClientPath(mesh, new(-638f, 30.75f, 435.5f),              new(-410.2f, 47.25f, 405.5f));
        LinkClientPath(mesh, new(-410.75f, 47.25f, 396.5f),           new(-638.8f, 30.75f, 426.5f));
        LinkClientPath(mesh, new(-383.5f, 47.25f, 377.8f),            new(-537.5f, 25.75f, 142.2f));
        LinkClientPath(mesh, new(-546.5341f, 25.75f, 142.89268f),     new(-392.5f, 47.25f, 378.8f));
        LinkClientPath(mesh, new(-365.12f, 47.25f, 405.54f),          new(69.8f, 100.25f, 357.5f));
        LinkClientPath(mesh, new(69.25f, 100.25f, 348.5f),            new(-365.8f, 47.25f, 396.5f));
        LinkClientPath(mesh, new(-537.5f, 25.75f, 97.25f),            new(-525.5f, -24.75f, -217.8f));
        LinkClientPath(mesh, new(-534.5341f, -24.75f, -217.1073f),    new(-546.5f, 25.75f, 97.8f));
        LinkClientPath(mesh, new(-525.5f, -24.75f, -262.75f),         new(-697.5f, -85.25f, -471.8f));
        LinkClientPath(mesh, new(-706.5342f, -85.25f, -471.1073f),    new(-534.5f, -24.75f, -262.2f));
        LinkClientPath(mesh, new(-507f, -24.75f, -235.5f),            new(-172.2f, -20.25f, -232.5f));
        LinkClientPath(mesh, new(-172.75f, -20.25f, -241.5f),         new(-507.8f, -24.858137f, -244.5f));
        LinkClientPath(mesh, new(-145.5f, -20.25f, -259.75f),         new(-127.5f, -72.75f, -559.8f));
        LinkClientPath(mesh, new(-136.53415f, -72.75f, -559.1073f),   new(-154.5f, -20.341175f, -259.2f));
        LinkClientPath(mesh, new(-127f, -20.25f, -232.5f),            new(169.8f, -51.75f, -395.5f));
        LinkClientPath(mesh, new(169.25f, -51.75f, -404.5f),          new(-127.8f, -20.25f, -241.5f));
        LinkClientPath(mesh, new(-154.75f, -72.75f, -586.5f),         new(-679.8f, -85.25f, -498.5f));
        LinkClientPath(mesh, new(-679f, -85.25f, -489.5f),            new(-154.2f, -72.75f, -577.5f));
        LinkClientPath(mesh, new(-127.5f, -72.75f, -605.2f),          new(-457.8f, -102.25f, -764.5f));
        LinkClientPath(mesh, new(-457.12f, -102.25f, -755.45996f),    new(-136.5f, -72.75f, -604.2f));
        LinkClientPath(mesh, new(-697.5f, -85.25f, -516.75f),         new(-502.2f, -102.25f, -755.5f));
        LinkClientPath(mesh, new(-502.75f, -102.25f, -764.5f),        new(-706.5f, -85.35814f, -516.2f));
    }

    private static void RecalculateMeshBounds
    (
        Mesh mesh
    )
    {
        mesh.LocalBounds = CalculateLocalBounds(mesh.Parts);
        foreach (var instance in mesh.Instances)
            instance.WorldBounds = TransformBounds(instance.WorldTransform, mesh.LocalBounds);
    }

    private static AABB CalculateLocalBounds
    (
        List<MeshPart> parts
    )
    {
        var bounds = new AABB { Min = new(float.MaxValue), Max = new(float.MinValue) };

        foreach (var part in parts)
        {
            bounds.Min = Vector3.Min(bounds.Min, part.LocalBounds.Min);
            bounds.Max = Vector3.Max(bounds.Max, part.LocalBounds.Max);
        }

        return bounds;
    }

    private static AABB CalculateLocalBounds
    (
        List<Vector3> vertices
    )
    {
        var bounds = new AABB { Min = new(float.MaxValue), Max = new(float.MinValue) };

        foreach (var vertex in vertices)
        {
            bounds.Min = Vector3.Min(bounds.Min, vertex);
            bounds.Max = Vector3.Max(bounds.Max, vertex);
        }

        return bounds;
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
