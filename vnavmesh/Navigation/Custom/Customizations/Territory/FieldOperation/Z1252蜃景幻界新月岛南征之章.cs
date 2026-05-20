using System.Numerics;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Custom.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.FieldOperation;

[CustomizationTerritory(1252)]
internal class Z1252蜃景幻界新月岛南征之章 : NavmeshCustomization
{
    public override int Version => 2;

    public override void CustomizeScene(SceneExtractor scene)
    {
        if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b1/collision/o6b1_a5_stc02.pcb", out var mesh0))
        {
            if (221 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[221];

                if (8 < part.Vertices.Count)
                {
                    part.Vertices[8] = new(57.90417f, 1.3432799E-11f, 19.401207f);
                    part.LocalBounds = CalculateLocalBounds(part.Vertices);
                    RecalculateMeshBounds(mesh0);
                }
            }

            if (221 < mesh0.Parts.Count)
            {
                var part = mesh0.Parts[221];

                if (16 < part.Vertices.Count)
                {
                    part.Vertices[16] = new(57.903984f, -2.2888169E-05f, 9.997324f);
                    part.LocalBounds  = CalculateLocalBounds(part.Vertices);
                    RecalculateMeshBounds(mesh0);
                }
            }
        }

        if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b1/collision/o6b1_a2_deb04.pcb", out var mesh1))
        {
            if (ResolveInstance(mesh1, 0xAA71A500000000ul, 246) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(-0.9668053f, 8.452079E-08f, -0.2130441f),
                    Row1 = new(0f, 0f, 0f),
                    Row2 = new(0.2130441f, 6.792364E-08f, -0.9668053f),
                    Row3 = new(-392.812f, 2.6f, -589.9389f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh1.LocalBounds);
            }
        }

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-300.31873f, -2.6500006f, -572.4582f),
                Max = new(-271.1427f, 5.6499996f, -549.54425f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(-466.953f, -2.95f, -577.3929f),
                Max = new(-422.05298f, 2.95f, -575.09283f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(295.64f, 101.79f, 322.61f),
            new(293.91f, 82.02f, 355.45f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(307.39f, 102.88f, 311.06f),
            new(339.73f, 69.75f, 321.51f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(309.04f, 102.88f, 314.5f),
            new(321.17f, 76.74f, 335.64f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(331.43f, 96f, 111.11f),
            new(342.42f, 88.9f, 91.92f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-337.27f, 47.34f, -419.95f),
            new(-333.29f, 7.06f, -451.97f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-175.51f, 6.5f, -607.24f),
            new(-183.04f, 3.85f, -607.21f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-416f, 3.8f, -562.77f),
            new(-439.071f, -0.3f, -556.1f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-500.08f, 3.5f, -552.53f),
            new(-509.95f, -0.3f, -552.91f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(5.23f, 106.65f, -390.92f),
            new(16.14f, 25.44f, -437.46f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-801.5f, 53.95f, 313.05f),
            new(-799.7f, 46.73f, 299.39f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(778.16f, 110f, 533.72f),
            new(770.54f, 80.3f, 534.08f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(831.25f, 98f, 722.47f),
            new(822.42f, 80f, 722.53f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(823.90704f, 75.62795f, -550.06836f),
            new(825.151f, 80.922905f, -538.5888f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(-429.767f, 3.8f, -593.997f),
            new(-436.77487f, 0f, -593.25397f),
            0.5f,
            false,
            0,
            NavmeshArea.GeneratedEdgeJump,
            NavmeshPolyFlags.GeneratedClimbDown | NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.GeneratedEdgeJump
        );
        config.AddOffMeshConnection
        (
            new(-492.6668f, 3.4977572f, -596.6317f),
            new(-495.37308f, 2.9727447f, -597.62134f),
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
        LinkPoints(mesh, new(831.99677f, 81.74532f, -523.0471f),      new(851.94464f, 95.71663f, -395.07745f));
        LinkPoints(mesh, new(-278.95834f, 3.4371157f, -590.45953f),   new(-401.257f, 3.8000011f, -589.865f));
        LinkPoints(mesh, new(-174.0232f, 6.247839f, -598.4698f),      new(-277.58643f, 3.8000011f, -590.52466f));
        LinkPoints(mesh, new(-510.286f, 3.791191f, -605.326f),        new(-492.67465f, 3.4225364f, -596.591f));
        LinkPoints(mesh, new(-425.0869f, 3.1132517f, -594.5297f),     new(-426.41736f, 3.7999992f, -594.3617f));
        LinkPoints(mesh, new(-405.04636f, 5.000005f, -590.0774f),     new(-423.07913f, 3f, -594.1107f),          true);
        LinkPoints(mesh, new(-437.25473f, 0f, -592.455f),             new(-453.648f, 0f, -590.415f),             true);
        LinkPoints(mesh, new(-453.55493f, 9.536743E-07f, -589.9885f), new(-461.67087f, 3.4977527f, -590.97705f), true);
        LinkClientPath(mesh, new(-461.95148f, 3.4977527f, -591.0292f), new(-492.27936f, 3.4977572f, -596.4955f));
        LinkPoints(mesh, new(-465.40982f, 2.999997f, -589.96594f), new(-492.25882f, 3.4977572f, -596.485f));
    }

    private static SceneExtractor.MeshInstance? ResolveInstance(SceneExtractor.Mesh mesh, ulong instanceId, int instanceIndex)
    {
        var index = ResolveInstanceIndex(mesh, instanceId, instanceIndex);
        return index >= 0 ? mesh.Instances[index] : null;
    }

    private static int ResolveInstanceIndex(SceneExtractor.Mesh mesh, ulong instanceId, int instanceIndex)
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

    private static void RecalculateMeshBounds(SceneExtractor.Mesh mesh)
    {
        mesh.LocalBounds = CalculateLocalBounds(mesh.Parts);
        foreach (var instance in mesh.Instances)
            instance.WorldBounds = TransformBounds(instance.WorldTransform, mesh.LocalBounds);
    }

    private static AABB CalculateLocalBounds(List<SceneExtractor.MeshPart> parts)
    {
        var bounds = new AABB { Min = new(float.MaxValue), Max = new(float.MinValue) };

        foreach (var part in parts)
        {
            bounds.Min = Vector3.Min(bounds.Min, part.LocalBounds.Min);
            bounds.Max = Vector3.Max(bounds.Max, part.LocalBounds.Max);
        }

        return bounds;
    }

    private static AABB CalculateLocalBounds(List<Vector3> vertices)
    {
        var bounds = new AABB { Min = new(float.MaxValue), Max = new(float.MinValue) };

        foreach (var vertex in vertices)
        {
            bounds.Min = Vector3.Min(bounds.Min, vertex);
            bounds.Max = Vector3.Max(bounds.Max, vertex);
        }

        return bounds;
    }

    private static AABB TransformBounds(Matrix4x3 worldTransform, AABB localBounds)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = axisX      * localCenter.X + axisY      * localCenter.Y + axisZ      * localCenter.Z + worldTransform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        return new() { Min = center                  - extent, Max = center       + extent };
    }

    private static Vector3 Abs(Vector3 value) => new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));
}
