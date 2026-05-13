using System.Numerics;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Overworld;

[CustomizationTerritory(139)]
public class Z0139拉诺西亚高地 : NavmeshCustomization
{
    public override int Version => 3;

    public Z0139拉诺西亚高地() =>
        ApplyNormalPrecisionSettings();

    public override void CustomizeScene(SceneExtractor scene)
    {
        if (scene.Meshes.TryGetValue("<box>", out var mesh0))
        {
            if (ResolveInstance(mesh0, 0x39F95800000000ul, 87) is { } inst)
            {
                inst.WorldTransform.Row0 = new(3.0521717f, 0f, -4.080866f);
                inst.WorldTransform.Row1 = new(0f, 3.078f, 0f);
                inst.WorldTransform.Row2 = new(0.15091036f, 0f, 0.11286924f);
                inst.WorldTransform.Row3 = new(399.99274f, 8.529965f, 28.993406f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh0.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh1))
        {
            if (ResolveInstance(mesh1, 0x39F95900000000ul, 88) is { } inst)
            {
                inst.WorldTransform.Row0 = new(1.804044f, 0f, -1.725368f);
                inst.WorldTransform.Row1 = new(0f, 3.778f, 0f);
                inst.WorldTransform.Row2 = new(0.13025147f, 0f, 0.13619088f);
                inst.WorldTransform.Row3 = new(403.3705f, 8.528974f, 25.155504f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh1.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh2))
        {
            if (ResolveInstance(mesh2, 0x39F8F400000000ul, 49) is { } inst)
            {
                inst.WorldTransform.Row0 = new(1.2109348f, 0f, -0.57903975f);
                inst.WorldTransform.Row1 = new(0f, 3.678f, 0f);
                inst.WorldTransform.Row2 = new(0.08150698f, 0f, 0.17045398f);
                inst.WorldTransform.Row3 = new(406.5698f, 8.528983f, 22.765104f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh2.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh3))
        {
            if (ResolveInstance(mesh3, 0x39F8F500000000ul, 50) is { } inst)
            {
                inst.WorldTransform.Row0 = new(1.2109348f, 0f, -0.57903975f);
                inst.WorldTransform.Row1 = new(0f, 3.778f, 0f);
                inst.WorldTransform.Row2 = new(0.08150698f, 0f, 0.17045398f);
                inst.WorldTransform.Row3 = new(409.1308f, 8.4353895f, 21.540106f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh3.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh4))
        {
            if (ResolveInstance(mesh4, 0x39F95A00000000ul, 89) is { } inst)
            {
                inst.WorldTransform.Row0 = new(2.0044036f, 0f, -1.4878964f);
                inst.WorldTransform.Row1 = new(0f, 3.878f, 0f);
                inst.WorldTransform.Row2 = new(0.11232427f, 0f, 0.15131642f);
                inst.WorldTransform.Row3 = new(412.47958f, 8.433727f, 19.522573f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh4.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh5))
        {
            if (ResolveInstance(mesh5, 0x39F8F600000000ul, 51) is { } inst)
            {
                inst.WorldTransform.Row0 = new(1.0185566f, 0f, -0.87418103f);
                inst.WorldTransform.Row1 = new(0f, 4.378f, 0f);
                inst.WorldTransform.Row2 = new(0.123051755f, 0f, 0.14337438f);
                inst.WorldTransform.Row3 = new(415.60382f, 8.512573f, 17.035158f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh5.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh6))
        {
            if (ResolveInstance(mesh6, 0x39F8F700000000ul, 52) is { } inst)
            {
                inst.WorldTransform.Row0 = new(1.2012515f, 0f, -0.5988695f);
                inst.WorldTransform.Row1 = new(0f, 4.378f, 0f);
                inst.WorldTransform.Row2 = new(0.08429826f, 0f, 0.16909096f);
                inst.WorldTransform.Row3 = new(417.8848f, 8.528983f, 15.425109f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh6.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("<box>", out var mesh7))
        {
            if (ResolveInstance(mesh7, 0x39F95B00000000ul, 90) is { } inst)
            {
                inst.WorldTransform.Row0 = new(2.3867178f, 0f, -0.73146963f);
                inst.WorldTransform.Row1 = new(0f, 4.178f, 0f);
                inst.WorldTransform.Row2 = new(0.055220105f, 0f, 0.18017808f);
                inst.WorldTransform.Row3 = new(421.58566f, 8.528974f, 14.065789f);
                inst.WorldBounds         = TransformBounds(inst.WorldTransform, mesh7.LocalBounds);
            }
        }

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(323.6f, -3.4f, -9f),
                Max = new(343.4f, 3.6f, 13.9f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(352.10004f, -3.4f, -16.4f),
                Max = new(381.7f, 2.2000003f, 19.9f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(560.6f, -3.4f, 131.7f),
                Max = new(654.1f, 0f, 147.7f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(561.1f, -3.4f, 158f),
                Max = new(618.6f, 0f, 182.1f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(403.7864f, 1.1228976f, 77.04017f),
                Max = new(404.7864f, 6.922898f, 78.04017f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(401.63702f, 0.6507976f, 82.471825f),
                Max = new(402.63702f, 7.2507973f, 83.471825f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(389.47458f, 8.413181f, 36.191235f),
            new(381.6874f, -3.1000004f, 26.508116f),
            0.5f,
            false,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }

    private static SceneExtractor.MeshInstance? ResolveInstance(SceneExtractor.Mesh mesh, ulong instanceId, int instanceIndex)
    {
        var index = ResolveInstanceIndex(mesh, instanceId, instanceIndex);
        return index >= 0 ? mesh.Instances[index] : null;
    }

    private static int ResolveInstanceIndex(SceneExtractor.Mesh mesh, ulong instanceId, int instanceIndex)
    {
        if (instanceId != 0)
        {
            for (var i = 0; i < mesh.Instances.Count; ++i)
                if (mesh.Instances[i].Id == instanceId)
                    return i;
        }

        return instanceIndex >= 0 && instanceIndex < mesh.Instances.Count ? instanceIndex : -1;
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
