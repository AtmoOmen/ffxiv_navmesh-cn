using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Build.Models;
using AABB = vnavmesh.Common.Models.AABB;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(139)]
internal class Z0139拉诺西亚高地 : NavmeshCustomization
{
    public override int Version => 4;

    public override void CustomizeBuildSettings
    (
        SceneDefinition definition,
        NavmeshSettings settings
    )
    {
        settings.AgentRadius     = 1f;
        settings.Filtering       = NavmeshFilter.LowHangingObstacles | NavmeshFilter.LedgeSpans | NavmeshFilter.WalkableLowHeightSpans;
        settings.RegionMinSize   = 4f;
        settings.RegionMergeSize = 40f;
    }

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("<box>", out var mesh0))
        {
            if (ResolveInstance(mesh0, 0x39F8F400000000ul, 45) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(1.2109348f, 0f, -0.57903975f),
                    Row1 = new(0f, 3.678f, 0f),
                    Row2 = new(0.08150698f, 0f, 0.17045398f),
                    Row3 = new(406.5698f, 8.528983f, 22.765104f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F8F500000000ul, 46) is { } instance1)
            {
                instance1.WorldTransform = new()
                {
                    Row0 = new(1.2109348f, 0f, -0.57903975f),
                    Row1 = new(0f, 3.778f, 0f),
                    Row2 = new(0.08150698f, 0f, 0.17045398f),
                    Row3 = new(409.1308f, 8.4353895f, 21.540106f)
                };
                instance1.WorldBounds = TransformBounds(instance1.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F8F600000000ul, 47) is { } instance2)
            {
                instance2.WorldTransform = new()
                {
                    Row0 = new(1.0185566f, 0f, -0.87418103f),
                    Row1 = new(0f, 4.378f, 0f),
                    Row2 = new(0.123051755f, 0f, 0.14337438f),
                    Row3 = new(415.60382f, 8.512573f, 17.035158f)
                };
                instance2.WorldBounds = TransformBounds(instance2.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F8F700000000ul, 48) is { } instance3)
            {
                instance3.WorldTransform = new()
                {
                    Row0 = new(1.2012515f, 0f, -0.5988695f),
                    Row1 = new(0f, 4.378f, 0f),
                    Row2 = new(0.08429826f, 0f, 0.16909096f),
                    Row3 = new(417.8848f, 8.528983f, 15.425109f)
                };
                instance3.WorldBounds = TransformBounds(instance3.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F95800000000ul, 83) is { } instance4)
            {
                instance4.WorldTransform = new()
                {
                    Row0 = new(3.0521717f, 0f, -4.080866f),
                    Row1 = new(0f, 3.078f, 0f),
                    Row2 = new(0.15091036f, 0f, 0.11286924f),
                    Row3 = new(399.99274f, 8.529965f, 28.993406f)
                };
                instance4.WorldBounds = TransformBounds(instance4.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F95900000000ul, 84) is { } instance5)
            {
                instance5.WorldTransform = new()
                {
                    Row0 = new(1.804044f, 0f, -1.725368f),
                    Row1 = new(0f, 3.778f, 0f),
                    Row2 = new(0.13025147f, 0f, 0.13619088f),
                    Row3 = new(403.3705f, 8.528974f, 25.155504f)
                };
                instance5.WorldBounds = TransformBounds(instance5.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F95A00000000ul, 85) is { } instance6)
            {
                instance6.WorldTransform = new()
                {
                    Row0 = new(2.0044036f, 0f, -1.4878964f),
                    Row1 = new(0f, 3.878f, 0f),
                    Row2 = new(0.11232427f, 0f, 0.15131642f),
                    Row3 = new(412.47958f, 8.433727f, 19.522573f)
                };
                instance6.WorldBounds = TransformBounds(instance6.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x39F95B00000000ul, 86) is { } instance7)
            {
                instance7.WorldTransform = new()
                {
                    Row0 = new(2.3867178f, 0f, -0.73146963f),
                    Row1 = new(0f, 4.178f, 0f),
                    Row2 = new(0.055220105f, 0f, 0.18017808f),
                    Row3 = new(421.58566f, 8.528974f, 14.065789f)
                };
                instance7.WorldBounds = TransformBounds(instance7.WorldTransform, mesh0.LocalBounds);
            }
        }

        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(401.63702f, 0.65079784f, 82.471825f),
                Max = new(402.63702f, 7.2507973f, 83.471825f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(403.7864f, 1.1228976f, 77.04017f),
                Max = new(404.7864f, 6.922898f, 78.04017f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(638.71655f, -3.6505f, 158.40239f),
                Max = new(647.1455f, -0.3935001f, 168.17238f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
        scene.InsertAABoxCollider
        (
            new AABB
            {
                Min = new(615.9527f, -3.6504998f, 156.4376f),
                Max = new(622.0457f, -0.5095f, 161.0796f)
            },
            PrimitiveFlags.ForceUnwalkable
        );
    }

    public override void CustomizeSettings
    (
        DtNavMeshCreateParams config
    )
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
        config.AddOffMeshConnection
        (
            new(652.77734f, -3.0527687f, 152.7778f),
            new(658.2084f, -3.052744f, 153.20773f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(588.88586f, -3.0527709f, 152.8568f),
            new(597.9923f, -3.002748f, 152.76297f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(616.8017f, -3.0527658f, 152.84084f),
            new(625.735f, -3.052744f, 152.95517f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(602.55554f, -3.052746f, 153.16356f),
            new(607.32764f, -3.0999994f, 152.94571f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(610.01025f, -3.0640388f, 152.81105f),
            new(613.78125f, -3.0527718f, 152.88957f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
        config.AddOffMeshConnection
        (
            new(619.02246f, -3.0527675f, 154.02652f),
            new(622.58417f, -3.0527458f, 154.46297f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh
        );
    }

    private static MeshInstance? ResolveInstance
    (
        Mesh mesh,
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
        Mesh mesh,
        ulong               instanceId,
        int                 instanceIndex
    )
    {
        if (instanceIndex >= 0 && instanceIndex < mesh.Instances.Count && (instanceId == 0 || mesh.Instances[instanceIndex].ID == instanceId))
            return instanceIndex;

        if (instanceId != 0)
        {
            for (var i = 0; i < mesh.Instances.Count; ++i)
                if (mesh.Instances[i].ID == instanceId)
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
