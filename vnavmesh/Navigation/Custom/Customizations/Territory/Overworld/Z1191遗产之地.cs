using System.Numerics;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Overworld;

[CustomizationTerritory(1191)]
internal class Z1191遗产之地 : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings) =>
        settings.AgentRadius = 1f;

    public override void CustomizeScene(SceneExtractor scene)
    {
        if (scene.Meshes.TryGetValue("bg/ex5/01_xkt_x6/fld/x6f2/collision/x6f2_d1_grk1c.pcb", out var mesh0))
        {
            // 地图左下角FATE小石头加高1
            if (ResolveInstance(mesh0, 0x99943900000000ul, 9) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(-0.8939932f, -7.815536E-08f, 0.29047602f),
                    Row1 = new(-4.1655397E-07f, 3.64f, -3.0264414E-07f),
                    Row2 = new(-0.29047602f, -1.0757164E-07f, -0.8939932f),
                    Row3 = new(-610.4479f, -14.00001f, 673.8964f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh0.LocalBounds);
            }

            // 地图左下角FATE小石头加高3
            if (ResolveInstance(mesh0, 0x9E809700000000ul, 41) is { } instance1)
            {
                instance1.WorldTransform = new()
                {
                    Row0 = new(1f, 0f, 0f),
                    Row1 = new(0f, 3.9f, 0f),
                    Row2 = new(0f, 0f, 1f),
                    Row3 = new(-577.0337f, -13.89243f, 674.726f)
                };
                instance1.WorldBounds = TransformBounds(instance1.WorldTransform, mesh0.LocalBounds);
            }

            if (ResolveInstance(mesh0, 0x9F079300000000ul, 114) is { } instance2)
            {
                instance2.WorldTransform = new()
                {
                    Row0 = new(0.9969984f, 0.077421784f, 0f),
                    Row1 = new(-0.27097633f, 3.4894946f, 0f),
                    Row2 = new(0f, 0f, 1f),
                    Row3 = new(-458.461f, -10.9592f, 511.9943f)
                };
                instance2.WorldBounds = TransformBounds(instance2.WorldTransform, mesh0.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("bg/ex5/01_xkt_x6/fld/x6f2/collision/x6f2_d1_grk2b.pcb", out var mesh1))
        {
            // 地图左下角FATE小石头加高2
            if (ResolveInstance(mesh1, 0x99AF3700000000ul, 4) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(0.99465585f, 0f, -0.1753846f),
                    Row1 = new(0f, 2.51f, 0f),
                    Row2 = new(0.1753846f, 0f, 0.99465585f),
                    Row3 = new(-576.5275f, -14.00001f, 671.7339f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh1.LocalBounds);
            }
        }

        if (scene.Meshes.TryGetValue("bg/ex5/01_xkt_x6/fld/x6f2/collision/x6f2_d0_gr2m2.pcb", out var mesh2))
        {
            // 蛇FATE加高墙
            if (ResolveInstance(mesh2, 0x99B66100000000ul, 2) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(-0.14698899f, 2.2f, -0.9891381f),
                    Row1 = new(-1.6142769E-09f, 2.1f, -2.184532E-08f),
                    Row2 = new(0.9891381f, 0.20000002f, -0.14698899f),
                    Row3 = new(-459.8482f, -6.892f, 511.1482f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh2.LocalBounds);
            }
        }
    }

    private static SceneExtractor.MeshInstance? ResolveInstance(SceneExtractor.Mesh mesh, ulong instanceId, int instanceIndex)
    {
        var index = ResolveInstanceIndex(mesh, instanceId, instanceIndex);
        return index >= 0 ?
                   mesh.Instances[index] :
                   null;
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
