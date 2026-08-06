using System.Numerics;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.FieldOperation;

[CustomizationTerritory(1346)]
internal class Z1346蜃景幻界新月岛北征之章 : NavmeshCustomization
{
    public override int Version => 2;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b2/collision/o6b2_t1_wal06.pcb", out var mesh0))
        {
            // 美杜莎实例1
            if (ResolveInstance(mesh0, 0xBCFC0C00000000ul, 41) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(-0.34325063f, 3.0007918E-08f, -0.9392439f),
                    Row1 = new(1.593441E-08f, 3f, 9.0023754E-08f),
                    Row2 = new(0.9392439f, 5.3114695E-09f, -0.34325063f),
                    Row3 = new(-723.8348f, 72.907f, -199.6791f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh0.LocalBounds);
            }
        }
        
        // 美杜莎墙体1
        scene.InsertWallCollider
        (
            new(6.3792725f, 9.099998f),
            new(-681.273f, 51.42f, -239.732f),
            96.54343f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        
        if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b2/collision/o6b2_t1_roc04.pcb", out var mesh1))
        {
            // 阿尔戈尔石头1
            if (ResolveInstance(mesh1, 0xBA960100000000ul, 0) is { } instance0)
            {
                instance0.WorldTransform = new()
                {
                    Row0 = new(1f, 0f, 0f),
                    Row1 = new(0f, 1.9f, 0f),
                    Row2 = new(0f, 0f, 1f),
                    Row3 = new(597.207f, 56.358f, 294.8287f)
                };
                instance0.WorldBounds = TransformBounds(instance0.WorldTransform, mesh1.LocalBounds);
            }
        }
        
        // 阿尔戈尔圆柱1
        scene.InsertCylinderCollider
        (
            new AABB
            {
                Min = new(664.433f, 60.08f, 226.588f),
                Max = new(672.54297f, 67.938f, 235.12599f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        // 阿尔戈尔圆柱2
        scene.InsertCylinderCollider
        (
            new AABB
            {
                Min = new(700.1213f, 61.23967f, 121.25908f),
                Max = new(706.0323f, 67.510666f, 125.984085f)
            },
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );

        // 浮游遗迹步道2
        scene.InsertWallCollider
        (
            new(80.568115f, 2.800003f),
            new(-489.15344f, 67.34456f, 629.6458f),
            -39.926067f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        // 浮游遗迹步道1
        scene.InsertWallCollider
        (
            new(78.06787f, 3.4499989f),
            new(-474.34308f, 66.93305f, 616.31244f),
            -40.068233f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        // 城塞入口1
        scene.InsertWallCollider
        (
            new(2.4105835f, 2.699997f),
            new(690.59766f, 131f, 622.6991f),
            -133.06546f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        // 城塞入口2
        scene.InsertWallCollider
        (
            new(2.3081665f, 2.350006f),
            new(695.38f, 131f, 627.4536f),
            -133.82306f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        // 城塞入口3
        scene.InsertWallCollider
        (
            new(2.4144287f, 2.649994f),
            new(622.4197f, 131f, 690.64075f),
            45.567398f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );
        // 城塞入口4
        scene.InsertWallCollider
        (
            new(2.3654175f, 2.699997f),
            new(627.3617f, 131f, 695.5911f),
            45.76367f,
            true,
            SceneExtractor.PrimitiveFlags.ForceUnwalkable
        );


        // 湖边的墙
        var blockers = new SceneExtractor.MeshPart();

        foreach (var (path, mesh) in scene.Meshes)
        {
            if (mesh.MeshType != SceneExtractor.MeshType.Terrain ||
                !path.StartsWith("bg/ex5/03_ocn_o6/btl/o6b2/collision/tr", StringComparison.Ordinal))
                continue;

            foreach (var part in mesh.Parts)
                AddWallBlockers(part, blockers, 1f, 1f);
        }

        if (blockers.Primitives.Count != 0)
        {
            blockers.LocalBounds = CalculateLocalBounds(blockers.Vertices);
            var blockerMesh = new SceneExtractor.Mesh
            {
                MeshType    = SceneExtractor.MeshType.FileMesh,
                LocalBounds = blockers.LocalBounds,
                Parts       = [blockers]
            };
            blockerMesh.Instances.Add
            (
                new
                (
                    0xBAADF00D13460001ul,
                    Matrix4x3.Identity,
                    blockerMesh.LocalBounds,
                    0,
                    SceneExtractor.PrimitiveFlags.ForceUnwalkable,
                    SceneExtractor.PrimitiveFlags.None
                )
            );
            scene.Meshes["<z1346 invisible wall blockers>"] = blockerMesh;
        }

    }

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        // 东侧浮岛
        LinkClientPath(mesh, new(-471.645f, 96.432f, 885.058f),  new(-502.403f, 158.678f, 880.735f));
        LinkClientPath(mesh, new(-502.411f, 158.576f, 894.453f), new(-452.72f, 96.33f, 886.656f));
        // 西侧浮岛
        LinkClientPath(mesh, new(-833.534f, 97.623f, 553.106f),  new(-912.932f, 157.793f, 630.335f));
        LinkClientPath(mesh, new(-900.858f, 157.8f, 629.249f),   new(-823.331f, 94.5f, 543.053f));
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

    private static void AddWallBlockers
    (
        SceneExtractor.MeshPart source,
        SceneExtractor.MeshPart target,
        float                   halfWidth,
        float                   heightOffset
    )
    {
        foreach (var primitive in source.Primitives)
        {
            if (primitive.Material != 0x2000ul)
                continue;

            var vertex1             = source.Vertices[primitive.V1];
            var vertex2             = source.Vertices[primitive.V2];
            var vertex3             = source.Vertices[primitive.V3];
            var normal              = Vector3.Cross(vertex2 - vertex1, vertex3 - vertex1);
            var normalLengthSquared = normal.LengthSquared();

            if (normalLengthSquared <= 0.0001f || normal.Y * normal.Y > normalLengthSquared * 0.0001f)
                continue;

            var point1           = new Vector2(vertex1.X, vertex1.Z);
            var point2           = new Vector2(vertex2.X, vertex2.Z);
            var point3           = new Vector2(vertex3.X, vertex3.Z);
            var start            = point1;
            var end              = point2;
            var maxLengthSquared = Vector2.DistanceSquared(point1, point2);
            var lengthSquared23  = Vector2.DistanceSquared(point2, point3);
            var lengthSquared31  = Vector2.DistanceSquared(point3, point1);

            if (lengthSquared23 > maxLengthSquared)
            {
                start            = point2;
                end              = point3;
                maxLengthSquared = lengthSquared23;
            }

            if (lengthSquared31 > maxLengthSquared)
            {
                start            = point3;
                end              = point1;
                maxLengthSquared = lengthSquared31;
            }

            if (maxLengthSquared <= 0.0001f)
                continue;

            var direction           = end - start;
            var normalizedDirection = direction                                                  / MathF.Sqrt(maxLengthSquared);
            var side                = new Vector2(-normalizedDirection.Y, normalizedDirection.X) * halfWidth;
            start -= normalizedDirection * halfWidth;
            end   += normalizedDirection * halfWidth;
            var height = MathF.Min(vertex1.Y, MathF.Min(vertex2.Y, vertex3.Y)) + heightOffset;
            var first  = target.Vertices.Count;
            target.Vertices.Add(new(start.X + side.X, height, start.Y + side.Y));
            target.Vertices.Add(new(start.X - side.X, height, start.Y - side.Y));
            target.Vertices.Add(new(end.X   + side.X, height, end.Y   + side.Y));
            target.Vertices.Add(new(end.X   - side.X, height, end.Y   - side.Y));

            var flags = primitive.Flags | SceneExtractor.PrimitiveFlags.ForceUnwalkable;
            target.Primitives.Add(new(first, first + 2, first + 1, flags, primitive.Material));
            target.Primitives.Add(new(first        + 1, first + 2, first + 3, flags, primitive.Material));
        }
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
}
