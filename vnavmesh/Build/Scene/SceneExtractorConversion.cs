using vnavmesh.Common.Build;
using AABB = vnavmesh.Common.Models.AABB;
using CommonMatrix4x3 = vnavmesh.Common.Models.Matrix4x3;
using Matrix4x3 = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.Matrix4x3;

namespace vnavmesh.Build.Scene;

public static class SceneExtractorConversion
{
    public static BuildScene ToBuildScene
    (
        this SceneExtractor extractor
    )
    {
        var scene = new BuildScene();

        foreach (var (key, mesh) in extractor.Meshes)
        {
            var convertedMesh = new BuildScene.Mesh
            {
                MeshType    = (BuildScene.MeshType)mesh.MeshType,
                LocalBounds = Convert(mesh.LocalBounds)
            };

            foreach (var part in mesh.Parts)
            {
                var convertedPart = new BuildScene.MeshPart
                {
                    LocalBounds = Convert(part.LocalBounds)
                };

                convertedPart.Vertices.AddRange(part.Vertices);

                foreach (var primitive in part.Primitives)
                    convertedPart.Primitives.Add(new(primitive.V1, primitive.V2, primitive.V3, (BuildScene.PrimitiveFlags)primitive.Flags, primitive.Material));

                convertedMesh.Parts.Add(convertedPart);
            }

            foreach (var instance in mesh.Instances)
            {
                convertedMesh.Instances.Add
                (
                    new
                    (
                        instance.Id,
                        Convert(instance.WorldTransform),
                        Convert(instance.WorldBounds),
                        instance.Material,
                        (BuildScene.PrimitiveFlags)instance.ForceSetPrimFlags,
                        (BuildScene.PrimitiveFlags)instance.ForceClearPrimFlags
                    )
                );
            }

            scene.Meshes[key] = convertedMesh;
        }

        return scene;
    }

    private static AABB Convert
    (
        FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB bounds
    ) =>
        new(bounds.Min, bounds.Max);

    private static CommonMatrix4x3 Convert
    (
        Matrix4x3 matrix
    ) =>
        new(matrix.Row0, matrix.Row1, matrix.Row2, matrix.Row3);
}
