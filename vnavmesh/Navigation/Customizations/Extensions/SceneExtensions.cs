using System.Numerics;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations;

public static class SceneExtensions
{
    extension(SceneExtractor scene)
    {
        private void InsertAxisAlignedCollider
        (
            string                        meshKey,
            Vector3                       scale,
            Vector3                       worldTransform,
            SceneExtractor.PrimitiveFlags forceSetFlags   = default,
            SceneExtractor.PrimitiveFlags forceClearFlags = default
        )
        {
            var transform = Matrix4x3.Identity;
            transform.M11  = scale.X;
            transform.M22  = scale.Y;
            transform.M33  = scale.Z;
            transform.Row3 = worldTransform;
            var aabb         = new AABB { Min = transform.Row3 - scale, Max = transform.Row3 + scale };
            var existingMesh = scene.Meshes[meshKey];
            var id           = 0xbaadf00d00000001ul + (uint)existingMesh.Instances.Count;
            existingMesh.Instances.Insert(0, new(id, transform, aabb, 0, forceSetFlags, forceClearFlags));
        }

        public void InsertAABoxCollider
        (
            Vector3                       scale,
            Vector3                       worldTransform,
            SceneExtractor.PrimitiveFlags forceSetFlags   = default,
            SceneExtractor.PrimitiveFlags forceClearFlags = default
        ) =>
            scene.InsertAxisAlignedCollider("<box>", scale, worldTransform, forceSetFlags, forceClearFlags);

        public void InsertAABoxCollider
        (
            AABB                          bounds,
            SceneExtractor.PrimitiveFlags forceSetFlags   = default,
            SceneExtractor.PrimitiveFlags forceClearFlags = default
        )
        {
            var scale     = (bounds.Max - bounds.Min) * 0.5f;
            var transform = (bounds.Min + bounds.Max) * 0.5f;
            scene.InsertAABoxCollider(scale, transform, forceSetFlags, forceClearFlags);
        }

        public void InsertCylinderCollider
        (
            Vector3                       scale,
            Vector3                       worldTransform,
            SceneExtractor.PrimitiveFlags forceSetFlags   = default,
            SceneExtractor.PrimitiveFlags forceClearFlags = default
        ) =>
            scene.InsertAxisAlignedCollider("<cylinder>", scale, worldTransform, forceSetFlags, forceClearFlags);

        public void InsertCylinderCollider
        (
            AABB                          bounds,
            SceneExtractor.PrimitiveFlags forceSetFlags   = default,
            SceneExtractor.PrimitiveFlags forceClearFlags = default
        )
        {
            var scale     = (bounds.Max - bounds.Min) * 0.5f;
            var transform = (bounds.Min + bounds.Max) * 0.5f;
            scene.InsertCylinderCollider(scale, transform, forceSetFlags, forceClearFlags);
        }
    }
}
