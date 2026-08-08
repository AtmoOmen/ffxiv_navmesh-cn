using System.Numerics;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;
using AABB = vnavmesh.Common.Models.AABB;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.Build.Custom.Extensions;

public static class SceneExtractorExtension
{
    extension
    (
        SceneExtractor scene
    )
    {
        private void InsertAxisAlignedCollider
        (
            string                        meshKey,
            Vector3                       scale,
            Vector3                       worldTransform,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            var transform = Matrix4x3.Identity;
            transform.Row0 = new(scale.X, 0, 0);
            transform.Row1 = new(0, scale.Y, 0);
            transform.Row2 = new(0, 0, scale.Z);
            transform.Row3 = worldTransform;
            var aabb = new AABB { Min = transform.Row3 - scale, Max = transform.Row3 + scale };
            scene.InsertCollider(meshKey, transform, aabb, forceSetFlags, forceClearFlags);
        }

        private void InsertCollider
        (
            string                        meshKey,
            Matrix4x3                     transform,
            AABB                          bounds,
            PrimitiveFlags forceSetFlags,
            PrimitiveFlags forceClearFlags
        )
        {
            var existingMesh = scene.Meshes[meshKey];
            var id           = 0xbaadf00d00000001ul + (uint)existingMesh.Instances.Count;
            existingMesh.Instances.Insert(0, new(id, transform, bounds, 0, forceSetFlags, forceClearFlags));
        }

        public void InsertMeshInstance
        (
            string                        meshKey,
            Matrix4x3                     transform,
            ulong                         material        = 0,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            if (!scene.Meshes.TryGetValue(meshKey, out var mesh))
                return;

            var localCenter = (mesh.LocalBounds.Min + mesh.LocalBounds.Max) * 0.5f;
            var localExtent = (mesh.LocalBounds.Max - mesh.LocalBounds.Min) * 0.5f;
            var center = (transform.Row0 * localCenter.X) +
                         (transform.Row1 * localCenter.Y) +
                         (transform.Row2 * localCenter.Z) +
                         transform.Row3;
            var extent = (Vector3.Abs(transform.Row0) * localExtent.X) +
                         (Vector3.Abs(transform.Row1) * localExtent.Y) +
                         (Vector3.Abs(transform.Row2) * localExtent.Z);
            var bounds = new AABB { Min = center - extent, Max = center + extent };
            var id     = 0xbaadf00d10000001ul + (uint)mesh.Instances.Count;
            mesh.Instances.Insert(0, new(id, transform, bounds, material, forceSetFlags, forceClearFlags));
        }

        public void InsertMeshInstances
        (
            string                        meshKey,
            Matrix4x3                     transform,
            int                           count,
            Vector3                       offset,
            ulong                         material        = 0,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            count = Math.Clamp(count, 1, 1024);
            for (var i = 0; i < count; ++i)
            {
                var instanceTransform = transform;
                instanceTransform.Row3 += offset * i;
                scene.InsertMeshInstance(meshKey, instanceTransform, material, forceSetFlags, forceClearFlags);
            }
        }

        public void RemoveMeshInstancesInBounds
        (
            AABB   bounds,
            string meshKeyContains = ""
        )
        {
            foreach (var (key, mesh) in scene.Meshes)
            {
                if (!string.IsNullOrWhiteSpace(meshKeyContains) && !key.Contains(meshKeyContains, StringComparison.OrdinalIgnoreCase))
                    continue;

                mesh.Instances.RemoveAll(instance => BoundsIntersect(instance.WorldBounds, bounds));
            }
        }

        public void SetMeshInstanceFlagsInBounds
        (
            AABB                          bounds,
            PrimitiveFlags forceSetFlags,
            PrimitiveFlags forceClearFlags,
            string                        meshKeyContains = ""
        )
        {
            foreach (var (key, mesh) in scene.Meshes)
            {
                if (!string.IsNullOrWhiteSpace(meshKeyContains) && !key.Contains(meshKeyContains, StringComparison.OrdinalIgnoreCase))
                    continue;

                foreach (var instance in mesh.Instances)
                {
                    if (!BoundsIntersect(instance.WorldBounds, bounds))
                        continue;

                    instance.ForceSetPrimFlags   = forceSetFlags;
                    instance.ForceClearPrimFlags = forceClearFlags;
                }
            }
        }

        private static bool BoundsIntersect
        (
            AABB a,
            AABB b
        ) =>
            a.Min.X <= b.Max.X && a.Max.X >= b.Min.X &&
            a.Min.Y <= b.Max.Y && a.Max.Y >= b.Min.Y &&
            a.Min.Z <= b.Max.Z && a.Max.Z >= b.Min.Z;

        private static Matrix4x3 ToCommon
        (
            Matrix4x4 matrix
        ) =>
            new
            (
                new(matrix.M11, matrix.M12, matrix.M13),
                new(matrix.M21, matrix.M22, matrix.M23),
                new(matrix.M31, matrix.M32, matrix.M33),
                new(matrix.M41, matrix.M42, matrix.M43)
            );

        public void InsertAABoxCollider
        (
            Vector3                       scale,
            Vector3                       worldTransform,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        ) =>
            scene.InsertAxisAlignedCollider("<box>", scale, worldTransform, forceSetFlags, forceClearFlags);

        public void InsertAABoxCollider
        (
            AABB                          bounds,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
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
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        ) =>
            scene.InsertAxisAlignedCollider("<cylinder>", scale, worldTransform, forceSetFlags, forceClearFlags);

        public void InsertCylinderCollider
        (
            AABB                          bounds,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            var scale     = (bounds.Max - bounds.Min) * 0.5f;
            var transform = (bounds.Min + bounds.Max) * 0.5f;
            scene.InsertCylinderCollider(scale, transform, forceSetFlags, forceClearFlags);
        }

        public void InsertOrientedCylinderCollider
        (
            Vector3                       start,
            Vector3                       end,
            float                         radius,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            var axis       = end - start;
            var axisLength = axis.Length();
            var axisDirection = axisLength > 0.0001f ?
                                    axis / axisLength :
                                    Vector3.UnitY;
            var reference = MathF.Abs(Vector3.Dot(axisDirection, Vector3.UnitY)) < 0.999f ?
                                Vector3.UnitY :
                                Vector3.UnitX;
            var radialX = Vector3.Normalize(Vector3.Cross(reference, axisDirection));
            var radialZ = Vector3.Normalize(Vector3.Cross(axisDirection, radialX));
            radius = MathF.Max(MathF.Abs(radius), 0.005f);

            var transform = Matrix4x3.Identity;
            transform.Row0 = radialX * radius;
            transform.Row1 = axisDirection * MathF.Max(axisLength * 0.5f, 0.005f);
            transform.Row2 = radialZ * radius;
            transform.Row3 = (start + end) * 0.5f;
            var extent = Vector3.Abs(transform.Row0) +
                         Vector3.Abs(transform.Row1) +
                         Vector3.Abs(transform.Row2);
            var bounds = new AABB { Min = transform.Row3 - extent, Max = transform.Row3 + extent };
            scene.InsertCollider("<cylinder>", transform, bounds, forceSetFlags, forceClearFlags);
        }

        public void InsertSphereCollider
        (
            Vector3                       scale,
            Vector3                       center,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        ) =>
            scene.InsertAxisAlignedCollider("<sphere>", Vector3.Max(Vector3.Abs(scale), new Vector3(0.005f)), center, forceSetFlags, forceClearFlags);

        public void InsertSphereCollider
        (
            AABB                          bounds,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        ) =>
            scene.InsertSphereCollider((bounds.Max - bounds.Min) * 0.5f, (bounds.Min + bounds.Max) * 0.5f, forceSetFlags, forceClearFlags);

        public void InsertWallCollider
        (
            Vector2                       halfSize,
            Vector3                       center,
            float                         rotationDegrees,
            bool                          doubleSided,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            halfSize = Vector2.Max(Vector2.Abs(halfSize), new Vector2(0.005f));
            var matrix = Matrix4x4.CreateScale(halfSize.X, halfSize.Y, 1f) *
                         Matrix4x4.CreateRotationY(rotationDegrees * (MathF.PI / 180f));
            matrix.Translation = center;

            var transform = ToCommon(matrix);
            var extent    = Vector3.Max(Vector3.Abs(transform.Row0) + Vector3.Abs(transform.Row1), new Vector3(0.005f));
            var bounds    = new AABB { Min = center - extent, Max = center + extent };
            scene.InsertCollider(doubleSided ? "<plane two-sided>" : "<plane one-sided>", transform, bounds, forceSetFlags, forceClearFlags);
        }

        public void InsertRampCollider
        (
            Vector3                       halfExtents,
            Vector3                       center,
            float                         rotationDegrees,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            halfExtents = Vector3.Max(Vector3.Abs(halfExtents), new Vector3(0.005f));
            forceSetFlags |= PrimitiveFlags.ForceWalkable;
            forceSetFlags &= ~PrimitiveFlags.ForceUnwalkable;
            forceClearFlags |= PrimitiveFlags.ForceUnwalkable;
            forceClearFlags &= ~PrimitiveFlags.ForceWalkable;
            var matrix = Matrix4x4.CreateScale(halfExtents) *
                         Matrix4x4.CreateRotationY(rotationDegrees * (MathF.PI / 180f));
            matrix.Translation = center;

            var transform = ToCommon(matrix);
            var extent = Vector3.Abs(transform.Row0) +
                         Vector3.Abs(transform.Row1) +
                         Vector3.Abs(transform.Row2);
            var bounds = new AABB { Min = center - extent, Max = center + extent };
            scene.InsertCollider("<ramp>", transform, bounds, forceSetFlags, forceClearFlags);
        }

        public void InsertWalkableFloor
        (
            Vector2                       halfSize,
            Vector3                       center,
            float                         rotationDegrees,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            halfSize = Vector2.Max(Vector2.Abs(halfSize), new Vector2(0.05f));
            forceSetFlags |= PrimitiveFlags.ForceWalkable;
            forceSetFlags &= ~PrimitiveFlags.ForceUnwalkable;
            forceClearFlags |= PrimitiveFlags.ForceUnwalkable;
            forceClearFlags &= ~PrimitiveFlags.ForceWalkable;
            var matrix = Matrix4x4.CreateScale(halfSize.X, 1f, halfSize.Y) *
                         Matrix4x4.CreateRotationY(rotationDegrees * (MathF.PI / 180f));
            matrix.Translation = center;

            var transform = ToCommon(matrix);
            var extent = Vector3.Max(Vector3.Abs(transform.Row0) + Vector3.Abs(transform.Row2), new Vector3(0.005f));
            var bounds = new AABB { Min = center - extent, Max = center + extent };
            scene.InsertCollider("<walkable floor>", transform, bounds, forceSetFlags, forceClearFlags);
        }

        public void InsertOrientedBoxCollider
        (
            Vector3                       halfExtents,
            Vector3                       center,
            float                         rotationDegrees,
            PrimitiveFlags forceSetFlags   = default,
            PrimitiveFlags forceClearFlags = default
        )
        {
            halfExtents = Vector3.Max(Vector3.Abs(halfExtents), new Vector3(0.005f));
            var matrix = Matrix4x4.CreateScale(halfExtents) *
                         Matrix4x4.CreateRotationY(rotationDegrees * (MathF.PI / 180f));
            matrix.Translation = center;

            var transform = ToCommon(matrix);
            var extent = Vector3.Abs(transform.Row0) +
                         Vector3.Abs(transform.Row1) +
                         Vector3.Abs(transform.Row2);
            var bounds = new AABB { Min = center - extent, Max = center + extent };
            scene.InsertCollider("<box>", transform, bounds, forceSetFlags, forceClearFlags);
        }
    }
}
