using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Models;
using AABB = vnavmesh.Common.Models.AABB;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.Build.Custom.Editor;

internal static class CustomizationDraftApplier
{
    public static void ApplyScene
    (
        SceneExtractor     scene,
        CustomizationDraft draft
    )
    {
        foreach (var removal in draft.MeshRemovals)
        {
            if (!removal.Enabled || string.IsNullOrWhiteSpace(removal.MeshKey))
                continue;

            scene.Meshes.Remove(removal.MeshKey);
        }

        foreach (var patch in draft.PartPatches)
            ApplyPartPatch(scene, patch);

        foreach (var patch in draft.InstancePatches)
            ApplyInstancePatch(scene, patch);

        foreach (var insertion in draft.ColliderInsertions)
            ApplyColliderInsertion(scene, insertion);
    }

    public static void ApplyBuildProfile
    (
        NavmeshBuildProfile profile,
        CustomizationDraft  draft
    )
    {
        var overrides = draft.BuildProfile;

        if (overrides.PartitioningOverride is { } partitioning)
            profile.PartitioningOverride = partitioning;
        if (overrides.CellSizeOverride is { } cellSize)
            profile.CellSizeOverride = cellSize;
        if (overrides.CellHeightOverride is { } cellHeight)
            profile.CellHeightOverride = cellHeight;
        if (overrides.RegionMinSizeOverride is { } regionMinSize)
            profile.RegionMinSizeOverride = regionMinSize;
        if (overrides.RegionMergeSizeOverride is { } regionMergeSize)
            profile.RegionMergeSizeOverride = regionMergeSize;
        if (overrides.PolyMaxEdgeLenOverride is { } polyMaxEdgeLen)
            profile.PolyMaxEdgeLenOverride = polyMaxEdgeLen;
        if (overrides.PolyMaxSimplificationErrorOverride is { } polyMaxSimplificationError)
            profile.PolyMaxSimplificationErrorOverride = polyMaxSimplificationError;
        if (overrides.AgentRadiusOverride is { } agentRadius)
            profile.AgentRadiusOverride = agentRadius;
        if (overrides.VolumeCellSizeOverride is { } volumeCellSize)
            profile.VolumeCellSizeOverride = volumeCellSize;
        if (overrides.VolumeVerticalPaddingOverride is { } volumeVerticalPadding)
            profile.VolumeVerticalPaddingOverride = volumeVerticalPadding;
        if (overrides.VolumeWallThickenNormalYThresholdOverride is { } volumeWallThickenNormalYThreshold)
            profile.VolumeWallThickenNormalYThresholdOverride = volumeWallThickenNormalYThreshold;
        if (overrides.VolumeWallThickenHorizontalRadiusOverride is { } volumeWallThickenHorizontalRadius)
            profile.VolumeWallThickenHorizontalRadiusOverride = volumeWallThickenHorizontalRadius;
        if (overrides.VolumeThinWallStripNormalYThresholdOverride is { } volumeThinWallStripNormalYThreshold)
            profile.VolumeThinWallStripNormalYThresholdOverride = volumeThinWallStripNormalYThreshold;
        if (overrides.VolumeThinWallStripMaxProjectedThicknessOverride is { } volumeThinWallStripMaxProjectedThickness)
            profile.VolumeThinWallStripMaxProjectedThicknessOverride = volumeThinWallStripMaxProjectedThickness;
        if (overrides.VolumeThinWallStripBaseRadiusOverride is { } volumeThinWallStripBaseRadius)
            profile.VolumeThinWallStripBaseRadiusOverride = volumeThinWallStripBaseRadius;
        if (overrides.VolumeThinWallStripExtraPaddingOverride is { } volumeThinWallStripExtraPadding)
            profile.VolumeThinWallStripExtraPaddingOverride = volumeThinWallStripExtraPadding;
        if (overrides.DetailSampleDistOverride is { } detailSampleDist)
            profile.DetailSampleDistOverride = detailSampleDist;
        if (overrides.GenerateEdgeClimbLinksOverride is { } generateEdgeClimbLinks)
            profile.GenerateEdgeClimbLinksOverride = generateEdgeClimbLinks;
        if (overrides.GenerateEdgeJumpLinksOverride is { } generateEdgeJumpLinks)
            profile.GenerateEdgeJumpLinksOverride = generateEdgeJumpLinks;
    }

    public static void ApplyBuildSettings
    (
        NavmeshSettings    settings,
        CustomizationDraft draft
    )
    {
        var overrides = draft.BuildSettings;

        if (overrides.CellSize is { } cellSize)
            settings.CellSize = cellSize;
        if (overrides.CellHeight is { } cellHeight)
            settings.CellHeight = cellHeight;
        if (overrides.AgentHeight is { } agentHeight)
            settings.AgentHeight = agentHeight;
        if (overrides.AgentRadius is { } agentRadius)
            settings.AgentRadius = agentRadius;
        if (overrides.AgentMaxClimb is { } agentMaxClimb)
            settings.AgentMaxClimb = agentMaxClimb;
        if (overrides.AgentMaxSlopeDeg is { } agentMaxSlopeDeg)
            settings.AgentMaxSlopeDeg = agentMaxSlopeDeg;
        if (overrides.Filtering is { } filtering)
            settings.Filtering = filtering;
        if (overrides.RegionMinSize is { } regionMinSize)
            settings.RegionMinSize = regionMinSize;
        if (overrides.RegionMergeSize is { } regionMergeSize)
            settings.RegionMergeSize = regionMergeSize;
        if (overrides.Partitioning is { } partitioning)
            settings.Partitioning = partitioning;
        if (overrides.PolyMaxEdgeLen is { } polyMaxEdgeLen)
            settings.PolyMaxEdgeLen = polyMaxEdgeLen;
        if (overrides.PolyMaxSimplificationError is { } polyMaxSimplificationError)
            settings.PolyMaxSimplificationError = polyMaxSimplificationError;
        if (overrides.PolyMaxVerts is { } polyMaxVerts)
            settings.PolyMaxVerts = polyMaxVerts;
        if (overrides.DetailSampleDist is { } detailSampleDist)
            settings.DetailSampleDist = detailSampleDist;
        if (overrides.DetailMaxSampleError is { } detailMaxSampleError)
            settings.DetailMaxSampleError = detailMaxSampleError;
        if (overrides.FastBuild is { } fastBuild)
            settings.FastBuild = fastBuild;
        if (overrides.GenerateEdgeClimbLinks is { } generateEdgeClimbLinks)
            settings.GenerateEdgeClimbLinks = generateEdgeClimbLinks;
        if (overrides.GenerateEdgeJumpLinks is { } generateEdgeJumpLinks)
            settings.GenerateEdgeJumpLinks = generateEdgeJumpLinks;
        if (overrides.GroundTolerance is { } groundTolerance)
            settings.GroundTolerance = groundTolerance;
        if (overrides.ClimbDownDistance is { } climbDownDistance)
            settings.ClimbDownDistance = climbDownDistance;
        if (overrides.ClimbDownMaxHeight is { } climbDownMaxHeight)
            settings.ClimbDownMaxHeight = climbDownMaxHeight;
        if (overrides.ClimbDownMinHeight is { } climbDownMinHeight)
            settings.ClimbDownMinHeight = climbDownMinHeight;
        if (overrides.EdgeJumpEndDistance is { } edgeJumpEndDistance)
            settings.EdgeJumpEndDistance = edgeJumpEndDistance;
        if (overrides.EdgeJumpHeight is { } edgeJumpHeight)
            settings.EdgeJumpHeight = edgeJumpHeight;
        if (overrides.EdgeJumpMaxDrop is { } edgeJumpMaxDrop)
            settings.EdgeJumpMaxDrop = edgeJumpMaxDrop;
        if (overrides.EdgeJumpMinDrop is { } edgeJumpMinDrop)
            settings.EdgeJumpMinDrop = edgeJumpMinDrop;
        if (overrides.GroundTileSize is { } groundTileSize)
            settings.GroundTileSize = groundTileSize;
        if (overrides.GroundTileCountMax is { } groundTileCountMax)
            settings.GroundTileCountMax = groundTileCountMax;
        if (overrides.VolumeCellSize is { } volumeCellSize)
            settings.VolumeCellSize = volumeCellSize;
    }

    public static void ApplySettings
    (
        DtNavMeshCreateParams config,
        CustomizationDraft    draft
    )
    {
        foreach (var connection in draft.OffMeshConnections)
        {
            if (!connection.Enabled)
                continue;

            config.AddOffMeshConnection
            (
                connection.Start,
                connection.End,
                connection.Radius,
                connection.Bidirectional,
                connection.UserId,
                connection.Area,
                connection.Flags,
                connection.Kind,
                connection.TraversalProfile
            );
        }
    }

    private static void ApplyColliderInsertion
    (
        SceneExtractor              scene,
        DraftSceneColliderInsertion insertion
    )
    {
        if (!insertion.Enabled)
            return;

        var (min, max) = NormalizeBounds(insertion.Min, insertion.Max);
        var bounds = new AABB
        {
            Min = min,
            Max = max
        };

        switch (insertion.Kind)
        {
            case DraftSceneColliderInsertionKind.Aabb:
                scene.InsertAABoxCollider(bounds, insertion.ForceSetPrimFlags, insertion.ForceClearPrimFlags);
                break;
            case DraftSceneColliderInsertionKind.Cylinder:
                scene.InsertCylinderCollider(bounds, insertion.ForceSetPrimFlags, insertion.ForceClearPrimFlags);
                break;
            case DraftSceneColliderInsertionKind.OrientedCylinder:
                scene.InsertOrientedCylinderCollider
                (
                    insertion.Start,
                    insertion.End,
                    insertion.Radius,
                    insertion.ForceSetPrimFlags,
                    insertion.ForceClearPrimFlags
                );
                break;
            case DraftSceneColliderInsertionKind.OrientedBox:
                scene.InsertOrientedBoxCollider
                (
                    (max - min) * 0.5f,
                    (min + max) * 0.5f,
                    insertion.RotationDegrees,
                    insertion.ForceSetPrimFlags,
                    insertion.ForceClearPrimFlags
                );
                break;
            case DraftSceneColliderInsertionKind.Sphere:
                scene.InsertSphereCollider(bounds, insertion.ForceSetPrimFlags, insertion.ForceClearPrimFlags);
                break;
            case DraftSceneColliderInsertionKind.Wall:
                var halfExtents = (max - min) * 0.5f;
                scene.InsertWallCollider
                (
                    new(halfExtents.X, halfExtents.Y),
                    (min + max) * 0.5f,
                    insertion.RotationDegrees,
                    insertion.DoubleSided,
                    insertion.ForceSetPrimFlags,
                    insertion.ForceClearPrimFlags
                );
                break;
            case DraftSceneColliderInsertionKind.Ramp:
                scene.InsertRampCollider
                (
                    (max - min) * 0.5f,
                    (min + max) * 0.5f,
                    insertion.RotationDegrees,
                    insertion.ForceSetPrimFlags,
                    insertion.ForceClearPrimFlags
                );
                break;
            case DraftSceneColliderInsertionKind.WalkableFloor:
                var floorHalfExtents = (max - min) * 0.5f;
                scene.InsertWalkableFloor
                (
                    new(floorHalfExtents.X, floorHalfExtents.Z),
                    (min + max) * 0.5f,
                    insertion.RotationDegrees,
                    insertion.ForceSetPrimFlags,
                    insertion.ForceClearPrimFlags
                );
                break;
            case DraftSceneColliderInsertionKind.RemoveInstances:
                scene.RemoveMeshInstancesInBounds(bounds, insertion.MeshKeyContains);
                break;
            case DraftSceneColliderInsertionKind.SetInstanceFlags:
                scene.SetMeshInstanceFlagsInBounds
                (
                    bounds,
                    insertion.ForceSetPrimFlags,
                    insertion.ForceClearPrimFlags,
                    insertion.MeshKeyContains
                );
                break;
        }
    }

    private static void ApplyInstancePatch
    (
        SceneExtractor          scene,
        DraftSceneInstancePatch patch
    )
    {
        if (!patch.Enabled || string.IsNullOrWhiteSpace(patch.MeshKey) || !scene.Meshes.TryGetValue(patch.MeshKey, out var mesh))
            return;

        if (patch.Kind == DraftSceneInstancePatchKind.Insert)
        {
            scene.InsertMeshInstances
            (
                patch.MeshKey,
                patch.WorldTransform.ToRuntime(),
                patch.Count,
                patch.Offset,
                patch.Material,
                patch.ForceSetPrimFlags,
                patch.ForceClearPrimFlags
            );
            return;
        }

        switch (patch.Kind)
        {
            case DraftSceneInstancePatchKind.ClearInstances:
                mesh.Instances.Clear();
                return;
            case DraftSceneInstancePatchKind.RemoveInstance:
            {
                var index = ResolveInstanceIndex(mesh, patch);
                if (index < 0)
                    return;
                mesh.Instances.RemoveAt(index);
                return;
            }
        }

        var targetIndex = ResolveInstanceIndex(mesh, patch);
        if (targetIndex < 0)
            return;

        var instance = mesh.Instances[targetIndex];

        if (patch.Kind == DraftSceneInstancePatchKind.Transform)
        {
            instance.WorldTransform = patch.WorldTransform.ToRuntime();
            instance.WorldBounds    = TransformBounds(instance.WorldTransform, mesh.LocalBounds);
        }
        else if (patch.Kind == DraftSceneInstancePatchKind.SetFlags)
        {
            instance.ForceSetPrimFlags   = patch.ForceSetPrimFlags;
            instance.ForceClearPrimFlags = patch.ForceClearPrimFlags;
        }
    }

    private static void ApplyPartPatch
    (
        SceneExtractor      scene,
        DraftScenePartPatch patch
    )
    {
        if (!patch.Enabled || string.IsNullOrWhiteSpace(patch.MeshKey) || !scene.Meshes.TryGetValue(patch.MeshKey, out var mesh))
            return;

        if (patch.PartIndex < 0 || patch.PartIndex >= mesh.Parts.Count)
            return;

        var part = mesh.Parts[patch.PartIndex];

        switch (patch.Kind)
        {
            case DraftScenePartPatchKind.Vertex:
            {
                if (patch.VertexIndex < 0 || patch.VertexIndex >= part.Vertices.Count)
                    return;

                part.Vertices[patch.VertexIndex] = patch.Position;
                part.LocalBounds                 = CalculateLocalBounds(part.Vertices);
                RecalculateMeshBounds(mesh);
                return;
            }
            case DraftScenePartPatchKind.PrimitiveFlags:
            {
                if (patch.PrimitiveIndex < 0 || patch.PrimitiveIndex >= part.Primitives.Count)
                    return;

                var primitive = part.Primitives[patch.PrimitiveIndex];
                primitive.Flags                       = patch.Flags;
                part.Primitives[patch.PrimitiveIndex] = primitive;
                return;
            }
            case DraftScenePartPatchKind.PrimitiveEdit:
            {
                if (patch.PrimitiveIndex < 0 || patch.PrimitiveIndex >= part.Primitives.Count)
                    return;

                part.Primitives[patch.PrimitiveIndex] = new
                (
                    patch.V1,
                    patch.V2,
                    patch.V3,
                    patch.Flags,
                    patch.Material
                );
                return;
            }
        }
    }

    private static int ResolveInstanceIndex
    (
        Mesh     mesh,
        DraftSceneInstancePatch patch
    )
    {
        if (patch.InstanceId != 0)
        {
            for (var i = 0; i < mesh.Instances.Count; ++i)
                if (mesh.Instances[i].ID == patch.InstanceId)
                    return i;
        }

        if (patch.InstanceIndex >= 0 && patch.InstanceIndex < mesh.Instances.Count)
            return patch.InstanceIndex;

        return -1;
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

    private static (Vector3 Min, Vector3 Max) NormalizeBounds
    (
        Vector3 a,
        Vector3 b
    ) =>
        (Vector3.Min(a, b), Vector3.Max(a, b));

    private static AABB CalculateLocalBounds
    (
        List<MeshPart> parts
    )
    {
        var bounds = new AABB
        {
            Min = new(float.MaxValue),
            Max = new(float.MinValue)
        };

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
        var bounds = new AABB
        {
            Min = new(float.MaxValue),
            Max = new(float.MinValue)
        };

        foreach (var vertex in vertices)
        {
            bounds.Min = Vector3.Min(bounds.Min, vertex);
            bounds.Max = Vector3.Max(bounds.Max, vertex);
        }

        return bounds;
    }
}
