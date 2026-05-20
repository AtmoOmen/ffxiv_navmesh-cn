using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Internal;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Editor;

internal static class CustomizationDraftSeedBuilder
{
    public static CustomizationDraft CreateFromCustomization(SceneDefinition scene, NavmeshCustomization customization, string territoryName, PluginConfig config)
    {
        var draft = new CustomizationDraft
        {
            TerritoryID   = scene.TerritoryID,
            TerritoryName = territoryName
        };

        var defaultCustomization = new NavmeshCustomization();
        var defaultSettings      = defaultCustomization.GetBuildSettings(scene);
        var customSettings       = customization.GetBuildSettings(scene);
        CopySettings(draft.BuildSettings, defaultSettings, customSettings);

        var defaultFlying = defaultCustomization.IsFlyingSupported(scene);
        var customFlying  = customization.IsFlyingSupported(scene);
        if (customFlying != defaultFlying)
            draft.FlyingSupportedOverride = customFlying;

        CopyBuildProfile(draft.BuildProfile, scene, customization);
        CopySceneDiffs(draft, scene, customization);
        CopyOffMeshConnections(draft, customization);
        return draft;
    }

    public static void CopyMeshLinksFromNavmesh(CustomizationDraft draft, Navmesh navmesh)
    {
        foreach (var link in navmesh.Links)
        {
            if (link.Kind is NavmeshOffMeshKind.GeneratedClimbDown or NavmeshOffMeshKind.GeneratedEdgeJump or NavmeshOffMeshKind.ManualOffMesh)
                continue;

            if (link.Kind is not (NavmeshOffMeshKind.Teleport or NavmeshOffMeshKind.ClientPath))
                continue;

            var kind = link.Kind == NavmeshOffMeshKind.ClientPath
                           ? DraftMeshLinkKind.ClientPath
                           : DraftMeshLinkKind.Points;

            draft.MeshLinks.Add
            (
                new()
                {
                    Kind             = kind,
                    Start            = link.Start,
                    End              = link.End,
                    Bidirectional    = link.Bidirectional,
                    TraversalProfile = link.TraversalProfile
                }
            );
        }
    }

    private static void CopyBuildProfile(DraftBuildProfileOverrides target, SceneDefinition scene, NavmeshCustomization customization)
    {
        var profile = new NavmeshBuildProfile();
        customization.CustomizeBuildProfile(scene, profile);

        if (profile.PartitioningOverride is { } partitioning)
            target.PartitioningOverride = partitioning;
        if (profile.CellSizeOverride is { } cellSize)
            target.CellSizeOverride = cellSize;
        if (profile.CellHeightOverride is { } cellHeight)
            target.CellHeightOverride = cellHeight;
        if (profile.RegionMinSizeOverride is { } regionMinSize)
            target.RegionMinSizeOverride = regionMinSize;
        if (profile.RegionMergeSizeOverride is { } regionMergeSize)
            target.RegionMergeSizeOverride = regionMergeSize;
        if (profile.PolyMaxEdgeLenOverride is { } polyMaxEdgeLen)
            target.PolyMaxEdgeLenOverride = polyMaxEdgeLen;
        if (profile.PolyMaxSimplificationErrorOverride is { } polyMaxSimplificationError)
            target.PolyMaxSimplificationErrorOverride = polyMaxSimplificationError;
        if (profile.AgentRadiusOverride is { } agentRadius)
            target.AgentRadiusOverride = agentRadius;
        if (profile.VolumeTilesOverride is { } volumeTiles)
            target.VolumeTilesOverride = (int[])volumeTiles.Clone();
        if (profile.DetailSampleDistOverride is { } detailSampleDist)
            target.DetailSampleDistOverride = detailSampleDist;
        if (profile.GenerateEdgeClimbLinksOverride is { } generateEdgeClimbLinks)
            target.GenerateEdgeClimbLinksOverride = generateEdgeClimbLinks;
        if (profile.GenerateEdgeJumpLinksOverride is { } generateEdgeJumpLinks)
            target.GenerateEdgeJumpLinksOverride = generateEdgeJumpLinks;
    }

    private static void CopySettings(DraftBuildSettingsOverrides target, NavmeshSettings defaults, NavmeshSettings current)
    {
        CopyIfChanged(current.CellSize, defaults.CellSize, v => target.CellSize = v);
        CopyIfChanged(current.CellHeight, defaults.CellHeight, v => target.CellHeight = v);
        CopyIfChanged(current.AgentHeight, defaults.AgentHeight, v => target.AgentHeight = v);
        CopyIfChanged(current.AgentRadius, defaults.AgentRadius, v => target.AgentRadius = v);
        CopyIfChanged(current.AgentMaxClimb, defaults.AgentMaxClimb, v => target.AgentMaxClimb = v);
        CopyIfChanged(current.AgentMaxSlopeDeg, defaults.AgentMaxSlopeDeg, v => target.AgentMaxSlopeDeg = v);
        if (current.Filtering != defaults.Filtering)
            target.Filtering = current.Filtering;
        CopyIfChanged(current.RegionMinSize, defaults.RegionMinSize, v => target.RegionMinSize = v);
        CopyIfChanged(current.RegionMergeSize, defaults.RegionMergeSize, v => target.RegionMergeSize = v);
        if (current.Partitioning != defaults.Partitioning)
            target.Partitioning = current.Partitioning;
        CopyIfChanged(current.PolyMaxEdgeLen, defaults.PolyMaxEdgeLen, v => target.PolyMaxEdgeLen = v);
        CopyIfChanged(current.PolyMaxSimplificationError, defaults.PolyMaxSimplificationError, v => target.PolyMaxSimplificationError = v);
        if (current.PolyMaxVerts != defaults.PolyMaxVerts)
            target.PolyMaxVerts = current.PolyMaxVerts;
        CopyIfChanged(current.DetailSampleDist, defaults.DetailSampleDist, v => target.DetailSampleDist = v);
        CopyIfChanged(current.DetailMaxSampleError, defaults.DetailMaxSampleError, v => target.DetailMaxSampleError = v);
        if (current.FastBuild != defaults.FastBuild)
            target.FastBuild = current.FastBuild;
        if (current.GenerateEdgeClimbLinks != defaults.GenerateEdgeClimbLinks)
            target.GenerateEdgeClimbLinks = current.GenerateEdgeClimbLinks;
        if (current.GenerateEdgeJumpLinks != defaults.GenerateEdgeJumpLinks)
            target.GenerateEdgeJumpLinks = current.GenerateEdgeJumpLinks;
        CopyIfChanged(current.GroundTolerance, defaults.GroundTolerance, v => target.GroundTolerance = v);
        CopyIfChanged(current.ClimbDownDistance, defaults.ClimbDownDistance, v => target.ClimbDownDistance = v);
        CopyIfChanged(current.ClimbDownMaxHeight, defaults.ClimbDownMaxHeight, v => target.ClimbDownMaxHeight = v);
        CopyIfChanged(current.ClimbDownMinHeight, defaults.ClimbDownMinHeight, v => target.ClimbDownMinHeight = v);
        CopyIfChanged(current.EdgeJumpEndDistance, defaults.EdgeJumpEndDistance, v => target.EdgeJumpEndDistance = v);
        CopyIfChanged(current.EdgeJumpHeight, defaults.EdgeJumpHeight, v => target.EdgeJumpHeight = v);
        CopyIfChanged(current.EdgeJumpMaxDrop, defaults.EdgeJumpMaxDrop, v => target.EdgeJumpMaxDrop = v);
        CopyIfChanged(current.EdgeJumpMinDrop, defaults.EdgeJumpMinDrop, v => target.EdgeJumpMinDrop = v);
        CopyIfChanged(current.GroundTileSize, defaults.GroundTileSize, v => target.GroundTileSize = v);
        if (current.GroundTileCountMax != defaults.GroundTileCountMax)
            target.GroundTileCountMax = current.GroundTileCountMax;
        if (!current.VolumeTiles.SequenceEqual(defaults.VolumeTiles))
            target.VolumeTiles = (int[])current.VolumeTiles.Clone();
    }

    private static void CopySceneDiffs(CustomizationDraft draft, SceneDefinition scene, NavmeshCustomization customization)
    {
        var defaultScene = new SceneExtractor(scene);
        var customScene   = new SceneExtractor(scene);
        customization.CustomizeScene(customScene);

        foreach (var key in defaultScene.Meshes.Keys.Where(key => !customScene.Meshes.ContainsKey(key)))
            draft.MeshRemovals.Add(new() { MeshKey = key });

        foreach (var (key, customMesh) in customScene.Meshes)
        {
            if (!defaultScene.Meshes.TryGetValue(key, out var defaultMesh))
                continue;

            CopyColliderInsertions(draft, key, defaultMesh, customMesh);
            CopyInstancePatches(draft, key, defaultMesh, customMesh);
            CopyPartPatches(draft, key, defaultMesh, customMesh);
        }
    }

    private static void CopyColliderInsertions(CustomizationDraft draft, string meshKey, SceneExtractor.Mesh defaultMesh, SceneExtractor.Mesh customMesh)
    {
        if (meshKey != "<box>" && meshKey != "<cylinder>")
            return;

        var baseIds = defaultMesh.Instances.Select(static instance => instance.Id).ToHashSet();
        foreach (var instance in customMesh.Instances.Where(instance => !baseIds.Contains(instance.Id)))
        {
            draft.ColliderInsertions.Add
            (
                new()
                {
                    Kind              = meshKey == "<cylinder>" ? DraftSceneColliderInsertionKind.Cylinder : DraftSceneColliderInsertionKind.Aabb,
                    Min               = instance.WorldBounds.Min,
                    Max               = instance.WorldBounds.Max,
                    ForceSetPrimFlags = instance.ForceSetPrimFlags,
                    ForceClearPrimFlags = instance.ForceClearPrimFlags
                }
            );
        }
    }

    private static void CopyInstancePatches(CustomizationDraft draft, string meshKey, SceneExtractor.Mesh defaultMesh, SceneExtractor.Mesh customMesh)
    {
        if (customMesh.Instances.Count == 0 && defaultMesh.Instances.Count > 0)
        {
            draft.InstancePatches.Add(new() { MeshKey = meshKey, Kind = DraftSceneInstancePatchKind.ClearInstances });
            return;
        }

        var customById = customMesh.Instances.ToDictionary(static instance => instance.Id);

        for (var i = 0; i < defaultMesh.Instances.Count; ++i)
        {
            var instance = defaultMesh.Instances[i];
            if (!customById.TryGetValue(instance.Id, out var customInstance))
            {
                draft.InstancePatches.Add
                (
                    new()
                    {
                        MeshKey       = meshKey,
                        Kind          = DraftSceneInstancePatchKind.RemoveInstance,
                        InstanceId    = instance.Id,
                        InstanceIndex = i
                    }
                );
                continue;
            }

            if (!TransformsEqual(instance.WorldTransform, customInstance.WorldTransform))
            {
                draft.InstancePatches.Add
                (
                    new()
                    {
                        MeshKey         = meshKey,
                        Kind            = DraftSceneInstancePatchKind.Transform,
                        InstanceId      = instance.Id,
                        InstanceIndex   = i,
                        WorldTransform  = DraftMatrix4x3.FromRuntime(customInstance.WorldTransform)
                    }
                );
            }

            if (instance.ForceSetPrimFlags != customInstance.ForceSetPrimFlags || instance.ForceClearPrimFlags != customInstance.ForceClearPrimFlags)
            {
                draft.InstancePatches.Add
                (
                    new()
                    {
                        MeshKey             = meshKey,
                        Kind                = DraftSceneInstancePatchKind.SetFlags,
                        InstanceId          = instance.Id,
                        InstanceIndex       = i,
                        ForceSetPrimFlags   = customInstance.ForceSetPrimFlags,
                        ForceClearPrimFlags = customInstance.ForceClearPrimFlags
                    }
                );
            }
        }
    }

    private static void CopyPartPatches(CustomizationDraft draft, string meshKey, SceneExtractor.Mesh defaultMesh, SceneExtractor.Mesh customMesh)
    {
        var partCount = Math.Min(defaultMesh.Parts.Count, customMesh.Parts.Count);

        for (var partIndex = 0; partIndex < partCount; ++partIndex)
        {
            var defaultPart = defaultMesh.Parts[partIndex];
            var customPart   = customMesh.Parts[partIndex];

            var vertexCount = Math.Min(defaultPart.Vertices.Count, customPart.Vertices.Count);
            for (var vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
            {
                if (defaultPart.Vertices[vertexIndex] == customPart.Vertices[vertexIndex])
                    continue;

                draft.PartPatches.Add
                (
                    new()
                    {
                        MeshKey     = meshKey,
                        PartIndex   = partIndex,
                        Kind        = DraftScenePartPatchKind.Vertex,
                        VertexIndex = vertexIndex,
                        Position    = customPart.Vertices[vertexIndex]
                    }
                );
            }

            var primitiveCount = Math.Min(defaultPart.Primitives.Count, customPart.Primitives.Count);
            for (var primitiveIndex = 0; primitiveIndex < primitiveCount; ++primitiveIndex)
            {
                var defaultPrimitive = defaultPart.Primitives[primitiveIndex];
                var customPrimitive  = customPart.Primitives[primitiveIndex];

                if (defaultPrimitive == customPrimitive)
                    continue;

                if (defaultPrimitive.V1 == customPrimitive.V1 &&
                    defaultPrimitive.V2 == customPrimitive.V2 &&
                    defaultPrimitive.V3 == customPrimitive.V3 &&
                    defaultPrimitive.Material == customPrimitive.Material)
                {
                    draft.PartPatches.Add
                    (
                        new()
                        {
                            MeshKey        = meshKey,
                            PartIndex      = partIndex,
                            Kind           = DraftScenePartPatchKind.PrimitiveFlags,
                            PrimitiveIndex = primitiveIndex,
                            Flags          = customPrimitive.Flags
                        }
                    );
                }
                else
                {
                    draft.PartPatches.Add
                    (
                        new()
                        {
                            MeshKey        = meshKey,
                            PartIndex      = partIndex,
                            Kind           = DraftScenePartPatchKind.PrimitiveEdit,
                            PrimitiveIndex = primitiveIndex,
                            V1             = customPrimitive.V1,
                            V2             = customPrimitive.V2,
                            V3             = customPrimitive.V3,
                            Flags          = customPrimitive.Flags,
                            Material       = customPrimitive.Material
                        }
                    );
                }
            }
        }
    }

    private static void CopyOffMeshConnections(CustomizationDraft draft, NavmeshCustomization customization)
    {
        var connections = OffMeshConnectionMetadataRegistry.Collect(customization);
        foreach (var connection in connections)
        {
            draft.OffMeshConnections.Add
            (
                new()
                {
                    Start            = connection.Start,
                    End              = connection.End,
                    Radius           = connection.Radius,
                    Bidirectional    = connection.Bidirectional,
                    UserId           = connection.UserId,
                    Area             = (NavmeshArea)connection.Area,
                    Flags            = (NavmeshPolyFlags)connection.Flags,
                    Kind             = (NavmeshOffMeshKind)connection.Kind,
                    TraversalProfile = connection.TraversalProfile
                }
            );
        }
    }

    private static bool TransformsEqual(Matrix4x3 left, Matrix4x3 right) =>
        left.Row0 == right.Row0 && left.Row1 == right.Row1 && left.Row2 == right.Row2 && left.Row3 == right.Row3;

    private static void CopyIfChanged<T>(T current, T defaults, Action<T> assign) where T : IEquatable<T>
    {
        if (!current.Equals(defaults))
            assign(current);
    }
}
