using System.Numerics;
using Dalamud.Bindings.ImGui;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Customizations.Editor;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Editor.Types;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorLeftPanel
{
    public delegate void AddMeshRemovalDelegate(string key);

    public delegate void AddInstancePatchDelegate(SceneExtractor.Mesh mesh, string key, int index, DraftSceneInstancePatchKind kind);

    public delegate void AddPartPatchDelegate(SceneExtractor.Mesh mesh, string key, int partIndex, DraftScenePartPatchKind kind, int subIndex = -1);

    public static void Draw
    (
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        ref Selection?                   pendingFocusSelection,
        CustomizationPreviewBuilder      previewBuilder,
        DebugGameCollision               collision,
        DebugDrawer                      dd,
        AddMeshRemovalDelegate           onAddMeshRemoval,
        AddInstancePatchDelegate         onAddInstancePatch,
        AddPartPatchDelegate             onAddPartPatch
    )
    {
        var focusSelection = pendingFocusSelection;
        var focusConsumed  = false;

        if (previewBuilder is { CurrentState: CustomizationPreviewBuilder.State.Ready, Extractor: not null })
        {
            if (ShouldAutoOpenPreviewRoot(focusSelection))
                ImGui.SetNextItemOpen(true);

            if (ImGui.TreeNodeEx("预览对象", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.TextDisabled("左键选择, 右键加入草稿补丁");
                DrawPreviewMeshes
                (
                    workspace,
                    previewBuilder.Extractor,
                    ref selection,
                    focusSelection,
                    ref focusConsumed,
                    collision,
                    dd,
                    onAddMeshRemoval,
                    onAddInstancePatch,
                    onAddPartPatch
                );
                ImGui.TreePop();
            }
        }

        if (ShouldAutoOpenDraftRoot(focusSelection))
            ImGui.SetNextItemOpen(true);

        if (ImGui.TreeNodeEx("草稿与设置", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftTree(ref selection, focusSelection, ref focusConsumed, workspace, previewBuilder, collision);
            ImGui.TreePop();
        }

        if (focusConsumed)
            pendingFocusSelection = null;
    }

    private static void DrawPreviewMeshes
    (
        CustomizationEditorWorkspace workspace,
        SceneExtractor               extractor,
        ref Selection                selection,
        Selection?                   focusSelection,
        ref bool                     focusConsumed,
        DebugGameCollision           collision,
        DebugDrawer                  dd,
        AddMeshRemovalDelegate       onAddMeshRemoval,
        AddInstancePatchDelegate     onAddInstancePatch,
        AddPartPatchDelegate         onAddPartPatch
    )
    {
        foreach (var (key, mesh) in extractor.Meshes.OrderBy(static x => x.Key, StringComparer.Ordinal))
        {
            var focusedPreviewInstance = TryGetFocusedPreviewInstance(focusSelection, key, out var focusedInstanceIndex);
            var visibleInstanceIndices = GetVisiblePreviewInstanceIndices(mesh, collision, focusedPreviewInstance, focusedInstanceIndex);
            if (visibleInstanceIndices.Count == 0)
                continue;

            var meshLabel    = $"{key} [{mesh.Parts.Count} parts, {visibleInstanceIndices.Count}/{mesh.Instances.Count} inst]";
            var meshSelected = selection is { Kind: SelectionKind.PreviewMesh, Key: var meshKey } && meshKey == key;
            if (focusedPreviewInstance)
                ImGui.SetNextItemOpen(true);

            var meshOpen     = ImGui.TreeNodeEx(meshLabel, meshSelected ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None);

            if (ImGui.IsItemClicked(ImGuiMouseButton.Left))
                selection = new(SelectionKind.PreviewMesh, Key: key);

            if (ImGui.BeginPopupContextItem($"##preview_mesh_popup_{key}"))
            {
                if (ImGui.MenuItem("加入 mesh 删除清单"))
                    onAddMeshRemoval(key);
                ImGui.EndPopup();
            }

            if (ImGui.IsItemHovered() && TryGetVisibleMeshBounds(mesh, visibleInstanceIndices, out var visibleMeshBounds))
                dd.DrawWorldAABB(visibleMeshBounds, 0xFF00FFFF);

            if (!meshOpen)
                continue;

            if (ImGui.TreeNodeEx($"组件##parts_{key}"))
            {
                var partIndex = 0;

                foreach (var part in mesh.Parts)
                {
                    var partLabel = $"Part {partIndex} [{part.Vertices.Count} v, {part.Primitives.Count} t]";
                    var partSelected = selection.Key   == key       &&
                                       selection.Index == partIndex &&
                                       selection.Kind is SelectionKind.PreviewPart or SelectionKind.PreviewVertex or SelectionKind.PreviewPrimitive;

                    var partOpen = ImGui.TreeNodeEx(partLabel, partSelected ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None);
                    if (ImGui.IsItemClicked(ImGuiMouseButton.Left))
                        selection = new(SelectionKind.PreviewPart, partIndex, Key: key);

                    if (ImGui.BeginPopupContextItem($"##preview_part_popup_{key}_{partIndex}"))
                    {
                        if (ImGui.MenuItem("加入顶点位置补丁"))
                            onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.Vertex);
                        if (ImGui.MenuItem("加入三角 flags 补丁"))
                            onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveFlags);
                        if (ImGui.MenuItem("加入三角高级编辑补丁"))
                            onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveEdit);
                        ImGui.EndPopup();
                    }

                    if (ImGui.IsItemHovered() && visibleInstanceIndices.Count > 0)
                        DrawMeshPreview(part, mesh.Instances[visibleInstanceIndices[0]].WorldTransform, dd);

                    if (partOpen)
                    {
                        if (ImGui.TreeNodeEx($"顶点##verts_{key}_{partIndex}", ImGuiTreeNodeFlags.DefaultOpen))
                        {
                            var vertexIndex = 0;

                            foreach (var vertex in part.Vertices)
                            {
                                var vertexLabel = $"[{vertexIndex}] {vertex:f3}";
                                var vertexSelected = selection is
                                                     {
                                                         Kind: SelectionKind.PreviewVertex, Key: var vertexKey, Index: var selectedPartIndex,
                                                         SubIndex: var selectedVertexIndex
                                                     }                                &&
                                                     vertexKey           == key       &&
                                                     selectedPartIndex   == partIndex &&
                                                     selectedVertexIndex == vertexIndex;
                                if (ImGui.Selectable(vertexLabel, vertexSelected))
                                    selection = new(SelectionKind.PreviewVertex, partIndex, vertexIndex, key);

                                if (ImGui.BeginPopupContextItem($"##preview_vertex_popup_{key}_{partIndex}_{vertexIndex}"))
                                {
                                    if (ImGui.MenuItem("加入顶点位置补丁"))
                                        onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.Vertex, vertexIndex);
                                    ImGui.EndPopup();
                                }

                                if (ImGui.IsItemHovered())
                                    DrawPreviewVertex(mesh, vertex, dd);
                                ++vertexIndex;
                            }

                            ImGui.TreePop();
                        }

                        if (ImGui.TreeNodeEx($"三角##prims_{key}_{partIndex}", ImGuiTreeNodeFlags.DefaultOpen))
                        {
                            var primIndex = 0;

                            foreach (var prim in part.Primitives)
                            {
                                var primLabel = $"[{primIndex}] {prim.V1}x{prim.V2}x{prim.V3} {prim.Flags}";
                                var primitiveSelected = selection is
                                                        {
                                                            Kind: SelectionKind.PreviewPrimitive, Key: var primitiveKey, Index: var selectedPartIndex,
                                                            SubIndex: var selectedPrimitiveIndex
                                                        }                                   &&
                                                        primitiveKey           == key       &&
                                                        selectedPartIndex      == partIndex &&
                                                        selectedPrimitiveIndex == primIndex;
                                if (ImGui.Selectable(primLabel, primitiveSelected))
                                    selection = new(SelectionKind.PreviewPrimitive, partIndex, primIndex, key);

                                if (ImGui.BeginPopupContextItem($"##preview_primitive_popup_{key}_{partIndex}_{primIndex}"))
                                {
                                    if (ImGui.MenuItem("加入三角 flags 补丁"))
                                        onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveFlags, primIndex);
                                    if (ImGui.MenuItem("加入三角高级编辑补丁"))
                                        onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveEdit, primIndex);
                                    ImGui.EndPopup();
                                }

                                if (ImGui.IsItemHovered())
                                    DrawPreviewPrimitive(mesh, part, prim, dd);
                                ++primIndex;
                            }

                            ImGui.TreePop();
                        }

                        ImGui.TreePop();
                    }

                    ++partIndex;
                }

                ImGui.TreePop();
            }

            if (focusedPreviewInstance)
                ImGui.SetNextItemOpen(true);

            if (ImGui.TreeNodeEx($"实例##inst_{key}"))
            {
                foreach (var instanceIndex in visibleInstanceIndices)
                {
                    var instance = mesh.Instances[instanceIndex];
                    var patchTag = BuildInstancePatchTag(workspace, key, instanceIndex, instance.Id);
                    var instanceLabel = $"[{instanceIndex}] {instance.Id:X} {instance.WorldBounds.Min:f1}-{instance.WorldBounds.Max:f1}{patchTag}";
                    var instanceSelected = selection is { Kind: SelectionKind.PreviewInstance, Key: var instanceKey, Index: var selectedIndex } &&
                                           instanceKey   == key                                                                                 &&
                                           selectedIndex == instanceIndex;

                    if (ImGui.Selectable(instanceLabel, instanceSelected))
                        selection = new(SelectionKind.PreviewInstance, instanceIndex, Key: key);

                    if (ImGui.BeginPopupContextItem($"##preview_instance_popup_{key}_{instanceIndex}"))
                    {
                        if (ImGui.MenuItem("加入实例变换补丁"))
                            onAddInstancePatch(mesh, key, instanceIndex, DraftSceneInstancePatchKind.Transform);
                        if (ImGui.MenuItem("加入实例 flags 补丁"))
                            onAddInstancePatch(mesh, key, instanceIndex, DraftSceneInstancePatchKind.SetFlags);
                        if (ImGui.MenuItem("移除这个实例"))
                            onAddInstancePatch(mesh, key, instanceIndex, DraftSceneInstancePatchKind.RemoveInstance);
                        if (ImGui.MenuItem("清空这个 mesh 的全部实例"))
                            onAddInstancePatch(mesh, key, instanceIndex, DraftSceneInstancePatchKind.ClearInstances);
                        ImGui.EndPopup();
                    }

                    if (ImGui.IsItemHovered())
                        dd.DrawWorldAABB(instance.WorldBounds, 0xFFFFAA00);

                    if (focusedPreviewInstance && instanceIndex == focusedInstanceIndex && !focusConsumed)
                    {
                        ImGui.SetScrollHereY();
                        focusConsumed = true;
                    }
                }

                ImGui.TreePop();
            }

            ImGui.TreePop();
        }
    }

    private static void DrawMeshPreview(SceneExtractor.MeshPart part, Matrix4x3 transform, DebugDrawer dd, uint color = 0xFF00FFAA)
    {
        foreach (var primitive in part.Primitives)
        {
            dd.DrawWorldTriangle
            (
                transform.TransformCoordinate(part.Vertices[primitive.V1]),
                transform.TransformCoordinate(part.Vertices[primitive.V2]),
                transform.TransformCoordinate(part.Vertices[primitive.V3]),
                color
            );
        }
    }

    private static void DrawPreviewVertex(SceneExtractor.Mesh mesh, Vector3 vertex, DebugDrawer dd)
    {
        if (mesh.Instances.Count == 0)
        {
            dd.DrawWorldPointFilled(vertex, 3, 0xFF00FF00);
            return;
        }

        dd.DrawWorldPointFilled(mesh.Instances[0].WorldTransform.TransformCoordinate(vertex), 3, 0xFF00FF00);
    }

    private static void DrawPreviewPrimitive(SceneExtractor.Mesh mesh, SceneExtractor.MeshPart part, SceneExtractor.Primitive primitive, DebugDrawer dd)
    {
        if (mesh.Instances.Count == 0)
            return;

        var transform = mesh.Instances[0].WorldTransform;
        dd.DrawWorldTriangle
        (
            transform.TransformCoordinate(part.Vertices[primitive.V1]),
            transform.TransformCoordinate(part.Vertices[primitive.V2]),
            transform.TransformCoordinate(part.Vertices[primitive.V3]),
            0xFF00FFAA
        );
    }

    private static void DrawDraftTree
    (
        ref Selection               selection,
        Selection?                  focusSelection,
        ref bool                    focusConsumed,
        CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder previewBuilder,
        DebugGameCollision          collision
    )
    {
        if (ImGui.Selectable("工作区", selection.Kind == SelectionKind.Workspace))
            selection = new(SelectionKind.Workspace);
        if (ImGui.Selectable("构建参数", selection.Kind == SelectionKind.BuildProfile))
            selection = new(SelectionKind.BuildProfile);
        if (ImGui.Selectable("构建设置", selection.Kind == SelectionKind.BuildSettings))
            selection = new(SelectionKind.BuildSettings);
        if (ImGui.Selectable("飞行支持", selection.Kind == SelectionKind.FlyingOverride))
            selection = new(SelectionKind.FlyingOverride);
        if (ImGui.Selectable("诊断", selection.Kind == SelectionKind.Diagnostics))
            selection = new(SelectionKind.Diagnostics);

        if (ShouldAutoOpenGeometrySection(focusSelection))
            ImGui.SetNextItemOpen(true);

        if (ImGui.TreeNodeEx("场景几何", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems
            (
                "mesh 删除",
                BuildMeshRemovalEntries(workspace, previewBuilder, collision),
                SelectionKind.MeshRemoval,
                ref selection,
                focusSelection,
                ref focusConsumed
            );
            DrawDraftItems
            (
                "实例补丁",
                BuildInstancePatchEntries(workspace, previewBuilder, collision),
                SelectionKind.InstancePatch,
                ref selection,
                focusSelection,
                ref focusConsumed
            );
            DrawDraftItems
            (
                "顶点 / 三角补丁",
                BuildPartPatchEntries(workspace, previewBuilder, collision),
                SelectionKind.PartPatch,
                ref selection,
                focusSelection,
                ref focusConsumed
            );
            DrawDraftItems
            (
                "碰撞插入",
                BuildColliderInsertionEntries(workspace, collision),
                SelectionKind.ColliderInsertion,
                ref selection,
                focusSelection,
                ref focusConsumed
            );
            ImGui.TreePop();
        }

        if (ShouldAutoOpenConnectivitySection(focusSelection))
            ImGui.SetNextItemOpen(true);

        if (ImGui.TreeNodeEx("连通规则", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems
            (
                "mesh link",
                BuildMeshLinkEntries(workspace, collision),
                SelectionKind.MeshLink,
                ref selection,
                focusSelection,
                ref focusConsumed
            );
            DrawDraftItems
            (
                "off-mesh 连接",
                BuildOffMeshConnectionEntries(workspace, collision),
                SelectionKind.OffMeshConnection,
                ref selection,
                focusSelection,
                ref focusConsumed
            );
            ImGui.TreePop();
        }
    }

    private static void DrawDraftItems
    (
        string                   title,
        List<DraftListEntry>     items,
        SelectionKind            kind,
        ref Selection            selection,
        Selection?               focusSelection,
        ref bool                 focusConsumed
    )
    {
        if (!ImGui.TreeNodeEx($"{title} ({items.Count})", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        foreach (var item in items)
        {
            if (!item.IsInRange)
                ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyle().Colors[(int)ImGuiCol.TextDisabled]);

            var label = $"[{item.Index}] {item.Label}";
            if (ImGui.Selectable(label, selection.Kind == kind && selection.Index == item.Index))
                selection = new(kind, item.Index);

            if (!item.IsInRange)
                ImGui.PopStyleColor();

            if (!focusConsumed && focusSelection is { Kind: var focusKind, Index: var focusIndex } && focusKind == kind && focusIndex == item.Index)
            {
                ImGui.SetScrollHereY();
                focusConsumed = true;
            }
        }

        ImGui.TreePop();
    }

    private static List<DraftListEntry> BuildMeshRemovalEntries
        (CustomizationEditorWorkspace workspace, CustomizationPreviewBuilder previewBuilder, DebugGameCollision collision)
    {
        List<DraftListEntry> entries = [];
        for (var i = 0; i < workspace.Draft.MeshRemovals.Count; ++i)
        {
            var item = workspace.Draft.MeshRemovals[i];
            var info = DescribeMeshRemoval(previewBuilder, collision, item);
            entries.Add(CreateDraftEntry(i, FormatMeshRemovalLabel(item), item.Note, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildInstancePatchEntries
        (CustomizationEditorWorkspace workspace, CustomizationPreviewBuilder previewBuilder, DebugGameCollision collision)
    {
        List<DraftListEntry> entries = [];
        for (var i = 0; i < workspace.Draft.InstancePatches.Count; ++i)
        {
            var item = workspace.Draft.InstancePatches[i];
            var info = DescribeInstancePatch(previewBuilder, collision, item);
            entries.Add(CreateDraftEntry(i, FormatInstancePatchLabel(item), item.Note, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildPartPatchEntries
        (CustomizationEditorWorkspace workspace, CustomizationPreviewBuilder previewBuilder, DebugGameCollision collision)
    {
        List<DraftListEntry> entries = [];
        for (var i = 0; i < workspace.Draft.PartPatches.Count; ++i)
        {
            var item = workspace.Draft.PartPatches[i];
            var info = DescribePartPatch(previewBuilder, collision, item);
            entries.Add(CreateDraftEntry(i, $"{item.MeshKey} p{item.PartIndex} {item.Kind}", item.Note, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildColliderInsertionEntries(CustomizationEditorWorkspace workspace, DebugGameCollision collision)
    {
        List<DraftListEntry> entries = [];
        for (var i = 0; i < workspace.Draft.ColliderInsertions.Count; ++i)
        {
            var item  = workspace.Draft.ColliderInsertions[i];
            var info  = DescribeBounds(collision, CustomizationEditorSpatial.CreateBounds(item.Min, item.Max));
            entries.Add(CreateDraftEntry(i, $"{item.Kind} {item.Min:f1} -> {item.Max:f1}", item.Note, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildMeshLinkEntries(CustomizationEditorWorkspace workspace, DebugGameCollision collision)
    {
        List<DraftListEntry> entries = [];
        for (var i = 0; i < workspace.Draft.MeshLinks.Count; ++i)
        {
            var item = workspace.Draft.MeshLinks[i];
            var info = DescribeBounds(collision, CustomizationEditorSpatial.CreateBounds(item.Start, item.End));
            entries.Add(CreateDraftEntry(i, $"{item.Kind} {item.Start:f1} -> {item.End:f1}", item.Note, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildOffMeshConnectionEntries(CustomizationEditorWorkspace workspace, DebugGameCollision collision)
    {
        List<DraftListEntry> entries = [];
        for (var i = 0; i < workspace.Draft.OffMeshConnections.Count; ++i)
        {
            var item = workspace.Draft.OffMeshConnections[i];
            var info = DescribeBounds(collision, CustomizationEditorSpatial.CreateBounds(item.Start, item.End));
            entries.Add(CreateDraftEntry(i, $"{item.Kind} {item.Start:f1} -> {item.End:f1}", item.Note, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static DraftDistanceInfo DescribeMeshRemoval
        (CustomizationPreviewBuilder previewBuilder, DebugGameCollision collision, DraftSceneMeshRemoval item)
    {
        if (previewBuilder.Extractor == null ||
            !previewBuilder.Extractor.Meshes.TryGetValue(item.MeshKey, out var mesh) ||
            !TryGetNearestInstanceBounds(mesh, collision, out var bounds))
        {
            return DraftDistanceInfo.Unknown;
        }

        return DescribeBounds(collision, bounds);
    }

    private static DraftDistanceInfo DescribeInstancePatch
        (CustomizationPreviewBuilder previewBuilder, DebugGameCollision collision, DraftSceneInstancePatch item)
    {
        if (previewBuilder.Extractor == null ||
            !previewBuilder.Extractor.Meshes.TryGetValue(item.MeshKey, out var mesh) ||
            !TryGetInstancePatchBounds(mesh, item, out var bounds))
        {
            return DraftDistanceInfo.Unknown;
        }

        return DescribeBounds(collision, bounds);
    }

    private static DraftDistanceInfo DescribePartPatch
        (CustomizationPreviewBuilder previewBuilder, DebugGameCollision collision, DraftScenePartPatch item)
    {
        if (previewBuilder.Extractor == null ||
            !previewBuilder.Extractor.Meshes.TryGetValue(item.MeshKey, out var mesh) ||
            !TryGetNearestInstanceBounds(mesh, collision, out var bounds))
        {
            return DraftDistanceInfo.Unknown;
        }

        return DescribeBounds(collision, bounds);
    }

    private static DraftDistanceInfo DescribeBounds(DebugGameCollision collision, AABB bounds) =>
        collision.HasRenderDistanceReferencePosition
            ? new(collision.GetHorizontalDistanceToBounds(bounds), collision.IsBoundsWithinEditorRenderDistance(bounds))
            : DraftDistanceInfo.Unknown;

    private static DraftListEntry CreateDraftEntry(int index, string label, string note, DraftDistanceInfo info) =>
        new(index, AppendNote(label, note), info.Distance, info.IsInRange);

    private static void SortDraftEntries(List<DraftListEntry> entries) =>
        entries.Sort
        (
            static (a, b) =>
            {
                if (a.Distance == null && b.Distance == null)
                    return a.Index.CompareTo(b.Index);
                if (a.Distance == null)
                    return 1;
                if (b.Distance == null)
                    return -1;

                var cmp = a.Distance.Value.CompareTo(b.Distance.Value);
                return cmp != 0 ? cmp : a.Index.CompareTo(b.Index);
            }
        );

    private static string AppendNote(string label, string note) =>
        string.IsNullOrWhiteSpace(note) ? label : $"{label} - {note.Trim()}";

    private static string FormatMeshRemovalLabel(DraftSceneMeshRemoval item) =>
        item.Enabled ? item.MeshKey : $"{item.MeshKey} (off)";

    private static string FormatInstancePatchLabel(DraftSceneInstancePatch item) =>
        item.Kind == DraftSceneInstancePatchKind.ClearInstances
            ? $"{item.MeshKey} {item.Kind}"
            : $"{item.MeshKey} #{item.InstanceIndex} {item.Kind}";

    private static bool TryGetNearestInstanceBounds(SceneExtractor.Mesh mesh, DebugGameCollision collision, out AABB bounds)
    {
        if (mesh.Instances.Count == 0)
        {
            bounds = default;
            return false;
        }

        bounds = mesh.Instances[0].WorldBounds;
        if (!collision.HasRenderDistanceReferencePosition)
            return true;

        var bestDistance = collision.GetHorizontalDistanceToBounds(bounds);
        for (var i = 1; i < mesh.Instances.Count; ++i)
        {
            var candidate = mesh.Instances[i].WorldBounds;
            var distance  = collision.GetHorizontalDistanceToBounds(candidate);
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bounds       = candidate;
        }

        return true;
    }

    private static bool TryGetInstancePatchBounds(SceneExtractor.Mesh mesh, DraftSceneInstancePatch patch, out AABB bounds)
    {
        if (patch.Kind == DraftSceneInstancePatchKind.ClearInstances)
        {
            bounds = default;
            return CustomizationEditorSpatial.TryUnionBounds(mesh.Instances.Select(static x => x.WorldBounds), out bounds);
        }

        if (patch.Kind == DraftSceneInstancePatchKind.Transform)
        {
            bounds = CustomizationEditorSpatial.CalculateTransformedBounds(mesh.LocalBounds, patch.WorldTransform.ToRuntime());
            return true;
        }

        if (TryResolveInstance(mesh, patch, out var instance))
        {
            bounds = instance.WorldBounds;
            return true;
        }

        bounds = default;
        return false;
    }

    private static bool TryResolveInstance(SceneExtractor.Mesh mesh, DraftSceneInstancePatch patch, out SceneExtractor.MeshInstance instance)
    {
        if (patch.InstanceId != 0)
        {
            instance = mesh.Instances.FirstOrDefault(x => x.Id == patch.InstanceId)!;
            if (instance != null)
                return true;
        }

        if (patch.InstanceIndex >= 0 && patch.InstanceIndex < mesh.Instances.Count)
        {
            instance = mesh.Instances[patch.InstanceIndex];
            return true;
        }

        instance = null!;
        return false;
    }

    private static List<int> GetVisiblePreviewInstanceIndices(SceneExtractor.Mesh mesh, DebugGameCollision collision, bool forceFocused, int focusedInstanceIndex)
    {
        List<int> visibleIndices = [];

        for (var i = 0; i < mesh.Instances.Count; ++i)
        {
            var visible = collision.IsBoundsWithinEditorRenderDistance(mesh.Instances[i].WorldBounds);
            if (visible || forceFocused && i == focusedInstanceIndex)
                visibleIndices.Add(i);
        }

        return visibleIndices;
    }

    private static bool TryGetVisibleMeshBounds(SceneExtractor.Mesh mesh, List<int> visibleInstanceIndices, out AABB bounds)
    {
        if (visibleInstanceIndices.Count == 0)
        {
            bounds = default;
            return false;
        }

        bounds = mesh.Instances[visibleInstanceIndices[0]].WorldBounds;
        for (var i = 1; i < visibleInstanceIndices.Count; ++i)
        {
            var candidate = mesh.Instances[visibleInstanceIndices[i]].WorldBounds;
            bounds.Min = Vector3.Min(bounds.Min, candidate.Min);
            bounds.Max = Vector3.Max(bounds.Max, candidate.Max);
        }

        return true;
    }

    private static bool TryGetFocusedPreviewInstance(Selection? focusSelection, string meshKey, out int focusedInstanceIndex)
    {
        if (focusSelection is { Kind: SelectionKind.PreviewInstance, Key: not null, Index: >= 0 } && focusSelection.Key == meshKey)
        {
            focusedInstanceIndex = focusSelection.Index;
            return true;
        }

        focusedInstanceIndex = -1;
        return false;
    }

    private static bool ShouldAutoOpenPreviewRoot(Selection? focusSelection) =>
        focusSelection?.Kind == SelectionKind.PreviewInstance;

    private static bool ShouldAutoOpenDraftRoot(Selection? focusSelection) =>
        focusSelection != null && IsDraftSelectionKind(focusSelection.Kind);

    private static bool ShouldAutoOpenGeometrySection(Selection? focusSelection) =>
        focusSelection is { Kind: SelectionKind.MeshRemoval or SelectionKind.InstancePatch or SelectionKind.PartPatch or SelectionKind.ColliderInsertion };

    private static bool ShouldAutoOpenConnectivitySection(Selection? focusSelection) =>
        focusSelection is { Kind: SelectionKind.MeshLink or SelectionKind.OffMeshConnection };

    private static bool IsDraftSelectionKind(SelectionKind kind) =>
        kind is SelectionKind.MeshRemoval or SelectionKind.InstancePatch or SelectionKind.PartPatch or SelectionKind.ColliderInsertion or SelectionKind.MeshLink or SelectionKind.OffMeshConnection;

    private static string BuildInstancePatchTag(CustomizationEditorWorkspace workspace, string meshKey, int instanceIndex, ulong instanceId)
    {
        List<string> tags = [];

        foreach (var patch in workspace.Draft.InstancePatches.Where(x => x.Enabled && x.MeshKey == meshKey))
        {
            var matches = patch.Kind == DraftSceneInstancePatchKind.ClearInstances ||
                          patch.InstanceId != 0
                              ? patch.InstanceId == instanceId
                              : patch.InstanceIndex == instanceIndex;
            if (!matches && patch.Kind != DraftSceneInstancePatchKind.ClearInstances)
                continue;

            switch (patch.Kind)
            {
                case DraftSceneInstancePatchKind.ClearInstances:
                    if (!tags.Contains("清空"))
                        tags.Add("清空");
                    break;
                case DraftSceneInstancePatchKind.RemoveInstance:
                    if (!tags.Contains("移除"))
                        tags.Add("移除");
                    break;
                case DraftSceneInstancePatchKind.Transform:
                    if (!tags.Contains("Transform"))
                        tags.Add("Transform");
                    break;
                case DraftSceneInstancePatchKind.SetFlags:
                    if (!tags.Contains("Flags"))
                        tags.Add("Flags");
                    break;
            }
        }

        return tags.Count == 0 ? string.Empty : $" [{string.Join(", ", tags)}]";
    }

    private readonly record struct DraftListEntry(int Index, string Label, float? Distance, bool IsInRange);

    private readonly record struct DraftDistanceInfo(float? Distance, bool IsInRange)
    {
        public static DraftDistanceInfo Unknown => new(null, true);
    }
}
