using System.Numerics;
using Dalamud.Bindings.ImGui;
using vnavmesh.Build.Custom.Editor;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Models;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Editor.Types;
using AABB = vnavmesh.Common.Models.AABB;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorLeftPanel
{
    public delegate void AddMeshRemovalDelegate
    (
        string key
    );

    public delegate void AddInstancePatchDelegate
    (
        Mesh         mesh,
        string                      key,
        int                         index,
        DraftSceneInstancePatchKind kind
    );

    public delegate void AddPartPatchDelegate
    (
        Mesh     mesh,
        string                  key,
        int                     partIndex,
        DraftScenePartPatchKind kind,
        int                     subIndex = -1
    );

    public static void Draw
    (
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        ref Selection?                   pendingFocusSelection,
        ref EditorPanelMode              panelMode,
        ref string                       searchText,
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

        if (focusSelection != null)
        {
            panelMode = IsPreviewSelectionKind(focusSelection.Kind) ? EditorPanelMode.Scene : EditorPanelMode.Draft;
            searchText = string.Empty;
        }

        var panelButtonWidth = (ImGui.GetContentRegionAvail().X - ImGui.GetStyle().ItemSpacing.X) * 0.5f;
        DrawPanelModeButton("草稿", EditorPanelMode.Draft, panelButtonWidth, ref panelMode);
        ImGui.SameLine();
        DrawPanelModeButton("场景", EditorPanelMode.Scene, panelButtonWidth, ref panelMode);

        ImGui.SetNextItemWidth(-1);
        ImGui.InputTextWithHint("##customization_editor_search", "搜索名称、编号或备注", ref searchText);
        ImGui.Separator();

        var query = searchText.Trim();

        if (panelMode == EditorPanelMode.Draft)
        {
            DrawDraftTree(ref selection, focusSelection, ref focusConsumed, workspace, previewBuilder, collision, query);
        }
        else if (previewBuilder is { CurrentState: CustomizationPreviewBuilder.State.Ready, Extractor: not null })
        {
            DrawPreviewMeshes
            (
                workspace,
                previewBuilder.Extractor,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query,
                collision,
                dd,
                onAddMeshRemoval,
                onAddInstancePatch,
                onAddPartPatch
            );
        }
        else
        {
            ImGui.TextDisabled($"场景预览 · {CustomizationEditorWidgets.FormatPreviewStateDisplayName(previewBuilder.CurrentState)}");
        }

        if (focusConsumed)
            pendingFocusSelection = null;
    }

    private static void DrawPanelModeButton
    (
        string              label,
        EditorPanelMode     mode,
        float               width,
        ref EditorPanelMode panelMode
    )
    {
        var active = panelMode == mode;

        if (active)
            ImGui.PushStyleColor(ImGuiCol.Button, ImGui.GetStyle().Colors[(int)ImGuiCol.Header]);

        if (ImGui.Button($"{label}##panel_{mode}", new Vector2(width, 0)))
            panelMode = mode;

        if (active)
            ImGui.PopStyleColor();
    }

    private static void DrawPreviewMeshes
    (
        CustomizationEditorWorkspace workspace,
        SceneExtractor               extractor,
        ref Selection                selection,
        Selection?                   focusSelection,
        ref bool                     focusConsumed,
        string                       query,
        DebugGameCollision           collision,
        DebugDrawer                  dd,
        AddMeshRemovalDelegate       onAddMeshRemoval,
        AddInstancePatchDelegate     onAddInstancePatch,
        AddPartPatchDelegate         onAddPartPatch
    )
    {
        foreach (var (key, mesh) in extractor.Meshes.OrderBy(static x => x.Key, StringComparer.Ordinal))
        {
            if (!MatchesSearch(query, key))
                continue;

            var focusedPreview         = IsFocusedPreviewSelection(focusSelection, key);
            var focusedPreviewInstance = TryGetFocusedPreviewInstance(focusSelection, key, out var focusedInstanceIndex);
            var visibleInstanceIndices = GetVisiblePreviewInstanceIndices(mesh, collision, focusedPreviewInstance, focusedInstanceIndex);
            if (visibleInstanceIndices.Count == 0)
                continue;

            var meshLabel    = $"{key} · {mesh.Parts.Count} 部件 · {visibleInstanceIndices.Count}/{mesh.Instances.Count} 实例";
            var meshSelected = selection is { Kind: SelectionKind.PreviewMesh, Key: var meshKey } && meshKey == key;
            if (focusedPreview)
                ImGui.SetNextItemOpen(true);

            var meshOpen = ImGui.TreeNodeEx
            (
                meshLabel,
                meshSelected ?
                    ImGuiTreeNodeFlags.Selected :
                    ImGuiTreeNodeFlags.None
            );

            if (ImGui.IsItemClicked(ImGuiMouseButton.Left))
                selection = new(SelectionKind.PreviewMesh, Key: key);

            if (ImGui.BeginPopupContextItem($"##preview_mesh_popup_{key}"))
            {
                if (ImGui.MenuItem("加入 mesh 删除清单"))
                    onAddMeshRemoval(key);
                ImGui.EndPopup();
            }

            if (ImGui.IsItemHovered() && TryGetVisibleMeshBounds(mesh, visibleInstanceIndices, out var visibleMeshBounds))
                dd.DrawWorldAABB(visibleMeshBounds.ToGame(), 0xFF00FFFF);

            if (!meshOpen)
                continue;

            var focusedPartIndex = GetFocusedPreviewPartIndex(focusSelection, key);
            if (focusedPartIndex >= 0)
                ImGui.SetNextItemOpen(true);

            if (ImGui.TreeNodeEx($"部件##parts_{key}"))
            {
                var partIndex = 0;

                foreach (var part in mesh.Parts)
                {
                    var partLabel = $"部件 {partIndex} · {part.Vertices.Count} 顶点 · {part.Primitives.Count} 三角";
                    var partSelected = selection.Key   == key       &&
                                       selection.Index == partIndex &&
                                       selection.Kind is SelectionKind.PreviewPart or SelectionKind.PreviewVertex or SelectionKind.PreviewPrimitive;

                    if (partIndex == focusedPartIndex)
                        ImGui.SetNextItemOpen(true);

                    var partOpen = ImGui.TreeNodeEx
                    (
                        partLabel,
                        partSelected ?
                            ImGuiTreeNodeFlags.Selected :
                            ImGuiTreeNodeFlags.None
                    );
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
                        var focusVertex = focusSelection is
                                          {
                                              Kind: SelectionKind.PreviewVertex, Key: var vertexKey, Index: var vertexPartIndex
                                          }                               &&
                                          vertexKey       == key           &&
                                          vertexPartIndex == partIndex;
                        if (focusVertex)
                            ImGui.SetNextItemOpen(true);

                        if (ImGui.TreeNodeEx($"顶点 ({part.Vertices.Count})##verts_{key}_{partIndex}"))
                        {
                            DrawPreviewVertices
                            (
                                mesh,
                                part,
                                key,
                                partIndex,
                                visibleInstanceIndices[0],
                                ref selection,
                                focusSelection,
                                ref focusConsumed,
                                dd,
                                onAddPartPatch
                            );
                            ImGui.TreePop();
                        }

                        var focusPrimitive = focusSelection is
                                             {
                                                 Kind: SelectionKind.PreviewPrimitive, Key: var primitiveKey, Index: var primitivePartIndex
                                             }                                  &&
                                             primitiveKey       == key           &&
                                             primitivePartIndex == partIndex;
                        if (focusPrimitive)
                            ImGui.SetNextItemOpen(true);

                        if (ImGui.TreeNodeEx($"三角 ({part.Primitives.Count})##prims_{key}_{partIndex}"))
                        {
                            DrawPreviewPrimitives
                            (
                                mesh,
                                part,
                                key,
                                partIndex,
                                visibleInstanceIndices[0],
                                ref selection,
                                focusSelection,
                                ref focusConsumed,
                                dd,
                                onAddPartPatch
                            );
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
                    var instance      = mesh.Instances[instanceIndex];
                    var patchTag      = BuildInstancePatchTag(workspace, key, instanceIndex, instance.ID);
                    var instanceLabel = $"[{instanceIndex}] {instance.ID:X} {instance.WorldBounds.Min:f1}-{instance.WorldBounds.Max:f1}{patchTag}";
                    var instanceSelected = selection is { Kind: SelectionKind.PreviewInstance, Key: var instanceKey, Index: var selectedIndex } &&
                                           instanceKey   == key                                                                                 &&
                                           selectedIndex == instanceIndex;

                    if (ImGui.Selectable(instanceLabel, instanceSelected))
                        selection = new(SelectionKind.PreviewInstance, instanceIndex, Key: key);

                    if (ImGui.BeginPopupContextItem($"##preview_instance_popup_{key}_{instanceIndex}"))
                    {
                        if (ImGui.MenuItem("复制为新实例"))
                            onAddInstancePatch(mesh, key, instanceIndex, DraftSceneInstancePatchKind.Insert);
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
                        dd.DrawWorldAABB(instance.WorldBounds.ToGame(), 0xFFFFAA00);

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

    private static void DrawMeshPreview
    (
        MeshPart part,
        Matrix4x3               transform,
        DebugDrawer             dd,
        uint                    color = 0xFF00FFAA
    )
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

    private static unsafe void DrawPreviewVertices
    (
        Mesh          mesh,
        MeshPart      part,
        string                       key,
        int                          partIndex,
        int                          instanceIndex,
        ref Selection                selection,
        Selection?                   focusSelection,
        ref bool                     focusConsumed,
        DebugDrawer                  dd,
        AddPartPatchDelegate         onAddPartPatch
    )
    {
        var clipper = ImGui.ImGuiListClipper();

        try
        {
            clipper.Begin(part.Vertices.Count);

            if (focusSelection is { Kind: SelectionKind.PreviewVertex, SubIndex: >= 0 } &&
                focusSelection.Key   == key                                                     &&
                focusSelection.Index == partIndex)
                clipper.ForceDisplayRangeByIndices(focusSelection.SubIndex, focusSelection.SubIndex + 1);

            while (clipper.Step())
            {
                for (var vertexIndex = clipper.DisplayStart; vertexIndex < clipper.DisplayEnd; ++vertexIndex)
                {
                    var vertex = part.Vertices[vertexIndex];
                    var vertexSelected = selection is
                                         {
                                             Kind: SelectionKind.PreviewVertex, Key: var vertexKey, Index: var selectedPartIndex,
                                             SubIndex: var selectedVertexIndex
                                         }                                &&
                                         vertexKey           == key       &&
                                         selectedPartIndex   == partIndex &&
                                         selectedVertexIndex == vertexIndex;

                    if (ImGui.Selectable($"[{vertexIndex}] {vertex:f3}", vertexSelected))
                        selection = new(SelectionKind.PreviewVertex, partIndex, vertexIndex, key);

                    if (ImGui.BeginPopupContextItem($"##preview_vertex_popup_{key}_{partIndex}_{vertexIndex}"))
                    {
                        if (ImGui.MenuItem("加入顶点位置补丁"))
                            onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.Vertex, vertexIndex);
                        ImGui.EndPopup();
                    }

                    if (ImGui.IsItemHovered())
                        DrawPreviewVertex(mesh, instanceIndex, vertex, dd);

                    if (!focusConsumed && focusSelection is
                        {
                            Kind: SelectionKind.PreviewVertex, Key: var focusKey, Index: var focusPartIndex, SubIndex: var focusVertexIndex
                        }                                 &&
                        focusKey         == key           &&
                        focusPartIndex   == partIndex     &&
                        focusVertexIndex == vertexIndex)
                    {
                        ImGui.SetScrollHereY();
                        focusConsumed = true;
                    }
                }
            }
        }
        finally
        {
            clipper.Destroy();
        }
    }

    private static unsafe void DrawPreviewPrimitives
    (
        Mesh          mesh,
        MeshPart      part,
        string                       key,
        int                          partIndex,
        int                          instanceIndex,
        ref Selection                selection,
        Selection?                   focusSelection,
        ref bool                     focusConsumed,
        DebugDrawer                  dd,
        AddPartPatchDelegate         onAddPartPatch
    )
    {
        var clipper = ImGui.ImGuiListClipper();

        try
        {
            clipper.Begin(part.Primitives.Count);

            if (focusSelection is { Kind: SelectionKind.PreviewPrimitive, SubIndex: >= 0 } &&
                focusSelection.Key   == key                                                        &&
                focusSelection.Index == partIndex)
                clipper.ForceDisplayRangeByIndices(focusSelection.SubIndex, focusSelection.SubIndex + 1);

            while (clipper.Step())
            {
                for (var primitiveIndex = clipper.DisplayStart; primitiveIndex < clipper.DisplayEnd; ++primitiveIndex)
                {
                    var primitive = part.Primitives[primitiveIndex];
                    var primitiveSelected = selection is
                                            {
                                                Kind: SelectionKind.PreviewPrimitive, Key: var primitiveKey, Index: var selectedPartIndex,
                                                SubIndex: var selectedPrimitiveIndex
                                            }                                   &&
                                            primitiveKey           == key       &&
                                            selectedPartIndex      == partIndex &&
                                            selectedPrimitiveIndex == primitiveIndex;

                    var label = $"[{primitiveIndex}] {primitive.V1} / {primitive.V2} / {primitive.V3} · {primitive.Flags}";
                    if (ImGui.Selectable(label, primitiveSelected))
                        selection = new(SelectionKind.PreviewPrimitive, partIndex, primitiveIndex, key);

                    if (ImGui.BeginPopupContextItem($"##preview_primitive_popup_{key}_{partIndex}_{primitiveIndex}"))
                    {
                        if (ImGui.MenuItem("加入三角标记补丁"))
                            onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveFlags, primitiveIndex);
                        if (ImGui.MenuItem("加入三角编辑补丁"))
                            onAddPartPatch(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveEdit, primitiveIndex);
                        ImGui.EndPopup();
                    }

                    if (ImGui.IsItemHovered())
                        DrawPreviewPrimitive(mesh, part, instanceIndex, primitive, dd);

                    if (!focusConsumed && focusSelection is
                        {
                            Kind: SelectionKind.PreviewPrimitive, Key: var focusKey, Index: var focusPartIndex,
                            SubIndex: var focusPrimitiveIndex
                        }                                    &&
                        focusKey            == key           &&
                        focusPartIndex      == partIndex     &&
                        focusPrimitiveIndex == primitiveIndex)
                    {
                        ImGui.SetScrollHereY();
                        focusConsumed = true;
                    }
                }
            }
        }
        finally
        {
            clipper.Destroy();
        }
    }

    private static void DrawPreviewVertex
    (
        Mesh mesh,
        int                 instanceIndex,
        Vector3             vertex,
        DebugDrawer         dd
    )
    {
        if (mesh.Instances.Count == 0)
        {
            dd.DrawWorldPointFilled(vertex, 3, 0xFF00FF00);
            return;
        }

        dd.DrawWorldPointFilled(mesh.Instances[instanceIndex].WorldTransform.TransformCoordinate(vertex), 3, 0xFF00FF00);
    }

    private static void DrawPreviewPrimitive
    (
        Mesh      mesh,
        MeshPart  part,
        int                      instanceIndex,
        Primitive primitive,
        DebugDrawer              dd
    )
    {
        if (mesh.Instances.Count == 0)
            return;

        var transform = mesh.Instances[instanceIndex].WorldTransform;
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
        ref Selection                selection,
        Selection?                   focusSelection,
        ref bool                     focusConsumed,
        CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder  previewBuilder,
        DebugGameCollision           collision,
        string                       query
    )
    {
        if (MatchesSearch(query, "工作区") && ImGui.Selectable("工作区", selection.Kind == SelectionKind.Workspace))
            selection = new(SelectionKind.Workspace);
        if (MatchesSearch(query, "构建参数") && ImGui.Selectable("构建参数", selection.Kind == SelectionKind.BuildProfile))
            selection = new(SelectionKind.BuildProfile);
        if (MatchesSearch(query, "构建设置") && ImGui.Selectable("构建设置", selection.Kind == SelectionKind.BuildSettings))
            selection = new(SelectionKind.BuildSettings);
        if (MatchesSearch(query, "飞行支持") && ImGui.Selectable("飞行支持", selection.Kind == SelectionKind.FlyingOverride))
            selection = new(SelectionKind.FlyingOverride);
        if (MatchesSearch(query, "诊断") && ImGui.Selectable("诊断", selection.Kind == SelectionKind.Diagnostics))
            selection = new(SelectionKind.Diagnostics);

        if (ShouldAutoOpenGeometrySection(focusSelection))
            ImGui.SetNextItemOpen(true);

        if (ImGui.TreeNodeEx("场景几何", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems
            (
                "网格删除",
                BuildMeshRemovalEntries(workspace, previewBuilder, collision),
                SelectionKind.MeshRemoval,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query
            );
            DrawDraftItems
            (
                "实例补丁",
                BuildInstancePatchEntries(workspace, previewBuilder, collision),
                SelectionKind.InstancePatch,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query
            );
            DrawDraftItems
            (
                "顶点 / 三角补丁",
                BuildPartPatchEntries(workspace, previewBuilder, collision),
                SelectionKind.PartPatch,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query
            );
            DrawDraftItems
            (
                "世界几何 / 区域",
                BuildColliderInsertionEntries(workspace, collision),
                SelectionKind.ColliderInsertion,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query
            );
            ImGui.TreePop();
        }

        if (ShouldAutoOpenConnectivitySection(focusSelection))
            ImGui.SetNextItemOpen(true);

        if (ImGui.TreeNodeEx("连通规则", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems
            (
                "网格连接",
                BuildMeshLinkEntries(workspace, collision),
                SelectionKind.MeshLink,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query
            );
            DrawDraftItems
            (
                "离网连接",
                BuildOffMeshConnectionEntries(workspace, collision),
                SelectionKind.OffMeshConnection,
                ref selection,
                focusSelection,
                ref focusConsumed,
                query
            );
            ImGui.TreePop();
        }
    }

    private static void DrawDraftItems
    (
        string               title,
        List<DraftListEntry> items,
        SelectionKind        kind,
        ref Selection        selection,
        Selection?           focusSelection,
        ref bool             focusConsumed,
        string               query
    )
    {
        var filteredCount = string.IsNullOrEmpty(query) ? items.Count : items.Count(item => MatchesSearch(query, item.Label));
        if (filteredCount == 0)
            return;

        var countLabel = filteredCount == items.Count ? items.Count.ToString() : $"{filteredCount}/{items.Count}";
        if (!ImGui.TreeNodeEx($"{title} ({countLabel})", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        foreach (var item in items)
        {
            if (!MatchesSearch(query, item.Label))
                continue;

            var pushedTextStyle = false;

            if (!item.IsEnabled)
            {
                ImGui.PushStyleColor(ImGuiCol.Text, new Vector4(0.82f, 0.58f, 0.34f, 1f));
                pushedTextStyle = true;
            }
            else if (!item.IsInRange)
            {
                ImGui.PushStyleColor(ImGuiCol.Text, ImGui.GetStyle().Colors[(int)ImGuiCol.TextDisabled]);
                pushedTextStyle = true;
            }

            var label = $"[{item.Index}] {item.Label}";
            if (ImGui.Selectable(label, selection.Kind == kind && selection.Index == item.Index))
                selection = new(kind, item.Index);

            if (pushedTextStyle)
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
    (
        CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder  previewBuilder,
        DebugGameCollision           collision
    )
    {
        List<DraftListEntry> entries = [];

        for (var i = 0; i < workspace.Draft.MeshRemovals.Count; ++i)
        {
            var item = workspace.Draft.MeshRemovals[i];
            var info = DescribeMeshRemoval(previewBuilder, collision, item);
            entries.Add(CreateDraftEntry(i, FormatMeshRemovalLabel(item), item.Note, item.Enabled, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildInstancePatchEntries
    (
        CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder  previewBuilder,
        DebugGameCollision           collision
    )
    {
        List<DraftListEntry> entries = [];

        for (var i = 0; i < workspace.Draft.InstancePatches.Count; ++i)
        {
            var item = workspace.Draft.InstancePatches[i];
            var info = DescribeInstancePatch(previewBuilder, collision, item);
            entries.Add(CreateDraftEntry(i, FormatInstancePatchLabel(item), item.Note, item.Enabled, info));
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildPartPatchEntries
    (
        CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder  previewBuilder,
        DebugGameCollision           collision
    )
    {
        List<DraftListEntry> entries = [];

        for (var i = 0; i < workspace.Draft.PartPatches.Count; ++i)
        {
            var item = workspace.Draft.PartPatches[i];
            var info = DescribePartPatch(previewBuilder, collision, item);
            entries.Add
            (
                CreateDraftEntry
                    (i, $"{item.MeshKey} 部件 {item.PartIndex} {CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)}", item.Note, item.Enabled, info)
            );
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildColliderInsertionEntries
    (
        CustomizationEditorWorkspace workspace,
        DebugGameCollision           collision
    )
    {
        List<DraftListEntry> entries = [];

        for (var i = 0; i < workspace.Draft.ColliderInsertions.Count; ++i)
        {
            var item = workspace.Draft.ColliderInsertions[i];
            var info = DescribeBounds(collision, CustomizationEditorSpatial.CreateColliderBounds(item));
            var center = (item.Min + item.Max) * 0.5f;
            var size   = Vector3.Abs(item.Max - item.Min);
            var label = item.Kind == DraftSceneColliderInsertionKind.OrientedCylinder ?
                            $"{CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)} {item.Start:f1} -> {item.End:f1} · 半径 {item.Radius:f1}" :
                        CustomizationEditorSpatial.UsesYRotation(item.Kind) ?
                            $"{CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)} 中心 {center:f1} · {size.X:f1} × {size.Y:f1} × {size.Z:f1} · {item.RotationDegrees:f1}°" :
                            $"{CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)} {item.Min:f1} -> {item.Max:f1}";
            entries.Add
            (
                CreateDraftEntry(i, label, item.Note, item.Enabled, info)
            );
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildMeshLinkEntries
    (
        CustomizationEditorWorkspace workspace,
        DebugGameCollision           collision
    )
    {
        List<DraftListEntry> entries = [];

        for (var i = 0; i < workspace.Draft.MeshLinks.Count; ++i)
        {
            var item = workspace.Draft.MeshLinks[i];
            var info = DescribeBounds(collision, CustomizationEditorSpatial.CreateBounds(item.Start, item.End));
            entries.Add
            (
                CreateDraftEntry(i, $"{CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)} {item.Start:f1} -> {item.End:f1}", item.Note, item.Enabled, info)
            );
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static List<DraftListEntry> BuildOffMeshConnectionEntries
    (
        CustomizationEditorWorkspace workspace,
        DebugGameCollision           collision
    )
    {
        List<DraftListEntry> entries = [];

        for (var i = 0; i < workspace.Draft.OffMeshConnections.Count; ++i)
        {
            var item = workspace.Draft.OffMeshConnections[i];
            var info = DescribeBounds(collision, CustomizationEditorSpatial.CreateBounds(item.Start, item.End));
            entries.Add
            (
                CreateDraftEntry(i, $"{CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)} {item.Start:f1} -> {item.End:f1}", item.Note, item.Enabled, info)
            );
        }

        SortDraftEntries(entries);
        return entries;
    }

    private static DraftDistanceInfo DescribeMeshRemoval
    (
        CustomizationPreviewBuilder previewBuilder,
        DebugGameCollision          collision,
        DraftSceneMeshRemoval       item
    )
    {
        if (previewBuilder.Extractor == null                                         ||
            !previewBuilder.Extractor.Meshes.TryGetValue(item.MeshKey, out var mesh) ||
            !TryGetNearestInstanceBounds(mesh, collision, out var bounds))
            return DraftDistanceInfo.Unknown;

        return DescribeBounds(collision, bounds);
    }

    private static DraftDistanceInfo DescribeInstancePatch
    (
        CustomizationPreviewBuilder previewBuilder,
        DebugGameCollision          collision,
        DraftSceneInstancePatch     item
    )
    {
        if (previewBuilder.Extractor == null                                         ||
            !previewBuilder.Extractor.Meshes.TryGetValue(item.MeshKey, out var mesh) ||
            !TryGetInstancePatchBounds(mesh, item, out var bounds))
            return DraftDistanceInfo.Unknown;

        return DescribeBounds(collision, bounds);
    }

    private static DraftDistanceInfo DescribePartPatch
    (
        CustomizationPreviewBuilder previewBuilder,
        DebugGameCollision          collision,
        DraftScenePartPatch         item
    )
    {
        if (previewBuilder.Extractor == null                                         ||
            !previewBuilder.Extractor.Meshes.TryGetValue(item.MeshKey, out var mesh) ||
            !TryGetNearestInstanceBounds(mesh, collision, out var bounds))
            return DraftDistanceInfo.Unknown;

        return DescribeBounds(collision, bounds);
    }

    private static DraftDistanceInfo DescribeBounds
    (
        DebugGameCollision collision,
        AABB               bounds
    ) =>
        collision.HasRenderDistanceReferencePosition ?
            new(collision.GetHorizontalDistanceToBounds(bounds.ToGame()), collision.IsBoundsWithinEditorRenderDistance(bounds.ToGame())) :
            DraftDistanceInfo.Unknown;

    private static DraftListEntry CreateDraftEntry
    (
        int               index,
        string            label,
        string            note,
        bool              enabled,
        DraftDistanceInfo info
    ) =>
        new(index, FormatDraftLabel(label, note), info.Distance, enabled, info.IsInRange);

    private static void SortDraftEntries
    (
        List<DraftListEntry> entries
    ) =>
        entries.Sort
        (static (a, b) =>
            {
                if (a.Distance == null && b.Distance == null)
                    return a.Index.CompareTo(b.Index);
                if (a.Distance == null)
                    return 1;
                if (b.Distance == null)
                    return -1;

                var cmp = a.Distance.Value.CompareTo(b.Distance.Value);
                return cmp != 0 ?
                           cmp :
                           a.Index.CompareTo(b.Index);
            }
        );

    private static string FormatDraftLabel
    (
        string fallbackLabel,
        string note
    ) =>
        string.IsNullOrWhiteSpace(note) ?
            fallbackLabel :
            note.Trim();

    private static string FormatMeshRemovalLabel
    (
        DraftSceneMeshRemoval item
    ) =>
        item.MeshKey;

    private static string FormatInstancePatchLabel
    (
        DraftSceneInstancePatch item
    ) =>
        item.Kind == DraftSceneInstancePatchKind.ClearInstances ?
            $"{item.MeshKey} {CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)}" :
        item.Kind == DraftSceneInstancePatchKind.Insert ?
            $"{item.MeshKey} {CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)} × {Math.Clamp(item.Count, 1, 1024)} @ {item.WorldTransform.Row3:f1}" :
            $"{item.MeshKey} #{item.InstanceIndex} {CustomizationEditorWidgets.FormatEnumDisplayName(item.Kind)}";

    private static bool TryGetNearestInstanceBounds
    (
        Mesh mesh,
        DebugGameCollision  collision,
        out AABB            bounds
    )
    {
        if (mesh.Instances.Count == 0)
        {
            bounds = default;
            return false;
        }

        bounds = mesh.Instances[0].WorldBounds;
        if (!collision.HasRenderDistanceReferencePosition)
            return true;

        var bestDistance = collision.GetHorizontalDistanceToBounds(bounds.ToGame());

        for (var i = 1; i < mesh.Instances.Count; ++i)
        {
            var candidate = mesh.Instances[i].WorldBounds;
            var distance  = collision.GetHorizontalDistanceToBounds(candidate.ToGame());
            if (distance >= bestDistance)
                continue;

            bestDistance = distance;
            bounds       = candidate;
        }

        return true;
    }

    private static bool TryGetInstancePatchBounds
    (
        Mesh     mesh,
        DraftSceneInstancePatch patch,
        out AABB                bounds
    )
    {
        if (patch.Kind == DraftSceneInstancePatchKind.ClearInstances)
        {
            bounds = default;
            return CustomizationEditorSpatial.TryUnionBounds(mesh.Instances.Select(static x => x.WorldBounds), out bounds);
        }

        if (patch.Kind is DraftSceneInstancePatchKind.Transform or DraftSceneInstancePatchKind.Insert)
        {
            bounds = CustomizationEditorSpatial.CreateInstancePatchBounds(mesh.LocalBounds, patch);
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

    private static bool TryResolveInstance
    (
        Mesh             mesh,
        DraftSceneInstancePatch         patch,
        out MeshInstance instance
    )
    {
        if (patch.InstanceId != 0)
        {
            instance = mesh.Instances.FirstOrDefault(x => x.ID == patch.InstanceId)!;
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

    private static List<int> GetVisiblePreviewInstanceIndices
    (
        Mesh mesh,
        DebugGameCollision  collision,
        bool                forceFocused,
        int                 focusedInstanceIndex
    )
    {
        List<int> visibleIndices = [];

        for (var i = 0; i < mesh.Instances.Count; ++i)
        {
            var visible = collision.IsBoundsWithinEditorRenderDistance(mesh.Instances[i].WorldBounds.ToGame());
            if (visible || (forceFocused && i == focusedInstanceIndex))
                visibleIndices.Add(i);
        }

        return visibleIndices;
    }

    private static bool TryGetVisibleMeshBounds
    (
        Mesh mesh,
        List<int>           visibleInstanceIndices,
        out AABB            bounds
    )
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

    private static bool TryGetFocusedPreviewInstance
    (
        Selection? focusSelection,
        string     meshKey,
        out int    focusedInstanceIndex
    )
    {
        if (focusSelection is { Kind: SelectionKind.PreviewInstance, Key: not null, Index: >= 0 } && focusSelection.Key == meshKey)
        {
            focusedInstanceIndex = focusSelection.Index;
            return true;
        }

        focusedInstanceIndex = -1;
        return false;
    }

    private static int GetFocusedPreviewPartIndex
    (
        Selection? focusSelection,
        string     meshKey
    ) =>
        focusSelection is
        {
            Kind: SelectionKind.PreviewPart or SelectionKind.PreviewVertex or SelectionKind.PreviewPrimitive,
            Key: not null,
            Index: >= 0
        } && focusSelection.Key == meshKey ?
            focusSelection.Index :
            -1;

    private static bool IsFocusedPreviewSelection
    (
        Selection? focusSelection,
        string     meshKey
    ) =>
        focusSelection != null &&
        focusSelection.Key == meshKey &&
        IsPreviewSelectionKind(focusSelection.Kind);

    private static bool ShouldAutoOpenGeometrySection
    (
        Selection? focusSelection
    ) =>
        focusSelection is { Kind: SelectionKind.MeshRemoval or SelectionKind.InstancePatch or SelectionKind.PartPatch or SelectionKind.ColliderInsertion };

    private static bool ShouldAutoOpenConnectivitySection
    (
        Selection? focusSelection
    ) =>
        focusSelection is { Kind: SelectionKind.MeshLink or SelectionKind.OffMeshConnection };

    private static bool IsPreviewSelectionKind
    (
        SelectionKind kind
    ) =>
        kind is SelectionKind.PreviewMesh or SelectionKind.PreviewInstance or SelectionKind.PreviewPart or SelectionKind.PreviewVertex
            or SelectionKind.PreviewPrimitive;

    private static bool MatchesSearch
    (
        string query,
        string value
    ) =>
        string.IsNullOrEmpty(query) || value.Contains(query, StringComparison.OrdinalIgnoreCase);

    private static string BuildInstancePatchTag
    (
        CustomizationEditorWorkspace workspace,
        string                       meshKey,
        int                          instanceIndex,
        ulong                        instanceId
    )
    {
        List<string> tags = [];

        foreach (var patch in workspace.Draft.InstancePatches.Where(x => x.Enabled && x.MeshKey == meshKey))
        {
            var matches = patch.Kind       == DraftSceneInstancePatchKind.ClearInstances ||
                          patch.InstanceId != 0 ?
                              patch.InstanceId    == instanceId :
                              patch.InstanceIndex == instanceIndex;
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

        return tags.Count == 0 ?
                   string.Empty :
                   $" [{string.Join(", ", tags)}]";
    }

    private readonly record struct DraftListEntry
    (
        int    Index,
        string Label,
        float? Distance,
        bool   IsEnabled,
        bool   IsInRange
    );

    private readonly record struct DraftDistanceInfo
    (
        float? Distance,
        bool   IsInRange
    )
    {
        public static DraftDistanceInfo Unknown => new(null, true);
    }
}
