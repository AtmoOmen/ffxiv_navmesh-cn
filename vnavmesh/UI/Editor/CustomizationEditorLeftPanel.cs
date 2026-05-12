using System.Numerics;
using Dalamud.Bindings.ImGui;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Customizations.Editor;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Debug.Common;

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
        CustomizationPreviewBuilder      previewBuilder,
        DebugDrawer                      dd,
        AddMeshRemovalDelegate           onAddMeshRemoval,
        AddInstancePatchDelegate         onAddInstancePatch,
        AddPartPatchDelegate             onAddPartPatch
    )
    {
        if (previewBuilder is { CurrentState: CustomizationPreviewBuilder.State.Ready, Extractor: not null })
        {
            if (ImGui.TreeNodeEx("预览对象", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.TextDisabled("左键选择, 右键加入对应草稿补丁");
                DrawPreviewMeshes(previewBuilder.Extractor, ref selection, dd, onAddMeshRemoval, onAddInstancePatch, onAddPartPatch);
                ImGui.TreePop();
            }
        }

        if (ImGui.TreeNodeEx("草稿与设置", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftTree(ref selection, workspace);
            ImGui.TreePop();
        }
    }

    private static void DrawPreviewMeshes
    (
        SceneExtractor           extractor,
        ref Selection            selection,
        DebugDrawer              dd,
        AddMeshRemovalDelegate   onAddMeshRemoval,
        AddInstancePatchDelegate onAddInstancePatch,
        AddPartPatchDelegate     onAddPartPatch
    )
    {
        foreach (var (key, mesh) in extractor.Meshes.OrderBy(static x => x.Key, StringComparer.Ordinal))
        {
            var meshLabel    = $"{key} [{mesh.Parts.Count} parts, {mesh.Instances.Count} inst]";
            var meshSelected = selection is { Kind: SelectionKind.PreviewMesh, Key: var meshKey } && meshKey == key;
            var meshOpen     = ImGui.TreeNodeEx(meshLabel, meshSelected ? ImGuiTreeNodeFlags.Selected : ImGuiTreeNodeFlags.None);

            if (ImGui.IsItemClicked(ImGuiMouseButton.Left))
                selection = new(SelectionKind.PreviewMesh, Key: key);

            if (ImGui.BeginPopupContextItem($"##preview_mesh_popup_{key}"))
            {
                if (ImGui.MenuItem("加入 mesh 删除清单"))
                    onAddMeshRemoval(key);
                ImGui.EndPopup();
            }

            if (ImGui.IsItemHovered())
                dd.DrawWorldAABB(mesh.LocalBounds, 0xFF00FFFF);

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

                    if (ImGui.IsItemHovered() && mesh.Instances.Count > 0)
                        DrawMeshPreview(part, mesh.Instances[0].WorldTransform, dd);

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

            if (ImGui.TreeNodeEx($"实例##inst_{key}"))
            {
                var instanceIndex = 0;

                foreach (var instance in mesh.Instances)
                {
                    var instanceLabel = $"[{instanceIndex}] {instance.Id:X} {instance.WorldBounds.Min:f1}-{instance.WorldBounds.Max:f1}";
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

                    ++instanceIndex;
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

    private static void DrawDraftTree(ref Selection selection, CustomizationEditorWorkspace workspace)
    {
        if (ImGui.Selectable("工作区", selection.Kind == SelectionKind.Workspace))
            selection = new(SelectionKind.Workspace);
        if (ImGui.Selectable("构建参数", selection.Kind == SelectionKind.BuildProfile))
            selection = new(SelectionKind.BuildProfile);
        if (ImGui.Selectable("构建设置", selection.Kind == SelectionKind.BuildSettings))
            selection = new(SelectionKind.BuildSettings);
        if (ImGui.Selectable("飞行支持", selection.Kind == SelectionKind.FlyingOverride))
            selection = new(SelectionKind.FlyingOverride);

        if (ImGui.TreeNodeEx("场景几何", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems
            (
                "mesh 删除",
                workspace.Draft.MeshRemovals,
                SelectionKind.MeshRemoval,
                ref selection,
                static item => $"{item.MeshKey} {(item.Enabled ? "" : "(off)")}"
            );
            DrawDraftItems
            (
                "实例补丁",
                workspace.Draft.InstancePatches,
                SelectionKind.InstancePatch,
                ref selection,
                static item => $"{item.MeshKey} #{item.InstanceIndex} {item.Kind}"
            );
            DrawDraftItems
                ("顶点 / 三角补丁", workspace.Draft.PartPatches, SelectionKind.PartPatch, ref selection, static item => $"{item.MeshKey} p{item.PartIndex} {item.Kind}");
            DrawDraftItems
            (
                "碰撞插入",
                workspace.Draft.ColliderInsertions,
                SelectionKind.ColliderInsertion,
                ref selection,
                static item => $"{item.Kind} {item.Min:f1} -> {item.Max:f1}"
            );
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("连通规则", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems
                ("mesh link", workspace.Draft.MeshLinks, SelectionKind.MeshLink, ref selection, static item => $"{item.Kind} {item.Start:f1} -> {item.End:f1}");
            DrawDraftItems
            (
                "off-mesh 连接",
                workspace.Draft.OffMeshConnections,
                SelectionKind.OffMeshConnection,
                ref selection,
                static item => $"{item.Kind} {item.Start:f1} -> {item.End:f1}"
            );
            ImGui.TreePop();
        }
    }

    private static void DrawDraftItems<T>(string title, List<T> items, SelectionKind kind, ref Selection selection, Func<T, string> itemLabel)
    {
        if (!ImGui.TreeNodeEx($"{title} ({items.Count})", ImGuiTreeNodeFlags.DefaultOpen))
            return;

        for (var i = 0; i < items.Count; ++i)
        {
            var label = $"[{i}] {itemLabel(items[i])}";
            if (ImGui.Selectable(label, selection.Kind == kind && selection.Index == i))
                selection = new(kind, i);
        }

        ImGui.TreePop();
    }
}
