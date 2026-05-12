using System.Globalization;
using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using Dalamud.Interface.Utility.Raii;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Customizations.Editor;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using Action = System.Action;

namespace vnavmesh.UI.Editor;

internal sealed unsafe class CustomizationEditorView
(
    Config             config,
    DebugDrawer        dd,
    DebugGameCollision collision,
    NavmeshManager     manager,
    DirectoryInfo      configDirectory
)
    : IDisposable
{
    private enum SelectionKind
    {
        Workspace,
        BuildProfile,
        BuildSettings,
        FlyingOverride,
        MeshRemoval,
        InstancePatch,
        PartPatch,
        ColliderInsertion,
        MeshLink,
        OffMeshConnection,
        PreviewMesh,
        PreviewInstance,
        PreviewPart,
        PreviewVertex,
        PreviewPrimitive
    }

    private enum PickKind
    {
        None,
        SelectCollider,
        Aabb,
        Cylinder,
        LinkPoints,
        LinkDrop,
        LinkClientPath,
        OffMesh
    }

    private sealed record Selection
    (
        SelectionKind Kind,
        int           Index    = -1,
        int           SubIndex = -1,
        string?       Key      = null
    );

    private readonly CustomizationDraftPersistence persistence      = new(new DirectoryInfo(Path.Combine(configDirectory.FullName, "customization-editor")));
    private readonly CustomizationPreviewBuilder   previewBuilder   = new(manager, config);
    private readonly NavmeshSettings               settingsDefaults = new();
    private readonly NavmeshBuildProfile           profileDefaults  = new();

    private          CustomizationEditorWorkspace    workspace = new();
    private          uint                            territoryID;
    private          string                          territoryLabel = "";
    private          bool                            workspaceLoaded;
    private          bool                            historySuspended;
    private          CustomizationDraft              historySnapshot = new();
    private readonly Stack<CustomizationDraft>       undo            = new();
    private readonly Stack<CustomizationDraft>       redo            = new();
    private          Selection                       selection       = new(SelectionKind.Workspace);
    private          PickKind                        pickKind       = PickKind.None;
    private          Vector3?                        pendingPickPoint;
    private          Vector3?                        currentPickPoint;
    private          bool                            lastPickMouseDown;
    private          bool                            lastPickEscapeDown;
    private          bool                            lastWorldSelectMouseDown;
    private          string                          statusText         = "未加载";
    private          string                          exportDirText = "";
    private          CustomizationDraftExportResult? lastExport;
    private          bool                            previewDirty  = true;
    private          DateTime                        nextRebuildAt = DateTime.MinValue;

    public void Dispose()
    {
        if (workspaceLoaded)
            SaveWorkspace(true);
        previewBuilder.Dispose();
    }

    public void Draw()
    {
        EnsureWorkspace();
        HandleKeyboardShortcuts();
        DrawToolbar();
        DrawStatus();

        if (workspaceLoaded                               &&
            previewDirty                                  &&
            workspace.Settings.AutoRebuild                &&
            DateTime.UtcNow              >= nextRebuildAt &&
            previewBuilder.CurrentState != CustomizationPreviewBuilder.State.InProgress)
            RebuildPreview();

        ImGui.BeginChild("##customization_editor_left", new Vector2(340, 0), true);
        DrawLeftPanel();
        ImGui.EndChild();

        ImGui.SameLine();

        ImGui.BeginChild("##customization_editor_right", new Vector2(0, 0), true);
        DrawInspector();
        ImGui.EndChild();

        DrawWorldOverlay();
    }

    private void EnsureWorkspace()
    {
        var territoryId = Service.ClientState.TerritoryType;
        if (territoryId == 0)
            return;

        if (workspaceLoaded && territoryId == territoryID)
            return;

        if (workspaceLoaded)
            SaveWorkspace(true);

        workspace   = persistence.Load(territoryId);
        territoryID = territoryId;
        var territory = Service.LuminaRow<TerritoryType>(territoryId);
        territoryLabel                = territory == null ? territoryId.ToString(CultureInfo.InvariantCulture) : territory.Value.Bg.ToString();
        workspace.Draft.TerritoryID   = territoryId;
        workspace.Draft.TerritoryName = territoryLabel;
        historySnapshot               = workspace.Draft.Clone();
        exportDirText = string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory)
                             ? Path.Combine(configDirectory.FullName, "customization-editor", "generated")
                             : workspace.Settings.ExportDirectory;
        workspace.Settings.ExportDirectory = exportDirText;
        workspaceLoaded                    = true;
        undo.Clear();
        redo.Clear();
        selection                = new(SelectionKind.Workspace);
        pendingPickPoint         = null;
        pickKind                 = PickKind.None;
        lastPickMouseDown        = IsKeyDown(VK_LBUTTON);
        lastWorldSelectMouseDown = lastPickMouseDown;
        lastPickEscapeDown       = IsKeyDown(VK_ESCAPE);
        previewDirty             = true;
        nextRebuildAt            = DateTime.MinValue;
        statusText                   = $"已加载 Territory {territoryId} ({territoryLabel})";
        if (workspace.Settings.AutoRebuild)
            RebuildPreview();
    }

    private void DrawToolbar()
    {
        ImGui.BeginGroup();

        ImGui.TextUnformatted("工具");
        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            DrawModeButton("浏览", PickKind.None, "只查看和选择对象; 在左侧选中后, 右侧会显示可编辑内容");
            ImGui.SameLine();
            DrawModeButton("选碰撞体", PickKind.SelectCollider, "进入世界点选模式; 在游戏画面点击碰撞体后, 右侧直接编辑");
            ImGui.SameLine();
            DrawModeButton("AABB 障碍", PickKind.Aabb, "在游戏画面点两个世界点, 生成一个轴对齐障碍体");
            ImGui.SameLine();
            DrawModeButton("圆柱障碍", PickKind.Cylinder, "在游戏画面点两个世界点, 生成一个圆柱障碍体");
            ImGui.SameLine();
            DrawModeButton("网格连线", PickKind.LinkPoints, "在游戏画面点两个世界点, 生成 LinkPoints 连接");
            ImGui.SameLine();
            DrawModeButton("下落连接", PickKind.LinkDrop, "在游戏画面点两个世界点, 生成 LinkDrop 连接");
            ImGui.SameLine();
            DrawModeButton("客户端路径", PickKind.LinkClientPath, "在游戏画面点两个世界点, 生成 LinkClientPath 连接");
            ImGui.SameLine();
            DrawModeButton("离网连接", PickKind.OffMesh, "在游戏画面点两个世界点, 生成构建期 off-mesh connection");
        }

        ImGui.TextUnformatted("草稿");
        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded || undo.Count == 0))
        {
            if (ImGui.Button("撤销"))
                Undo();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded || redo.Count == 0))
        {
            if (ImGui.Button("重做"))
                Redo();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            if (ImGui.Button("重建预览"))
                RebuildPreview();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            if (ImGui.Button("保存草稿"))
                SaveWorkspace(true);
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            if (ImGui.Button("导出 C#"))
                ExportCurrentDraft();
        }

        if (pickKind != PickKind.None)
        {
            ImGui.SameLine();
            if (ImGui.Button("退出模式 (Esc)"))
                CancelPick("已退出当前工具模式");
        }

        ImGui.EndGroup();

        ImGui.TextWrapped(GetToolbarHint());
    }

    private void DrawModeButton(string label, PickKind kind, string tooltip)
    {
        var active = pickKind == kind;

        if (active)
        {
            ImGui.PushStyleColor(ImGuiCol.Button,        new Vector4(0.22f, 0.45f, 0.75f, 1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new Vector4(0.28f, 0.55f, 0.9f,  1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonActive,  new Vector4(0.18f, 0.38f, 0.66f, 1f));
        }

        if (ImGui.Button(label))
        {
            if (kind == PickKind.None)
                CancelPick("已切换到浏览模式");
            else
                BeginPick(kind);
        }

        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(tooltip);

        if (active)
            ImGui.PopStyleColor(3);
    }

    private void CancelPick(string status)
    {
        pickKind                 = PickKind.None;
        pendingPickPoint         = null;
        currentPickPoint         = null;
        lastPickMouseDown        = IsKeyDown(VK_LBUTTON);
        lastWorldSelectMouseDown = lastPickMouseDown;
        lastPickEscapeDown       = IsKeyDown(VK_ESCAPE);
        this.statusText               = status;
    }

    private void HandleKeyboardShortcuts()
    {
        if (pickKind != PickKind.None && TakeKeyPress(VK_ESCAPE, ref lastPickEscapeDown))
            CancelPick("已退出当前工具模式");
    }

    private static bool IsKeyDown(int virtualKey) =>
        (GetAsyncKeyState(virtualKey) & 0x8000) != 0;

    private static bool TakeKeyPress(int virtualKey, ref bool lastDown)
    {
        var down    = IsKeyDown(virtualKey);
        var pressed = down && !lastDown;
        lastDown = down;
        return pressed;
    }

    private bool TakeWorldPickClick()
    {
        var clicked = TakeKeyPress(VK_LBUTTON, ref lastPickMouseDown);
        if (!clicked)
            return false;

        return IsWorldClickAllowed();
    }

    private bool TakeWorldSelectClick()
    {
        var clicked = TakeKeyPress(VK_LBUTTON, ref lastWorldSelectMouseDown);
        if (!clicked)
            return false;

        return IsWorldClickAllowed();
    }

    private static bool IsWorldClickAllowed() =>
        !ImGui.GetIO().WantCaptureMouse && !ImGui.IsAnyItemHovered() && !ImGui.IsAnyItemActive();

    private void BeginPick(PickKind kind)
    {
        pickKind                 = kind;
        pendingPickPoint         = null;
        currentPickPoint         = null;
        lastPickMouseDown        = IsKeyDown(VK_LBUTTON);
        lastWorldSelectMouseDown = lastPickMouseDown;
        lastPickEscapeDown       = IsKeyDown(VK_ESCAPE);
        statusText = kind == PickKind.SelectCollider
                      ? "选中碰撞体: 在游戏画面点击一个碰撞体"
                      : $"{GetPickKindTitle(kind)}: 等待第 1 个世界点, 在游戏画面点击";
    }

    private string GetToolbarHint()
    {
        if (!workspaceLoaded)
            return "进入一个游戏区域后, 编辑器会自动加载该 Territory 的草稿";

        if (pickKind == PickKind.None)
            return "浏览模式: 世界点击不会选中对象; 需要编辑现有碰撞体时, 先点击“选碰撞体”";

        if (pickKind == PickKind.SelectCollider)
            return "选中碰撞体: 鼠标指向的碰撞体会高亮, 左键选中后在右侧编辑, Esc 可退出";

        var step = pendingPickPoint == null ? "点击第 1 个世界点" : "点击第 2 个世界点完成创建";
        return $"{GetPickKindTitle(pickKind)}: {step}; 在游戏画面点击落点, 点在插件窗口或其他 UI 上不会落点, Esc 可取消";
    }

    private static string GetPickKindTitle(PickKind kind) =>
        kind switch
        {
            PickKind.SelectCollider => "选中碰撞体",
            PickKind.Aabb           => "AABB 障碍",
            PickKind.Cylinder       => "圆柱障碍",
            PickKind.LinkPoints     => "网格连线",
            PickKind.LinkDrop       => "下落连接",
            PickKind.LinkClientPath => "客户端路径",
            PickKind.OffMesh        => "离网连接",
            _                       => "浏览"
        };

    private void DrawStatus()
    {
        ImGui.TextUnformatted($"Territory: {territoryID} {territoryLabel}");
        ImGui.SameLine();
        ImGui.TextUnformatted($"Preview: {previewBuilder.CurrentState}");
        ImGui.SameLine();
        ImGui.TextUnformatted(statusText);

        if (previewBuilder is { CurrentState: CustomizationPreviewBuilder.State.Failed, LastError: not null })
            ImGui.TextColored(new Vector4(1, 0.5f, 0.5f, 1), previewBuilder.LastError.Message);

        if (lastExport != null)
            ImGui.TextDisabled($"导出: {lastExport.ClassName} v{lastExport.Version} {lastExport.ContentHash[..16]} -> {lastExport.File.FullName}");
    }

    private void DrawLeftPanel()
    {
        if (!workspaceLoaded)
        {
            ImGui.TextDisabled("尚未加载 Territory");
            ImGui.TextWrapped("进入游戏区域后, 这里会显示当前 Territory 的预览对象和本地草稿");
            return;
        }

        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
        {
            ImGui.TextDisabled("预览对象未就绪");
            ImGui.TextWrapped("点击“重建预览”或等待自动重建完成后, 可在这里选择 mesh、实例、顶点和三角");
        }
        else
        {
            if (ImGui.TreeNodeEx("预览对象", ImGuiTreeNodeFlags.DefaultOpen))
            {
                ImGui.TextDisabled("左键选择, 右键加入对应草稿补丁");
                DrawPreviewMeshes(previewBuilder.Extractor);
                ImGui.TreePop();
            }
        }

        if (ImGui.TreeNodeEx("草稿与设置", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.TextDisabled("选中草稿项后, 在右侧调整数值并触发重建预览");
            DrawDraftTree();
            ImGui.TreePop();
        }
    }

    private void DrawPreviewMeshes(SceneExtractor extractor)
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
                    AddMeshRemovalFromPreview(key);
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
                            AddPartPatchFromPreview(mesh, key, partIndex, DraftScenePartPatchKind.Vertex);
                        if (ImGui.MenuItem("加入三角 flags 补丁"))
                            AddPartPatchFromPreview(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveFlags);
                        if (ImGui.MenuItem("加入三角高级编辑补丁"))
                            AddPartPatchFromPreview(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveEdit);
                        ImGui.EndPopup();
                    }

                    if (ImGui.IsItemHovered() && mesh.Instances.Count > 0)
                        DrawMeshPreview(part, mesh.Instances[0].WorldTransform);

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
                                        AddPartPatchFromPreview(mesh, key, partIndex, DraftScenePartPatchKind.Vertex, vertexIndex);
                                    ImGui.EndPopup();
                                }

                                if (ImGui.IsItemHovered())
                                    DrawPreviewVertex(mesh, vertex);
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
                                        AddPartPatchFromPreview(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveFlags, primIndex);
                                    if (ImGui.MenuItem("加入三角高级编辑补丁"))
                                        AddPartPatchFromPreview(mesh, key, partIndex, DraftScenePartPatchKind.PrimitiveEdit, primIndex);
                                    ImGui.EndPopup();
                                }

                                if (ImGui.IsItemHovered())
                                    DrawPreviewPrimitive(mesh, part, prim);
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
                                           instanceKey   == key                                                                                  &&
                                           selectedIndex == instanceIndex;

                    if (ImGui.Selectable(instanceLabel, instanceSelected))
                        selection = new(SelectionKind.PreviewInstance, instanceIndex, Key: key);

                    if (ImGui.BeginPopupContextItem($"##preview_instance_popup_{key}_{instanceIndex}"))
                    {
                        if (ImGui.MenuItem("加入实例变换补丁"))
                            AddInstancePatchFromPreview(mesh, key, instanceIndex, DraftSceneInstancePatchKind.Transform);
                        if (ImGui.MenuItem("加入实例 flags 补丁"))
                            AddInstancePatchFromPreview(mesh, key, instanceIndex, DraftSceneInstancePatchKind.SetFlags);
                        if (ImGui.MenuItem("移除这个实例"))
                            AddInstancePatchFromPreview(mesh, key, instanceIndex, DraftSceneInstancePatchKind.RemoveInstance);
                        if (ImGui.MenuItem("清空这个 mesh 的全部实例"))
                            AddInstancePatchFromPreview(mesh, key, instanceIndex, DraftSceneInstancePatchKind.ClearInstances);
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

    private void DrawMeshPreview(SceneExtractor.MeshPart part, Matrix4x3 transform, uint color = 0xFF00FFAA)
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

    private void DrawPreviewVertex(SceneExtractor.Mesh mesh, Vector3 vertex)
    {
        if (mesh.Instances.Count == 0)
        {
            dd.DrawWorldPointFilled(vertex, 3, 0xFF00FF00);
            return;
        }

        dd.DrawWorldPointFilled(mesh.Instances[0].WorldTransform.TransformCoordinate(vertex), 3, 0xFF00FF00);
    }

    private void DrawPreviewPrimitive(SceneExtractor.Mesh mesh, SceneExtractor.MeshPart part, SceneExtractor.Primitive primitive)
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

    private void DrawDraftTree()
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
            DrawDraftItems("mesh 删除", workspace.Draft.MeshRemovals, SelectionKind.MeshRemoval, static item => $"{item.MeshKey} {(item.Enabled ? "" : "(off)")}");
            DrawDraftItems
                ("实例补丁", workspace.Draft.InstancePatches, SelectionKind.InstancePatch, static item => $"{item.MeshKey} #{item.InstanceIndex} {item.Kind}");
            DrawDraftItems("顶点 / 三角补丁", workspace.Draft.PartPatches, SelectionKind.PartPatch, static item => $"{item.MeshKey} p{item.PartIndex} {item.Kind}");
            DrawDraftItems
                ("碰撞插入", workspace.Draft.ColliderInsertions, SelectionKind.ColliderInsertion, static item => $"{item.Kind} {item.Min:f1} -> {item.Max:f1}");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("连通规则", ImGuiTreeNodeFlags.DefaultOpen))
        {
            DrawDraftItems("mesh link", workspace.Draft.MeshLinks, SelectionKind.MeshLink, static item => $"{item.Kind} {item.Start:f1} -> {item.End:f1}");
            DrawDraftItems
            (
                "off-mesh 连接",
                workspace.Draft.OffMeshConnections,
                SelectionKind.OffMeshConnection,
                static item => $"{item.Kind} {item.Start:f1} -> {item.End:f1}"
            );
            ImGui.TreePop();
        }
    }

    private void DrawDraftItems<T>(string title, List<T> items, SelectionKind kind, Func<T, string> itemLabel)
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

    private void DrawInspector()
    {
        if (!workspaceLoaded)
        {
            ImGui.TextDisabled("等待 Territory");
            ImGui.TextWrapped("进入游戏区域后, 编辑器会自动加载当前 Territory 的草稿; 加载后可直接在游戏画面点两点创建障碍或连接");
            return;
        }

        DrawInspectorHeader();

        switch (selection.Kind)
        {
            case SelectionKind.Workspace:
                DrawWorkspaceInspector();
                break;
            case SelectionKind.BuildProfile:
                DrawBuildProfileInspector();
                break;
            case SelectionKind.BuildSettings:
                DrawBuildSettingsInspector();
                break;
            case SelectionKind.FlyingOverride:
                DrawFlyingInspector();
                break;
            case SelectionKind.MeshRemoval:
                DrawMeshRemovalInspector();
                break;
            case SelectionKind.InstancePatch:
                DrawInstancePatchInspector();
                break;
            case SelectionKind.PartPatch:
                DrawPartPatchInspector();
                break;
            case SelectionKind.ColliderInsertion:
                DrawColliderInsertionInspector();
                break;
            case SelectionKind.MeshLink:
                DrawMeshLinkInspector();
                break;
            case SelectionKind.OffMeshConnection:
                DrawOffMeshInspector();
                break;
            case SelectionKind.PreviewMesh:
                DrawPreviewMeshInspector();
                break;
            case SelectionKind.PreviewInstance:
                DrawPreviewInstanceInspector();
                break;
            case SelectionKind.PreviewPart:
            case SelectionKind.PreviewVertex:
            case SelectionKind.PreviewPrimitive:
                DrawPreviewPartInspector();
                break;
        }
    }

    private void DrawInspectorHeader()
    {
        ImGui.TextUnformatted(GetSelectionTitle());
        ImGui.TextWrapped(GetSelectionHelp());
        ImGui.Separator();
    }

    private string GetSelectionTitle() =>
        selection.Kind switch
        {
            SelectionKind.Workspace         => $"工作区 / Territory {territoryID} {territoryLabel}",
            SelectionKind.BuildProfile      => "构建参数覆盖",
            SelectionKind.BuildSettings     => "构建设置覆盖",
            SelectionKind.FlyingOverride    => "飞行支持",
            SelectionKind.MeshRemoval       => $"mesh 删除 [{selection.Index}]",
            SelectionKind.InstancePatch     => $"实例补丁 [{selection.Index}]",
            SelectionKind.PartPatch         => $"顶点 / 三角补丁 [{selection.Index}]",
            SelectionKind.ColliderInsertion => $"碰撞插入 [{selection.Index}]",
            SelectionKind.MeshLink          => $"mesh link [{selection.Index}]",
            SelectionKind.OffMeshConnection => $"off-mesh 连接 [{selection.Index}]",
            SelectionKind.PreviewMesh       => $"预览 mesh: {selection.Key}",
            SelectionKind.PreviewInstance   => $"预览实例: {selection.Key} #{selection.Index}",
            SelectionKind.PreviewPart       => $"预览 part: {selection.Key} p{selection.Index}",
            SelectionKind.PreviewVertex     => $"预览顶点: {selection.Key} p{selection.Index} v{selection.SubIndex}",
            SelectionKind.PreviewPrimitive  => $"预览三角: {selection.Key} p{selection.Index} t{selection.SubIndex}",
            _                               => "自定义编辑器"
        };

    private string GetSelectionHelp() =>
        selection.Kind switch
        {
            SelectionKind.Workspace         => "管理这个 Territory 的草稿保存、自动重建和 C# 导出位置",
            SelectionKind.BuildProfile      => "覆盖 BuildProfile, 适合调 Recast profile 级别的默认值",
            SelectionKind.BuildSettings     => "直接覆盖本 Territory 的 NavmeshSettings, 修改后可自动重建预览",
            SelectionKind.FlyingOverride    => "覆盖该 Territory 是否支持飞行导航",
            SelectionKind.MeshRemoval       => "把整个 mesh 从提取场景中移除, 适合门、碎石等不该参与导航的碰撞",
            SelectionKind.InstancePatch     => "修改或移除某个 mesh 实例, 也可以给该实例强制写入或清除 PrimitiveFlags",
            SelectionKind.PartPatch         => "修改 mesh 内部顶点或三角, 适合小范围几何修补和 flags 调试",
            SelectionKind.ColliderInsertion => "两点生成的 AABB / 圆柱障碍体; 用中心和尺寸调整高度、宽度后会按设置重建预览",
            SelectionKind.MeshLink          => "在已构建 navmesh 上追加 LinkPoints / LinkDrop / LinkClientPath",
            SelectionKind.OffMeshConnection => "在 Recast 构建参数中追加 AddOffMeshConnection",
            SelectionKind.PreviewMesh       => "这是实时预览对象; 右键或点击按钮可把它加入草稿删除清单",
            SelectionKind.PreviewInstance   => "这是实时预览实例; 可在世界中直接点选并在右侧编辑 transform / flags / 删除实例补丁",
            SelectionKind.PreviewPart       => "这是实时预览 part; 展开左侧顶点或三角后可直接编辑对应内容",
            SelectionKind.PreviewVertex     => "这是实时预览顶点; 调整 Position 会自动生成或更新顶点补丁",
            SelectionKind.PreviewPrimitive  => "这是实时预览三角; 调整 flags 或顶点索引会自动生成或更新三角补丁",
            _                               => "左侧选择对象或草稿项后, 这里会显示可执行操作"
        };

    private void DrawWorkspaceInspector()
    {
        ImGui.TextUnformatted($"Territory {territoryID} {territoryLabel}");
        ImGui.Separator();
        DrawBool("自动重建", ref workspace.Settings.AutoRebuild);
        DrawBool("自动保存", ref workspace.Settings.AutoSave);
        DrawFloat("重建延迟", ref workspace.Settings.RebuildDelaySeconds, 0.05f, 0.1f, 5f);

        ImGui.SetNextItemWidth(-1);

        if (ImGui.InputText("导出目录", ref exportDirText))
        {
            workspace.Settings.ExportDirectory = exportDirText;
            SaveWorkspace();
        }

        if (ImGui.Button("手动重建预览"))
            RebuildPreview();

        ImGui.SameLine();
        if (ImGui.Button("导出 C#"))
            ExportCurrentDraft();
    }

    private void DrawBuildProfileInspector()
    {
        var changed = false;
        changed |= DrawNullableEnum("Partitioning", ref workspace.Draft.BuildProfile.PartitioningOverride, profileDefaults.PartitioningOverride);
        changed |= DrawNullableFloat("Cell Size",         ref workspace.Draft.BuildProfile.CellSizeOverride,        settingsDefaults.CellSize);
        changed |= DrawNullableFloat("Cell Height",       ref workspace.Draft.BuildProfile.CellHeightOverride,      settingsDefaults.CellHeight);
        changed |= DrawNullableFloat("Region Min Size",   ref workspace.Draft.BuildProfile.RegionMinSizeOverride,   settingsDefaults.RegionMinSize);
        changed |= DrawNullableFloat("Region Merge Size", ref workspace.Draft.BuildProfile.RegionMergeSizeOverride, settingsDefaults.RegionMergeSize);
        changed |= DrawNullableFloat("Poly Max Edge Len", ref workspace.Draft.BuildProfile.PolyMaxEdgeLenOverride,  settingsDefaults.PolyMaxEdgeLen);
        changed |= DrawNullableFloat
            ("Poly Max Simplification Error", ref workspace.Draft.BuildProfile.PolyMaxSimplificationErrorOverride, settingsDefaults.PolyMaxSimplificationError);
        changed |= DrawNullableFloat("Agent Radius", ref workspace.Draft.BuildProfile.AgentRadiusOverride, settingsDefaults.AgentRadius);
        changed |= DrawNullableIntArray("Volume Tiles", ref workspace.Draft.BuildProfile.VolumeTilesOverride, settingsDefaults.VolumeTiles);
        changed |= DrawNullableFloat("Detail Sample Dist", ref workspace.Draft.BuildProfile.DetailSampleDistOverride, settingsDefaults.DetailSampleDist);
        changed |= DrawNullableBool
            ("Generate Edge Climb Links", ref workspace.Draft.BuildProfile.GenerateEdgeClimbLinksOverride, settingsDefaults.GenerateEdgeClimbLinks);
        changed |= DrawNullableBool
            ("Generate Edge Jump Links", ref workspace.Draft.BuildProfile.GenerateEdgeJumpLinksOverride, settingsDefaults.GenerateEdgeJumpLinks);

        if (changed)
            CommitDraftChange();
    }

    private void DrawBuildSettingsInspector()
    {
        var changed = false;
        changed |= DrawNullableFloat("Cell Size",       ref workspace.Draft.BuildSettings.CellSize,         settingsDefaults.CellSize);
        changed |= DrawNullableFloat("Cell Height",     ref workspace.Draft.BuildSettings.CellHeight,       settingsDefaults.CellHeight);
        changed |= DrawNullableFloat("Agent Height",    ref workspace.Draft.BuildSettings.AgentHeight,      settingsDefaults.AgentHeight);
        changed |= DrawNullableFloat("Agent Radius",    ref workspace.Draft.BuildSettings.AgentRadius,      settingsDefaults.AgentRadius);
        changed |= DrawNullableFloat("Agent Max Climb", ref workspace.Draft.BuildSettings.AgentMaxClimb,    settingsDefaults.AgentMaxClimb);
        changed |= DrawNullableFloat("Agent Max Slope", ref workspace.Draft.BuildSettings.AgentMaxSlopeDeg, settingsDefaults.AgentMaxSlopeDeg);
        changed |= DrawNullableFlags("Filtering", ref workspace.Draft.BuildSettings.Filtering, settingsDefaults.Filtering);
        changed |= DrawNullableFloat("Region Min Size",   ref workspace.Draft.BuildSettings.RegionMinSize,   settingsDefaults.RegionMinSize);
        changed |= DrawNullableFloat("Region Merge Size", ref workspace.Draft.BuildSettings.RegionMergeSize, settingsDefaults.RegionMergeSize);
        changed |= DrawNullableEnum("Partitioning", ref workspace.Draft.BuildSettings.Partitioning, settingsDefaults.Partitioning);
        changed |= DrawNullableFloat("Poly Max Edge Len", ref workspace.Draft.BuildSettings.PolyMaxEdgeLen, settingsDefaults.PolyMaxEdgeLen);
        changed |= DrawNullableFloat
            ("Poly Max Simplification Error", ref workspace.Draft.BuildSettings.PolyMaxSimplificationError, settingsDefaults.PolyMaxSimplificationError);
        changed |= DrawNullableInt("Poly Max Verts", ref workspace.Draft.BuildSettings.PolyMaxVerts, settingsDefaults.PolyMaxVerts);
        changed |= DrawNullableFloat("Detail Sample Dist",      ref workspace.Draft.BuildSettings.DetailSampleDist,     settingsDefaults.DetailSampleDist);
        changed |= DrawNullableFloat("Detail Max Sample Error", ref workspace.Draft.BuildSettings.DetailMaxSampleError, settingsDefaults.DetailMaxSampleError);
        changed |= DrawNullableBool("Fast Build", ref workspace.Draft.BuildSettings.FastBuild, settingsDefaults.FastBuild);
        changed |= DrawNullableBool
            ("Generate Edge Climb Links", ref workspace.Draft.BuildSettings.GenerateEdgeClimbLinks, settingsDefaults.GenerateEdgeClimbLinks);
        changed |= DrawNullableBool("Generate Edge Jump Links", ref workspace.Draft.BuildSettings.GenerateEdgeJumpLinks, settingsDefaults.GenerateEdgeJumpLinks);
        changed |= DrawNullableFloat("Ground Tolerance",       ref workspace.Draft.BuildSettings.GroundTolerance,     settingsDefaults.GroundTolerance);
        changed |= DrawNullableFloat("Climb Down Distance",    ref workspace.Draft.BuildSettings.ClimbDownDistance,   settingsDefaults.ClimbDownDistance);
        changed |= DrawNullableFloat("Climb Down Max Height",  ref workspace.Draft.BuildSettings.ClimbDownMaxHeight,  settingsDefaults.ClimbDownMaxHeight);
        changed |= DrawNullableFloat("Climb Down Min Height",  ref workspace.Draft.BuildSettings.ClimbDownMinHeight,  settingsDefaults.ClimbDownMinHeight);
        changed |= DrawNullableFloat("Edge Jump End Distance", ref workspace.Draft.BuildSettings.EdgeJumpEndDistance, settingsDefaults.EdgeJumpEndDistance);
        changed |= DrawNullableFloat("Edge Jump Height",       ref workspace.Draft.BuildSettings.EdgeJumpHeight,      settingsDefaults.EdgeJumpHeight);
        changed |= DrawNullableFloat("Edge Jump Max Drop",     ref workspace.Draft.BuildSettings.EdgeJumpMaxDrop,     settingsDefaults.EdgeJumpMaxDrop);
        changed |= DrawNullableFloat("Edge Jump Min Drop",     ref workspace.Draft.BuildSettings.EdgeJumpMinDrop,     settingsDefaults.EdgeJumpMinDrop);
        changed |= DrawNullableFloat("Ground Tile Size",       ref workspace.Draft.BuildSettings.GroundTileSize,      settingsDefaults.GroundTileSize);
        changed |= DrawNullableInt("Ground Tile Count Max", ref workspace.Draft.BuildSettings.GroundTileCountMax, settingsDefaults.GroundTileCountMax);
        changed |= DrawNullableIntArray("Volume Tiles", ref workspace.Draft.BuildSettings.VolumeTiles, settingsDefaults.VolumeTiles);

        if (changed)
            CommitDraftChange();
    }

    private void DrawFlyingInspector()
    {
        var current = workspace.Draft.FlyingSupportedOverride;
        var next = current switch
        {
            true  => 1,
            false => 2,
            _     => 0
        };

        if (DrawEnumCombo("Flying Support", ref next, ["默认", "启用", "禁用"]))
        {
            workspace.Draft.FlyingSupportedOverride = next switch
            {
                1 => true,
                2 => false,
                _ => null
            };
            CommitDraftChange();
        }
    }

    private void DrawMeshRemovalInspector()
    {
        if (!TryGetItem(workspace.Draft.MeshRemovals, selection.Index, out var item))
            return;

        var changed = false;
        changed |= DrawBool("Enabled", ref item.Enabled);
        changed |= DrawString("Mesh Key", ref item.MeshKey);

        if (ImGui.Button("删除这一项"))
        {
            RemoveItem(workspace.Draft.MeshRemovals, selection.Index);
            return;
        }

        if (changed)
            CommitDraftChange();
    }

    private void DrawInstancePatchInspector()
    {
        if (!TryGetItem(workspace.Draft.InstancePatches, selection.Index, out var item))
            return;

        var changed = false;
        changed |= DrawBool("Enabled", ref item.Enabled);
        changed |= DrawString("Mesh Key", ref item.MeshKey);
        changed |= DrawEnumCombo("Kind", ref item.Kind);
        changed |= DrawInt("Instance Index", ref item.InstanceIndex);
        changed |= DrawUInt64("Instance ID", ref item.InstanceId);
        changed |= DrawMatrix("World Transform", ref item.WorldTransform);
        changed |= DrawFlags("Set Flags",   ref item.ForceSetPrimFlags);
        changed |= DrawFlags("Clear Flags", ref item.ForceClearPrimFlags);

        if (ImGui.Button("删除这一项"))
        {
            RemoveItem(workspace.Draft.InstancePatches, selection.Index);
            return;
        }

        if (changed)
            CommitDraftChange();
    }

    private void DrawPartPatchInspector()
    {
        if (!TryGetItem(workspace.Draft.PartPatches, selection.Index, out var item))
            return;

        var changed = false;
        changed |= DrawBool("Enabled", ref item.Enabled);
        changed |= DrawString("Mesh Key", ref item.MeshKey);
        changed |= DrawInt("Part Index", ref item.PartIndex);
        changed |= DrawEnumCombo("Kind", ref item.Kind);
        changed |= DrawInt("Vertex Index",    ref item.VertexIndex);
        changed |= DrawInt("Primitive Index", ref item.PrimitiveIndex);
        changed |= DrawVector3("Position", ref item.Position);
        changed |= DrawInt("V1", ref item.V1);
        changed |= DrawInt("V2", ref item.V2);
        changed |= DrawInt("V3", ref item.V3);
        changed |= DrawUInt64("Material", ref item.Material);
        changed |= DrawFlags("Flags",       ref item.Flags);
        changed |= DrawFlags("Set Flags",   ref item.ForceSetPrimFlags);
        changed |= DrawFlags("Clear Flags", ref item.ForceClearPrimFlags);

        if (ImGui.Button("删除这一项"))
        {
            RemoveItem(workspace.Draft.PartPatches, selection.Index);
            return;
        }

        if (changed)
            CommitDraftChange();
    }

    private void DrawColliderInsertionInspector()
    {
        if (!TryGetItem(workspace.Draft.ColliderInsertions, selection.Index, out var item))
            return;

        var changed = false;
        changed |= DrawBool("Enabled", ref item.Enabled);
        changed |= DrawEnumCombo("Kind", ref item.Kind);
        changed |= DrawBoundsEditor("几何", ref item.Min, ref item.Max);
        changed |= DrawFlags("Set Flags",   ref item.ForceSetPrimFlags);
        changed |= DrawFlags("Clear Flags", ref item.ForceClearPrimFlags);

        if (ImGui.Button("删除这一项"))
        {
            RemoveItem(workspace.Draft.ColliderInsertions, selection.Index);
            return;
        }

        if (changed)
            CommitDraftChange();
    }

    private void DrawMeshLinkInspector()
    {
        if (!TryGetItem(workspace.Draft.MeshLinks, selection.Index, out var item))
            return;

        var changed = false;
        changed |= DrawBool("Enabled", ref item.Enabled);
        changed |= DrawEnumCombo("Kind", ref item.Kind);
        changed |= DrawVector3("Start", ref item.Start);
        changed |= DrawVector3("End",   ref item.End);

        if (ImGui.Button("删除这一项"))
        {
            RemoveItem(workspace.Draft.MeshLinks, selection.Index);
            return;
        }

        if (changed)
            CommitDraftChange();
    }

    private void DrawOffMeshInspector()
    {
        if (!TryGetItem(workspace.Draft.OffMeshConnections, selection.Index, out var item))
            return;

        var changed = false;
        changed |= DrawBool("Enabled", ref item.Enabled);
        changed |= DrawVector3("Start", ref item.Start);
        changed |= DrawVector3("End",   ref item.End);
        changed |= DrawFloat("Radius", ref item.Radius, 0.05f, 0.01f, 10f);
        changed |= DrawBool("Bidirectional", ref item.Bidirectional);
        changed |= DrawInt("UserId", ref item.UserId);
        changed |= DrawEnumCombo("Area", ref item.Area);
        changed |= DrawFlags("Flags", ref item.Flags);
        changed |= DrawEnumCombo("Kind", ref item.Kind);

        if (ImGui.Button("删除这一项"))
        {
            RemoveItem(workspace.Draft.OffMeshConnections, selection.Index);
            return;
        }

        if (changed)
            CommitDraftChange();
    }

    private void DrawPreviewMeshInspector()
    {
        if (selection.Key == null || previewBuilder.Extractor == null || !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh))
            return;

        ImGui.TextUnformatted(selection.Key);
        ImGui.TextUnformatted($"{mesh.Parts.Count} parts, {mesh.Instances.Count} instances");
        ImGui.TextUnformatted($"Bounds: {mesh.LocalBounds.Min:f3} - {mesh.LocalBounds.Max:f3}");

        if (ImGui.Button("加入 mesh 删除清单"))
            AddMeshRemovalFromPreview(selection.Key);
    }

    private void DrawPreviewInstanceInspector()
    {
        if (selection.Key            == null                                           ||
            previewBuilder.Extractor == null                                           ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                        ||
            selection.Index >= mesh.Instances.Count)
            return;

        var instance = mesh.Instances[selection.Index];
        ImGui.TextUnformatted($"Mesh: {selection.Key}");
        ImGui.TextUnformatted($"Instance: {instance.Id:X}");
        ImGui.TextUnformatted($"Bounds: {instance.WorldBounds.Min:f3} - {instance.WorldBounds.Max:f3}");

        var transformPatch = workspace.Draft.InstancePatches.FirstOrDefault
            (x => x.MeshKey == selection.Key && x.InstanceIndex == selection.Index && x.Kind == DraftSceneInstancePatchKind.Transform);
        var transform = transformPatch?.WorldTransform ?? DraftMatrix4x3.FromRuntime(instance.WorldTransform);

        if (DrawMatrix("Transform", ref transform))
        {
            transformPatch ??= new()
            {
                MeshKey       = selection.Key,
                InstanceIndex = selection.Index,
                InstanceId    = instance.Id,
                Kind          = DraftSceneInstancePatchKind.Transform
            };
            transformPatch.WorldTransform = transform;
            if (!workspace.Draft.InstancePatches.Contains(transformPatch))
                workspace.Draft.InstancePatches.Add(transformPatch);
            CommitDraftChange();
        }

        var flagsPatch = workspace.Draft.InstancePatches.FirstOrDefault
                             (x => x.MeshKey == selection.Key && x.InstanceIndex == selection.Index && x.Kind == DraftSceneInstancePatchKind.SetFlags) ??
                         new()
                         {
                             MeshKey       = selection.Key,
                             InstanceIndex = selection.Index,
                             InstanceId    = instance.Id,
                             Kind          = DraftSceneInstancePatchKind.SetFlags
                         };

        var flagsChanged = false;
        var setFlags     = flagsPatch.ForceSetPrimFlags;
        var clearFlags   = flagsPatch.ForceClearPrimFlags;
        flagsChanged |= DrawFlags("Set Flags",   ref setFlags);
        flagsChanged |= DrawFlags("Clear Flags", ref clearFlags);

        if (flagsChanged)
        {
            flagsPatch.ForceSetPrimFlags   = setFlags;
            flagsPatch.ForceClearPrimFlags = clearFlags;
            if (!workspace.Draft.InstancePatches.Contains(flagsPatch))
                workspace.Draft.InstancePatches.Add(flagsPatch);
            CommitDraftChange();
        }

        if (ImGui.Button("加入实例变换补丁"))
            AddInstancePatchFromPreview(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.Transform);
        ImGui.SameLine();
        if (ImGui.Button("加入实例 flags 补丁"))
            AddInstancePatchFromPreview(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.SetFlags);
        ImGui.SameLine();
        if (ImGui.Button("移除这个实例"))
            AddInstancePatchFromPreview(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.RemoveInstance);
        ImGui.SameLine();
        if (ImGui.Button("清空这个 mesh 的全部实例"))
            AddInstancePatchFromPreview(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.ClearInstances);
    }

    private void DrawPreviewPartInspector()
    {
        if (selection.Key            == null                                           ||
            previewBuilder.Extractor == null                                           ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                        ||
            selection.Index >= mesh.Parts.Count)
            return;

        var part                   = mesh.Parts[selection.Index];
        var selectedVertexIndex    = selection.Kind == SelectionKind.PreviewVertex ? selection.SubIndex : -1;
        var selectedPrimitiveIndex = selection.Kind == SelectionKind.PreviewPrimitive ? selection.SubIndex : -1;
        ImGui.TextUnformatted($"Mesh: {selection.Key}");
        ImGui.TextUnformatted($"Part: {selection.Index}");
        ImGui.TextUnformatted($"{part.Vertices.Count} vertices, {part.Primitives.Count} primitives");
        ImGui.TextUnformatted($"Bounds: {part.LocalBounds.Min:f3} - {part.LocalBounds.Max:f3}");

        if (selectedVertexIndex >= 0 && selectedVertexIndex < part.Vertices.Count)
        {
            var vertexPatch = workspace.Draft.PartPatches.FirstOrDefault
                (x => PartPatchMatches(x, selection.Key, selection.Index, DraftScenePartPatchKind.Vertex, selectedVertexIndex));
            var position = vertexPatch?.Position ?? part.Vertices[selectedVertexIndex];
            ImGui.TextUnformatted($"Selected vertex: {selectedVertexIndex}");

            if (DrawVector3("Position", ref position))
            {
                vertexPatch ??= new()
                {
                    MeshKey     = selection.Key,
                    PartIndex   = selection.Index,
                    Kind        = DraftScenePartPatchKind.Vertex,
                    VertexIndex = selectedVertexIndex
                };
                vertexPatch.Position = position;
                if (!workspace.Draft.PartPatches.Contains(vertexPatch))
                    workspace.Draft.PartPatches.Add(vertexPatch);
                CommitDraftChange();
            }

            if (ImGui.Button("删除顶点位置补丁"))
                RemoveMatchingPartPatch(selection.Key, selection.Index, DraftScenePartPatchKind.Vertex, selectedVertexIndex);
        }
        else if (ImGui.Button("加入顶点位置补丁")) AddPartPatchFromPreview(mesh, selection.Key, selection.Index, DraftScenePartPatchKind.Vertex);

        ImGui.SameLine();

        if (selectedPrimitiveIndex >= 0 && selectedPrimitiveIndex < part.Primitives.Count)
        {
            var primitive = part.Primitives[selectedPrimitiveIndex];
            var flagsPatch = workspace.Draft.PartPatches.FirstOrDefault
                (x => PartPatchMatches(x, selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveFlags, selectedPrimitiveIndex));
            var primitiveFlags = flagsPatch?.Flags ?? primitive.Flags;
            ImGui.TextUnformatted($"Selected primitive: {selectedPrimitiveIndex} {primitive.V1}x{primitive.V2}x{primitive.V3}");
            var flagsChanged = DrawFlags("Flags", ref primitiveFlags);

            if (flagsChanged)
            {
                flagsPatch ??= new()
                {
                    MeshKey        = selection.Key,
                    PartIndex      = selection.Index,
                    Kind           = DraftScenePartPatchKind.PrimitiveFlags,
                    PrimitiveIndex = selectedPrimitiveIndex
                };
                flagsPatch.Flags = primitiveFlags;
                if (!workspace.Draft.PartPatches.Contains(flagsPatch))
                    workspace.Draft.PartPatches.Add(flagsPatch);
                CommitDraftChange();
            }

            var editPatch = workspace.Draft.PartPatches.FirstOrDefault
                (x => PartPatchMatches(x, selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveEdit, selectedPrimitiveIndex));
            var v1          = editPatch?.V1       ?? primitive.V1;
            var v2          = editPatch?.V2       ?? primitive.V2;
            var v3          = editPatch?.V3       ?? primitive.V3;
            var editFlags   = editPatch?.Flags    ?? primitive.Flags;
            var material    = editPatch?.Material ?? primitive.Material;
            var editChanged = false;
            editChanged |= DrawInt("V1", ref v1);
            editChanged |= DrawInt("V2", ref v2);
            editChanged |= DrawInt("V3", ref v3);
            editChanged |= DrawFlags("Edit Flags", ref editFlags);
            editChanged |= DrawUInt64("Material", ref material);

            if (editChanged)
            {
                editPatch ??= new()
                {
                    MeshKey        = selection.Key,
                    PartIndex      = selection.Index,
                    Kind           = DraftScenePartPatchKind.PrimitiveEdit,
                    PrimitiveIndex = selectedPrimitiveIndex
                };
                editPatch.V1       = v1;
                editPatch.V2       = v2;
                editPatch.V3       = v3;
                editPatch.Flags    = editFlags;
                editPatch.Material = material;
                if (!workspace.Draft.PartPatches.Contains(editPatch))
                    workspace.Draft.PartPatches.Add(editPatch);
                CommitDraftChange();
            }

            if (ImGui.Button("删除三角 flags 补丁"))
                RemoveMatchingPartPatch(selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveFlags, selectedPrimitiveIndex);
            ImGui.SameLine();
            if (ImGui.Button("删除三角高级编辑补丁"))
                RemoveMatchingPartPatch(selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveEdit, selectedPrimitiveIndex);
        }
        else
        {
            if (ImGui.Button("加入三角 flags 补丁"))
                AddPartPatchFromPreview(mesh, selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveFlags);
            ImGui.SameLine();
            if (ImGui.Button("加入三角高级编辑补丁"))
                AddPartPatchFromPreview(mesh, selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveEdit);
        }
    }

    private void HandlePicking()
    {
        if (pickKind is PickKind.None or PickKind.SelectCollider)
            return;

        var clicked = TakeWorldPickClick();
        if (!IsWorldClickAllowed())
            return;

        if (!collision.TryGetMouseRaycastHit(out var hit))
            return;

        currentPickPoint = hit.Point;
        dd.DrawWorldPointFilled(hit.Point, 5, 0xFFFFFF00);
        collision.VisualizeCollider(hit.Object, default, default);

        if (!clicked)
            return;

        var point = hit.Point;

        if (pendingPickPoint == null)
        {
            pendingPickPoint = point;
            statusText           = $"{GetPickKindTitle(pickKind)}: 已记录第 1 个点 {point:f3}, 等待第 2 个世界点";
            return;
        }

        var first = pendingPickPoint.Value;
        pendingPickPoint = null;
        var completedKind = pickKind;

        switch (pickKind)
        {
            case PickKind.Aabb:
                AddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Aabb);
                break;
            case PickKind.Cylinder:
                AddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Cylinder);
                break;
            case PickKind.LinkPoints:
                AddMeshLink(first, point, DraftMeshLinkKind.Points);
                break;
            case PickKind.LinkDrop:
                AddMeshLink(first, point, DraftMeshLinkKind.Drop);
                break;
            case PickKind.LinkClientPath:
                AddMeshLink(first, point, DraftMeshLinkKind.ClientPath);
                break;
            case PickKind.OffMesh:
                AddOffMeshConnection(first, point);
                break;
        }

        statusText           = $"已创建 {GetPickKindTitle(completedKind)}";
        pickKind         = PickKind.None;
        currentPickPoint = null;
    }

    private void HandleWorldSelection()
    {
        var clicked = TakeWorldSelectClick();
        if (!IsWorldClickAllowed())
            return;

        if (!TryGetWorldSelectionRay(out var rayOrigin, out var rayDirection))
            return;

        var hasHit = collision.TryGetMouseRaycastHit(out var hit);

        if (hasHit)
        {
            currentPickPoint = hit.Point;
            dd.DrawWorldPointFilled(hit.Point, 5, 0xFFFFFF00);
            collision.VisualizeCollider(hit.Object, default, default);
        }

        if (!clicked)
            return;

        if (TrySelectColliderInsertion(rayOrigin, rayDirection))
            return;

        if (hasHit && TrySelectPreviewInstance(hit.Object))
            return;

        statusText = "未命中可编辑对象";
    }

    private bool TrySelectPreviewInstance(Collider* collider)
    {
        if (collider == null || previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return false;

        var layoutObjectId = collider->LayoutObjectId << 32 | collider->LayoutObjectId >> 32;

        foreach (var (key, mesh) in previewBuilder.Extractor.Meshes.OrderBy(static x => x.Key, StringComparer.Ordinal))
        {
            var instanceIndex = 0;

            foreach (var instance in mesh.Instances)
            {
                if (instance.Id != layoutObjectId)
                {
                    ++instanceIndex;
                    continue;
                }

                selection = new(SelectionKind.PreviewInstance, instanceIndex, Key: key);
                statusText    = $"已选中预览实例: {key} #{instanceIndex}";
                return true;
            }
        }

        return false;
    }

    private bool TrySelectColliderInsertion(Vector3 rayOrigin, Vector3 rayDirection)
    {
        Selection? bestSelection = null;
        var        bestDistance  = float.MaxValue;

        for (var i = 0; i < workspace.Draft.ColliderInsertions.Count; ++i)
        {
            var item = workspace.Draft.ColliderInsertions[i];
            if (!item.Enabled)
                continue;

            var bounds = new AABB { Min = item.Min, Max = item.Max };
            if (!RayIntersectsAabb(rayOrigin, rayDirection, bounds, out var distance))
                continue;

            if (distance >= bestDistance)
                continue;

            bestDistance  = distance;
            bestSelection = new(SelectionKind.ColliderInsertion, i);
        }

        if (bestSelection == null)
            return false;

        selection = bestSelection;
        statusText    = $"已选中碰撞插入: {workspace.Draft.ColliderInsertions[selection.Index].Kind}";
        return true;
    }

    private bool TryGetWorldSelectionRay(out Vector3 origin, out Vector3 direction)
    {
        origin    = default;
        direction = default;

        if (!TryGetViewportCursorPosition(out var screenPos))
            return false;

        if (dd.ViewportSize.X <= 0 || dd.ViewportSize.Y <= 0)
            return false;

        var clipPos = new Vector3(2 * screenPos.X / dd.ViewportSize.X - 1, 1 - 2 * screenPos.Y / dd.ViewportSize.Y, 1);
        Matrix4x4.Invert(dd.ViewProj, out var invViewProj);
        var cameraPosAtPlaneP = Vector4.Transform(clipPos, invViewProj);
        if (MathF.Abs(cameraPosAtPlaneP.W) < 0.0001f)
            return false;

        var cameraPosAtPlane = new Vector3
            (cameraPosAtPlaneP.X / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Y / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Z / cameraPosAtPlaneP.W);
        var dir = cameraPosAtPlane - dd.Origin;
        if (dir.LengthSquared() < 0.0001f)
            return false;

        origin    = dd.Origin;
        direction = Vector3.Normalize(dir);
        return true;
    }

    private static bool TryGetViewportCursorPosition(out Vector2 screenPos)
    {
        if (!GetCursorPos(out var cursor))
        {
            screenPos = default;
            return false;
        }

        screenPos = new Vector2(cursor.X, cursor.Y) - ImGuiHelpers.MainViewport.Pos;
        var windowSize = ImGuiHelpers.MainViewport.Size;
        return screenPos.X >= 0 && screenPos.X <= windowSize.X && screenPos.Y >= 0 && screenPos.Y <= windowSize.Y;
    }

    private static bool RayIntersectsAabb(Vector3 origin, Vector3 direction, AABB bounds, out float distance)
    {
        var tMin = 0f;
        var tMax = float.MaxValue;

        if (!IntersectsAxis(origin.X, direction.X, bounds.Min.X, bounds.Max.X, ref tMin, ref tMax) ||
            !IntersectsAxis(origin.Y, direction.Y, bounds.Min.Y, bounds.Max.Y, ref tMin, ref tMax) ||
            !IntersectsAxis(origin.Z, direction.Z, bounds.Min.Z, bounds.Max.Z, ref tMin, ref tMax))
        {
            distance = 0f;
            return false;
        }

        distance = tMin;
        return tMax >= 0f;
    }

    private static bool IntersectsAxis(float origin, float direction, float min, float max, ref float tMin, ref float tMax)
    {
        const float EPSILON = 0.000001f;
        if (MathF.Abs(direction) < EPSILON)
            return origin >= min && origin <= max;

        var invDir = 1f             / direction;
        var t1     = (min - origin) * invDir;
        var t2     = (max - origin) * invDir;
        if (t1 > t2)
            (t1, t2) = (t2, t1);

        tMin = MathF.Max(tMin, t1);
        tMax = MathF.Min(tMax, t2);
        return tMin <= tMax;
    }

    private void DrawPreviewInstancesOverlay()
    {
        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        if (selection is not { Kind: SelectionKind.PreviewInstance, Key: not null }    ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                        ||
            selection.Index >= mesh.Instances.Count)
            return;

        var instance = mesh.Instances[selection.Index];
        dd.DrawWorldAABB(instance.WorldBounds, 0xFFFFD94A, 3);

        if (mesh.Parts.Count > 0)
        {
            foreach (var part in mesh.Parts)
                DrawMeshPreview(part, instance.WorldTransform, 0xFFFFD94A);
        }
    }

    private void DrawWorldOverlay()
    {
        currentPickPoint = null;

        switch (pickKind)
        {
            case PickKind.SelectCollider:
                HandleWorldSelection();
                break;
            case PickKind.None:
                break;
            default:
                HandlePicking();
                break;
        }

        DrawPreviewInstancesOverlay();

        if (pendingPickPoint is { } pending)
        {
            dd.DrawWorldPointFilled(pending, 6, 0xFFFF00FF);
            if (currentPickPoint is { } current)
                DrawPendingPickPreview(pending, current);
        }

        foreach (var insertion in workspace.Draft.ColliderInsertions.Where(static x => x.Enabled))
        {
            var selected = selection is { Kind: SelectionKind.ColliderInsertion, Index: var selectedIndex and >= 0 } &&
                           selectedIndex < workspace.Draft.ColliderInsertions.Count                                  &&
                           ReferenceEquals(workspace.Draft.ColliderInsertions[selectedIndex], insertion);
            var color = selected
                            ? 0xFFFFD94A
                            : insertion.Kind == DraftSceneColliderInsertionKind.Cylinder
                                ? 0xFF00FF00
                                : 0xFF00FFFF;
            if (insertion.Kind == DraftSceneColliderInsertionKind.Cylinder)
                dd.DrawWorldCylinder((insertion.Min + insertion.Max) * 0.5f, (insertion.Max - insertion.Min) * 0.5f, color, selected ? 3 : 2);
            else
                dd.DrawWorldAABB((insertion.Min + insertion.Max) * 0.5f, (insertion.Max - insertion.Min) * 0.5f, color, selected ? 3 : 2);
        }

        foreach (var link in workspace.Draft.MeshLinks.Where(static x => x.Enabled))
            dd.DrawWorldLine(link.Start, link.End, 0xFFAAFF00, 2);

        foreach (var link in workspace.Draft.OffMeshConnections.Where(static x => x.Enabled))
            dd.DrawWorldArc(link.Start, link.End, 0.15f, 3f, 3f, 0xFFFF8800, 2);
    }

    private void DrawPendingPickPreview(Vector3 first, Vector3 current)
    {
        switch (pickKind)
        {
            case PickKind.Aabb:
            case PickKind.Cylinder:
            {
                var min = Vector3.Min(first, current);
                var max = Vector3.Max(first, current);
                NormalizeBounds(ref min, ref max);
                var color = pickKind == PickKind.Cylinder ? 0xFF33FF66 : 0xFF33DDFF;
                if (pickKind == PickKind.Cylinder)
                    dd.DrawWorldCylinder((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                else
                    dd.DrawWorldAABB((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                break;
            }
            case PickKind.LinkPoints:
            case PickKind.LinkDrop:
            case PickKind.LinkClientPath:
                dd.DrawWorldLine(first, current, 0xFFAAFF00, 2);
                break;
            case PickKind.OffMesh:
                dd.DrawWorldArc(first, current, 0.15f, 3f, 3f, 0xFFFF8800, 2);
                break;
        }
    }

    private void AddColliderInsertion(Vector3 a, Vector3 b, DraftSceneColliderInsertionKind kind)
    {
        var min = Vector3.Min(a, b);
        var max = Vector3.Max(a, b);
        NormalizeBounds(ref min, ref max);

        ApplyDraftChange
        (() =>
            {
                workspace.Draft.ColliderInsertions.Add
                (
                    new()
                    {
                        Kind              = kind,
                        Min               = min,
                        Max               = max,
                        ForceSetPrimFlags = SceneExtractor.PrimitiveFlags.ForceUnwalkable
                    }
                );
                selection = new(SelectionKind.ColliderInsertion, workspace.Draft.ColliderInsertions.Count - 1);
                statusText    = kind == DraftSceneColliderInsertionKind.Cylinder ? "已添加圆柱障碍" : "已添加 AABB 障碍";
            }
        );
    }

    private void AddMeshLink(Vector3 a, Vector3 b, DraftMeshLinkKind kind)
    {
        ApplyDraftChange
        (() =>
            {
                workspace.Draft.MeshLinks.Add(new() { Kind = kind, Start = a, End = b });
                selection = new(SelectionKind.MeshLink, workspace.Draft.MeshLinks.Count - 1);
                statusText = kind switch
                {
                    DraftMeshLinkKind.Points     => "已添加网格连线",
                    DraftMeshLinkKind.Drop       => "已添加下落连接",
                    DraftMeshLinkKind.ClientPath => "已添加客户端路径连接",
                    _                            => "已添加 mesh link"
                };
            }
        );
    }

    private void AddOffMeshConnection(Vector3 a, Vector3 b)
    {
        ApplyDraftChange
        (() =>
            {
                workspace.Draft.OffMeshConnections.Add(new() { Start = a, End = b });
                selection = new(SelectionKind.OffMeshConnection, workspace.Draft.OffMeshConnections.Count - 1);
                statusText    = "已添加 off-mesh 连接";
            }
        );
    }

    private void AddMeshRemovalFromPreview(string key)
    {
        var existingIndex = workspace.Draft.MeshRemovals.FindIndex(x => x.MeshKey == key);

        if (existingIndex >= 0)
        {
            selection = new(SelectionKind.MeshRemoval, existingIndex);
            statusText    = "已选中已有 mesh 删除项";
            return;
        }

        ApplyDraftChange
        (() =>
            {
                workspace.Draft.MeshRemovals.Add(new() { MeshKey = key });
                selection = new(SelectionKind.MeshRemoval, workspace.Draft.MeshRemovals.Count - 1);
                statusText    = "已加入 mesh 删除清单";
            }
        );
    }

    private void AddInstancePatchFromPreview(SceneExtractor.Mesh mesh, string key, int index, DraftSceneInstancePatchKind kind)
    {
        var inst = mesh.Instances[index];
        var existingIndex = workspace.Draft.InstancePatches.FindIndex
            (x => x.MeshKey == key && x.Kind == kind && (kind == DraftSceneInstancePatchKind.ClearInstances || x.InstanceIndex == index));

        if (existingIndex >= 0)
        {
            selection = new(SelectionKind.InstancePatch, existingIndex);
            statusText    = "已选中已有实例补丁";
            return;
        }

        ApplyDraftChange
        (() =>
            {
                workspace.Draft.InstancePatches.Add
                (
                    new()
                    {
                        MeshKey             = key,
                        InstanceIndex       = index,
                        InstanceId          = inst.Id,
                        Kind                = kind,
                        WorldTransform      = DraftMatrix4x3.FromRuntime(inst.WorldTransform),
                        ForceSetPrimFlags   = inst.ForceSetPrimFlags,
                        ForceClearPrimFlags = inst.ForceClearPrimFlags
                    }
                );
                selection = new(SelectionKind.InstancePatch, workspace.Draft.InstancePatches.Count - 1);
                statusText    = $"已加入实例补丁: {kind}";
            }
        );
    }

    private void AddPartPatchFromPreview(SceneExtractor.Mesh mesh, string key, int partIndex, DraftScenePartPatchKind kind, int subIndex = -1)
    {
        var part        = mesh.Parts[partIndex];
        var vertexIndex = kind == DraftScenePartPatchKind.Vertex && subIndex >= 0 && subIndex < part.Vertices.Count ? subIndex : 0;
        var primitiveIndex = (kind == DraftScenePartPatchKind.PrimitiveFlags || kind == DraftScenePartPatchKind.PrimitiveEdit) &&
                             subIndex >= 0                                                                                     &&
                             subIndex < part.Primitives.Count
                                 ? subIndex
                                 : 0;
        var existingIndex = workspace.Draft.PartPatches.FindIndex
            (x => PartPatchMatches(x, key, partIndex, kind, kind == DraftScenePartPatchKind.Vertex ? vertexIndex : primitiveIndex));

        if (existingIndex >= 0)
        {
            selection = new(SelectionKind.PartPatch, existingIndex);
            statusText    = "已选中已有顶点 / 三角补丁";
            return;
        }

        ApplyDraftChange
        (() =>
            {
                workspace.Draft.PartPatches.Add
                (
                    new()
                    {
                        MeshKey        = key,
                        PartIndex      = partIndex,
                        Kind           = kind,
                        VertexIndex    = vertexIndex,
                        PrimitiveIndex = primitiveIndex,
                        Position       = part.Vertices.Count   > 0 ? part.Vertices[vertexIndex] : default,
                        V1             = part.Primitives.Count > 0 ? part.Primitives[primitiveIndex].V1 : 0,
                        V2             = part.Primitives.Count > 0 ? part.Primitives[primitiveIndex].V2 : 1,
                        V3             = part.Primitives.Count > 0 ? part.Primitives[primitiveIndex].V3 : 2,
                        Flags          = part.Primitives.Count > 0 ? part.Primitives[primitiveIndex].Flags : SceneExtractor.PrimitiveFlags.None,
                        Material       = part.Primitives.Count > 0 ? part.Primitives[primitiveIndex].Material : 0
                    }
                );
                selection = new(SelectionKind.PartPatch, workspace.Draft.PartPatches.Count - 1);
                statusText    = $"已加入顶点 / 三角补丁: {kind}";
            }
        );
    }

    private void RemoveMatchingPartPatch(string key, int partIndex, DraftScenePartPatchKind kind, int subIndex)
    {
        var patch = workspace.Draft.PartPatches.FirstOrDefault(x => PartPatchMatches(x, key, partIndex, kind, subIndex));
        if (patch == null)
            return;

        ApplyDraftChange(() => { workspace.Draft.PartPatches.Remove(patch); });
    }

    private static bool PartPatchMatches(DraftScenePartPatch patch, string key, int partIndex, DraftScenePartPatchKind kind, int subIndex)
    {
        if (patch.MeshKey != key || patch.PartIndex != partIndex || patch.Kind != kind)
            return false;

        return kind == DraftScenePartPatchKind.Vertex ? patch.VertexIndex == subIndex : patch.PrimitiveIndex == subIndex;
    }

    private void RebuildPreview()
    {
        if (!workspaceLoaded)
            return;

        var scene = new SceneDefinition();
        scene.FillFromActiveLayout();
        scene.TerritoryID = territoryID;

        var customization = new CustomizationDraftCustomization(NavmeshCustomizationRegistry.GetForScene(scene), workspace.Draft.Clone());
        previewDirty  = false;
        nextRebuildAt = DateTime.MinValue;
        statusText        = "重建中";
        previewBuilder.Rebuild(scene, customization, true);
    }

    private void ExportCurrentDraft()
    {
        if (string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory))
            workspace.Settings.ExportDirectory = Path.Combine(configDirectory.FullName, "customization-editor", "generated");

        var outputDirectory = new DirectoryInfo(workspace.Settings.ExportDirectory);
        lastExport = CustomizationDraftExporter.Export(workspace.Draft.Clone(), outputDirectory);
        statusText     = $"已导出 {lastExport.File.FullName}";
        SaveWorkspace(true);
    }

    private void CommitDraftChange()
    {
        if (historySuspended)
            return;

        undo.Push(historySnapshot.Clone());
        historySnapshot = workspace.Draft.Clone();
        redo.Clear();
        previewDirty  = true;
        nextRebuildAt = DateTime.UtcNow + TimeSpan.FromSeconds(Math.Clamp(workspace.Settings.RebuildDelaySeconds, 0.05f, 10f));
        SaveWorkspace();
    }

    private void ApplyDraftChange(Action action)
    {
        action();
        CommitDraftChange();
    }

    private void Undo()
    {
        if (undo.Count == 0)
            return;

        historySuspended = true;
        redo.Push(workspace.Draft.Clone());
        workspace.Draft  = undo.Pop();
        historySnapshot  = workspace.Draft.Clone();
        selection        = new(SelectionKind.Workspace);
        previewDirty     = true;
        nextRebuildAt    = DateTime.UtcNow;
        historySuspended = false;
        SaveWorkspace();
    }

    private void Redo()
    {
        if (redo.Count == 0)
            return;

        historySuspended = true;
        undo.Push(workspace.Draft.Clone());
        workspace.Draft  = redo.Pop();
        historySnapshot  = workspace.Draft.Clone();
        selection        = new(SelectionKind.Workspace);
        previewDirty     = true;
        nextRebuildAt    = DateTime.UtcNow;
        historySuspended = false;
        SaveWorkspace();
    }

    private void SaveWorkspace(bool force = false)
    {
        if (!workspaceLoaded)
            return;

        workspace.Draft.TerritoryID        = territoryID;
        workspace.Draft.TerritoryName      = territoryLabel;
        workspace.Settings.ExportDirectory = exportDirText;

        if (force || workspace.Settings.AutoSave)
            persistence.Save(workspace);
    }

    private static void NormalizeBounds(ref Vector3 min, ref Vector3 max)
    {
        if (MathF.Abs(max.Y - min.Y) < 0.1f)
        {
            var center = (min + max) * 0.5f;
            min = new(center.X - 0.5f, center.Y - 1f, center.Z - 0.5f);
            max = new(center.X + 0.5f, center.Y + 1f, center.Z + 0.5f);
        }
    }

    private bool TryGetItem<T>(List<T> items, int index, out T item)
    {
        if (index < 0 || index >= items.Count)
        {
            item = default!;
            return false;
        }

        item = items[index];
        return true;
    }

    private void RemoveItem<T>(List<T> items, int index)
    {
        ApplyDraftChange
        (() =>
            {
                if (index < 0 || index >= items.Count)
                    return;

                items.RemoveAt(index);
                selection = new(SelectionKind.Workspace);
            }
        );
    }

    private static bool DrawBool(string label, ref bool value)
    {
        var changed = ImGui.Checkbox(label, ref value);
        return changed;
    }

    private static bool DrawString(string label, ref string value)
    {
        var next = value;
        if (!ImGui.InputText(label, ref next))
            return false;

        value = next;
        return true;
    }

    private static bool DrawInt(string label, ref int value) =>
        ImGui.InputInt(label, ref value);

    private static bool DrawUInt64(string label, ref ulong value)
    {
        var text = value.ToString("X", CultureInfo.InvariantCulture);
        if (!ImGui.InputText(label, ref text))
            return false;

        if (string.IsNullOrWhiteSpace(text))
            return false;

        if (ulong.TryParse
                (text.StartsWith("0x", StringComparison.OrdinalIgnoreCase) ? text[2..] : text, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var parsed))
        {
            value = parsed;
            return true;
        }

        return false;
    }

    private static bool DrawFloat(string label, ref float value, float speed, float min = -10000f, float max = 10000f) =>
        ImGui.DragFloat(label, ref value, speed, min, max, "%.3f");

    private static bool DrawVector3(string label, ref Vector3 value)
    {
        var changed = false;

        if (ImGui.TreeNodeEx(label, ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.PushItemWidth(90);
            changed |= ImGui.DragFloat("X", ref value.X, 0.1f, -100000, 100000, "%.3f");
            changed |= ImGui.DragFloat("Y", ref value.Y, 0.1f, -100000, 100000, "%.3f");
            changed |= ImGui.DragFloat("Z", ref value.Z, 0.1f, -100000, 100000, "%.3f");
            ImGui.PopItemWidth();
            ImGui.TreePop();
        }

        return changed;
    }

    private static bool DrawBoundsEditor(string label, ref Vector3 min, ref Vector3 max)
    {
        var changed = false;
        if (!ImGui.TreeNodeEx(label, ImGuiTreeNodeFlags.DefaultOpen))
            return false;

        var center = (min + max) * 0.5f;
        var size   = max - min;
        changed |= DrawVector3("中心", ref center);
        changed |= DrawVector3("尺寸", ref size);

        if (changed)
        {
            size = Vector3.Max(size, new(0.01f));
            min  = center - size * 0.5f;
            max  = center + size * 0.5f;
        }

        if (ImGui.TreeNodeEx("原始 Min / Max"))
        {
            changed |= DrawVector3("Min", ref min);
            changed |= DrawVector3("Max", ref max);
            ImGui.TreePop();
        }

        ImGui.TreePop();
        return changed;
    }

    private static bool DrawMatrix(string label, ref DraftMatrix4x3 matrix)
    {
        var changed = false;
        if (!ImGui.TreeNodeEx(label, ImGuiTreeNodeFlags.DefaultOpen))
            return false;

        var translation = matrix.Translation;
        var scale       = matrix.GetScale();
        changed |= DrawVector3("Translation", ref translation);
        changed |= DrawVector3("Scale",       ref scale);
        if (changed)
            matrix.SetTranslationScale(translation, scale);

        if (ImGui.TreeNodeEx("Raw Matrix4x3"))
        {
            changed |= DrawVector3("Row0", ref matrix.Row0);
            changed |= DrawVector3("Row1", ref matrix.Row1);
            changed |= DrawVector3("Row2", ref matrix.Row2);
            changed |= DrawVector3("Row3", ref matrix.Row3);
            ImGui.TreePop();
        }

        ImGui.TreePop();
        return changed;
    }

    private static bool DrawNullableFloat(string label, ref float? value, float fallback)
    {
        var enabled = value.HasValue;
        var changed = ImGui.Checkbox($"启用##{label}", ref enabled);
        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (value.HasValue)
            {
                value = null;
                return true;
            }

            return changed;
        }

        var current = value ?? fallback;
        ImGui.PushItemWidth(200);

        if (ImGui.DragFloat($"##{label}_value", ref current, 0.1f, -100000, 100000, "%.3f"))
        {
            value   = current;
            changed = true;
        }

        ImGui.PopItemWidth();
        return changed;
    }

    private static bool DrawNullableInt(string label, ref int? value, int fallback)
    {
        var enabled = value.HasValue;
        var changed = ImGui.Checkbox($"启用##{label}", ref enabled);
        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (value.HasValue)
            {
                value = null;
                return true;
            }

            return changed;
        }

        var current = value ?? fallback;
        ImGui.PushItemWidth(200);

        if (ImGui.InputInt($"##{label}_value", ref current))
        {
            value   = current;
            changed = true;
        }

        ImGui.PopItemWidth();
        return changed;
    }

    private static bool DrawNullableBool(string label, ref bool? value, bool fallback)
    {
        var enabled = value.HasValue;
        var changed = ImGui.Checkbox($"启用##{label}", ref enabled);
        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (value.HasValue)
            {
                value = null;
                return true;
            }

            return changed;
        }

        var current = value ?? fallback;

        if (ImGui.Checkbox($"##{label}_value", ref current))
        {
            value   = current;
            changed = true;
        }

        return changed;
    }

    private static bool DrawNullableEnum<T>(string label, ref T? value, T? fallback) where T : struct, Enum
    {
        var enabled = value.HasValue;
        var changed = ImGui.Checkbox($"启用##{label}", ref enabled);
        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (value.HasValue)
            {
                value = null;
                return true;
            }

            return changed;
        }

        var current = value ?? fallback.GetValueOrDefault();

        if (DrawEnumCombo($"##{label}_value", ref current))
        {
            value   = current;
            changed = true;
        }

        return changed;
    }

    private static bool DrawNullableIntArray(string label, ref int[]? value, int[] fallback)
    {
        var enabled = value != null;
        var changed = ImGui.Checkbox($"启用##{label}", ref enabled);
        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (value != null)
            {
                value = null;
                return true;
            }

            return changed;
        }

        var current = value != null ? (int[])value.Clone() : (int[])fallback.Clone();
        ImGui.PushItemWidth(120);

        if (current.Length > 0 && ImGui.InputInt($"##{label}_0", ref current[0]))
        {
            value   = current;
            changed = true;
        }

        if (current.Length > 1)
        {
            ImGui.SameLine();

            if (ImGui.InputInt($"##{label}_1", ref current[1]))
            {
                value   = current;
                changed = true;
            }
        }

        ImGui.PopItemWidth();
        return changed;
    }

    private static bool DrawNullableFlags<T>(string label, ref T? value, T? fallback) where T : struct, Enum
    {
        var enabled = value.HasValue;
        var changed = ImGui.Checkbox($"启用##{label}", ref enabled);
        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (value.HasValue)
            {
                value = null;
                return true;
            }

            return changed;
        }

        var current = value ?? fallback.GetValueOrDefault();

        if (DrawFlags($"##{label}_value", ref current))
        {
            value   = current;
            changed = true;
        }

        return changed;
    }

    private static bool DrawEnumCombo<T>(string label, ref T value) where T : struct, Enum
    {
        var       changed = false;
        using var combo   = ImRaii.Combo(label, value.ToString());
        if (!combo)
            return false;

        foreach (var candidate in Enum.GetValues<T>())
        {
            var isSelected = EqualityComparer<T>.Default.Equals(candidate, value);

            if (ImGui.Selectable(candidate.ToString(), isSelected) && !isSelected)
            {
                value   = candidate;
                changed = true;
            }

            if (isSelected)
                ImGui.SetItemDefaultFocus();
        }

        return changed;
    }

    private static bool DrawEnumCombo(string label, ref int value, string[] options)
    {
        var       changed = false;
        using var combo   = ImRaii.Combo(label, options[Math.Clamp(value, 0, options.Length - 1)]);
        if (!combo)
            return false;

        for (var i = 0; i < options.Length; ++i)
        {
            var isSelected = i == value;

            if (ImGui.Selectable(options[i], isSelected) && !isSelected)
            {
                value   = i;
                changed = true;
            }

            if (i == value)
                ImGui.SetItemDefaultFocus();
        }

        return changed;
    }

    private static bool DrawFlags<T>(string label, ref T value) where T : struct, Enum
    {
        using var combo = ImRaii.Combo(label, value.ToString());
        if (!combo)
            return false;

        var changed = false;
        var current = Convert.ToUInt64(value);

        foreach (var candidate in Enum.GetValues<T>())
        {
            var raw = Convert.ToUInt64(candidate);
            if (raw == 0 || (raw & raw - 1) != 0)
                continue;

            var enabled = (current & raw) == raw;

            if (ImGui.Checkbox(candidate.ToString(), ref enabled))
            {
                current = enabled ? current | raw : current & ~raw;
                changed = true;
            }
        }

        if (changed)
            value = (T)Enum.ToObject(typeof(T), current);

        return changed;
    }

    private static bool DrawNullableFlags<T>(string label, ref T? value, T fallback, string[] dummy) where T : struct, Enum =>
        DrawNullableFlags(label, ref value, fallback);

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern short GetAsyncKeyState(int virtualKey);

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool GetCursorPos(out CursorPoint point);

    private struct CursorPoint
    {
        public int X;
        public int Y;
    }
    
    #region 常量
    
    private const int VK_LBUTTON = 0x01;
    private const int VK_ESCAPE  = 0x1B;
    
    #endregion
}
