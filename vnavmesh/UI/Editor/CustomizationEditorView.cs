using System.Drawing;
using System.Globalization;
using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface;
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

internal sealed class CustomizationEditorView
(
    Config             config,
    DebugDrawer        dd,
    DebugGameCollision collision,
    NavmeshManager     manager,
    DirectoryInfo      configDirectory
)
    : IDisposable
{
    private readonly CustomizationDraftPersistence persistence      = new(new DirectoryInfo(Path.Combine(configDirectory.FullName, "customization-editor")));
    private readonly CustomizationPreviewBuilder   previewBuilder   = new(manager, config);
    private readonly NavmeshSettings               settingsDefaults = new();
    private readonly NavmeshBuildProfile           profileDefaults  = new();

    private          CustomizationEditorWorkspace    workspace = new();
    
    private uint   territoryID;
    private string territoryKey  = string.Empty;
    private string territoryName = string.Empty;
    
    private          bool                            workspaceLoaded;
    private          bool                            historySuspended;
    private          CustomizationDraft              historySnapshot = new();
    private readonly Stack<CustomizationDraft>       undo            = new();
    private readonly Stack<CustomizationDraft>       redo            = new();
    private          Selection                       selection       = new(SelectionKind.Workspace);
    private          PickKind                        pickKind        = PickKind.None;
    private          Vector3?                        pendingPickPoint;
    private          Vector3?                        currentPickPoint;
    private          bool                            lastPickMouseDown;
    private          bool                            lastPickEscapeDown;
    private          bool                            lastWorldSelectMouseDown;
    private          string                          statusText    = string.Empty;
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
        CustomizationEditorToolbar.Draw
        (
            ref pickKind,
            ref pendingPickPoint,
            ref currentPickPoint,
            ref lastPickMouseDown,
            ref lastWorldSelectMouseDown,
            ref lastPickEscapeDown,
            ref statusText,
            workspaceLoaded,
            undo.Count,
            redo.Count,
            Undo,
            Redo,
            RebuildPreview,
            () => SaveWorkspace(true),
            ExportCurrentDraft
        );
        DrawStatus();

        if (workspaceLoaded                              &&
            previewDirty                                 &&
            workspace.Settings.AutoRebuild               &&
            DateTime.UtcNow             >= nextRebuildAt &&
            previewBuilder.CurrentState != CustomizationPreviewBuilder.State.InProgress)
            RebuildPreview();

        ImGui.BeginChild("##customization_editor_left", new Vector2(340, 0), true);

        if (workspaceLoaded)
        {
            CustomizationEditorLeftPanel.Draw
            (
                ref workspace,
                ref selection,
                previewBuilder,
                dd,
                AddMeshRemovalFromPreview,
                AddInstancePatchFromPreview,
                AddPartPatchFromPreview
            );
        }
        else
        {
            ImGui.TextDisabled("尚未加载 Territory");
            ImGui.TextWrapped("进入游戏区域后, 这里会显示当前 Territory 的预览对象和本地草稿");
        }

        ImGui.EndChild();

        ImGui.SameLine();

        ImGui.BeginChild("##customization_editor_right", new Vector2(0, 0), true);

        if (workspaceLoaded)
        {
            CustomizationEditorInspector.Draw
            (
                ref selection,
                ref workspace,
                previewBuilder,
                ref statusText,
                territoryID,
                territoryKey,
                ref exportDirText,
                settingsDefaults,
                profileDefaults,
                CommitDraftChange,
                AddMeshRemovalFromPreview,
                AddInstancePatchFromPreview,
                AddPartPatchFromPreview,
                RemoveMatchingPartPatch
            );
        }
        else
        {
            ImGui.TextDisabled("等待 Territory");
            ImGui.TextWrapped("进入游戏区域后, 编辑器会自动加载当前 Territory 的草稿; 加载后可直接在游戏画面点两点创建障碍或连接");
        }

        ImGui.EndChild();

        CustomizationEditorWorldOverlay.Draw
        (
            ref pickKind,
            ref pendingPickPoint,
            ref currentPickPoint,
            ref lastPickMouseDown,
            ref lastWorldSelectMouseDown,
            ref lastPickEscapeDown,
            ref workspace,
            ref selection,
            ref statusText,
            collision,
            dd,
            previewBuilder,
            AddColliderInsertion,
            AddMeshLink,
            AddOffMeshConnection
        );
    }

    private void EnsureWorkspace()
    {
        var zoneID = Service.ClientState.TerritoryType;
        if (zoneID == 0)
            return;

        switch (workspaceLoaded)
        {
            case true when zoneID == territoryID:
                return;
            case true:
                SaveWorkspace(true);
                break;
        }

        workspace = persistence.Load(zoneID);
        
        var territory = Service.LuminaRow<TerritoryType>(zoneID);
        
        territoryID   = zoneID;
        territoryKey  = territory == null ? zoneID.ToString(CultureInfo.InvariantCulture) : territory.Value.Bg.ToString();
        territoryName = territory == null ? zoneID.ToString(CultureInfo.InvariantCulture) : territory.Value.PlaceName.Value.Name.ToString();
        
        workspace.Draft.TerritoryID   = zoneID;
        workspace.Draft.TerritoryName = territoryName;
        
        
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
        lastPickMouseDown        = CustomizationEditorWorldOverlay.TakeKeyPress(0x01, ref lastPickMouseDown);
        lastWorldSelectMouseDown = lastPickMouseDown;
        lastPickEscapeDown       = CustomizationEditorWorldOverlay.TakeKeyPress(0x1B, ref lastPickEscapeDown);
        previewDirty             = true;
        nextRebuildAt            = DateTime.MinValue;
        statusText               = string.Empty;
        if (workspace.Settings.AutoRebuild)
            RebuildPreview();
    }

    private void DrawStatus()
    {
        ImGui.TextUnformatted($"区域: [{territoryID}] [{territoryKey}] [{territoryName}] ");
        
        ImGui.TextUnformatted($"预览: {previewBuilder.CurrentState}");

        if (!string.IsNullOrEmpty(statusText))
            ImGui.TextUnformatted($"状态: {statusText}");

        if (previewBuilder is { CurrentState: CustomizationPreviewBuilder.State.Failed, LastError: not null })
            ImGui.TextColored(KnownColor.Red.Vector(), previewBuilder.LastError.Message);

        if (lastExport != null)
            ImGui.TextDisabled($"导出: {lastExport.ClassName} v{lastExport.Version} {lastExport.ContentHash[..16]} -> {lastExport.File.FullName}");
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
                selection  = new(SelectionKind.ColliderInsertion, workspace.Draft.ColliderInsertions.Count - 1);
                statusText = kind == DraftSceneColliderInsertionKind.Cylinder ? "已添加圆柱障碍" : "已添加 AABB 障碍";
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
                selection  = new(SelectionKind.OffMeshConnection, workspace.Draft.OffMeshConnections.Count - 1);
                statusText = "已添加 off-mesh 连接";
            }
        );
    }

    private void AddMeshRemovalFromPreview(string key)
    {
        var existingIndex = workspace.Draft.MeshRemovals.FindIndex(x => x.MeshKey == key);

        if (existingIndex >= 0)
        {
            selection  = new(SelectionKind.MeshRemoval, existingIndex);
            statusText = "已选中已有 mesh 删除项";
            return;
        }

        ApplyDraftChange
        (() =>
            {
                workspace.Draft.MeshRemovals.Add(new() { MeshKey = key });
                selection  = new(SelectionKind.MeshRemoval, workspace.Draft.MeshRemovals.Count - 1);
                statusText = "已加入 mesh 删除清单";
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
            selection  = new(SelectionKind.InstancePatch, existingIndex);
            statusText = "已选中已有实例补丁";
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
                selection  = new(SelectionKind.InstancePatch, workspace.Draft.InstancePatches.Count - 1);
                statusText = $"已加入实例补丁: {kind}";
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
            (x => CustomizationEditorInspector.PartPatchMatches(x, key, partIndex, kind, kind == DraftScenePartPatchKind.Vertex ? vertexIndex : primitiveIndex));

        if (existingIndex >= 0)
        {
            selection  = new(SelectionKind.PartPatch, existingIndex);
            statusText = "已选中已有顶点 / 三角补丁";
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
                selection  = new(SelectionKind.PartPatch, workspace.Draft.PartPatches.Count - 1);
                statusText = $"已加入顶点 / 三角补丁: {kind}";
            }
        );
    }

    private void RemoveMatchingPartPatch(string key, int partIndex, DraftScenePartPatchKind kind, int subIndex)
    {
        var patch = workspace.Draft.PartPatches.FirstOrDefault(x => CustomizationEditorInspector.PartPatchMatches(x, key, partIndex, kind, subIndex));
        if (patch == null)
            return;

        ApplyDraftChange(() => { workspace.Draft.PartPatches.Remove(patch); });
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
        statusText    = string.Empty;
        previewBuilder.Rebuild(scene, customization, true);
    }

    private void ExportCurrentDraft()
    {
        if (string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory))
            workspace.Settings.ExportDirectory = Path.Combine(configDirectory.FullName, "customization-editor", "generated");

        var outputDirectory = new DirectoryInfo(workspace.Settings.ExportDirectory);
        lastExport = CustomizationDraftExporter.Export(workspace.Draft.Clone(), outputDirectory);
        statusText = $"已导出 {lastExport.File.FullName}";
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
        workspace.Draft.TerritoryName      = territoryKey;
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
}
