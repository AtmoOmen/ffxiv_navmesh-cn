using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Utility.Numerics;
using Lumina.Excel.Sheets;
using vnavmesh.Build;
using vnavmesh.Build.Custom;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Editor;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Models;
using vnavmesh.Internal;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Editor.Types;
using Action = System.Action;

namespace vnavmesh.UI.Editor;

internal sealed class CustomizationEditorView
(
    PluginConfig       config,
    DebugDrawer        dd,
    DebugGameCollision collision,
    NavmeshManager     manager,
    DirectoryInfo      configDirectory
)
    : IDisposable
{
    private const int MAX_HISTORY_ENTRIES = 128;

    private readonly CustomizationDraftPersistence persistence      = new(new DirectoryInfo(Path.Combine(configDirectory.FullName, "customization-editor")));
    private readonly CustomizationPreviewBuilder   previewBuilder   = new(manager, config);
    private readonly NavmeshSettings               settingsDefaults = new();
    private readonly NavmeshBuildProfile           profileDefaults  = new();

    private CustomizationEditorTerritoryStore store     = new();
    private CustomizationEditorWorkspace      workspace = new();

    private uint   territoryID;
    private string territoryKey  = string.Empty;
    private string territoryName = string.Empty;

    private          bool workspaceLoaded;
    private          bool hasWorkspace;
    private          bool historySuspended;
    private          bool inspectorEditActive;
    private          CustomizationDraft historySnapshot = new();
    private readonly Stack<CustomizationDraft> undo = new();
    private readonly Stack<CustomizationDraft> redo = new();
    private          Selection selection = new(SelectionKind.Workspace);
    private          Selection? pendingLeftPanelFocusSelection;
    private          EditorPanelMode leftPanelMode = EditorPanelMode.Draft;
    private          string leftPanelSearch = string.Empty;
    private          PickKind pickKind = PickKind.None;
    private          Vector3? pendingPickPoint;
    private          Vector3? currentPickPoint;
    private          bool lastPickMouseDown;
    private          bool lastPickEscapeDown;
    private          bool lastWorldSelectMouseDown;
    private          CustomizationEditorWorldOverlay.DraftEditState draftEditState;
    private          string statusText = string.Empty;
    private          string exportDirText = "";
    private          CustomizationDraftExportResult? lastExport;
    private          Task<(CustomizationEditorWorkspace Workspace, SceneDefinition Scene, SceneExtractor Extractor, Navmesh Navmesh)>? pendingWorkspaceCreation;
    private          CancellationTokenSource? pendingWorkspaceCreationCancel;
    private          float leftPaneWidth = 360;

    public void Dispose()
    {
        pendingWorkspaceCreationCancel?.Cancel();
        pendingWorkspaceCreationCancel?.Dispose();
        if (workspaceLoaded)
            SaveWorkspace(true);
        previewBuilder.Dispose();
    }

    public void Draw()
    {
        EnsureWorkspace();
        PollPendingWorkspaceCreation();
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
            hasWorkspace,
            undo.Count,
            redo.Count,
            Undo,
            Redo,
            RebuildPreview,
            () => SaveWorkspace(true),
            ExportCurrentDraft,
            OpenExportedDirectory
        );
        DrawStatus();

        var childSize = ImGui.GetContentRegionAvail().WithX(0) - new Vector2(0, 0.5f * ImGui.GetTextLineHeight());

        if (ImGui.BeginTable("##customization_editor_split", 2, ImGuiTableFlags.Resizable | ImGuiTableFlags.BordersInnerV))
        {
            ImGui.TableSetupColumn("left",  ImGuiTableColumnFlags.WidthFixed, leftPaneWidth);
            ImGui.TableSetupColumn("right", ImGuiTableColumnFlags.WidthStretch);

            ImGui.TableNextColumn();
            ImGui.BeginChild("##customization_editor_left", childSize);

            if (workspaceLoaded && hasWorkspace)
            {
                CustomizationEditorLeftPanel.Draw
                (
                    ref workspace,
                    ref selection,
                    ref pendingLeftPanelFocusSelection,
                    ref leftPanelMode,
                    ref leftPanelSearch,
                    previewBuilder,
                    collision,
                    dd,
                    AddMeshRemovalFromPreview,
                    AddInstancePatchFromPreview,
                    AddPartPatchFromPreview
                );
            }
            else if (pendingWorkspaceCreation != null)
            {
                ImGui.TextDisabled("正在创建工作区");
                var progress = manager.ExternalBuildProgress;
                if (progress >= 0)
                    ImGui.TextWrapped($"正在通过外置 worker 准备初始数据 ({progress * 100:f0}%), 完成后会自动切换到新工作区");
                else
                    ImGui.TextWrapped("正在通过外置 worker 准备初始数据, 完成后会自动切换到新工作区");
            }
            else if (workspaceLoaded)
            {
                ImGui.TextDisabled("当前区域暂无工作区");
                ImGui.TextWrapped("右侧先新建一个工作区, 再开始编辑和预览自定义");
            }
            else
            {
                ImGui.TextDisabled("等待区域加载");
                ImGui.TextWrapped("进入游戏区域后, 左侧会显示场景预览对象和本地草稿");
            }

            ImGui.EndChild();

            if (ImGui.TableGetColumnFlags(0).HasFlag(ImGuiTableColumnFlags.IsEnabled))
                leftPaneWidth = Math.Clamp(ImGui.GetColumnWidth(0), 280, 560);

            ImGui.TableNextColumn();
            ImGui.BeginChild("##customization_editor_right", childSize);

            if (workspaceLoaded)
            {
                CustomizationEditorInspector.Draw
                (
                    ref selection,
                    store,
                    hasWorkspace,
                    ref workspace,
                    previewBuilder,
                    ref statusText,
                    territoryID,
                    territoryKey,
                    ref exportDirText,
                    settingsDefaults,
                    profileDefaults,
                    CommitDraftChange,
                    () => SaveWorkspace(true),
                    CreateWorkspace,
                    DeleteCurrentWorkspace,
                    SelectWorkspace,
                    AddMeshRemovalFromPreview,
                    AddInstancePatchFromPreview,
                    AddPartPatchFromPreview,
                    RemoveMatchingPartPatch,
                    RebuildSceneExtract,
                    RebuildPreview
                );

                if (inspectorEditActive && !ImGui.IsAnyItemActive())
                    inspectorEditActive = false;
            }
            else
            {
                ImGui.TextDisabled("等待区域加载");
                ImGui.TextWrapped("进入游戏区域后自动加载草稿, 可直接在画面中点两点创建障碍或连接");
            }

            ImGui.EndChild();
            ImGui.EndTable();
        }

        if (workspaceLoaded && hasWorkspace)
        {
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
                ref pendingLeftPanelFocusSelection,
                ref statusText,
                ref draftEditState,
                CommitDraftChange,
                collision,
                dd,
                previewBuilder,
                AddColliderInsertion,
                AddMeshLink,
                AddOffMeshConnection
            );
        }
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
                CancelPendingWorkspaceCreation();
                SaveWorkspace(true);
                previewBuilder.Clear();
                break;
        }

        var territory = Service.LuminaRow<TerritoryType>(zoneID);

        territoryID = zoneID;
        territoryKey = territory == null ?
                           zoneID.ToString(CultureInfo.InvariantCulture) :
                           territory.Value.Bg.ToString();
        territoryName = territory == null ?
                            zoneID.ToString(CultureInfo.InvariantCulture) :
                            territory.Value.PlaceName.Value.Name.ToString();

        store               = persistence.Load(zoneID);
        store.TerritoryID   = zoneID;
        store.TerritoryKey  = territoryKey;
        store.TerritoryName = territoryName;
        EnsureWorkspaceSelection();
        workspaceLoaded = true;

        undo.Clear();
        redo.Clear();

        selection                      = new(SelectionKind.Workspace);
        pendingLeftPanelFocusSelection = null;
        leftPanelMode                  = EditorPanelMode.Draft;
        leftPanelSearch                = string.Empty;
        pendingPickPoint               = null;
        pickKind                       = PickKind.None;
        lastPickMouseDown              = CustomizationEditorWorldOverlay.TakeKeyPress(0x01, ref lastPickMouseDown);
        lastWorldSelectMouseDown       = lastPickMouseDown;
        lastPickEscapeDown             = CustomizationEditorWorldOverlay.TakeKeyPress(0x1B, ref lastPickEscapeDown);
        draftEditState                 = default;
        statusText                     = string.Empty;
    }

    private void PollPendingWorkspaceCreation()
    {
        if (pendingWorkspaceCreation is not { IsCompleted: true })
            return;

        try
        {
            if (pendingWorkspaceCreation.IsCanceled)
                statusText = "工作区创建已取消";
            else if (pendingWorkspaceCreation.IsFaulted)
                statusText = $"工作区创建失败: {pendingWorkspaceCreation.Exception?.GetBaseException().Message}";
            else
            {
                var (created, scene, extractor, navmesh) = pendingWorkspaceCreation.Result;
                store.Workspaces.Add(created);
                store.CurrentWorkspaceId = created.WorkspaceId;
                store.SchemaVersion      = 1;
                SaveWorkspace(true);
                SelectWorkspace(created.WorkspaceId, true);
                previewBuilder.Publish(scene, extractor, navmesh);
                statusText   = $"已新建工作区并自动激活预览: {workspace.WorkspaceName}";
            }
        }
        finally
        {
            pendingWorkspaceCreationCancel?.Dispose();
            pendingWorkspaceCreationCancel = null;
            pendingWorkspaceCreation       = null;
        }
    }

    private void CancelPendingWorkspaceCreation()
    {
        pendingWorkspaceCreationCancel?.Cancel();
        pendingWorkspaceCreationCancel?.Dispose();
        pendingWorkspaceCreationCancel = null;
        pendingWorkspaceCreation       = null;
    }

    private void EnsureWorkspaceSelection()
    {
        if (store.Workspaces.Count == 0)
        {
            workspace                = new();
            store.CurrentWorkspaceId = string.Empty;
            exportDirText            = Path.Combine(configDirectory.FullName, "customization-editor", "generated");
            historySnapshot          = new();
            hasWorkspace             = false;
            return;
        }

        if (store.SchemaVersion == 0)
            UpgradeLegacyWorkspace();

        workspace                     = ResolveCurrentWorkspace() ?? store.Workspaces[0];
        store.CurrentWorkspaceId      = workspace.WorkspaceId;
        workspace.Draft.TerritoryID   = territoryID;
        workspace.Draft.TerritoryName = territoryName;
        exportDirText = string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory) ?
                            Path.Combine(configDirectory.FullName, "customization-editor", "generated") :
                            workspace.Settings.ExportDirectory;
        workspace.Settings.ExportDirectory = exportDirText;
        historySnapshot                    = workspace.Draft.Clone();
        hasWorkspace                       = true;
    }

    private CustomizationEditorWorkspace? ResolveCurrentWorkspace() =>
        store.Workspaces.FirstOrDefault(x => x.WorkspaceId == store.CurrentWorkspaceId);

    private async Task<(CustomizationEditorWorkspace Workspace, SceneDefinition Scene, SceneExtractor Extractor, Navmesh Navmesh)> CreateWorkspaceAsync
    (
        string            name,
        CancellationToken cancel
    )
    {
        var scene = new SceneDefinition();
        scene.FillFromActiveLayout();
        scene.TerritoryID = territoryID;
        var baseCustomization = NavmeshCustomizationRegistry.GetForScene(scene);
        var draft             = await Task.Run(() => CustomizationDraftSeedBuilder.CreateFromCustomization(scene, baseCustomization, territoryName, config), cancel);
        cancel.ThrowIfCancellationRequested();

        var settings = baseCustomization.GetBuildSettings(scene);
        settings.Flyable              = baseCustomization.IsFlyingSupported(scene);
        settings.CustomizationVersion = baseCustomization.Version;
        settings.OffMeshConnections.AddRange(OffMeshConnectionMetadataRegistry.Collect(baseCustomization));

        SceneExtractor? extractor = null;
        var customizedScene = await Task.Run
                              (
                                  () =>
                                      {
                                          extractor = new SceneExtractor(scene);
                                          baseCustomization.CustomizeScene(extractor);
                                          return extractor;
                                      },
                                  cancel
                              );
        cancel.ThrowIfCancellationRequested();

        var     buildSignature = NavmeshBuilder.ComputeBuildSignature(customizedScene, settings);
        Navmesh navmesh;
        manager.ExternalBuildProgress = 0f;

        try
        {
            navmesh = await manager.BuildExternalNavmesh
                      (
                          $"editor-seed-{territoryID}-{Guid.NewGuid():N}",
                          customizedScene,
                          settings,
                          baseCustomization.Version,
                          buildSignature,
                          cancel,
                          progress => manager.ExternalBuildProgress = Math.Clamp((float)progress, 0f, 0.99f)
                      );
        }
        finally
        {
            manager.ExternalBuildProgress = -1f;
        }

        cancel.ThrowIfCancellationRequested();
        navmesh.RegisterBuildTimeOffMeshConnections(settings.OffMeshConnections);
        baseCustomization.CustomizeMesh(navmesh, [.. scene.FestivalLayers]);
        CustomizationDraftSeedBuilder.CopyMeshLinksFromNavmesh(draft, navmesh);

        var createdWorkspace = new CustomizationEditorWorkspace
        {
            WorkspaceId = Convert.ToHexString(Guid.NewGuid().ToByteArray()),
            WorkspaceName = string.IsNullOrWhiteSpace(name) ?
                                $"工作区 {store.Workspaces.Count + 1}" :
                                name,
            IsApplied = true,
            Draft     = draft,
            Settings = new()
            {
                ExportDirectory = Path.Combine(configDirectory.FullName, "customization-editor", "generated"),
                AutoSave        = true
            }
        };

        return (createdWorkspace, scene, extractor!, navmesh);
    }

    private void CreateWorkspace()
    {
        if (pendingWorkspaceCreation != null)
            return;

        pendingWorkspaceCreationCancel?.Cancel();
        pendingWorkspaceCreationCancel?.Dispose();
        pendingWorkspaceCreationCancel = new();
        var workspaceName = $"工作区 {store.Workspaces.Count + 1}";
        pendingWorkspaceCreation = CreateWorkspaceAsync(workspaceName, pendingWorkspaceCreationCancel.Token);
        statusText               = $"正在创建工作区: {workspaceName}";
    }

    private void DeleteCurrentWorkspace()
    {
        CancelPendingWorkspaceCreation();

        if (!hasWorkspace)
            return;

        var currentIndex = store.Workspaces.FindIndex(x => x.WorkspaceId == workspace.WorkspaceId);
        if (currentIndex < 0)
            return;

        store.Workspaces.RemoveAt(currentIndex);

        if (store.Workspaces.Count == 0)
        {
            workspace                = new();
            store.CurrentWorkspaceId = string.Empty;
            exportDirText            = Path.Combine(configDirectory.FullName, "customization-editor", "generated");
            historySnapshot          = new();
            hasWorkspace             = false;
            undo.Clear();
            redo.Clear();
            selection                      = new(SelectionKind.Workspace);
            pendingLeftPanelFocusSelection = null;
            leftPanelMode                  = EditorPanelMode.Draft;
            leftPanelSearch                = string.Empty;
            pickKind                       = PickKind.None;
            pendingPickPoint               = null;
            currentPickPoint               = null;
            draftEditState                 = default;
            previewBuilder.Clear();
            SaveWorkspace(true);
            statusText = "已删除当前工作区, 当前区域暂无工作区";
            return;
        }

        var nextIndex = Math.Clamp(currentIndex, 0, store.Workspaces.Count - 1);
        store.CurrentWorkspaceId = store.Workspaces[nextIndex].WorkspaceId;
        SelectWorkspace(store.CurrentWorkspaceId);
        statusText = $"已删除工作区, 当前为: {workspace.WorkspaceName}";
    }

    private void SelectWorkspace
    (
        string workspaceId
    ) => SelectWorkspace(workspaceId, false);

    private void SelectWorkspace
    (
        string workspaceId,
        bool   avoidClear
    )
    {
        CancelPendingWorkspaceCreation();

        var selected = store.Workspaces.FirstOrDefault(x => x.WorkspaceId == workspaceId);
        if (selected == null)
            return;

        if (hasWorkspace && store.Workspaces.Any(x => x.WorkspaceId == workspace.WorkspaceId))
            SaveWorkspace(true);
        workspace                     = selected;
        hasWorkspace                  = true;
        store.CurrentWorkspaceId      = workspace.WorkspaceId;
        workspace.Draft.TerritoryID   = territoryID;
        workspace.Draft.TerritoryName = territoryName;
        exportDirText = string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory) ?
                            Path.Combine(configDirectory.FullName, "customization-editor", "generated") :
                            workspace.Settings.ExportDirectory;
        workspace.Settings.ExportDirectory = exportDirText;
        historySnapshot                    = workspace.Draft.Clone();
        undo.Clear();
        redo.Clear();
        selection                      = new(SelectionKind.Workspace);
        pendingLeftPanelFocusSelection = null;
        leftPanelMode                  = EditorPanelMode.Draft;
        leftPanelSearch                = string.Empty;
        draftEditState                 = default;

        if (!avoidClear)
            previewBuilder.Clear();
    }

    private void UpgradeLegacyWorkspace()
    {
        if (store.Workspaces.Count == 0)
            return;

        var legacyWorkspace = store.Workspaces[0];
        var scene           = new SceneDefinition();
        scene.FillFromActiveLayout();
        scene.TerritoryID = territoryID;
        var baseCustomization = NavmeshCustomizationRegistry.GetForScene(scene);
        var baseDraft         = CustomizationDraftSeedBuilder.CreateFromCustomization(scene, baseCustomization, territoryName, config);
        MergeDraft(baseDraft, legacyWorkspace.Draft);
        legacyWorkspace.Draft = baseDraft;
        store.SchemaVersion   = 1;
        SaveWorkspace(true);
    }

    private static void MergeDraft
    (
        CustomizationDraft target,
        CustomizationDraft overlay
    )
    {
        target.FlyingSupportedOverride = overlay.FlyingSupportedOverride ?? target.FlyingSupportedOverride;
        target.BuildProfile            = overlay.BuildProfile;
        target.BuildSettings           = overlay.BuildSettings;
        target.MeshRemovals            = overlay.MeshRemovals;
        target.InstancePatches         = overlay.InstancePatches;
        target.PartPatches             = overlay.PartPatches;
        target.ColliderInsertions      = overlay.ColliderInsertions;
        target.MeshLinks               = overlay.MeshLinks;
        target.OffMeshConnections      = overlay.OffMeshConnections;
    }

    private void DrawStatus()
    {
        var workspaceSummary = hasWorkspace ? workspace.WorkspaceName : "暂无工作区";
        var sourceSummary = hasWorkspace && workspace.IsApplied ? workspace.WorkspaceName : "默认场景";
        var previewSummary = CustomizationEditorWidgets.FormatPreviewStateDisplayName(previewBuilder.CurrentState);
        var statusSummary = previewBuilder.CurrentState switch
        {
            CustomizationPreviewBuilder.State.InProgress when previewBuilder.BuildProgress >= 0
                => $"正在构建预览 · {previewBuilder.BuildProgress * 100:f0}%",
            CustomizationPreviewBuilder.State.Failed when previewBuilder.LastError is not null
                => previewBuilder.LastError.Message,
            _ => string.IsNullOrWhiteSpace(statusText) ?
                     "编辑器已就绪" :
                     statusText
        };

        if (ImGui.BeginTable("##customization_editor_status", 5, ImGuiTableFlags.SizingStretchProp | ImGuiTableFlags.BordersInnerV))
        {
            ImGui.TableSetupColumn("区域",   ImGuiTableColumnFlags.WidthStretch, 2.4f);
            ImGui.TableSetupColumn("工作区", ImGuiTableColumnFlags.WidthStretch, 1.2f);
            ImGui.TableSetupColumn("来源",   ImGuiTableColumnFlags.WidthStretch, 1.2f);
            ImGui.TableSetupColumn("预览",   ImGuiTableColumnFlags.WidthStretch, 0.9f);
            ImGui.TableSetupColumn("状态",   ImGuiTableColumnFlags.WidthStretch, 1.8f);
            ImGui.TableNextRow();
            DrawStatusCell("区域", $"{territoryName} · {territoryID} · {territoryKey}");
            DrawStatusCell("工作区", workspaceSummary);
            DrawStatusCell("来源", sourceSummary);
            DrawStatusCell("预览", previewSummary, GetPreviewStateColor(previewBuilder.CurrentState));
            DrawStatusCell
            (
                "状态",
                statusSummary,
                previewBuilder.CurrentState == CustomizationPreviewBuilder.State.Failed ?
                    GetPreviewStateColor(previewBuilder.CurrentState) :
                    null
            );
            ImGui.EndTable();
        }

        if (previewBuilder is { CurrentState: CustomizationPreviewBuilder.State.InProgress, BuildProgress: >= 0 })
            ImGui.ProgressBar(previewBuilder.BuildProgress, new Vector2(-1, 3), string.Empty);
    }

    private static void DrawStatusCell
    (
        string label,
        string value,
        Vector4? color = null
    )
    {
        ImGui.TableNextColumn();
        ImGui.AlignTextToFramePadding();
        ImGui.TextDisabled($"{label}:");
        ImGui.SameLine(0, 4);

        if (color is { } textColor)
            ImGui.TextColored(textColor, value);
        else
            ImGui.TextUnformatted(value);

        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(value);
    }

    private static Vector4 GetPreviewStateColor
    (
        CustomizationPreviewBuilder.State state
    ) =>
        state switch
        {
            CustomizationPreviewBuilder.State.Ready      => new(0.35f, 0.78f, 0.48f, 1f),
            CustomizationPreviewBuilder.State.InProgress => new(0.95f, 0.72f, 0.28f, 1f),
            CustomizationPreviewBuilder.State.Failed     => new(0.95f, 0.35f, 0.35f, 1f),
            _                                            => ImGui.GetStyle().Colors[(int)ImGuiCol.TextDisabled]
        };

    private void RebuildSceneExtract
    (
        uint territoryID
    )
    {
        var scene = new SceneDefinition();
        scene.FillFromActiveLayout();
        scene.TerritoryID = territoryID;
        var customization = BuildPreviewCustomization(scene);
        previewBuilder.Rebuild(scene, customization, false);
    }

    private void AddColliderInsertion
    (
        Vector3                         a,
        Vector3                         b,
        DraftSceneColliderInsertionKind kind
    )
    {
        var center          = (a + b) * 0.5f;
        var min             = Vector3.Min(a, b);
        var max             = Vector3.Max(a, b);
        var rotationDegrees = 0f;
        var doubleSided     = true;

        if (kind is DraftSceneColliderInsertionKind.OrientedBox or DraftSceneColliderInsertionKind.Wall)
        {
            var delta       = b - a;
            var length      = MathF.Max(MathF.Sqrt((delta.X * delta.X) + (delta.Z * delta.Z)), 0.1f);
            var halfExtents = kind == DraftSceneColliderInsertionKind.Wall ?
                                  new Vector3(length * 0.5f, 1f, 0.025f) :
                                  new Vector3(length * 0.5f, 1f, 0.5f);
            min             = center - halfExtents;
            max             = center + halfExtents;
            rotationDegrees = MathF.Atan2(-delta.Z, delta.X) * (180f / MathF.PI);
        }
        else if (kind == DraftSceneColliderInsertionKind.Sphere)
        {
            var radius  = MathF.Max(Vector3.Distance(a, b), 0.05f);
            var extents = new Vector3(radius);
            center      = a;
            min         = center - extents;
            max         = center + extents;
        }
        else if (kind == DraftSceneColliderInsertionKind.OrientedCylinder)
        {
            min = Vector3.Min(a, b) - new Vector3(0.5f);
            max = Vector3.Max(a, b) + new Vector3(0.5f);
        }
        else if (kind == DraftSceneColliderInsertionKind.WalkableFloor)
        {
            var halfExtents = new Vector3
            (
                MathF.Max(MathF.Abs(b.X - a.X) * 0.5f, 0.05f),
                0.005f,
                MathF.Max(MathF.Abs(b.Z - a.Z) * 0.5f, 0.05f)
            );
            center = (a + b) * 0.5f;
            min    = center - halfExtents;
            max    = center + halfExtents;
        }
        else if (kind == DraftSceneColliderInsertionKind.Ramp)
        {
            var low  = a.Y <= b.Y ? a : b;
            var high = a.Y <= b.Y ? b : a;
            var delta = high - low;
            var length = MathF.Max(MathF.Sqrt((delta.X * delta.X) + (delta.Z * delta.Z)), 0.1f);
            var height = MathF.Max(high.Y - low.Y, 0.5f);
            center = new((low.X + high.X) * 0.5f, low.Y + (height * 0.5f), (low.Z + high.Z) * 0.5f);
            var halfExtents = new Vector3(length * 0.5f, height * 0.5f, 0.75f);
            min             = center - halfExtents;
            max             = center + halfExtents;
            rotationDegrees = MathF.Atan2(-delta.Z, delta.X) * (180f / MathF.PI);
        }
        else
        {
            NormalizeBounds(ref min, ref max);
        }

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
                        Start             = a,
                        End               = b,
                        Radius            = 0.5f,
                        RotationDegrees   = rotationDegrees,
                        DoubleSided       = doubleSided,
                        ForceSetPrimFlags = kind switch
                        {
                            DraftSceneColliderInsertionKind.Ramp or
                            DraftSceneColliderInsertionKind.WalkableFloor  => PrimitiveFlags.ForceWalkable,
                            DraftSceneColliderInsertionKind.RemoveInstances => PrimitiveFlags.None,
                            _                                               => PrimitiveFlags.ForceUnwalkable
                        },
                        ForceClearPrimFlags = kind is DraftSceneColliderInsertionKind.Ramp or
                                                      DraftSceneColliderInsertionKind.WalkableFloor ?
                                                  PrimitiveFlags.ForceUnwalkable :
                                                  PrimitiveFlags.None
                    }
                );
                selection = new(SelectionKind.ColliderInsertion, workspace.Draft.ColliderInsertions.Count - 1);
                statusText = $"已添加 {CustomizationEditorWidgets.FormatEnumDisplayName(kind)}";
            }
        );
    }

    private void AddMeshLink
    (
        Vector3           a,
        Vector3           b,
        DraftMeshLinkKind kind
    ) =>
        ApplyDraftChange
        (() =>
            {
                workspace.Draft.MeshLinks.Add(new() { Kind = kind, Start = a, End = b, Bidirectional = false });
                selection = new(SelectionKind.MeshLink, workspace.Draft.MeshLinks.Count - 1);
                statusText = kind switch
                {
                    DraftMeshLinkKind.Points     => "已添加网格连线",
                    DraftMeshLinkKind.Shortcut   => "已添加普通移动捷径",
                    DraftMeshLinkKind.ClientPath => "已添加客户端路径连接",
                    _                            => "已添加网格连接"
                };
            }
        );

    private void AddOffMeshConnection
    (
        Vector3 a,
        Vector3 b
    ) =>
        ApplyDraftChange
        (() =>
            {
                workspace.Draft.OffMeshConnections.Add(new() { Start = a, End = b });
                selection  = new(SelectionKind.OffMeshConnection, workspace.Draft.OffMeshConnections.Count - 1);
                statusText = "已添加离网连接";
            }
        );

    private void AddMeshRemovalFromPreview
    (
        string key
    )
    {
        var existingIndex = workspace.Draft.MeshRemovals.FindIndex(x => x.MeshKey == key);

        if (existingIndex >= 0)
        {
            selection  = new(SelectionKind.MeshRemoval, existingIndex);
            statusText = "已选现有网格删除项";
            return;
        }

        ApplyDraftChange
        (() =>
            {
                workspace.Draft.MeshRemovals.Add(new() { MeshKey = key });
                selection  = new(SelectionKind.MeshRemoval, workspace.Draft.MeshRemovals.Count - 1);
                statusText = "已加入网格删除清单";
            }
        );
    }

    private void AddInstancePatchFromPreview
    (
        Mesh         mesh,
        string                      key,
        int                         index,
        DraftSceneInstancePatchKind kind
    )
    {
        var inst = mesh.Instances[index];
        var existingIndex = kind == DraftSceneInstancePatchKind.Insert ?
                                -1 :
                                workspace.Draft.InstancePatches.FindIndex
                                    (x => x.MeshKey == key && x.Kind == kind && (kind == DraftSceneInstancePatchKind.ClearInstances || x.InstanceIndex == index));

        if (existingIndex >= 0)
        {
            selection  = new(SelectionKind.InstancePatch, existingIndex);
            statusText = "已选现有实例补丁";
            return;
        }

        ApplyDraftChange
        (() =>
            {
                var worldTransform = DraftMatrix4x3.FromRuntime(inst.WorldTransform);
                if (kind == DraftSceneInstancePatchKind.Insert)
                    worldTransform.Row3.X += 1f;

                workspace.Draft.InstancePatches.Add
                (
                    new()
                    {
                        MeshKey             = key,
                        InstanceIndex       = kind == DraftSceneInstancePatchKind.Insert ? -1 : index,
                        InstanceId          = kind == DraftSceneInstancePatchKind.Insert ? 0 : inst.ID,
                        Kind                = kind,
                        WorldTransform      = worldTransform,
                        Material            = inst.Material,
                        Count               = 1,
                        Offset              = Vector3.UnitX,
                        ForceSetPrimFlags   = inst.ForceSetPrimFlags,
                        ForceClearPrimFlags = inst.ForceClearPrimFlags
                    }
                );
                selection  = new(SelectionKind.InstancePatch, workspace.Draft.InstancePatches.Count - 1);
                statusText = $"已加入实例补丁: {CustomizationEditorWidgets.FormatEnumDisplayName(kind)}";
            }
        );
    }

    private void AddPartPatchFromPreview
    (
        Mesh     mesh,
        string                  key,
        int                     partIndex,
        DraftScenePartPatchKind kind,
        int                     subIndex = -1
    )
    {
        var part = mesh.Parts[partIndex];
        var vertexIndex = kind == DraftScenePartPatchKind.Vertex && subIndex >= 0 && subIndex < part.Vertices.Count ?
                              subIndex :
                              0;
        var primitiveIndex = (kind == DraftScenePartPatchKind.PrimitiveFlags || kind == DraftScenePartPatchKind.PrimitiveEdit) &&
                             subIndex >= 0                                                                                     &&
                             subIndex < part.Primitives.Count ?
                                 subIndex :
                                 0;
        var existingIndex = workspace.Draft.PartPatches.FindIndex
        (x => CustomizationEditorInspector.PartPatchMatches
         (
             x,
             key,
             partIndex,
             kind,
             kind == DraftScenePartPatchKind.Vertex ?
                 vertexIndex :
                 primitiveIndex
         )
        );

        if (existingIndex >= 0)
        {
            selection  = new(SelectionKind.PartPatch, existingIndex);
            statusText = "已选现有顶点或三角补丁";
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
                        Position = part.Vertices.Count > 0 ?
                                       part.Vertices[vertexIndex] :
                                       default,
                        V1 = part.Primitives.Count > 0 ?
                                 part.Primitives[primitiveIndex].V1 :
                                 0,
                        V2 = part.Primitives.Count > 0 ?
                                 part.Primitives[primitiveIndex].V2 :
                                 1,
                        V3 = part.Primitives.Count > 0 ?
                                 part.Primitives[primitiveIndex].V3 :
                                 2,
                        Flags = part.Primitives.Count > 0 ?
                                    part.Primitives[primitiveIndex].Flags :
                                    PrimitiveFlags.None,
                        Material = part.Primitives.Count > 0 ?
                                       part.Primitives[primitiveIndex].Material :
                                       0
                    }
                );
                selection  = new(SelectionKind.PartPatch, workspace.Draft.PartPatches.Count - 1);
                statusText = $"已加入顶点 / 三角补丁: {kind}";
            }
        );
    }

    private void RemoveMatchingPartPatch
    (
        string                  key,
        int                     partIndex,
        DraftScenePartPatchKind kind,
        int                     subIndex
    )
    {
        var patch = workspace.Draft.PartPatches.FirstOrDefault(x => CustomizationEditorInspector.PartPatchMatches(x, key, partIndex, kind, subIndex));
        if (patch == null)
            return;

        ApplyDraftChange(() => { workspace.Draft.PartPatches.Remove(patch); });
    }

    private void RebuildPreview()
    {
        if (!workspaceLoaded || !hasWorkspace)
            return;

        var scene = new SceneDefinition();
        scene.FillFromActiveLayout();
        scene.TerritoryID = territoryID;

        var customization = BuildPreviewCustomization(scene);
        statusText   = string.Empty;
        previewBuilder.Rebuild(scene, customization, true);
    }

    private NavmeshCustomization BuildPreviewCustomization
    (
        SceneDefinition scene
    )
    {
        if (!workspace.IsApplied)
            return new NavmeshCustomization();

        return new CustomizationDraftCustomization(new NavmeshCustomization(), workspace.Draft.Clone());
    }

    private void ExportCurrentDraft()
    {
        if (!hasWorkspace)
            return;

        if (string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory))
            workspace.Settings.ExportDirectory = Path.Combine(configDirectory.FullName, "customization-editor", "generated");

        var outputDirectory = new DirectoryInfo(workspace.Settings.ExportDirectory);
        lastExport = CustomizationDraftExporter.Export(workspace.Draft.Clone(), outputDirectory);
        statusText = $"已导出 {lastExport.File.FullName}";
        SaveWorkspace(true);
    }

    private void OpenExportedDirectory()
    {
        if (!hasWorkspace)
            return;

        if (string.IsNullOrWhiteSpace(workspace.Settings.ExportDirectory))
            workspace.Settings.ExportDirectory = Path.Combine(configDirectory.FullName, "customization-editor", "generated");

        Process.Start(new ProcessStartInfo(workspace.Settings.ExportDirectory) { UseShellExecute = true });
    }

    private void CommitDraftChange()
    {
        if (historySuspended || !hasWorkspace)
            return;

        if (!inspectorEditActive)
            PushUndoSnapshot(historySnapshot.Clone());

        inspectorEditActive = ImGui.IsAnyItemActive();
        historySnapshot = workspace.Draft.Clone();
        redo.Clear();
        SaveWorkspace();
    }

    private void PushUndoSnapshot
    (
        CustomizationDraft snapshot
    )
    {
        if (undo.Count >= MAX_HISTORY_ENTRIES)
        {
            var retained = undo.Take(MAX_HISTORY_ENTRIES - 1).ToArray();
            undo.Clear();

            for (var i = retained.Length - 1; i >= 0; --i)
                undo.Push(retained[i]);
        }

        undo.Push(snapshot);
    }

    private void ApplyDraftChange
    (
        Action action
    )
    {
        action();
        CommitDraftChange();
    }

    private void Undo()
    {
        if (!hasWorkspace || undo.Count == 0)
            return;

        historySuspended = true;
        redo.Push(workspace.Draft.Clone());
        workspace.Draft                = undo.Pop();
        historySnapshot                = workspace.Draft.Clone();
        selection                      = new(SelectionKind.Workspace);
        pendingLeftPanelFocusSelection = null;
        draftEditState                 = default;
        inspectorEditActive            = false;
        historySuspended               = false;
        SaveWorkspace();
    }

    private void Redo()
    {
        if (!hasWorkspace || redo.Count == 0)
            return;

        historySuspended = true;
        PushUndoSnapshot(workspace.Draft.Clone());
        workspace.Draft                = redo.Pop();
        historySnapshot                = workspace.Draft.Clone();
        selection                      = new(SelectionKind.Workspace);
        pendingLeftPanelFocusSelection = null;
        draftEditState                 = default;
        inspectorEditActive            = false;
        historySuspended               = false;
        SaveWorkspace();
    }

    private void SaveWorkspace
    (
        bool force = false
    )
    {
        if (!workspaceLoaded)
            return;

        if (hasWorkspace)
        {
            workspace.Draft.TerritoryID        = territoryID;
            workspace.Draft.TerritoryName      = territoryName;
            workspace.Settings.ExportDirectory = exportDirText;
            store.CurrentWorkspaceId           = workspace.WorkspaceId;
        }

        store.TerritoryID   = territoryID;
        store.TerritoryKey  = territoryKey;
        store.TerritoryName = territoryName;
        store.SchemaVersion = 1;

        if (force || (hasWorkspace && workspace.Settings.AutoSave))
            persistence.Save(store);
    }

    private static void NormalizeBounds
    (
        ref Vector3 min,
        ref Vector3 max
    )
    {
        if (MathF.Abs(max.Y - min.Y) < 0.1f)
        {
            var center = (min + max) * 0.5f;
            min = new(center.X - 0.5f, center.Y - 1f, center.Z - 0.5f);
            max = new(center.X + 0.5f, center.Y + 1f, center.Z + 0.5f);
        }
    }
}
