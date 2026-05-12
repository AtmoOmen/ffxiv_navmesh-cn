using Dalamud.Bindings.ImGui;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Customizations.Editor;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorInspector
{
    public delegate void CommitDelegate();

    public delegate void AddMeshRemovalDelegate(string key);

    public delegate void AddInstancePatchDelegate(SceneExtractor.Mesh mesh, string key, int index, DraftSceneInstancePatchKind kind);

    public delegate void AddPartPatchDelegate(SceneExtractor.Mesh mesh, string key, int partIndex, DraftScenePartPatchKind kind, int subIndex = -1);

    public delegate void RemoveMatchingPartPatchDelegate(string key, int partIndex, DraftScenePartPatchKind kind, int subIndex);

    public static void Draw
    (
        ref Selection                    selection,
        ref CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder      previewBuilder,
        ref string                       statusText,
        uint                             territoryID,
        string                           territoryLabel,
        ref string                       exportDirText,
        NavmeshSettings                  settingsDefaults,
        NavmeshBuildProfile              profileDefaults,
        CommitDelegate                   onCommit,
        AddMeshRemovalDelegate           onAddMeshRemoval,
        AddInstancePatchDelegate         onAddInstancePatch,
        AddPartPatchDelegate             onAddPartPatch,
        RemoveMatchingPartPatchDelegate  onRemoveMatchingPartPatch
    )
    {
        DrawInspectorHeader(selection);

        switch (selection.Kind)
        {
            case SelectionKind.Workspace:
                DrawWorkspaceInspector(ref workspace, ref exportDirText);
                break;
            case SelectionKind.BuildProfile:
                DrawBuildProfileInspector(ref workspace, profileDefaults, settingsDefaults, onCommit);
                break;
            case SelectionKind.BuildSettings:
                DrawBuildSettingsInspector(ref workspace, settingsDefaults, onCommit);
                break;
            case SelectionKind.FlyingOverride:
                DrawFlyingInspector(ref workspace, onCommit);
                break;
            case SelectionKind.MeshRemoval:
                DrawMeshRemovalInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.InstancePatch:
                DrawInstancePatchInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.PartPatch:
                DrawPartPatchInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.ColliderInsertion:
                DrawColliderInsertionInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.MeshLink:
                DrawMeshLinkInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.OffMeshConnection:
                DrawOffMeshInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.PreviewMesh:
                DrawPreviewMeshInspector(selection, previewBuilder, onAddMeshRemoval);
                break;
            case SelectionKind.PreviewInstance:
                DrawPreviewInstanceInspector(ref workspace, ref selection, previewBuilder, onCommit, onAddInstancePatch);
                break;
            case SelectionKind.PreviewPart:
            case SelectionKind.PreviewVertex:
            case SelectionKind.PreviewPrimitive:
                DrawPreviewPartInspector(ref workspace, ref selection, previewBuilder, onCommit, onAddPartPatch, onRemoveMatchingPartPatch);
                break;
        }
    }

    private static void DrawInspectorHeader(Selection selection)
    {
        ImGui.TextUnformatted(GetSelectionTitle(selection));
        ImGui.TextDisabled(GetSelectionHelp(selection));
        ImGui.Separator();
    }

    private static string GetSelectionTitle(Selection selection) =>
        selection.Kind switch
        {
            SelectionKind.Workspace         => "工作区",
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

    private static string GetSelectionHelp(Selection selection) =>
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

    private static void DrawWorkspaceInspector
    (
        ref CustomizationEditorWorkspace workspace,
        ref string                       exportDirText
    )
    {
        CustomizationEditorWidgets.DrawBool("自动重建", ref workspace.Settings.AutoRebuild);
        CustomizationEditorWidgets.DrawBool("自动保存", ref workspace.Settings.AutoSave);
        CustomizationEditorWidgets.DrawFloat("重建延迟", ref workspace.Settings.RebuildDelaySeconds, 0.05f, 0.1f, 5f);

        if (ImGui.InputText("导出目录", ref exportDirText))
            workspace.Settings.ExportDirectory = exportDirText;
    }

    private static void DrawBuildProfileInspector
        (ref CustomizationEditorWorkspace workspace, NavmeshBuildProfile profileDefaults, NavmeshSettings settingsDefaults, CommitDelegate onCommit)
    {
        var changed = false;

        if (ImGui.TreeNodeEx("Cell", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Cell Size",   ref workspace.Draft.BuildProfile.CellSizeOverride,   settingsDefaults.CellSize);
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Cell Height", ref workspace.Draft.BuildProfile.CellHeightOverride, settingsDefaults.CellHeight);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Region", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableEnum
                ("Partitioning", ref workspace.Draft.BuildProfile.PartitioningOverride, profileDefaults.PartitioningOverride);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Region Min Size", ref workspace.Draft.BuildProfile.RegionMinSizeOverride, settingsDefaults.RegionMinSize);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Region Merge Size", ref workspace.Draft.BuildProfile.RegionMergeSizeOverride, settingsDefaults.RegionMergeSize);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Polygonization", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Poly Max Edge Len", ref workspace.Draft.BuildProfile.PolyMaxEdgeLenOverride, settingsDefaults.PolyMaxEdgeLen);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Poly Max Simplification Error", ref workspace.Draft.BuildProfile.PolyMaxSimplificationErrorOverride, settingsDefaults.PolyMaxSimplificationError);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Agent", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Agent Radius", ref workspace.Draft.BuildProfile.AgentRadiusOverride, settingsDefaults.AgentRadius);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Detail", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Detail Sample Dist", ref workspace.Draft.BuildProfile.DetailSampleDistOverride, settingsDefaults.DetailSampleDist);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Edge Links", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("Generate Edge Climb Links", ref workspace.Draft.BuildProfile.GenerateEdgeClimbLinksOverride, settingsDefaults.GenerateEdgeClimbLinks);
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("Generate Edge Jump Links", ref workspace.Draft.BuildProfile.GenerateEdgeJumpLinksOverride, settingsDefaults.GenerateEdgeJumpLinks);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Volume", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableIntArray
                ("Volume Tiles", ref workspace.Draft.BuildProfile.VolumeTilesOverride, settingsDefaults.VolumeTiles);
            ImGui.TreePop();
        }

        if (changed) onCommit();
    }

    private static void DrawBuildSettingsInspector(ref CustomizationEditorWorkspace workspace, NavmeshSettings settingsDefaults, CommitDelegate onCommit)
    {
        var changed = false;

        if (ImGui.TreeNodeEx("通用", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableBool("Fast Build", ref workspace.Draft.BuildSettings.FastBuild, settingsDefaults.FastBuild);
            changed |= CustomizationEditorWidgets.DrawNullableFlags("Filtering", ref workspace.Draft.BuildSettings.Filtering, settingsDefaults.Filtering);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Cell", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Cell Size",   ref workspace.Draft.BuildSettings.CellSize,   settingsDefaults.CellSize);
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Cell Height", ref workspace.Draft.BuildSettings.CellHeight, settingsDefaults.CellHeight);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Agent", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Agent Height",    ref workspace.Draft.BuildSettings.AgentHeight,   settingsDefaults.AgentHeight);
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Agent Radius",    ref workspace.Draft.BuildSettings.AgentRadius,   settingsDefaults.AgentRadius);
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Agent Max Climb", ref workspace.Draft.BuildSettings.AgentMaxClimb, settingsDefaults.AgentMaxClimb);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Agent Max Slope", ref workspace.Draft.BuildSettings.AgentMaxSlopeDeg, settingsDefaults.AgentMaxSlopeDeg);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Region", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("Region Min Size",   ref workspace.Draft.BuildSettings.RegionMinSize,   settingsDefaults.RegionMinSize);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Region Merge Size", ref workspace.Draft.BuildSettings.RegionMergeSize, settingsDefaults.RegionMergeSize);
            changed |= CustomizationEditorWidgets.DrawNullableEnum("Partitioning", ref workspace.Draft.BuildSettings.Partitioning, settingsDefaults.Partitioning);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Polygonization", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Poly Max Edge Len", ref workspace.Draft.BuildSettings.PolyMaxEdgeLen, settingsDefaults.PolyMaxEdgeLen);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Poly Max Simplification Error", ref workspace.Draft.BuildSettings.PolyMaxSimplificationError, settingsDefaults.PolyMaxSimplificationError);
            changed |= CustomizationEditorWidgets.DrawNullableInt("Poly Max Verts", ref workspace.Draft.BuildSettings.PolyMaxVerts, settingsDefaults.PolyMaxVerts);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Detail Mesh", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Detail Sample Dist", ref workspace.Draft.BuildSettings.DetailSampleDist, settingsDefaults.DetailSampleDist);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Detail Max Sample Error", ref workspace.Draft.BuildSettings.DetailMaxSampleError, settingsDefaults.DetailMaxSampleError);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Edge Links", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("Generate Edge Climb Links", ref workspace.Draft.BuildSettings.GenerateEdgeClimbLinks, settingsDefaults.GenerateEdgeClimbLinks);
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("Generate Edge Jump Links", ref workspace.Draft.BuildSettings.GenerateEdgeJumpLinks, settingsDefaults.GenerateEdgeJumpLinks);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Climb Down", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Ground Tolerance", ref workspace.Draft.BuildSettings.GroundTolerance, settingsDefaults.GroundTolerance);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Climb Down Distance", ref workspace.Draft.BuildSettings.ClimbDownDistance, settingsDefaults.ClimbDownDistance);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Climb Down Max Height", ref workspace.Draft.BuildSettings.ClimbDownMaxHeight, settingsDefaults.ClimbDownMaxHeight);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Climb Down Min Height", ref workspace.Draft.BuildSettings.ClimbDownMinHeight, settingsDefaults.ClimbDownMinHeight);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Edge Jump", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Edge Jump End Distance", ref workspace.Draft.BuildSettings.EdgeJumpEndDistance, settingsDefaults.EdgeJumpEndDistance);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Edge Jump Height", ref workspace.Draft.BuildSettings.EdgeJumpHeight, settingsDefaults.EdgeJumpHeight);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Edge Jump Max Drop", ref workspace.Draft.BuildSettings.EdgeJumpMaxDrop, settingsDefaults.EdgeJumpMaxDrop);
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Edge Jump Min Drop", ref workspace.Draft.BuildSettings.EdgeJumpMinDrop, settingsDefaults.EdgeJumpMinDrop);
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("Tiles", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("Ground Tile Size", ref workspace.Draft.BuildSettings.GroundTileSize, settingsDefaults.GroundTileSize);
            changed |= CustomizationEditorWidgets.DrawNullableInt
                ("Ground Tile Count Max", ref workspace.Draft.BuildSettings.GroundTileCountMax, settingsDefaults.GroundTileCountMax);
            changed |= CustomizationEditorWidgets.DrawNullableIntArray("Volume Tiles", ref workspace.Draft.BuildSettings.VolumeTiles, settingsDefaults.VolumeTiles);
            ImGui.TreePop();
        }

        if (changed) onCommit();
    }

    private static void DrawFlyingInspector(ref CustomizationEditorWorkspace workspace, CommitDelegate onCommit)
    {
        var current = workspace.Draft.FlyingSupportedOverride;
        var next = current switch
        {
            true  => 1,
            false => 2,
            _     => 0
        };

        if (CustomizationEditorWidgets.DrawEnumCombo("Flying Support", ref next, ["默认", "启用", "禁用"]))
        {
            workspace.Draft.FlyingSupportedOverride = next switch
            {
                1 => true,
                2 => false,
                _ => null
            };
            onCommit();
        }
    }

    private static void DrawMeshRemovalInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.MeshRemovals, selection.Index, out var item))
            return;

        var changed = false;
        changed |= CustomizationEditorWidgets.DrawBool("Enabled", ref item.Enabled);
        changed |= CustomizationEditorWidgets.DrawString("Mesh Key", ref item.MeshKey);

        if (ImGui.Button("删除这一项"))
        {
            workspace.Draft.MeshRemovals.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }

        if (changed) onCommit();
    }

    private static void DrawInstancePatchInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.InstancePatches, selection.Index, out var item))
            return;

        var changed = false;
        changed |= CustomizationEditorWidgets.DrawBool("Enabled", ref item.Enabled);
        changed |= CustomizationEditorWidgets.DrawString("Mesh Key", ref item.MeshKey);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("Kind", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawInt("Instance Index", ref item.InstanceIndex);
        changed |= CustomizationEditorWidgets.DrawUInt64("Instance ID", ref item.InstanceId);
        changed |= CustomizationEditorWidgets.DrawMatrix("World Transform", ref item.WorldTransform);
        changed |= CustomizationEditorWidgets.DrawFlags("Set Flags",   ref item.ForceSetPrimFlags);
        changed |= CustomizationEditorWidgets.DrawFlags("Clear Flags", ref item.ForceClearPrimFlags);

        if (ImGui.Button("删除这一项"))
        {
            workspace.Draft.InstancePatches.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }

        if (changed) onCommit();
    }

    private static void DrawPartPatchInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.PartPatches, selection.Index, out var item))
            return;

        var changed = false;
        changed |= CustomizationEditorWidgets.DrawBool("Enabled", ref item.Enabled);
        changed |= CustomizationEditorWidgets.DrawString("Mesh Key", ref item.MeshKey);
        changed |= CustomizationEditorWidgets.DrawInt("Part Index", ref item.PartIndex);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("Kind", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawInt("Vertex Index",    ref item.VertexIndex);
        changed |= CustomizationEditorWidgets.DrawInt("Primitive Index", ref item.PrimitiveIndex);
        changed |= CustomizationEditorWidgets.DrawVector3("Position", ref item.Position);
        changed |= CustomizationEditorWidgets.DrawInt("V1", ref item.V1);
        changed |= CustomizationEditorWidgets.DrawInt("V2", ref item.V2);
        changed |= CustomizationEditorWidgets.DrawInt("V3", ref item.V3);
        changed |= CustomizationEditorWidgets.DrawUInt64("Material", ref item.Material);
        changed |= CustomizationEditorWidgets.DrawFlags("Flags",       ref item.Flags);
        changed |= CustomizationEditorWidgets.DrawFlags("Set Flags",   ref item.ForceSetPrimFlags);
        changed |= CustomizationEditorWidgets.DrawFlags("Clear Flags", ref item.ForceClearPrimFlags);

        if (ImGui.Button("删除这一项"))
        {
            workspace.Draft.PartPatches.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }

        if (changed) onCommit();
    }

    private static void DrawColliderInsertionInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.ColliderInsertions, selection.Index, out var item))
            return;

        var changed = false;
        changed |= CustomizationEditorWidgets.DrawBool("Enabled", ref item.Enabled);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("Kind", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawBoundsEditor("几何", ref item.Min, ref item.Max);
        changed |= CustomizationEditorWidgets.DrawFlags("Set Flags",   ref item.ForceSetPrimFlags);
        changed |= CustomizationEditorWidgets.DrawFlags("Clear Flags", ref item.ForceClearPrimFlags);

        if (ImGui.Button("删除这一项"))
        {
            workspace.Draft.ColliderInsertions.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }

        if (changed) onCommit();
    }

    private static void DrawMeshLinkInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.MeshLinks, selection.Index, out var item))
            return;

        var changed = false;
        changed |= CustomizationEditorWidgets.DrawBool("Enabled", ref item.Enabled);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("Kind", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawVector3("Start", ref item.Start);
        changed |= CustomizationEditorWidgets.DrawVector3("End",   ref item.End);

        if (ImGui.Button("删除这一项"))
        {
            workspace.Draft.MeshLinks.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }

        if (changed) onCommit();
    }

    private static void DrawOffMeshInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.OffMeshConnections, selection.Index, out var item))
            return;

        var changed = false;
        changed |= CustomizationEditorWidgets.DrawBool("Enabled", ref item.Enabled);
        changed |= CustomizationEditorWidgets.DrawVector3("Start", ref item.Start);
        changed |= CustomizationEditorWidgets.DrawVector3("End",   ref item.End);
        changed |= CustomizationEditorWidgets.DrawFloat("Radius", ref item.Radius, 0.05f, 0.01f, 10f);
        changed |= CustomizationEditorWidgets.DrawBool("Bidirectional", ref item.Bidirectional);
        changed |= CustomizationEditorWidgets.DrawInt("UserId", ref item.UserId);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("Area", ref item.Area);
        changed |= CustomizationEditorWidgets.DrawFlags("Flags", ref item.Flags);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("Kind", ref item.Kind);

        if (ImGui.Button("删除这一项"))
        {
            workspace.Draft.OffMeshConnections.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }

        if (changed) onCommit();
    }

    private static void DrawPreviewMeshInspector(Selection selection, CustomizationPreviewBuilder previewBuilder, AddMeshRemovalDelegate onAddMeshRemoval)
    {
        if (selection.Key == null || previewBuilder.Extractor == null || !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh))
            return;

        ImGui.TextUnformatted(selection.Key);
        ImGui.TextUnformatted($"{mesh.Parts.Count} parts, {mesh.Instances.Count} instances");
        ImGui.TextUnformatted($"Bounds: {mesh.LocalBounds.Min:f3} - {mesh.LocalBounds.Max:f3}");

        if (ImGui.Button("加入 mesh 删除清单"))
            onAddMeshRemoval(selection.Key);
    }

    private static void DrawPreviewInstanceInspector
    (
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        CustomizationPreviewBuilder      previewBuilder,
        CommitDelegate                   onCommit,
        AddInstancePatchDelegate         onAddInstancePatch
    )
    {
        if (selection.Key            == null                                          ||
            previewBuilder.Extractor == null                                          ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                       ||
            selection.Index >= mesh.Instances.Count)
            return;

        var selKey   = selection.Key;
        var selIndex = selection.Index;
        var instance = mesh.Instances[selIndex];
        ImGui.TextUnformatted($"Mesh: {selection.Key}");
        ImGui.TextUnformatted($"Instance: {instance.Id:X}");
        ImGui.TextUnformatted($"Bounds: {instance.WorldBounds.Min:f3} - {instance.WorldBounds.Max:f3}");

        var transformPatch = workspace.Draft.InstancePatches.FirstOrDefault
            (x => x.MeshKey == selKey && x.InstanceIndex == selIndex && x.Kind == DraftSceneInstancePatchKind.Transform);
        var transform = transformPatch?.WorldTransform ?? DraftMatrix4x3.FromRuntime(instance.WorldTransform);

        if (CustomizationEditorWidgets.DrawMatrix("Transform", ref transform))
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
            onCommit();
        }

        var flagsPatch = workspace.Draft.InstancePatches.FirstOrDefault
                             (x => x.MeshKey == selKey && x.InstanceIndex == selIndex && x.Kind == DraftSceneInstancePatchKind.SetFlags) ??
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
        flagsChanged |= CustomizationEditorWidgets.DrawFlags("Set Flags",   ref setFlags);
        flagsChanged |= CustomizationEditorWidgets.DrawFlags("Clear Flags", ref clearFlags);

        if (flagsChanged)
        {
            flagsPatch.ForceSetPrimFlags   = setFlags;
            flagsPatch.ForceClearPrimFlags = clearFlags;
            if (!workspace.Draft.InstancePatches.Contains(flagsPatch))
                workspace.Draft.InstancePatches.Add(flagsPatch);
            onCommit();
        }

        if (ImGui.Button("加入实例变换补丁"))
            onAddInstancePatch(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.Transform);
        ImGui.SameLine();
        if (ImGui.Button("加入实例 flags 补丁"))
            onAddInstancePatch(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.SetFlags);
        ImGui.SameLine();
        if (ImGui.Button("移除这个实例"))
            onAddInstancePatch(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.RemoveInstance);
        ImGui.SameLine();
        if (ImGui.Button("清空这个 mesh 的全部实例"))
            onAddInstancePatch(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.ClearInstances);
    }

    private static void DrawPreviewPartInspector
    (
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        CustomizationPreviewBuilder      previewBuilder,
        CommitDelegate                   onCommit,
        AddPartPatchDelegate             onAddPartPatch,
        RemoveMatchingPartPatchDelegate  onRemoveMatchingPartPatch
    )
    {
        if (selection.Key            == null                                          ||
            previewBuilder.Extractor == null                                          ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                       ||
            selection.Index >= mesh.Parts.Count)
            return;

        var selKey                 = selection.Key;
        var selIndex               = selection.Index;
        var part                   = mesh.Parts[selIndex];
        var selectedVertexIndex    = selection.Kind == SelectionKind.PreviewVertex ? selection.SubIndex : -1;
        var selectedPrimitiveIndex = selection.Kind == SelectionKind.PreviewPrimitive ? selection.SubIndex : -1;
        ImGui.TextUnformatted($"Mesh: {selection.Key}");
        ImGui.TextUnformatted($"Part: {selection.Index}");
        ImGui.TextUnformatted($"{part.Vertices.Count} vertices, {part.Primitives.Count} primitives");
        ImGui.TextUnformatted($"Bounds: {part.LocalBounds.Min:f3} - {part.LocalBounds.Max:f3}");

        if (selectedVertexIndex >= 0 && selectedVertexIndex < part.Vertices.Count)
        {
            var vertexPatch = workspace.Draft.PartPatches.FirstOrDefault
                (x => PartPatchMatches(x, selKey, selIndex, DraftScenePartPatchKind.Vertex, selectedVertexIndex));
            var position = vertexPatch?.Position ?? part.Vertices[selectedVertexIndex];
            ImGui.TextUnformatted($"Selected vertex: {selectedVertexIndex}");

            if (CustomizationEditorWidgets.DrawVector3("Position", ref position))
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
                onCommit();
            }

            if (ImGui.Button("删除顶点位置补丁"))
                onRemoveMatchingPartPatch(selection.Key, selection.Index, DraftScenePartPatchKind.Vertex, selectedVertexIndex);
        }
        else if (ImGui.Button("加入顶点位置补丁")) onAddPartPatch(mesh, selection.Key, selection.Index, DraftScenePartPatchKind.Vertex);

        ImGui.SameLine();

        if (selectedPrimitiveIndex >= 0 && selectedPrimitiveIndex < part.Primitives.Count)
        {
            var primitive = part.Primitives[selectedPrimitiveIndex];
            var flagsPatch = workspace.Draft.PartPatches.FirstOrDefault
                (x => PartPatchMatches(x, selKey, selIndex, DraftScenePartPatchKind.PrimitiveFlags, selectedPrimitiveIndex));
            var primitiveFlags = flagsPatch?.Flags ?? primitive.Flags;
            ImGui.TextUnformatted($"Selected primitive: {selectedPrimitiveIndex} {primitive.V1}x{primitive.V2}x{primitive.V3}");
            var flagsChanged = CustomizationEditorWidgets.DrawFlags("Flags", ref primitiveFlags);

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
                onCommit();
            }

            var editPatch = workspace.Draft.PartPatches.FirstOrDefault
                (x => PartPatchMatches(x, selKey, selIndex, DraftScenePartPatchKind.PrimitiveEdit, selectedPrimitiveIndex));
            var v1          = editPatch?.V1       ?? primitive.V1;
            var v2          = editPatch?.V2       ?? primitive.V2;
            var v3          = editPatch?.V3       ?? primitive.V3;
            var editFlags   = editPatch?.Flags    ?? primitive.Flags;
            var material    = editPatch?.Material ?? primitive.Material;
            var editChanged = false;
            editChanged |= CustomizationEditorWidgets.DrawInt("V1", ref v1);
            editChanged |= CustomizationEditorWidgets.DrawInt("V2", ref v2);
            editChanged |= CustomizationEditorWidgets.DrawInt("V3", ref v3);
            editChanged |= CustomizationEditorWidgets.DrawFlags("Edit Flags", ref editFlags);
            editChanged |= CustomizationEditorWidgets.DrawUInt64("Material", ref material);

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
                onCommit();
            }

            if (ImGui.Button("删除三角 flags 补丁"))
                onRemoveMatchingPartPatch(selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveFlags, selectedPrimitiveIndex);
            ImGui.SameLine();
            if (ImGui.Button("删除三角高级编辑补丁"))
                onRemoveMatchingPartPatch(selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveEdit, selectedPrimitiveIndex);
        }
        else
        {
            if (ImGui.Button("加入三角 flags 补丁"))
                onAddPartPatch(mesh, selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveFlags);
            ImGui.SameLine();
            if (ImGui.Button("加入三角高级编辑补丁"))
                onAddPartPatch(mesh, selection.Key, selection.Index, DraftScenePartPatchKind.PrimitiveEdit);
        }
    }

    internal static bool PartPatchMatches(DraftScenePartPatch patch, string key, int partIndex, DraftScenePartPatchKind kind, int subIndex)
    {
        if (patch.MeshKey != key || patch.PartIndex != partIndex || patch.Kind != kind)
            return false;

        return kind == DraftScenePartPatchKind.Vertex ? patch.VertexIndex == subIndex : patch.PrimitiveIndex == subIndex;
    }

    private static bool TryGetItem<T>(List<T> items, int index, out T item)
    {
        if (index < 0 || index >= items.Count)
        {
            item = default!;
            return false;
        }

        item = items[index];
        return true;
    }
}
