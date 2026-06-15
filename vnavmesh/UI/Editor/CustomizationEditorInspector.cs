using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility.Raii;
using vnavmesh.Navigation;
using vnavmesh.Navigation.Custom;
using vnavmesh.Navigation.Custom.Editor;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Editor.Types;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorInspector
{
    public delegate void CommitDelegate();
    public delegate void WorkspaceCreateDelegate();
    public delegate void WorkspaceDeleteDelegate();
    public delegate void WorkspaceSelectDelegate(string workspaceId);

    public delegate void AddMeshRemovalDelegate(string key);

    public delegate void AddInstancePatchDelegate(SceneExtractor.Mesh mesh, string key, int index, DraftSceneInstancePatchKind kind);

    public delegate void AddPartPatchDelegate(SceneExtractor.Mesh mesh, string key, int partIndex, DraftScenePartPatchKind kind, int subIndex = -1);

    public delegate void RemoveMatchingPartPatchDelegate(string key, int partIndex, DraftScenePartPatchKind kind, int subIndex);

    public delegate void RebuildSceneExtractDelegate(uint territoryID);

    public delegate void RebuildFullDelegate();

    public static void Draw
    (
        ref Selection                    selection,
        CustomizationEditorTerritoryStore store,
        bool                             hasWorkspace,
        ref CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder      previewBuilder,
        ref string                       statusText,
        uint                             territoryID,
        string                           territoryLabel,
        ref string                       exportDirText,
        NavmeshSettings                  settingsDefaults,
        NavmeshBuildProfile              profileDefaults,
        CommitDelegate                   onCommit,
        WorkspaceCreateDelegate          onCreateWorkspace,
        WorkspaceDeleteDelegate          onDeleteWorkspace,
        WorkspaceSelectDelegate          onSelectWorkspace,
        AddMeshRemovalDelegate           onAddMeshRemoval,
        AddInstancePatchDelegate         onAddInstancePatch,
        AddPartPatchDelegate             onAddPartPatch,
        RemoveMatchingPartPatchDelegate  onRemoveMatchingPartPatch,
        RebuildSceneExtractDelegate      onRebuildSceneExtract,
        RebuildFullDelegate              onRebuildFull
    )
    {
        DrawInspectorHeader(selection);

        switch (selection.Kind)
        {
            case SelectionKind.Workspace:
                DrawWorkspaceInspector(store, hasWorkspace, ref workspace, ref exportDirText, onCommit, onCreateWorkspace, onDeleteWorkspace, onSelectWorkspace);
                break;
            case SelectionKind.BuildProfile:
                if (!hasWorkspace)
                    break;
                DrawBuildProfileInspector(ref workspace, profileDefaults, settingsDefaults, onCommit);
                break;
            case SelectionKind.BuildSettings:
                if (!hasWorkspace)
                    break;
                DrawBuildSettingsInspector(ref workspace, settingsDefaults, onCommit);
                break;
            case SelectionKind.FlyingOverride:
                if (!hasWorkspace)
                    break;
                DrawFlyingInspector(ref workspace, onCommit);
                break;
            case SelectionKind.MeshRemoval:
                if (!hasWorkspace)
                    break;
                DrawMeshRemovalInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.InstancePatch:
                if (!hasWorkspace)
                    break;
                DrawInstancePatchInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.PartPatch:
                if (!hasWorkspace)
                    break;
                DrawPartPatchInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.ColliderInsertion:
                if (!hasWorkspace)
                    break;
                DrawColliderInsertionInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.MeshLink:
                if (!hasWorkspace)
                    break;
                DrawMeshLinkInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.OffMeshConnection:
                if (!hasWorkspace)
                    break;
                DrawOffMeshInspector(ref workspace, ref selection, onCommit);
                break;
            case SelectionKind.PreviewMesh:
                if (!hasWorkspace)
                    break;
                DrawPreviewMeshInspector(selection, previewBuilder, onAddMeshRemoval);
                break;
            case SelectionKind.PreviewInstance:
                if (!hasWorkspace)
                    break;
                DrawPreviewInstanceInspector(ref workspace, ref selection, previewBuilder, onCommit, onAddInstancePatch);
                break;
            case SelectionKind.PreviewPart:
            case SelectionKind.PreviewVertex:
            case SelectionKind.PreviewPrimitive:
                if (!hasWorkspace)
                    break;
                DrawPreviewPartInspector(ref workspace, ref selection, previewBuilder, onCommit, onAddPartPatch, onRemoveMatchingPartPatch);
                break;
            case SelectionKind.Diagnostics:
                DrawDiagnosticsPanel(previewBuilder, territoryID, onRebuildSceneExtract, onRebuildFull, ref statusText);
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
            SelectionKind.MeshRemoval       => $"网格删除 [{selection.Index}]",
            SelectionKind.InstancePatch     => $"实例补丁 [{selection.Index}]",
            SelectionKind.PartPatch         => $"顶点 / 三角补丁 [{selection.Index}]",
            SelectionKind.ColliderInsertion => $"碰撞插入 [{selection.Index}]",
            SelectionKind.MeshLink          => $"网格连接 [{selection.Index}]",
            SelectionKind.OffMeshConnection => $"离网连接 [{selection.Index}]",
            SelectionKind.PreviewMesh       => $"预览网格: {selection.Key}",
            SelectionKind.PreviewInstance   => $"预览实例: {selection.Key} #{selection.Index}",
            SelectionKind.PreviewPart       => $"预览部分: {selection.Key} p{selection.Index}",
            SelectionKind.PreviewVertex     => $"预览顶点: {selection.Key} p{selection.Index} v{selection.SubIndex}",
            SelectionKind.PreviewPrimitive  => $"预览三角: {selection.Key} p{selection.Index} t{selection.SubIndex}",
            SelectionKind.Diagnostics       => "诊断",
            _                               => "自定义编辑器"
        };

    private static string GetSelectionHelp(Selection selection) =>
        selection.Kind switch
        {
            SelectionKind.Workspace         => "管理草稿保存、自动重建和 C# 导出位置",
            SelectionKind.BuildProfile      => "覆盖 Recast 构建参数, 如像素尺寸、区域合并、边缘长度等",
            SelectionKind.BuildSettings     => "覆盖本区域的构建设置, 修改后自动更新预览",
            SelectionKind.FlyingOverride    => "指定本区域是否支持飞行导航",
            SelectionKind.Diagnostics       => "构建统计、阶段耗时与慢瓦片分析",
            SelectionKind.MeshRemoval       => "将整个模型从提取场景移除, 适合门、碎石等不应参与导航的碰撞体",
            SelectionKind.InstancePatch     => "修改或移除某模型实例, 可强制写入或清除碰撞标记",
            SelectionKind.PartPatch         => "修改模型内部顶点或三角, 适合局部几何修补",
            SelectionKind.ColliderInsertion => "两点生成的障碍体, 用中心和尺寸调整后自动重建预览",
            SelectionKind.MeshLink          => "在已构建导航网格上追加连线或客户端路径",
            SelectionKind.OffMeshConnection => "在构建参数中追加离网连接",
            SelectionKind.PreviewMesh       => "实时预览模型, 右键可加入草稿删除清单",
            SelectionKind.PreviewInstance   => "实时预览实例, 可在世界中点选并编辑变换或标记",
            SelectionKind.PreviewPart       => "实时预览部件, 展开顶点或三角后可直接编辑",
            SelectionKind.PreviewVertex     => "实时预览顶点, 调整位置后自动生成顶点补丁",
            SelectionKind.PreviewPrimitive  => "实时预览三角, 调整标记或顶点索引后自动生成三角补丁",
            _                               => "左侧选择对象或草稿项后, 这里显示可执行操作"
        };

    private static void DrawWorkspaceInspector
    (
        CustomizationEditorTerritoryStore store,
        bool                             hasWorkspace,
        ref CustomizationEditorWorkspace workspace,
        ref string                       exportDirText,
        CommitDelegate                   onCommit,
        WorkspaceCreateDelegate          onCreateWorkspace,
        WorkspaceDeleteDelegate          onDeleteWorkspace,
        WorkspaceSelectDelegate          onSelectWorkspace
    )
    {
        if (ImGui.Button("新建工作区"))
            onCreateWorkspace();

        ImGui.SameLine();
        using (ImRaii.Disabled(!hasWorkspace))
        {
            if (ImGui.Button("删除当前工作区"))
                onDeleteWorkspace();
        }

        if (!hasWorkspace)
        {
            ImGui.Separator();
            ImGui.TextDisabled("当前区域暂无工作区");
            ImGui.TextWrapped("新建工作区后, 编辑、保存、导出和预览都会只针对该工作区");
            return;
        }

        ImGui.TextUnformatted($"当前工作区: {workspace.WorkspaceName}");
        ImGui.TextDisabled(workspace.IsApplied ? "当前生效结果: 当前工作区" : "当前生效结果: 默认场景");
        ImGui.TextDisabled("当前编辑、保存和导出都只针对当前工作区");
        ImGui.Separator();

        if (ImGui.BeginCombo("切换工作区", workspace.WorkspaceName))
        {
            foreach (var item in store.Workspaces)
            {
                var selected = item.WorkspaceId == workspace.WorkspaceId;
                if (ImGui.Selectable(item.WorkspaceName, selected))
                    onSelectWorkspace(item.WorkspaceId);
                if (selected)
                    ImGui.SetItemDefaultFocus();
            }

            ImGui.EndCombo();
        }

        var renamed = workspace.WorkspaceName;
        if (ImGui.InputText("工作区名称", ref renamed) && !string.IsNullOrWhiteSpace(renamed))
        {
            workspace.WorkspaceName = renamed.Trim();
            onCommit();
        }

        if (CustomizationEditorWidgets.DrawBool("当前工作区生效", ref workspace.IsApplied))
            onCommit();

        CustomizationEditorWidgets.DrawBool("自动保存", ref workspace.Settings.AutoSave);

        if (ImGui.InputText("导出目录", ref exportDirText))
        {
            workspace.Settings.ExportDirectory = exportDirText;
            onCommit();
        }
    }

    private static void DrawBuildProfileInspector
        (ref CustomizationEditorWorkspace workspace, NavmeshBuildProfile profileDefaults, NavmeshSettings settingsDefaults, CommitDelegate onCommit)
    {
        var changed = false;

        if (ImGui.TreeNodeEx("像素", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("像素尺寸", ref workspace.Draft.BuildProfile.CellSizeOverride, settingsDefaults.CellSize,
                "XZ 平面体素尺寸. 越小细节越高但构建越慢, 推荐角色半径 /2 或 /3");
            changed |= CustomizationEditorWidgets.DrawNullableFloat("像素高度", ref workspace.Draft.BuildProfile.CellHeightOverride, settingsDefaults.CellHeight,
                "Y 轴体素高度. 推荐为像素尺寸一半");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("区域", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableEnum
                ("分区算法", ref workspace.Draft.BuildProfile.PartitioningOverride, settingsDefaults.Partitioning,
                    "Watershed = 质量最高最慢, Monotone = 最快, Layer = 折中");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("区域最小尺寸", ref workspace.Draft.BuildProfile.RegionMinSizeOverride, settingsDefaults.RegionMinSize,
                    "小于此体素数的孤立区域会被移除");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("区域合并尺寸", ref workspace.Draft.BuildProfile.RegionMergeSizeOverride, settingsDefaults.RegionMergeSize,
                    "体素数小于此值的区域尽量合并到相邻大区域");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("多边形化", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大边长", ref workspace.Draft.BuildProfile.PolyMaxEdgeLenOverride, settingsDefaults.PolyMaxEdgeLen,
                    "轮廓边最大长度, 防止生成长条三角. 推荐角色半径 *8, 0=不限制");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大简化误差", ref workspace.Draft.BuildProfile.PolyMaxSimplificationErrorOverride, settingsDefaults.PolyMaxSimplificationError,
                    "轮廓简化允许的最大偏离. 推荐 1.1 ~ 1.5");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("代理", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("半径", ref workspace.Draft.BuildProfile.AgentRadiusOverride, settingsDefaults.AgentRadius,
                "网格边缘离障碍物距离. >0 自动收缩, =0 需自行碰撞检测");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("细节网格", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("采样距离", ref workspace.Draft.BuildProfile.DetailSampleDistOverride, settingsDefaults.DetailSampleDist,
                    "细节网格采样距离. 推荐约 6, 0=禁用");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("边缘链接", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("生成向下攀爬", ref workspace.Draft.BuildProfile.GenerateEdgeClimbLinksOverride, settingsDefaults.GenerateEdgeClimbLinks,
                    "在边缘生成向下攀爬链接");
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("生成向下跳跃", ref workspace.Draft.BuildProfile.GenerateEdgeJumpLinksOverride, settingsDefaults.GenerateEdgeJumpLinks,
                    "在边缘生成向下跳跃链接");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("体积", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableIntArray
                ("体积瓦片", ref workspace.Draft.BuildProfile.VolumeTilesOverride, settingsDefaults.VolumeTiles,
                    "体积细分每轴数量 [L2 瓦片数, L3 体素数]");
            ImGui.TreePop();
        }

        if (changed) onCommit();
    }

    private static void DrawBuildSettingsInspector(ref CustomizationEditorWorkspace workspace, NavmeshSettings settingsDefaults, CommitDelegate onCommit)
    {
        var changed = false;

        if (ImGui.TreeNodeEx("通用", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableBool("快速构建", ref workspace.Draft.BuildSettings.FastBuild, settingsDefaults.FastBuild,
                "关闭细节网格以加速构建");
            changed |= CustomizationEditorWidgets.DrawNullableFlags("过滤", ref workspace.Draft.BuildSettings.Filtering, settingsDefaults.Filtering,
                "选择过滤通道以移除体素化伪影");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("像素", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("像素尺寸", ref workspace.Draft.BuildSettings.CellSize, settingsDefaults.CellSize,
                "XZ 平面体素尺寸. 越小细节越高但构建越慢, 推荐角色半径 /2 或 /3");
            changed |= CustomizationEditorWidgets.DrawNullableFloat("像素高度", ref workspace.Draft.BuildSettings.CellHeight, settingsDefaults.CellHeight,
                "Y 轴体素高度. 推荐为像素尺寸一半");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("代理", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("高度", ref workspace.Draft.BuildSettings.AgentHeight, settingsDefaults.AgentHeight,
                "可行走区域最小净空, 低于此高度的区域不可行走");
            changed |= CustomizationEditorWidgets.DrawNullableFloat("半径", ref workspace.Draft.BuildSettings.AgentRadius, settingsDefaults.AgentRadius,
                "网格边缘离障碍物距离. >0 自动收缩, =0 需自行碰撞检测");
            changed |= CustomizationEditorWidgets.DrawNullableFloat("最大攀爬", ref workspace.Draft.BuildSettings.AgentMaxClimb, settingsDefaults.AgentMaxClimb,
                "台阶最大高度, 高于此值的台阶视为障碍物需绕行");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大坡度", ref workspace.Draft.BuildSettings.AgentMaxSlopeDeg, settingsDefaults.AgentMaxSlopeDeg,
                    "可行走最大地面坡度 (度), 超出标记为不可行走");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("区域", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat("最小尺寸", ref workspace.Draft.BuildSettings.RegionMinSize, settingsDefaults.RegionMinSize,
                "小于此体素数的孤立区域会被移除");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("合并尺寸", ref workspace.Draft.BuildSettings.RegionMergeSize, settingsDefaults.RegionMergeSize,
                    "体素数小于此值的区域尽量合并到相邻大区域");
            changed |= CustomizationEditorWidgets.DrawNullableEnum("分区算法", ref workspace.Draft.BuildSettings.Partitioning, settingsDefaults.Partitioning,
                "Watershed = 质量最高最慢, Monotone = 最快, Layer = 折中");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("多边形化", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大边长", ref workspace.Draft.BuildSettings.PolyMaxEdgeLen, settingsDefaults.PolyMaxEdgeLen,
                    "轮廓边最大长度. 推荐角色半径 *8, 0=不限制");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大简化误差", ref workspace.Draft.BuildSettings.PolyMaxSimplificationError, settingsDefaults.PolyMaxSimplificationError,
                    "轮廓简化允许最大偏离. 推荐 1.1 ~ 1.5");
            changed |= CustomizationEditorWidgets.DrawNullableInt("每多边形最大顶点数", ref workspace.Draft.BuildSettings.PolyMaxVerts, settingsDefaults.PolyMaxVerts,
                "单个多边形允许的最大顶点数, 至少 3");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("细节网格", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("采样距离", ref workspace.Draft.BuildSettings.DetailSampleDist, settingsDefaults.DetailSampleDist,
                    "细节网格采样距离. 推荐约 6, 0=禁用");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大采样误差", ref workspace.Draft.BuildSettings.DetailMaxSampleError, settingsDefaults.DetailMaxSampleError,
                    "细节网格偏离高度场的最大距离");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("边缘链接", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("生成向下攀爬", ref workspace.Draft.BuildSettings.GenerateEdgeClimbLinks, settingsDefaults.GenerateEdgeClimbLinks,
                    "在边缘生成向下攀爬链接");
            changed |= CustomizationEditorWidgets.DrawNullableBool
                ("生成向下跳跃", ref workspace.Draft.BuildSettings.GenerateEdgeJumpLinks, settingsDefaults.GenerateEdgeJumpLinks,
                    "在边缘生成向下跳跃链接");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("向下攀爬", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("地面容差", ref workspace.Draft.BuildSettings.GroundTolerance, settingsDefaults.GroundTolerance,
                    "边缘链接检测的地面容差范围");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("攀爬距离", ref workspace.Draft.BuildSettings.ClimbDownDistance, settingsDefaults.ClimbDownDistance,
                    "边缘攀爬采样的水平搜索距离");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大高度", ref workspace.Draft.BuildSettings.ClimbDownMaxHeight, settingsDefaults.ClimbDownMaxHeight,
                    "向下攀爬链接允许的最大落差值");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最小高度", ref workspace.Draft.BuildSettings.ClimbDownMinHeight, settingsDefaults.ClimbDownMinHeight,
                    "向下攀爬链接所需的最小落差值");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("边缘跳跃", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("结束距离", ref workspace.Draft.BuildSettings.EdgeJumpEndDistance, settingsDefaults.EdgeJumpEndDistance,
                    "边缘跳跃终点水平搜索范围");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("高度", ref workspace.Draft.BuildSettings.EdgeJumpHeight, settingsDefaults.EdgeJumpHeight,
                    "边缘跳跃高度阈值");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最大落差", ref workspace.Draft.BuildSettings.EdgeJumpMaxDrop, settingsDefaults.EdgeJumpMaxDrop,
                    "边缘跳跃允许的最大垂直落差");
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("最小落差", ref workspace.Draft.BuildSettings.EdgeJumpMinDrop, settingsDefaults.EdgeJumpMinDrop,
                    "边缘跳跃所需的最小垂直落差");
            ImGui.TreePop();
        }

        if (ImGui.TreeNodeEx("瓦片", ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= CustomizationEditorWidgets.DrawNullableFloat
                ("地面区块目标尺寸", ref workspace.Draft.BuildSettings.GroundTileSize, settingsDefaults.GroundTileSize,
                    "自动推导地面区块数量时使用的目标世界尺寸");
            changed |= CustomizationEditorWidgets.DrawNullableInt
                ("地面区块数量上限", ref workspace.Draft.BuildSettings.GroundTileCountMax, settingsDefaults.GroundTileCountMax,
                    "自动推导地面区块数量时的单轴上限");
            changed |= CustomizationEditorWidgets.DrawNullableIntArray("体积细分", ref workspace.Draft.BuildSettings.VolumeTiles, settingsDefaults.VolumeTiles,
                "体积细分每轴数量 [L2 瓦片数, L3 体素数]");
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

        if (CustomizationEditorWidgets.DrawEnumCombo("支持飞行", ref next, ["默认", "启用", "禁用"]))
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
        var enabledBefore = item.Enabled;
        if (DrawEnabledWithDelete(ref item.Enabled))
        {
            workspace.Draft.MeshRemovals.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }
        changed |= enabledBefore != item.Enabled;

        changed |= CustomizationEditorWidgets.DrawString("网格键", ref item.MeshKey);
        changed |= CustomizationEditorWidgets.DrawString("备注", ref item.Note);

        if (changed) 
            onCommit();
    }

    private static void DrawInstancePatchInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.InstancePatches, selection.Index, out var item))
            return;

        var changed = false;
        var enabledBefore = item.Enabled;
        if (DrawEnabledWithDelete(ref item.Enabled))
        {
            workspace.Draft.InstancePatches.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }
        changed |= enabledBefore != item.Enabled;

        changed |= CustomizationEditorWidgets.DrawString("网格键", ref item.MeshKey);
        changed |= CustomizationEditorWidgets.DrawString("备注", ref item.Note);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("类型", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawInt("实例索引", ref item.InstanceIndex);
        changed |= CustomizationEditorWidgets.DrawUInt64("实例编号", ref item.InstanceId);
        changed |= CustomizationEditorWidgets.DrawMatrix("世界变换", ref item.WorldTransform);
        changed |= CustomizationEditorWidgets.DrawFlags("设置标记", ref item.ForceSetPrimFlags);
        changed |= CustomizationEditorWidgets.DrawFlags("清除标记", ref item.ForceClearPrimFlags);

        if (changed) onCommit();
    }

    private static void DrawPartPatchInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.PartPatches, selection.Index, out var item))
            return;

        var changed = false;
        var enabledBefore = item.Enabled;
        if (DrawEnabledWithDelete(ref item.Enabled))
        {
            workspace.Draft.PartPatches.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }
        changed |= enabledBefore != item.Enabled;

        changed |= CustomizationEditorWidgets.DrawString("网格键", ref item.MeshKey);
        changed |= CustomizationEditorWidgets.DrawString("备注", ref item.Note);
        changed |= CustomizationEditorWidgets.DrawInt("部件索引", ref item.PartIndex);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("类型", ref item.Kind);

        ImGui.Separator();

        switch (item.Kind)
        {
            case DraftScenePartPatchKind.Vertex:
                changed |= CustomizationEditorWidgets.DrawInt("顶点索引", ref item.VertexIndex);
                changed |= CustomizationEditorWidgets.DrawVector3("位置", ref item.Position);
                break;

            case DraftScenePartPatchKind.PrimitiveFlags:
                changed |= CustomizationEditorWidgets.DrawInt("三角索引", ref item.PrimitiveIndex);
                changed |= CustomizationEditorWidgets.DrawFlags("标记", ref item.Flags);

                if (ImGui.Button("不可行走"))
                {
                    item.Flags = SceneExtractor.PrimitiveFlags.ForceUnwalkable;
                    changed = true;
                }
                ImGui.SameLine();
                if (ImGui.Button("可行走"))
                {
                    item.Flags = SceneExtractor.PrimitiveFlags.ForceWalkable;
                    changed = true;
                }
                ImGui.SameLine();
                if (ImGui.Button("飞行穿透"))
                {
                    item.Flags = SceneExtractor.PrimitiveFlags.FlyThrough;
                    changed = true;
                }
                ImGui.SameLine();
                if (ImGui.Button("不可降落"))
                {
                    item.Flags = SceneExtractor.PrimitiveFlags.Unlandable;
                    changed = true;
                }
                ImGui.SameLine();
                if (ImGui.Button("清除"))
                {
                    item.Flags = SceneExtractor.PrimitiveFlags.None;
                    changed = true;
                }
                break;

            case DraftScenePartPatchKind.PrimitiveEdit:
                changed |= CustomizationEditorWidgets.DrawInt("三角索引", ref item.PrimitiveIndex);
                changed |= CustomizationEditorWidgets.DrawInt("顶点 1", ref item.V1);
                changed |= CustomizationEditorWidgets.DrawInt("顶点 2", ref item.V2);
                changed |= CustomizationEditorWidgets.DrawInt("顶点 3", ref item.V3);
                changed |= CustomizationEditorWidgets.DrawUInt64("材质", ref item.Material);
                changed |= CustomizationEditorWidgets.DrawFlags("标记", ref item.Flags);

                if (ImGui.Button("移除三角形"))
                {
                    item.V1 = item.V2 = item.V3 = 0;
                    changed = true;
                }
                if (ImGui.IsItemHovered())
                    ImGui.SetTooltip("将三个顶点索引设为相同值");
                break;
        }

        if (changed)
            onCommit();
    }

    private static void DrawColliderInsertionInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.ColliderInsertions, selection.Index, out var item))
            return;

        var changed = false;
        var enabledBefore = item.Enabled;
        if (DrawEnabledWithDelete(ref item.Enabled))
        {
            workspace.Draft.ColliderInsertions.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }
        changed |= enabledBefore != item.Enabled;

        changed |= CustomizationEditorWidgets.DrawString("备注", ref item.Note);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("类型", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawBoundsEditor("几何", ref item.Min, ref item.Max);
        changed |= CustomizationEditorWidgets.DrawFlags("设置标记", ref item.ForceSetPrimFlags);
        changed |= CustomizationEditorWidgets.DrawFlags("清除标记", ref item.ForceClearPrimFlags);

        if (changed) onCommit();
    }

    private static void DrawMeshLinkInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.MeshLinks, selection.Index, out var item))
            return;

        var changed = false;
        var enabledBefore = item.Enabled;
        if (DrawEnabledWithDelete(ref item.Enabled))
        {
            workspace.Draft.MeshLinks.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }
        changed |= enabledBefore != item.Enabled;

        changed |= CustomizationEditorWidgets.DrawString("备注", ref item.Note);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("类型", ref item.Kind);
        changed |= CustomizationEditorWidgets.DrawVector3("起点", ref item.Start);
        changed |= CustomizationEditorWidgets.DrawVector3("终点", ref item.End);
        changed |= CustomizationEditorWidgets.DrawBool("双向", ref item.Bidirectional);

        if (changed) onCommit();
    }

    private static void DrawOffMeshInspector(ref CustomizationEditorWorkspace workspace, ref Selection selection, CommitDelegate onCommit)
    {
        if (!TryGetItem(workspace.Draft.OffMeshConnections, selection.Index, out var item))
            return;

        var changed = false;
        var enabledBefore = item.Enabled;
        if (DrawEnabledWithDelete(ref item.Enabled))
        {
            workspace.Draft.OffMeshConnections.RemoveAt(selection.Index);
            selection = new(SelectionKind.Workspace);
            onCommit();
            return;
        }
        changed |= enabledBefore != item.Enabled;

        changed |= CustomizationEditorWidgets.DrawString("备注", ref item.Note);
        changed |= CustomizationEditorWidgets.DrawVector3("起点", ref item.Start);
        changed |= CustomizationEditorWidgets.DrawVector3("终点", ref item.End);
        changed |= CustomizationEditorWidgets.DrawFloat("半径", ref item.Radius, 0.05f, 0.01f, 10f);
        changed |= CustomizationEditorWidgets.DrawBool("双向", ref item.Bidirectional);
        changed |= CustomizationEditorWidgets.DrawInt("用户编号", ref item.UserId);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("区域", ref item.Area);
        changed |= CustomizationEditorWidgets.DrawFlags("标记", ref item.Flags);
        changed |= CustomizationEditorWidgets.DrawEnumCombo("类型", ref item.Kind);

        if (changed) onCommit();
    }

    private static void DrawDiagnosticsPanel
    (
        CustomizationPreviewBuilder previewBuilder,
        uint                        territoryID,
        RebuildSceneExtractDelegate onRebuildSceneExtract,
        RebuildFullDelegate         onRebuildFull,
        ref string                  statusText
    )
    {
        if (previewBuilder.CurrentState == CustomizationPreviewBuilder.State.InProgress)
        {
            ImGui.TextDisabled("构建进行中...");
            return;
        }

        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready)
        {
            ImGui.TextDisabled("尚无构建数据, 请先构建导航网格");
            return;
        }

        using (var d = ImRaii.Disabled(false))
        {
            if (ImGui.Button("重建场景提取"))
            {
                onRebuildSceneExtract(territoryID);
                statusText = "已触发场景提取重建";
            }

            ImGui.SameLine();
            if (ImGui.Button("重建完整导航网格"))
            {
                onRebuildFull();
                statusText = "已触发完整重建";
            }
        }

        ImGui.Separator();

        if (previewBuilder is { BuildTelemetry: { } telemetry })
        {
            ImGui.TextUnformatted($"核心: {telemetry.ConfiguredBuildMaxCores} (可用 {telemetry.MaxAvailableCores})");
            ImGui.TextUnformatted($"线程: {telemetry.ThreadCount}");
            ImGui.TextUnformatted($"并行构建耗时: {telemetry.ParallelTicks / (double)TimeSpan.TicksPerMillisecond:f1} ms");

            if (ImGui.TreeNodeEx("构建阶段", ImGuiTreeNodeFlags.DefaultOpen))
            {
                foreach (var phase in telemetry.Phases)
                    ImGui.TextUnformatted($"{phase.Name}: {phase.TotalTicks / (double)TimeSpan.TicksPerMillisecond:f1} ms, " +
                                           $"占比 {phase.ShareOfPhaseTicks:P1}, 最慢 {phase.SlowestTileX}x{phase.SlowestTileZ}");
                ImGui.TreePop();
            }

            if (telemetry.SlowTiles.Count > 0 && ImGui.TreeNodeEx("慢瓦片"))
            {
                foreach (var tile in telemetry.SlowTiles)
                    ImGui.TextUnformatted($"{tile.TileX}x{tile.TileZ}: {tile.TotalTicks / (double)TimeSpan.TicksPerMillisecond:f1} ms, " +
                                           $"几何 {tile.GeometryJobCount}, 地形 {tile.TerrainJobCount}, " +
                                           $"Poly {tile.PolyCount}, Vert {tile.VertCount}, Tri {tile.DetailTriCount}");
                ImGui.TreePop();
            }
        }
    }

    private static void DrawPreviewMeshInspector(Selection selection, CustomizationPreviewBuilder previewBuilder, AddMeshRemovalDelegate onAddMeshRemoval)
    {
        if (selection.Key == null || previewBuilder.Extractor == null || !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh))
            return;

        ImGui.TextUnformatted(selection.Key);
        ImGui.TextUnformatted($"{mesh.Parts.Count} 个部件, {mesh.Instances.Count} 个实例");
        ImGui.TextUnformatted($"边界: {mesh.LocalBounds.Min:f3} - {mesh.LocalBounds.Max:f3}");

        if (ImGui.Button("加入网格删除清单"))
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
        ImGui.TextUnformatted($"网格: {selection.Key}");
        ImGui.TextUnformatted($"实例: {instance.Id:X}");
        ImGui.TextUnformatted($"边界: {instance.WorldBounds.Min:f3} - {instance.WorldBounds.Max:f3}");

        var transformPatch = workspace.Draft.InstancePatches.FirstOrDefault
            (x => x.MeshKey == selKey && x.InstanceIndex == selIndex && x.Kind == DraftSceneInstancePatchKind.Transform);
        var transform = transformPatch?.WorldTransform ?? DraftMatrix4x3.FromRuntime(instance.WorldTransform);

        if (CustomizationEditorWidgets.DrawMatrix("变换", ref transform))
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
        flagsChanged |= CustomizationEditorWidgets.DrawFlags("设置标记", ref setFlags);
        flagsChanged |= CustomizationEditorWidgets.DrawFlags("清除标记", ref clearFlags);

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
        if (ImGui.Button("加入实例标记补丁"))
            onAddInstancePatch(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.SetFlags);
        ImGui.SameLine();
        if (ImGui.Button("移除这个实例"))
            onAddInstancePatch(mesh, selection.Key, selection.Index, DraftSceneInstancePatchKind.RemoveInstance);
        ImGui.SameLine();
        if (ImGui.Button("清空这个网格的全部实例"))
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

        ImGui.TextUnformatted($"网格: {selection.Key}");
        ImGui.TextUnformatted($"部件: {selection.Index}");
        ImGui.TextUnformatted($"{part.Vertices.Count} 个顶点, {part.Primitives.Count} 个三角");
        ImGui.TextUnformatted($"边界: {part.LocalBounds.Min:f3} - {part.LocalBounds.Max:f3}");
        ImGui.Separator();

        if (selectedVertexIndex >= 0 && selectedVertexIndex < part.Vertices.Count)
        {
            DrawPreviewVertexInfo(workspace, part, selKey, selIndex, selectedVertexIndex, mesh, onAddPartPatch);
        }
        else if (selectedPrimitiveIndex >= 0 && selectedPrimitiveIndex < part.Primitives.Count)
        {
            DrawPreviewPrimitiveInfo(workspace, part, selKey, selIndex, selectedPrimitiveIndex, mesh, onAddPartPatch);
        }
        else
        {
            ImGui.TextDisabled("在左侧树中选择顶点或三角以查看详情");
        }
    }

    private static void DrawPreviewVertexInfo
    (
        CustomizationEditorWorkspace workspace,
        SceneExtractor.MeshPart      part,
        string                       meshKey,
        int                          partIndex,
        int                          vertexIndex,
        SceneExtractor.Mesh          mesh,
        AddPartPatchDelegate         onAddPartPatch
    )
    {
        var originalPosition = part.Vertices[vertexIndex];
        var existingPatch = workspace.Draft.PartPatches.FirstOrDefault
            (x => PartPatchMatches(x, meshKey, partIndex, DraftScenePartPatchKind.Vertex, vertexIndex));

        ImGui.TextUnformatted($"顶点索引: {vertexIndex}");
        ImGui.TextUnformatted($"原始位置: {originalPosition:f3}");

        if (existingPatch != null)
        {
            ImGui.Separator();
            ImGui.TextColored(new Vector4(0.3f, 1f, 0.3f, 1f), "✓ 已有位置补丁");
            ImGui.TextUnformatted($"补丁位置: {existingPatch.Position:f3}");
            ImGui.TextDisabled("要编辑此补丁, 请在左侧草稿区选择对应的补丁项");
        }
        else
        {
            ImGui.Separator();
            if (ImGui.Button("创建位置补丁"))
            {
                onAddPartPatch(mesh, meshKey, partIndex, DraftScenePartPatchKind.Vertex, vertexIndex);
            }
            ImGui.SameLine();
            ImGui.TextDisabled("将以当前位置创建补丁");
        }
    }

    private static void DrawPreviewPrimitiveInfo
    (
        CustomizationEditorWorkspace workspace,
        SceneExtractor.MeshPart      part,
        string                       meshKey,
        int                          partIndex,
        int                          primitiveIndex,
        SceneExtractor.Mesh          mesh,
        AddPartPatchDelegate         onAddPartPatch
    )
    {
        var primitive = part.Primitives[primitiveIndex];
        var flagsPatch = workspace.Draft.PartPatches.FirstOrDefault
            (x => PartPatchMatches(x, meshKey, partIndex, DraftScenePartPatchKind.PrimitiveFlags, primitiveIndex));
        var editPatch = workspace.Draft.PartPatches.FirstOrDefault
            (x => PartPatchMatches(x, meshKey, partIndex, DraftScenePartPatchKind.PrimitiveEdit, primitiveIndex));

        ImGui.TextUnformatted($"三角索引: {primitiveIndex}");
        ImGui.TextUnformatted($"顶点索引: {primitive.V1} / {primitive.V2} / {primitive.V3}");
        ImGui.TextUnformatted($"标记: {primitive.Flags}");
        ImGui.TextUnformatted($"材质: {primitive.Material}");

        ImGui.Separator();

        if (flagsPatch != null)
        {
            ImGui.TextColored(new Vector4(0.3f, 1f, 1f, 1f), "✓ 已有标记补丁");
            ImGui.TextUnformatted($"补丁标记: {flagsPatch.Flags}");
        }
        else
        {
            if (ImGui.Button("创建标记补丁"))
            {
                onAddPartPatch(mesh, meshKey, partIndex, DraftScenePartPatchKind.PrimitiveFlags, primitiveIndex);
            }
            ImGui.SameLine();
            ImGui.TextDisabled("将以当前标记创建补丁");
        }

        ImGui.Separator();

        if (editPatch != null)
        {
            ImGui.TextColored(new Vector4(1f, 0.3f, 1f, 1f), "✓ 已有高级编辑补丁");
            ImGui.TextUnformatted($"补丁顶点: {editPatch.V1} / {editPatch.V2} / {editPatch.V3}");
            ImGui.TextUnformatted($"补丁标记: {editPatch.Flags}");
            ImGui.TextUnformatted($"补丁材质: {editPatch.Material}");
        }
        else
        {
            if (ImGui.Button("创建高级编辑补丁"))
            {
                onAddPartPatch(mesh, meshKey, partIndex, DraftScenePartPatchKind.PrimitiveEdit, primitiveIndex);
            }
            ImGui.SameLine();
            ImGui.TextDisabled("将以当前值创建补丁");
        }

        if (flagsPatch != null || editPatch != null)
        {
            ImGui.Separator();
            ImGui.TextDisabled("要编辑补丁, 请在左侧草稿区选择对应的补丁项");
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

    private static bool DrawEnabledWithDelete(ref bool enabled)
    {
        CustomizationEditorWidgets.DrawBool("启用", ref enabled);
        ImGui.SameLine();
        return ImGui.Button("删除");
    }
}
