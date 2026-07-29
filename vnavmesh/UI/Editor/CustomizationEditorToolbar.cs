using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface;
using Dalamud.Interface.Utility.Raii;
using vnavmesh.UI.Editor.Types;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorToolbar
{
    public delegate void VoidDelegate();

    private const int VK_LBUTTON = 0x01;
    private const int VK_ESCAPE  = 0x1B;

    private static readonly ToolDefinition[] SelectionTools =
    [
        new(FontAwesomeIcon.MousePointer, "浏览",   PickKind.None,           "查看和选择对象, 左侧选中后右侧显示可编辑内容"),
        new(FontAwesomeIcon.Cube,         "碰撞体", PickKind.SelectCollider, "在游戏画面点选碰撞体, 选中后右侧直接编辑"),
        new(FontAwesomeIcon.DrawPolygon,  "三角形", PickKind.SelectTriangle, "在游戏画面点选三角形, 选中后右侧直接编辑")
    ];

    private static readonly ToolDefinition[] VolumeTools =
    [
        new(FontAwesomeIcon.Cube,   "AABB",     PickKind.Aabb,             "在画面点两个世界点生成轴对齐障碍体"),
        new(FontAwesomeIcon.Cube,   "旋转箱体", PickKind.OrientedBox,      "沿画面中两个世界点的水平连线生成旋转箱体障碍"),
        new(FontAwesomeIcon.Circle, "圆柱",     PickKind.Cylinder,         "在画面点两个世界点生成圆柱障碍体"),
        new(FontAwesomeIcon.Circle, "定向圆柱", PickKind.OrientedCylinder, "沿画面中两个世界点生成任意方向圆柱体"),
        new(FontAwesomeIcon.Circle, "球体",     PickKind.Sphere,           "先选择中心, 再选择表面点生成球形体积")
    ];

    private static readonly ToolDefinition[] SurfaceTools =
    [
        new(FontAwesomeIcon.DrawPolygon, "地面", PickKind.WalkableFloor, "在画面点两个世界点生成可行走地面"),
        new(FontAwesomeIcon.DrawPolygon, "墙体", PickKind.Wall,          "沿两个世界点的水平连线生成双面墙"),
        new(FontAwesomeIcon.Route,       "斜坡", PickKind.Ramp,          "以较低点和较高点生成可行走斜坡")
    ];

    private static readonly ToolDefinition[] RegionTools =
    [
        new(FontAwesomeIcon.Cube, "移除实例", PickKind.RemoveInstancesVolume, "框选世界范围并批量移除相交的场景实例"),
        new(FontAwesomeIcon.Cube, "标记实例", PickKind.SetInstanceFlagsVolume, "框选世界范围并批量覆盖相交实例的碰撞标记")
    ];

    private static readonly ToolDefinition[] ConnectionTools =
    [
        new(FontAwesomeIcon.Link,  "直连",       PickKind.LinkPoints,     "在画面点两个世界点生成网格连接点"),
        new(FontAwesomeIcon.Route, "捷径",       PickKind.LinkShortcut,   "在画面点两个世界点生成普通移动捷径"),
        new(FontAwesomeIcon.Route, "客户端路径", PickKind.LinkClientPath, "在画面点两个世界点生成客户端路径连接"),
        new(FontAwesomeIcon.Link,  "离网",       PickKind.OffMesh,        "在画面点两个世界点生成构建期离网连接")
    ];

    private static readonly (string Label, ToolDefinition[] Tools)[] ToolSections =
    [
        ("选择", SelectionTools),
        ("体积", VolumeTools),
        ("表面", SurfaceTools),
        ("区域", RegionTools),
        ("连接", ConnectionTools)
    ];

    private static readonly FontAwesomeIcon[] CommandIcons =
    [
        FontAwesomeIcon.Undo,
        FontAwesomeIcon.Redo,
        FontAwesomeIcon.SyncAlt,
        FontAwesomeIcon.Save,
        FontAwesomeIcon.FileExport,
        FontAwesomeIcon.FolderOpen,
        FontAwesomeIcon.Times
    ];

    public static void Draw
    (
        ref PickKind pickKind,
        ref Vector3? pendingPickPoint,
        ref Vector3? currentPickPoint,
        ref bool     lastPickMouseDown,
        ref bool     lastWorldSelectMouseDown,
        ref bool     lastPickEscapeDown,
        ref string   statusText,
        bool         workspaceLoaded,
        bool         hasWorkspace,
        int          undoCount,
        int          redoCount,
        VoidDelegate onUndo,
        VoidDelegate onRedo,
        VoidDelegate onRebuildPreview,
        VoidDelegate onSaveWorkspace,
        VoidDelegate onExportDraft,
        VoidDelegate onOpenExportedDirectory
    )
    {
        HandleKeyboardShortcuts
        (
            ref pickKind,
            ref pendingPickPoint,
            ref currentPickPoint,
            ref lastPickMouseDown,
            ref lastWorldSelectMouseDown,
            ref lastPickEscapeDown,
            ref statusText
        );

        var availableWidth = ImGui.GetContentRegionAvail().X;
        var commandBarWidth = CalculateCommandBarWidth();
        var toolbarSpacing = ImGui.GetStyle().ItemSpacing.X;
        var useCompactPicker = availableWidth < CalculateExpandedToolAreaWidth() + commandBarWidth + toolbarSpacing;
        var stackCommands = availableWidth < CalculateCompactToolAreaWidth() + commandBarWidth + toolbarSpacing;

        using (ImRaii.Group())
        {
            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
            {
                using var group = ImRaii.Group();

                DrawGroupLabel("工具");
                if (useCompactPicker)
                {
                    DrawCompactToolPicker
                    (
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );
                }
                else
                {
                    DrawModeButton
                    (
                        SelectionTools[0].Icon,
                        SelectionTools[0].Label,
                        SelectionTools[0].Kind,
                        SelectionTools[0].Tooltip,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );

                    ImGui.SameLine();
                    DrawModeButton
                    (
                        SelectionTools[1].Icon,
                        SelectionTools[1].Label,
                        SelectionTools[1].Kind,
                        SelectionTools[1].Tooltip,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );

                    ImGui.SameLine();
                    DrawModeButton
                    (
                        SelectionTools[2].Icon,
                        SelectionTools[2].Label,
                        SelectionTools[2].Kind,
                        SelectionTools[2].Tooltip,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );

                    DrawToolGroup
                    (
                        FontAwesomeIcon.Cube,
                        "体积",
                        "volume",
                        VolumeTools,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );
                    DrawToolGroup
                    (
                        FontAwesomeIcon.DrawPolygon,
                        "表面",
                        "surface",
                        SurfaceTools,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );
                    DrawToolGroup
                    (
                        FontAwesomeIcon.Cube,
                        "区域",
                        "region",
                        RegionTools,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );
                    DrawToolGroup
                    (
                        FontAwesomeIcon.Link,
                        "连接",
                        "connection",
                        ConnectionTools,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );
                }
            }

            if (!stackCommands)
                ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace || undoCount == 0))
            {
                if (DrawCommandButton(FontAwesomeIcon.Undo, "undo", $"撤销上一步 ({undoCount})"))
                    onUndo();
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace || redoCount == 0))
            {
                if (DrawCommandButton(FontAwesomeIcon.Redo, "redo", $"重做下一步 ({redoCount})"))
                    onRedo();
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
            {
                if (DrawCommandButton(FontAwesomeIcon.SyncAlt, "rebuild", "重建导航预览"))
                    onRebuildPreview();
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
            {
                if (DrawCommandButton(FontAwesomeIcon.Save, "save", "保存当前工作区"))
                    onSaveWorkspace();
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
            {
                if (DrawCommandButton(FontAwesomeIcon.FileExport, "export", "导出 C# 自定义代码"))
                    onExportDraft();
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
            {
                if (DrawCommandButton(FontAwesomeIcon.FolderOpen, "open_export", "打开导出目录"))
                    onOpenExportedDirectory();
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace || pickKind == PickKind.None))
            {
                if (DrawCommandButton(FontAwesomeIcon.Times, "cancel_tool", "退出当前工具"))
                {
                    CancelPick
                    (
                        PickKind.None,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText,
                        "已退出当前工具"
                    );
                }
            }

        }
    }

    private static void DrawCompactToolPicker
    (
        ref PickKind pickKind,
        ref Vector3? pendingPickPoint,
        ref Vector3? currentPickPoint,
        ref bool     lastPickMouseDown,
        ref bool     lastWorldSelectMouseDown,
        ref bool     lastPickEscapeDown,
        ref string   statusText
    )
    {
        var activeTool = FindActiveTool(pickKind) ?? SelectionTools[0];
        var buttonText = $"{activeTool.Icon.ToIconString()}  {activeTool.Label}  {FontAwesomeIcon.CaretDown.ToIconString()}";

        ImGui.PushStyleColor(ImGuiCol.Button,        new Vector4(0.22f, 0.45f, 0.75f, 1f));
        ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new Vector4(0.28f, 0.55f, 0.9f,  1f));
        ImGui.PushStyleColor(ImGuiCol.ButtonActive,  new Vector4(0.18f, 0.38f, 0.66f, 1f));

        if (ImGui.Button($"{buttonText}##compact_tool_picker", new Vector2(CalculateCompactToolPickerWidth(), 0)))
            ImGui.OpenPopup("##compact_tool_picker_popup");

        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(activeTool.Tooltip);

        ImGui.PopStyleColor(3);

        if (!ImGui.BeginPopup("##compact_tool_picker_popup"))
            return;

        if (ImGui.BeginTable("##compact_tool_picker_sections", 2, ImGuiTableFlags.SizingFixedFit | ImGuiTableFlags.BordersInnerV))
        {
            ImGui.TableSetupColumn("left",  ImGuiTableColumnFlags.WidthFixed);
            ImGui.TableSetupColumn("right", ImGuiTableColumnFlags.WidthFixed);
            ImGui.TableNextRow();

            for (var column = 0; column < 2; column++)
            {
                ImGui.TableNextColumn();
                var firstSection = column == 0 ? 0 : 2;
                var lastSection = column == 0 ? 2 : ToolSections.Length;

                for (var i = firstSection; i < lastSection; i++)
                {
                    if (i > firstSection)
                        ImGui.Spacing();

                    ImGui.TextDisabled(ToolSections[i].Label);
                    ImGui.Separator();
                    DrawToolOptions
                    (
                        ToolSections[i].Tools,
                        ref pickKind,
                        ref pendingPickPoint,
                        ref currentPickPoint,
                        ref lastPickMouseDown,
                        ref lastWorldSelectMouseDown,
                        ref lastPickEscapeDown,
                        ref statusText
                    );
                }
            }

            ImGui.EndTable();
        }

        ImGui.EndPopup();
    }

    private static void DrawToolGroup
    (
        FontAwesomeIcon  icon,
        string           label,
        string           id,
        ToolDefinition[] tools,
        ref PickKind     pickKind,
        ref Vector3?     pendingPickPoint,
        ref Vector3?     currentPickPoint,
        ref bool         lastPickMouseDown,
        ref bool         lastWorldSelectMouseDown,
        ref bool         lastPickEscapeDown,
        ref string       statusText
    )
    {
        ImGui.SameLine();

        var activeTool = FindTool(tools, pickKind);
        var buttonIcon = activeTool?.Icon ?? icon;
        var buttonText = activeTool?.Label ?? label;
        var buttonWidth = CalculateToolGroupWidth(icon, label, tools);
        if (activeTool is not null)
        {
            ImGui.PushStyleColor(ImGuiCol.Button,        new Vector4(0.22f, 0.45f, 0.75f, 1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new Vector4(0.28f, 0.55f, 0.9f,  1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonActive,  new Vector4(0.18f, 0.38f, 0.66f, 1f));
        }

        if (ImGui.Button($"{buttonIcon.ToIconString()}  {buttonText}##tool_group_{id}", new Vector2(buttonWidth, 0)))
            ImGui.OpenPopup($"##tool_group_popup_{id}");

        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(activeTool?.Tooltip ?? $"选择{label}工具");

        if (activeTool is not null)
            ImGui.PopStyleColor(3);

        if (!ImGui.BeginPopup($"##tool_group_popup_{id}"))
            return;

        ImGui.TextDisabled(label);
        ImGui.Separator();
        DrawToolOptions
        (
            tools,
            ref pickKind,
            ref pendingPickPoint,
            ref currentPickPoint,
            ref lastPickMouseDown,
            ref lastWorldSelectMouseDown,
            ref lastPickEscapeDown,
            ref statusText
        );

        ImGui.EndPopup();
    }

    private static void DrawToolOptions
    (
        ToolDefinition[] tools,
        ref PickKind     pickKind,
        ref Vector3?     pendingPickPoint,
        ref Vector3?     currentPickPoint,
        ref bool         lastPickMouseDown,
        ref bool         lastWorldSelectMouseDown,
        ref bool         lastPickEscapeDown,
        ref string       statusText
    )
    {
        foreach (var tool in tools)
        {
            var selected = ImGui.Selectable($"{tool.Icon.ToIconString()}  {tool.Label}", pickKind == tool.Kind);

            if (ImGui.IsItemHovered())
                ImGui.SetTooltip(tool.Tooltip);

            if (!selected)
                continue;

            if (tool.Kind == PickKind.None)
            {
                CancelPick
                (
                    PickKind.None,
                    ref pickKind,
                    ref pendingPickPoint,
                    ref currentPickPoint,
                    ref lastPickMouseDown,
                    ref lastWorldSelectMouseDown,
                    ref lastPickEscapeDown,
                    ref statusText,
                    "已切换为浏览模式"
                );
            }
            else
            {
                BeginPick
                (
                    tool.Kind,
                    ref pickKind,
                    ref pendingPickPoint,
                    ref currentPickPoint,
                    ref lastPickMouseDown,
                    ref lastWorldSelectMouseDown,
                    ref lastPickEscapeDown,
                    ref statusText
                );
            }

            ImGui.CloseCurrentPopup();
        }
    }

    private static ToolDefinition? FindActiveTool
    (
        PickKind pickKind
    )
    {
        foreach (var section in ToolSections)
            if (FindTool(section.Tools, pickKind) is { } tool)
                return tool;

        return null;
    }

    private static ToolDefinition? FindTool
    (
        ToolDefinition[] tools,
        PickKind         pickKind
    )
    {
        foreach (var tool in tools)
            if (tool.Kind == pickKind)
                return tool;

        return null;
    }

    private static float CalculateToolGroupWidth
    (
        FontAwesomeIcon  icon,
        string           label,
        ToolDefinition[] tools
    )
    {
        var width = ImGui.CalcTextSize($"{icon.ToIconString()}  {label}").X;

        foreach (var tool in tools)
            width = Math.Max(width, ImGui.CalcTextSize($"{tool.Icon.ToIconString()}  {tool.Label}").X);

        return width + ImGui.GetStyle().FramePadding.X * 2;
    }

    private static float CalculateExpandedToolAreaWidth()
    {
        var style = ImGui.GetStyle();
        var width = ImGui.CalcTextSize("工具").X;

        foreach (var tool in SelectionTools)
            width += ImGui.CalcTextSize($"{tool.Icon.ToIconString()}  {tool.Label}").X + style.FramePadding.X * 2;

        width += CalculateToolGroupWidth(FontAwesomeIcon.Cube,        "体积", VolumeTools);
        width += CalculateToolGroupWidth(FontAwesomeIcon.DrawPolygon, "表面", SurfaceTools);
        width += CalculateToolGroupWidth(FontAwesomeIcon.Cube,        "区域", RegionTools);
        width += CalculateToolGroupWidth(FontAwesomeIcon.Link,        "连接", ConnectionTools);

        return width + style.ItemSpacing.X * 7;
    }

    private static float CalculateCompactToolAreaWidth() =>
        ImGui.CalcTextSize("工具").X + ImGui.GetStyle().ItemSpacing.X + CalculateCompactToolPickerWidth();

    private static float CalculateCompactToolPickerWidth()
    {
        var width = 0f;
        var caret = FontAwesomeIcon.CaretDown.ToIconString();

        foreach (var section in ToolSections)
            foreach (var tool in section.Tools)
                width = Math.Max(width, ImGui.CalcTextSize($"{tool.Icon.ToIconString()}  {tool.Label}  {caret}").X);

        return width + ImGui.GetStyle().FramePadding.X * 2;
    }

    private static float CalculateCommandBarWidth()
    {
        var frameHeight = ImGui.GetFrameHeight();
        var width = 0f;

        foreach (var icon in CommandIcons)
            width += frameHeight + ImGui.CalcTextSize(icon.ToIconString()).X;

        return width + ImGui.GetStyle().ItemSpacing.X * (CommandIcons.Length - 1);
    }

    private static void DrawGroupLabel
    (
        string label
    )
    {
        ImGui.AlignTextToFramePadding();
        ImGui.TextDisabled(label);
        ImGui.SameLine();
    }

    internal static void HandleKeyboardShortcuts
    (
        ref PickKind pickKind,
        ref Vector3? pendingPickPoint,
        ref Vector3? currentPickPoint,
        ref bool     lastPickMouseDown,
        ref bool     lastWorldSelectMouseDown,
        ref bool     lastPickEscapeDown,
        ref string   statusText
    )
    {
        if (pickKind != PickKind.None && CustomizationEditorWorldOverlay.TakeKeyPress(VK_ESCAPE, ref lastPickEscapeDown))
        {
            CancelPick
            (
                PickKind.None,
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText,
                "已退出当前工具模式"
            );
        }
    }

    private static void DrawModeButton
    (
        FontAwesomeIcon icon,
        string       label,
        PickKind     kind,
        string       tooltip,
        ref PickKind pickKind,
        ref Vector3? pendingPickPoint,
        ref Vector3? currentPickPoint,
        ref bool     lastPickMouseDown,
        ref bool     lastWorldSelectMouseDown,
        ref bool     lastPickEscapeDown,
        ref string   statusText
    )
    {
        var active = pickKind == kind;
        var buttonLabel = $"{icon.ToIconString()}  {label}";

        if (active)
        {
            ImGui.PushStyleColor(ImGuiCol.Button,        new Vector4(0.22f, 0.45f, 0.75f, 1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new Vector4(0.28f, 0.55f, 0.9f,  1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonActive,  new Vector4(0.18f, 0.38f, 0.66f, 1f));
        }

        if (ImGui.Button(buttonLabel))
        {
            if (kind == PickKind.None)
            {
                CancelPick
                (
                    PickKind.None,
                    ref pickKind,
                    ref pendingPickPoint,
                    ref currentPickPoint,
                    ref lastPickMouseDown,
                    ref lastWorldSelectMouseDown,
                    ref lastPickEscapeDown,
                    ref statusText,
                    "已切换为浏览模式"
                );
            }
            else
            {
                BeginPick
                (
                    kind,
                    ref pickKind,
                    ref pendingPickPoint,
                    ref currentPickPoint,
                    ref lastPickMouseDown,
                    ref lastWorldSelectMouseDown,
                    ref lastPickEscapeDown,
                    ref statusText
                );
            }
        }

        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(tooltip);

        if (active)
            ImGui.PopStyleColor(3);
    }

    private static bool DrawCommandButton
    (
        FontAwesomeIcon icon,
        string          id,
        string          tooltip
    )
    {
        var size     = ImGui.GetFrameHeight();
        var iconSize = ImGui.CalcTextSize(icon.ToIconString());
        var clicked  = ImGui.Button($"{icon.ToIconString()}##{id}", new Vector2(size + iconSize.X, size));

        if (ImGui.IsItemHovered(ImGuiHoveredFlags.AllowWhenDisabled))
            ImGui.SetTooltip(tooltip);

        return clicked;
    }

    private static string GetActiveToolStatus
    (
        PickKind kind,
        bool     hasFirstPoint
    ) =>
        kind switch
        {
            PickKind.SelectCollider => "选择碰撞体",
            PickKind.SelectTriangle => "选择三角形",
            _ when hasFirstPoint    => $"{CustomizationEditorWorldOverlay.GetPickKindTitle(kind)} · 选择终点",
            _                       => $"{CustomizationEditorWorldOverlay.GetPickKindTitle(kind)} · 选择起点"
        };

    internal static void CancelPick
    (
        PickKind     newKind,
        ref PickKind pickKind,
        ref Vector3? pendingPickPoint,
        ref Vector3? currentPickPoint,
        ref bool     lastPickMouseDown,
        ref bool     lastWorldSelectMouseDown,
        ref bool     lastPickEscapeDown,
        ref string   statusText,
        string       status
    )
    {
        pickKind                 = newKind;
        pendingPickPoint         = null;
        currentPickPoint         = null;
        lastPickMouseDown        = IsKeyDown(VK_LBUTTON);
        lastWorldSelectMouseDown = lastPickMouseDown;
        lastPickEscapeDown       = IsKeyDown(VK_ESCAPE);
        statusText               = status;
    }

    internal static void BeginPick
    (
        PickKind     kind,
        ref PickKind pickKind,
        ref Vector3? pendingPickPoint,
        ref Vector3? currentPickPoint,
        ref bool     lastPickMouseDown,
        ref bool     lastWorldSelectMouseDown,
        ref bool     lastPickEscapeDown,
        ref string   statusText
    )
    {
        pickKind                 = kind;
        pendingPickPoint         = null;
        currentPickPoint         = null;
        lastPickMouseDown        = IsKeyDown(VK_LBUTTON);
        lastWorldSelectMouseDown = lastPickMouseDown;
        lastPickEscapeDown       = IsKeyDown(VK_ESCAPE);
        statusText = kind switch
        {
            PickKind.SelectCollider => "点击游戏画面中的碰撞体以选中",
            PickKind.SelectTriangle => "点击游戏画面中的三角形以选中",
            PickKind.LinkShortcut   => "点击游戏画面中的两个世界点以生成普通移动捷径",
            _                       => $"{CustomizationEditorWorldOverlay.GetPickKindTitle(kind)}: 在画面点击第 1 个世界点"
        };
    }

    private static bool IsKeyDown
    (
        int virtualKey
    ) =>
        (GetAsyncKeyState(virtualKey) & 0x8000) != 0;

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern short GetAsyncKeyState
    (
        int virtualKey
    );

    private readonly record struct ToolDefinition
    (
        FontAwesomeIcon Icon,
        string          Label,
        PickKind        Kind,
        string          Tooltip
    );
}
