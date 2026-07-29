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

        using var group = ImRaii.Group();

        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
        {
            DrawGroupLabel("选择");
            DrawModeButton
            (
                FontAwesomeIcon.MousePointer,
                "浏览",
                PickKind.None,
                "查看和选择对象, 左侧选中后右侧显示可编辑内容",
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
                FontAwesomeIcon.Cube,
                "碰撞体",
                PickKind.SelectCollider,
                "在游戏画面点选碰撞体, 选中后右侧直接编辑",
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
                FontAwesomeIcon.DrawPolygon,
                "三角形",
                PickKind.SelectTriangle,
                "在游戏画面点选三角形, 选中后右侧直接编辑",
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText
            );

            DrawGroupLabel("体积");
            DrawModeButton
            (
                FontAwesomeIcon.Cube,
                "AABB",
                PickKind.Aabb,
                "在画面点两个世界点生成轴对齐障碍体",
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
                FontAwesomeIcon.Cube,
                "旋转箱体",
                PickKind.OrientedBox,
                "沿画面中两个世界点的水平连线生成旋转箱体障碍",
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
                FontAwesomeIcon.Circle,
                "圆柱",
                PickKind.Cylinder,
                "在画面点两个世界点生成圆柱障碍体",
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
                FontAwesomeIcon.Circle,
                "定向圆柱",
                PickKind.OrientedCylinder,
                "沿画面中两个世界点生成任意方向圆柱体",
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
                FontAwesomeIcon.Circle,
                "球体",
                PickKind.Sphere,
                "先选择中心, 再选择表面点生成球形体积",
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText
            );

            DrawGroupLabel("表面");
            DrawModeButton
            (
                FontAwesomeIcon.DrawPolygon,
                "墙体",
                PickKind.Wall,
                "沿两个世界点的水平连线生成双面墙",
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
                FontAwesomeIcon.Route,
                "斜坡",
                PickKind.Ramp,
                "以较低点和较高点生成可行走斜坡",
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText
            );

            DrawGroupLabel("区域");
            DrawModeButton
            (
                FontAwesomeIcon.Cube,
                "移除实例",
                PickKind.RemoveInstancesVolume,
                "框选世界范围并批量移除相交的场景实例",
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
                FontAwesomeIcon.Cube,
                "标记实例",
                PickKind.SetInstanceFlagsVolume,
                "框选世界范围并批量覆盖相交实例的碰撞标记",
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText
            );

            DrawGroupLabel("连接");
            DrawModeButton
            (
                FontAwesomeIcon.Link,
                "直连",
                PickKind.LinkPoints,
                "在画面点两个世界点生成网格连接点",
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
                FontAwesomeIcon.Route,
                "捷径",
                PickKind.LinkShortcut,
                "在画面点两个世界点生成普通移动捷径",
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
                FontAwesomeIcon.Route,
                "客户端路径",
                PickKind.LinkClientPath,
                "在画面点两个世界点生成客户端路径连接",
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
                FontAwesomeIcon.Link,
                "离网",
                PickKind.OffMesh,
                "在画面点两个世界点生成构建期离网连接",
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText
            );
        }

        ImGui.Separator();
        ImGui.AlignTextToFramePadding();
        ImGui.TextDisabled("草稿");

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

        if (pickKind != PickKind.None)
        {
            ImGui.SameLine();

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

            ImGui.SameLine();
            ImGui.AlignTextToFramePadding();
            ImGui.TextColored(new Vector4(0.35f, 0.72f, 1f, 1f), GetActiveToolStatus(pickKind, pendingPickPoint.HasValue));
        }
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
        var size    = ImGui.GetFrameHeight();
        var clicked = ImGui.Button($"{icon.ToIconString()}##{id}", new Vector2(size, size));

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
}
