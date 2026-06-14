using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
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
        
        ImGui.AlignTextToFramePadding();
        ImGui.TextUnformatted("工具");
        
        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
        {
            DrawModeButton
            (
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
                "选碰撞体",
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
                "选三角形",
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

            ImGui.SameLine();
            DrawModeButton
            (
                "AABB 障碍",
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
                "圆柱障碍",
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
                "网格连线",
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
                "离网连接",
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

        ImGui.AlignTextToFramePadding();
        ImGui.TextUnformatted("草稿");
        
        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace || undoCount == 0))
        {
            if (ImGui.Button("撤销"))
                onUndo();
        }

        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace || redoCount == 0))
        {
            if (ImGui.Button("重做"))
                onRedo();
        }

        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
        {
            if (ImGui.Button("重建预览"))
                onRebuildPreview();
        }

        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
        {
            if (ImGui.Button("保存草稿"))
                onSaveWorkspace();
        }

        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
        {
            if (ImGui.Button("导出 C#"))
                onExportDraft();
        }
        
        ImGui.SameLine();
        using (ImRaii.Disabled(!workspaceLoaded || !hasWorkspace))
        {
            if (ImGui.Button("打开导出文件夹"))
                onOpenExportedDirectory();
        }

        if (pickKind != PickKind.None)
        {
            ImGui.SameLine();
            if (ImGui.Button("退出模式 (Esc)"))
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

    private static void DrawModeButton
    (
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

        if (active)
        {
            ImGui.PushStyleColor(ImGuiCol.Button,        new Vector4(0.22f, 0.45f, 0.75f, 1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonHovered, new Vector4(0.28f, 0.55f, 0.9f,  1f));
            ImGui.PushStyleColor(ImGuiCol.ButtonActive,  new Vector4(0.18f, 0.38f, 0.66f, 1f));
        }

        if (ImGui.Button(label))
        {
            if (kind == PickKind.None)
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
            else
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

        if (ImGui.IsItemHovered())
            ImGui.SetTooltip(tooltip);

        if (active)
            ImGui.PopStyleColor(3);
    }

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
            PickKind.LinkShortcut => "点击游戏画面中的两个世界点以生成普通移动捷径",
            _                       => $"{CustomizationEditorWorldOverlay.GetPickKindTitle(kind)}: 在画面点击第 1 个世界点"
        };
    }

    private static bool IsKeyDown(int virtualKey) =>
        (GetAsyncKeyState(virtualKey) & 0x8000) != 0;

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern short GetAsyncKeyState(int virtualKey);
}
