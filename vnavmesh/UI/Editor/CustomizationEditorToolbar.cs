using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility.Raii;

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
        int          undoCount,
        int          redoCount,
        VoidDelegate onUndo,
        VoidDelegate onRedo,
        VoidDelegate onRebuildPreview,
        VoidDelegate onSaveWorkspace,
        VoidDelegate onExportDraft
    )
    {
        HandleKeyboardShortcuts
            (ref pickKind, ref pendingPickPoint, ref currentPickPoint, ref lastPickMouseDown, ref lastWorldSelectMouseDown, ref lastPickEscapeDown, ref statusText);

        ImGui.BeginGroup();

        ImGui.TextUnformatted("工具");
        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            DrawModeButton
            (
                "浏览",
                PickKind.None,
                "只查看和选择对象; 在左侧选中后, 右侧会显示可编辑内容",
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
                "进入世界点选模式; 在游戏画面点击碰撞体后, 右侧直接编辑",
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
                "在游戏画面点两个世界点, 生成一个轴对齐障碍体",
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
                "在游戏画面点两个世界点, 生成一个圆柱障碍体",
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
                "在游戏画面点两个世界点, 生成 LinkPoints 连接",
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
                "下落连接",
                PickKind.LinkDrop,
                "在游戏画面点两个世界点, 生成 LinkDrop 连接",
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
                "在游戏画面点两个世界点, 生成 LinkClientPath 连接",
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
                "在游戏画面点两个世界点, 生成构建期 off-mesh connection",
                ref pickKind,
                ref pendingPickPoint,
                ref currentPickPoint,
                ref lastPickMouseDown,
                ref lastWorldSelectMouseDown,
                ref lastPickEscapeDown,
                ref statusText
            );
        }

        ImGui.TextUnformatted("草稿");
        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded || undoCount == 0))
        {
            if (ImGui.Button("撤销"))
                onUndo();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded || redoCount == 0))
        {
            if (ImGui.Button("重做"))
                onRedo();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            if (ImGui.Button("重建预览"))
                onRebuildPreview();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            if (ImGui.Button("保存草稿"))
                onSaveWorkspace();
        }

        ImGui.SameLine();

        using (ImRaii.Disabled(!workspaceLoaded))
        {
            if (ImGui.Button("导出 C#"))
                onExportDraft();
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
                    "已退出当前工具模式"
                );
        }

        ImGui.EndGroup();

        ImGui.TextWrapped(GetToolbarHint(pickKind, pendingPickPoint, workspaceLoaded));
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
                    "已切换到浏览模式"
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
        statusText = kind == PickKind.SelectCollider
                         ? "选中碰撞体: 在游戏画面点击一个碰撞体"
                         : $"{CustomizationEditorWorldOverlay.GetPickKindTitle(kind)}: 等待第 1 个世界点, 在游戏画面点击";
    }

    private static string GetToolbarHint(PickKind pickKind, Vector3? pendingPickPoint, bool workspaceLoaded)
    {
        if (!workspaceLoaded)
            return "进入一个游戏区域后, 编辑器会自动加载该 Territory 的草稿";

        if (pickKind == PickKind.None)
            return "浏览模式: 世界点击不会选中对象; 需要编辑现有碰撞体时, 先点击“选碰撞体”";

        if (pickKind == PickKind.SelectCollider)
            return "选中碰撞体: 鼠标指向的碰撞体会高亮, 左键选中后在右侧编辑, Esc 可退出";

        var step = pendingPickPoint == null ? "点击第 1 个世界点" : "点击第 2 个世界点完成创建";
        return $"{CustomizationEditorWorldOverlay.GetPickKindTitle(pickKind)}: {step}; 在游戏画面点击落点, 点在插件窗口或其他 UI 上不会落点, Esc 可取消";
    }

    private static bool IsKeyDown(int virtualKey) =>
        (GetAsyncKeyState(virtualKey) & 0x8000) != 0;

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern short GetAsyncKeyState(int virtualKey);
}
