using Dalamud.Interface.Utility.Raii;
using ImGuiNET;

namespace Navmesh.Utilities;

public static class ImGuiOm
{
    public static void TooltipHover(string text, float warpPos = 20f)
    {
        if (string.IsNullOrWhiteSpace(text)) return;
        using var id = ImRaii.PushId($"TooltipHover_{text}_{warpPos}");
        
        if (ImGui.IsItemHovered())
        {
            ImGui.BeginTooltip();
            ImGui.PushTextWrapPos(ImGui.GetFontSize() * warpPos);
            ImGui.Text(text);
            ImGui.PopTextWrapPos();
            ImGui.EndTooltip();
        }
    }
}
