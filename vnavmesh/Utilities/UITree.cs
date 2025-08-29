using System;
using ImGuiNET;

namespace Navmesh.Utilities;

public class UITree
{
    private uint selectedID;

    public struct NodeRaii(bool selected, bool opened, bool hovered, bool realOpened) : IDisposable
    {
        public bool RealOpened { get; init; } = realOpened;
        public bool Selected   { get; init; } = selected;
        public bool Opened     { get; init; } = opened;
        public bool Hovered    { get; init; } = hovered;
        
        private bool disposed;

        public bool SelectedOrHovered => Selected || Hovered;

        public void Dispose()
        {
            if (disposed)
                return;
            if (RealOpened)
                ImGui.TreePop();
            ImGui.PopID();
            disposed = true;
        }
    }

    public NodeRaii Node(string text, bool leaf = false, uint color = 0xffffffff)
    {
        var id = ImGui.GetID(text);
        var flags = ImGuiTreeNodeFlags.None;
        if (id == selectedID)
            flags |= ImGuiTreeNodeFlags.Selected;
        if (leaf)
            flags |= ImGuiTreeNodeFlags.Leaf;

        ImGui.PushID((int)id);
        ImGui.PushStyleColor(ImGuiCol.Text, color);
        var open = ImGui.TreeNodeEx(text, flags);
        ImGui.PopStyleColor();
        if (ImGui.IsItemClicked(ImGuiMouseButton.Left))
            selectedID = id;
        if (ImGui.IsItemClicked(ImGuiMouseButton.Right))
            ImGui.SetClipboardText(text);
        return new(id == selectedID, open && !leaf, ImGui.IsItemHovered(), open);
    }

    // returned node is auto disposed
    public NodeRaii LeafNode(string text, uint color = 0xffffffff)
    {
        var n = Node(text, true, color);
        n.Dispose();
        return n;
    }
}
