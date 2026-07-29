namespace vnavmesh.Build.Custom.Editor;

public sealed class CustomizationEditorWorkspace
{
    public string                      WorkspaceId   = "";
    public string                      WorkspaceName = "";
    public bool                        IsApplied     = true;
    public CustomizationDraft          Draft         = new();
    public CustomizationEditorSettings Settings      = new();
}
