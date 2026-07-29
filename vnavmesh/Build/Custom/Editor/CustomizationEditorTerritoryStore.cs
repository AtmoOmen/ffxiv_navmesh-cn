namespace vnavmesh.Build.Custom.Editor;

public sealed class CustomizationEditorTerritoryStore
{
    public int                                SchemaVersion = 1;
    public uint                               TerritoryID;
    public string                             TerritoryKey       = "";
    public string                             TerritoryName      = "";
    public string                             CurrentWorkspaceId = "";
    public List<CustomizationEditorWorkspace> Workspaces         = [];
}
