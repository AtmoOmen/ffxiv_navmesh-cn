using System.Text;
using System.Text.Json;

namespace vnavmesh.Navigation.Custom.Editor;

internal sealed class CustomizationDraftPersistence
(
    DirectoryInfo rootDirectory
)
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        IncludeFields = true,
        WriteIndented = true
    };

    public CustomizationEditorTerritoryStore Load(uint territoryId)
    {
        var storePath = GetStorePath(territoryId);
        if (storePath.Exists)
        {
            try
            {
                var json = File.ReadAllText(storePath.FullName, Encoding.UTF8);
                var store = JsonSerializer.Deserialize<CustomizationEditorTerritoryStore>(json, JsonOptions);
                return EnsureValid(store, territoryId);
            }
            catch (Exception ex)
            {
                Service.Log.Error($"读取自定义编辑器工作区失败: {ex}");
            }
        }

        var legacy = TryLoadLegacyDraft(territoryId);
        return legacy ?? CreateEmptyStore(territoryId);
    }

    public void Save(CustomizationEditorTerritoryStore store)
    {
        try
        {
            rootDirectory.Create();
            var path = GetStorePath(store.TerritoryID);
            var json = JsonSerializer.Serialize(store, JsonOptions);
            File.WriteAllText(path.FullName, json, new UTF8Encoding(false));
        }
        catch (Exception ex)
        {
            Service.Log.Error($"保存自定义编辑器工作区失败: {ex}");
        }
    }

    public FileInfo GetStorePath(uint territoryId) =>
        new(Path.Combine(rootDirectory.FullName, $"{territoryId:D4}.workspace.json"));

    private CustomizationEditorTerritoryStore? TryLoadLegacyDraft(uint territoryId)
    {
        var legacyPath = GetLegacyPath(territoryId);
        if (!legacyPath.Exists)
            return null;

        try
        {
            var json = File.ReadAllText(legacyPath.FullName, Encoding.UTF8);
            var workspace = JsonSerializer.Deserialize<CustomizationEditorWorkspace>(json, JsonOptions);
            if (workspace == null)
                return null;

            workspace.WorkspaceId   = string.IsNullOrWhiteSpace(workspace.WorkspaceId) ? CreateWorkspaceId() : workspace.WorkspaceId;
            workspace.WorkspaceName = string.IsNullOrWhiteSpace(workspace.WorkspaceName) ? "默认工作区" : workspace.WorkspaceName;
            workspace.IsApplied     = true;
            workspace.Draft.TerritoryID = territoryId;

            var store = CreateEmptyStore(territoryId);
            store.SchemaVersion = 0;
            store.Workspaces.Add(workspace);
            store.CurrentWorkspaceId = workspace.WorkspaceId;
            return store;
        }
        catch (Exception ex)
        {
            Service.Log.Error($"迁移旧自定义草稿失败: {ex}");
            return null;
        }
    }

    private static CustomizationEditorTerritoryStore EnsureValid(CustomizationEditorTerritoryStore? store, uint territoryId)
    {
        store ??= new();
        store.TerritoryID = territoryId;
        if (string.IsNullOrWhiteSpace(store.TerritoryKey))
            store.TerritoryKey = territoryId.ToString();

        if (string.IsNullOrWhiteSpace(store.TerritoryName))
            store.TerritoryName = territoryId.ToString();

        if (store.Workspaces.Count == 0)
        {
            store.CurrentWorkspaceId = "";
            return store;
        }

        foreach (var workspace in store.Workspaces)
            EnsureWorkspaceValid(workspace, territoryId);

        if (string.IsNullOrWhiteSpace(store.CurrentWorkspaceId) || store.Workspaces.All(x => x.WorkspaceId != store.CurrentWorkspaceId))
            store.CurrentWorkspaceId = store.Workspaces[0].WorkspaceId;

        return store;
    }

    private static void EnsureWorkspaceValid(CustomizationEditorWorkspace workspace, uint territoryId)
    {
        if (string.IsNullOrWhiteSpace(workspace.WorkspaceId))
            workspace.WorkspaceId = CreateWorkspaceId();

        if (string.IsNullOrWhiteSpace(workspace.WorkspaceName))
            workspace.WorkspaceName = "默认工作区";

        workspace.Draft.TerritoryID = territoryId;
        workspace.Draft.TerritoryName = string.IsNullOrWhiteSpace(workspace.Draft.TerritoryName) ? territoryId.ToString() : workspace.Draft.TerritoryName;
        workspace.Draft.MeshLinks.RemoveAll(static x => !Enum.IsDefined(x.Kind));
    }

    private static CustomizationEditorTerritoryStore CreateEmptyStore(uint territoryId)
    {
        var store = new CustomizationEditorTerritoryStore
        {
            TerritoryID = territoryId,
            TerritoryKey = territoryId.ToString(),
            TerritoryName = territoryId.ToString()
        };
        return store;
    }

    private FileInfo GetLegacyPath(uint territoryId) =>
        new(Path.Combine(rootDirectory.FullName, $"{territoryId:D4}.draft.json"));

    private static string CreateWorkspaceId() =>
        Convert.ToHexString(Guid.NewGuid().ToByteArray());
}
