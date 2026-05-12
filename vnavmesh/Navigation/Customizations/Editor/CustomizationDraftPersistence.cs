using System.Text.Json;
using System.Text;
using vnavmesh.Bootstrap;

namespace vnavmesh.Navigation.Customizations.Editor;

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

    public CustomizationEditorWorkspace Load(uint territoryId)
    {
        var path = GetPath(territoryId);
        if (!path.Exists)
        {
            var empty = new CustomizationEditorWorkspace();
            empty.Draft.TerritoryID = territoryId;
            return empty;
        }

        try
        {
            var json = File.ReadAllText(path.FullName, Encoding.UTF8);
            var workspace = JsonSerializer.Deserialize<CustomizationEditorWorkspace>(json, JsonOptions) ?? new CustomizationEditorWorkspace();
            workspace.Draft.TerritoryID = territoryId;
            return workspace;
        }
        catch (Exception ex)
        {
            Service.Log.Error($"读取自定义编辑器草稿失败: {ex}");
            var empty = new CustomizationEditorWorkspace();
            empty.Draft.TerritoryID = territoryId;
            return empty;
        }
    }

    public void Save(CustomizationEditorWorkspace workspace)
    {
        try
        {
            rootDirectory.Create();
            var path = GetPath(workspace.Draft.TerritoryID);
            var json = JsonSerializer.Serialize(workspace, JsonOptions);
            File.WriteAllText(path.FullName, json, new UTF8Encoding(false));
        }
        catch (Exception ex)
        {
            Service.Log.Error($"保存自定义编辑器草稿失败: {ex}");
        }
    }

    public FileInfo GetPath(uint territoryId) =>
        new(Path.Combine(rootDirectory.FullName, $"{territoryId:D4}.draft.json"));
}
