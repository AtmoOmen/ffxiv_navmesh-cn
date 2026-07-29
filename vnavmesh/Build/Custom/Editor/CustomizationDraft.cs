namespace vnavmesh.Build.Custom.Editor;

public sealed class CustomizationDraft
{
    public int    SchemaVersion = 1;
    public uint   TerritoryID;
    public string TerritoryName = "";

    public bool? FlyingSupportedOverride;

    public DraftBuildProfileOverrides  BuildProfile  = new();
    public DraftBuildSettingsOverrides BuildSettings = new();

    public List<DraftSceneMeshRemoval>       MeshRemovals       = [];
    public List<DraftSceneInstancePatch>     InstancePatches    = [];
    public List<DraftScenePartPatch>         PartPatches        = [];
    public List<DraftSceneColliderInsertion> ColliderInsertions = [];
    public List<DraftMeshLinkPatch>          MeshLinks          = [];
    public List<DraftOffMeshConnectionPatch> OffMeshConnections = [];

    public string DisplayName =>
        string.IsNullOrWhiteSpace(TerritoryName) ?
            TerritoryID.ToString() :
            TerritoryName;

    public CustomizationDraft Clone() =>
        CustomizationDraftJson.Clone(this);

    public int ComputeVersion() =>
        CustomizationDraftJson.ComputeVersion(this);

    public string ComputeContentHash() =>
        CustomizationDraftJson.ComputeContentHash(this);
}
