using System.Buffers.Binary;
using System.Numerics;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;

namespace vnavmesh.Navigation.Customizations.Editor;

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
        string.IsNullOrWhiteSpace(TerritoryName) ? TerritoryID.ToString() : TerritoryName;

    public CustomizationDraft Clone() =>
        CustomizationDraftJson.Clone(this);

    public int ComputeVersion() =>
        CustomizationDraftJson.ComputeVersion(this);

    public string ComputeContentHash() =>
        CustomizationDraftJson.ComputeContentHash(this);
}

public sealed class CustomizationEditorWorkspace
{
    public CustomizationDraft          Draft    = new();
    public CustomizationEditorSettings Settings = new();
}

public sealed class CustomizationEditorSettings
{
    public string ExportDirectory               = "";
    public bool   IncludeExistingCustomizations = true;
    public bool   AutoRebuild                   = true;
    public bool   AutoSave                      = true;
    public float  RebuildDelaySeconds           = 0.4f;
}

public struct DraftMatrix4x3
(
    Vector3 row0,
    Vector3 row1,
    Vector3 row2,
    Vector3 row3
)
{
    public Vector3 Row0 = row0;
    public Vector3 Row1 = row1;
    public Vector3 Row2 = row2;
    public Vector3 Row3 = row3;

    public static DraftMatrix4x3 Identity => new(new(1, 0, 0), new(0, 1, 0), new(0, 0, 1), default);

    public static DraftMatrix4x3 FromRuntime(Matrix4x3 matrix) =>
        new(matrix.Row0, matrix.Row1, matrix.Row2, matrix.Row3);

    public Matrix4x3 ToRuntime() =>
        new
        (
            new
            (
                Row0.X,
                Row0.Y,
                Row0.Z,
                0,
                Row1.X,
                Row1.Y,
                Row1.Z,
                0,
                Row2.X,
                Row2.Y,
                Row2.Z,
                0,
                Row3.X,
                Row3.Y,
                Row3.Z,
                1
            )
        );

    public Vector3 Translation
    {
        readonly get => Row3;
        set => Row3 = value;
    }

    public Vector3 GetScale() =>
        new(Row0.Length(), Row1.Length(), Row2.Length());

    public void SetTranslationScale(Vector3 translation, Vector3 scale)
    {
        Row3 = translation;
        Row0 = RescaleAxis(Row0, scale.X, new(1, 0, 0));
        Row1 = RescaleAxis(Row1, scale.Y, new(0, 1, 0));
        Row2 = RescaleAxis(Row2, scale.Z, new(0, 0, 1));
    }

    private static Vector3 RescaleAxis(Vector3 axis, float length, Vector3 fallback)
    {
        if (length <= 0f)
            return default;

        var currentLength = axis.Length();
        if (currentLength <= 0.0001f)
            return fallback * length;

        return axis / currentLength * length;
    }
}

internal static class CustomizationDraftJson
{
    private static readonly JsonSerializerOptions Options = new()
    {
        IncludeFields = true,
        WriteIndented = false
    };

    public static CustomizationDraft Clone(CustomizationDraft draft)
    {
        var json = JsonSerializer.Serialize(draft, Options);
        return JsonSerializer.Deserialize<CustomizationDraft>(json, Options) ?? new();
    }

    public static string ComputeContentHash(CustomizationDraft draft)
    {
        var json  = JsonSerializer.Serialize(draft, Options);
        var bytes = Encoding.UTF8.GetBytes(json);
        var hash  = SHA256.HashData(bytes);
        return Convert.ToHexString(hash);
    }

    public static int ComputeVersion(CustomizationDraft draft)
    {
        var json  = JsonSerializer.Serialize(draft, Options);
        var bytes = Encoding.UTF8.GetBytes(json);
        var hash  = SHA256.HashData(bytes);
        var value = BinaryPrimitives.ReadUInt32LittleEndian(hash.AsSpan(0, 4)) & 0x7FFF_FFFF;
        return value == 0 ? 1 : (int)value;
    }
}
