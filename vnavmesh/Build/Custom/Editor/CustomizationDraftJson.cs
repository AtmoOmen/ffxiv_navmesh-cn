using System.Buffers.Binary;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;

namespace vnavmesh.Build.Custom.Editor;

internal static class CustomizationDraftJson
{
    private static readonly JsonSerializerOptions Options = new()
    {
        IncludeFields = true,
        WriteIndented = false
    };

    public static CustomizationDraft Clone
    (
        CustomizationDraft draft
    )
    {
        var json = JsonSerializer.Serialize(draft, Options);
        return JsonSerializer.Deserialize<CustomizationDraft>(json, Options) ?? new();
    }

    public static string ComputeContentHash
    (
        CustomizationDraft draft
    )
    {
        var json  = JsonSerializer.Serialize(draft, Options);
        var bytes = Encoding.UTF8.GetBytes(json);
        var hash  = SHA256.HashData(bytes);
        return Convert.ToHexString(hash);
    }

    public static int ComputeVersion
    (
        CustomizationDraft draft
    )
    {
        var json  = JsonSerializer.Serialize(draft, Options);
        var bytes = Encoding.UTF8.GetBytes(json);
        var hash  = SHA256.HashData(bytes);
        var value = BinaryPrimitives.ReadUInt32LittleEndian(hash.AsSpan(0, 4)) & 0x7FFF_FFFF;
        return value == 0 ?
                   1 :
                   (int)value;
    }
}
