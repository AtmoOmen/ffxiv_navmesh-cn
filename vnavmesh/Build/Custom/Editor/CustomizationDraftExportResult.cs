namespace vnavmesh.Build.Custom.Editor;

internal sealed record CustomizationDraftExportResult
(
    FileInfo File,
    string   ClassName,
    int      Version,
    string   ContentHash
);
