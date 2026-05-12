namespace vnavmesh.UI.Editor;

internal sealed record Selection
(
    SelectionKind Kind,
    int           Index    = -1,
    int           SubIndex = -1,
    string?       Key      = null
);
