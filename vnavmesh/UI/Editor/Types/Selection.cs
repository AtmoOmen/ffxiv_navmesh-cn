namespace vnavmesh.UI.Editor.Types;

internal sealed record Selection
(
    SelectionKind Kind,
    int           Index    = -1,
    int           SubIndex = -1,
    string?       Key      = null
);
