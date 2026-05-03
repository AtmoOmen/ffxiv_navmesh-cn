using System.Runtime.InteropServices;

namespace vnavmesh.UI.Debug.Layout;

[StructLayout(LayoutKind.Explicit, Size = 0x70)]
internal unsafe struct PathDefinition
{
    [FieldOffset(0x18)]
    public PathSegment* SegmentsArray;

    [FieldOffset(0x20)]
    public ushort NumSegments;

    public readonly Span<PathSegment> Segments => new(SegmentsArray, NumSegments);
}
