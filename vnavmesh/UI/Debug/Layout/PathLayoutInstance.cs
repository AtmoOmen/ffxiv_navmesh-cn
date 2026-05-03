using System.Runtime.InteropServices;

namespace vnavmesh.UI.Debug.Layout;

[StructLayout(LayoutKind.Explicit, Size = 0x70)]
internal unsafe struct PathLayoutInstance
{
    [FieldOffset(0x60)]
    public PathDefinition* Definition;
}
