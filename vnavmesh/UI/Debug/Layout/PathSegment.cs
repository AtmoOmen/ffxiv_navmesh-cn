using System.Numerics;
using System.Runtime.InteropServices;

namespace vnavmesh.UI.Debug.Layout;

[StructLayout(LayoutKind.Explicit, Size = 0x10)]
public unsafe struct PathSegment
{
    [FieldOffset(0x0)]
    public Vector3 Position;

    [FieldOffset(0xC)]
    public ushort UnkWord;

    [FieldOffset(0xE)]
    public byte UnkByte;
}
