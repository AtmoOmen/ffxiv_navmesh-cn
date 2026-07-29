using System.Numerics;

namespace vnavmesh.Common.Models;

public struct AABB
(
    Vector3 min,
    Vector3 max
)
{
    public Vector3 Min = min;
    public Vector3 Max = max;

    public static AABB Empty => new(new(float.MaxValue), new(float.MinValue));

    public static AABB Read
    (
        BinaryReader reader
    ) =>
        new(new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()), new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()));

    public void Write
    (
        BinaryWriter writer
    )
    {
        writer.Write(Min.X);
        writer.Write(Min.Y);
        writer.Write(Min.Z);
        writer.Write(Max.X);
        writer.Write(Max.Y);
        writer.Write(Max.Z);
    }
}
