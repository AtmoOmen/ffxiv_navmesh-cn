using System.Numerics;

namespace vnavmesh.Common.Numerics;

public struct Aabb
{
    public Vector3 Min;
    public Vector3 Max;

    public Aabb(Vector3 min, Vector3 max)
    {
        Min = min;
        Max = max;
    }

    public static Aabb Empty => new(new(float.MaxValue), new(float.MinValue));

    public static Aabb Read(BinaryReader reader) =>
        new(new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()), new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()));

    public void Write(BinaryWriter writer)
    {
        writer.Write(Min.X);
        writer.Write(Min.Y);
        writer.Write(Min.Z);
        writer.Write(Max.X);
        writer.Write(Max.Y);
        writer.Write(Max.Z);
    }
}
