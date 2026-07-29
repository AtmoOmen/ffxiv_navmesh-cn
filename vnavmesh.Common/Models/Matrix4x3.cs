using System.Numerics;

namespace vnavmesh.Common.Models;

public struct Matrix4x3
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

    public static Matrix4x3 Identity => new(new(1, 0, 0), new(0, 1, 0), new(0, 0, 1), default);

    public Vector3 TransformCoordinate
    (
        Vector3 value
    ) =>
        (Row0 * value.X) + (Row1 * value.Y) + (Row2 * value.Z) + Row3;

    public static Matrix4x3 Read
    (
        BinaryReader reader
    ) =>
        new
        (
            new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()),
            new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()),
            new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()),
            new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle())
        );

    public void Write
    (
        BinaryWriter writer
    )
    {
        WriteVector(writer, Row0);
        WriteVector(writer, Row1);
        WriteVector(writer, Row2);
        WriteVector(writer, Row3);
    }

    private static void WriteVector
    (
        BinaryWriter writer,
        Vector3      value
    )
    {
        writer.Write(value.X);
        writer.Write(value.Y);
        writer.Write(value.Z);
    }
}
