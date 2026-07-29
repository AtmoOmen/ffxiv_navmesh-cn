using System.Numerics;

namespace vnavmesh.Build.Ground;

public record struct JsonVec
(
    float X,
    float Y,
    float Z
)
{
    public static implicit operator Vector3
    (
        JsonVec v
    ) => new(v.X, v.Y, v.Z);

    public static implicit operator JsonVec
    (
        Vector3 v
    ) => new(v.X, v.Y, v.Z);
}
