using System.Numerics;

namespace vnavmesh.Common.Models;

public struct Angle
(
    float radians = 0
) : IEquatable<Angle>
{
    public const float RAD_TO_DEG = 180      / MathF.PI;
    public const float DEG_TO_RAD = MathF.PI / 180;

    public float Rad = radians;
    public float Deg => Rad * RAD_TO_DEG;

    public static Angle FromDirection
    (
        Vector2 dir
    ) => new(MathF.Atan2(dir.X, dir.Y));

    public static Angle FromDirectionXZ
    (
        Vector3 dir
    ) => new(MathF.Atan2(dir.X, dir.Z));

    public Vector2 ToDirection() => new(Sin(), Cos());

    public Vector3 ToDirectionXZ() => new(Sin(), 0, Cos());

    public static Angle operator +
    (
        Angle a,
        Angle b
    ) => new(a.Rad + b.Rad);

    public static Angle operator -
    (
        Angle a,
        Angle b
    ) => new(a.Rad - b.Rad);

    public static Angle operator -
    (
        Angle a
    ) => new(-a.Rad);

    public static Angle operator *
    (
        Angle a,
        float b
    ) => new(a.Rad * b);

    public static Angle operator *
    (
        float a,
        Angle b
    ) => new(a * b.Rad);

    public static Angle operator /
    (
        Angle a,
        float b
    ) => new(a.Rad / b);

    public Angle Abs() => new(Math.Abs(Rad));

    public float Sin() => MathF.Sin(Rad);

    public float Cos() => MathF.Cos(Rad);

    public float Tan() => MathF.Tan(Rad);

    public static Angle Asin
    (
        float x
    ) => new(MathF.Asin(x));

    public static Angle Acos
    (
        float x
    ) => new(MathF.Acos(x));

    public Angle Normalized()
    {
        var r = Rad;
        while (r < -MathF.PI)
            r += 2 * MathF.PI;
        while (r > MathF.PI)
            r -= 2 * MathF.PI;
        return new(r);
    }

    public bool AlmostEqual
    (
        Angle other,
        float epsRad
    ) => Math.Abs((this - other).Normalized().Rad) <= epsRad;

    public static bool operator ==
    (
        Angle l,
        Angle r
    ) => l.Rad == r.Rad;

    public static bool operator !=
    (
        Angle l,
        Angle r
    ) => l.Rad != r.Rad;

    public override bool Equals
    (
        object? obj
    ) => obj is Angle angle && this == angle;

    public override int GetHashCode() => Rad.GetHashCode();

    public override string ToString() => Deg.ToString("f0");

    public bool Equals
    (
        Angle other
    ) =>
        Rad.Equals(other.Rad);
}
