using System.Numerics;

namespace vnavmesh.Query.Flight.Utils;

public static class VoxelMathUtil
{
    public const float EPSILON = 0.00001f;

    public static bool TryNormalize
    (
        Vector2     value,
        out Vector2 normalized
    )
    {
        if (value.LengthSquared() <= EPSILON * EPSILON)
        {
            normalized = default;
            return false;
        }

        normalized = Vector2.Normalize(value);
        return true;
    }

    public static bool TryNormalize
    (
        Vector3     value,
        out Vector3 normalized
    )
    {
        if (value.LengthSquared() <= EPSILON * EPSILON)
        {
            normalized = default;
            return false;
        }

        normalized = Vector3.Normalize(value);
        return true;
    }

    public static float HorizontalDistanceXZ
    (
        Vector3 left,
        Vector3 right
    )
    {
        var dx = left.X             - right.X;
        var dz = left.Z             - right.Z;
        return MathF.Sqrt((dx * dx) + (dz * dz));
    }

    public static float WallPressure
    (
        float clearance,
        float preferredClearance
    )
    {
        if (preferredClearance <= EPSILON)
            return 0f;

        return Math.Clamp((preferredClearance - clearance) / preferredClearance, 0f, 1f);
    }

    public static int RoundUpPowerOf2
    (
        int value
    )
    {
        --value;
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        return value + 1;
    }
}
