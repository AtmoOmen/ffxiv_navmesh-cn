using System.Numerics;
using vnavmesh.Common.Models;

namespace vnavmesh.Build.Custom.Editor;

public struct DraftMatrix4x3
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

    public static DraftMatrix4x3 Identity => new(new(1, 0, 0), new(0, 1, 0), new(0, 0, 1), default);

    public static DraftMatrix4x3 FromRuntime
    (
        Matrix4x3 matrix
    ) =>
        new(matrix.Row0, matrix.Row1, matrix.Row2, matrix.Row3);

    public Matrix4x3 ToRuntime() => new(Row0, Row1, Row2, Row3);

    public Vector3 Translation
    {
        readonly get => Row3;
        set => Row3 = value;
    }

    public Vector3 GetScale() =>
        new(Row0.Length(), Row1.Length(), Row2.Length());

    public void SetTranslationScale
    (
        Vector3 translation,
        Vector3 scale
    )
    {
        Row3 = translation;
        Row0 = RescaleAxis(Row0, scale.X, new(1, 0, 0));
        Row1 = RescaleAxis(Row1, scale.Y, new(0, 1, 0));
        Row2 = RescaleAxis(Row2, scale.Z, new(0, 0, 1));
    }

    private static Vector3 RescaleAxis
    (
        Vector3 axis,
        float   length,
        Vector3 fallback
    )
    {
        if (length <= 0f)
            return default;

        var currentLength = axis.Length();
        if (currentLength <= 0.0001f)
            return fallback * length;

        return axis / currentLength * length;
    }
}
