using System.Numerics;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorSpatial
{
    public static AABB CreateBounds(Vector3 a, Vector3 b) =>
        new() { Min = Vector3.Min(a, b), Max = Vector3.Max(a, b) };

    public static AABB CalculateTransformedBounds(AABB localBounds, Matrix4x3 transform)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = transform.Row0;
        var axisY       = transform.Row1;
        var axisZ       = transform.Row2;
        var center      = axisX      * localCenter.X + axisY      * localCenter.Y + axisZ      * localCenter.Z + transform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        return new() { Min = center                  - extent, Max = center       + extent };
    }

    public static bool TryUnionBounds(IEnumerable<AABB> bounds, out AABB unionBounds)
    {
        using var enumerator = bounds.GetEnumerator();

        if (!enumerator.MoveNext())
        {
            unionBounds = default;
            return false;
        }

        unionBounds = enumerator.Current;

        while (enumerator.MoveNext())
        {
            var current = enumerator.Current;
            unionBounds.Min = Vector3.Min(unionBounds.Min, current.Min);
            unionBounds.Max = Vector3.Max(unionBounds.Max, current.Max);
        }

        return true;
    }

    private static Vector3 Abs(Vector3 value) =>
        new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));
}
