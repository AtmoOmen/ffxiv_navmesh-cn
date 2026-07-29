using System.Numerics;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Build.Custom.Editor;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorSpatial
{
    public static AABB CreateBounds
    (
        Vector3 a,
        Vector3 b
    ) =>
        new() { Min = Vector3.Min(a, b), Max = Vector3.Max(a, b) };

    public static AABB CreateColliderBounds
    (
        DraftSceneColliderInsertion insertion
    )
    {
        if (insertion.Kind == DraftSceneColliderInsertionKind.OrientedCylinder)
            return CreateOrientedCylinderBounds(insertion.Start, insertion.End, insertion.Radius);

        var bounds = CreateBounds(insertion.Min, insertion.Max);
        if (!UsesYRotation(insertion.Kind))
            return bounds;

        var center      = (bounds.Min + bounds.Max) * 0.5f;
        var halfExtents = (bounds.Max - bounds.Min) * 0.5f;
        var radians     = insertion.RotationDegrees * (MathF.PI / 180f);
        var sin         = MathF.Abs(MathF.Sin(radians));
        var cos         = MathF.Abs(MathF.Cos(radians));
        var worldExtents = new Vector3
        (
            (halfExtents.X * cos) + (halfExtents.Z * sin),
            halfExtents.Y,
            (halfExtents.X * sin) + (halfExtents.Z * cos)
        );
        return new() { Min = center - worldExtents, Max = center + worldExtents };
    }

    public static AABB CreateOrientedCylinderBounds
    (
        Vector3 start,
        Vector3 end,
        float   radius
    )
    {
        var center     = (start + end) * 0.5f;
        var halfAxis   = (end - start) * 0.5f;
        var axisLength = halfAxis.Length();
        var axis = axisLength > 0.0001f ?
                       halfAxis / axisLength :
                       Vector3.UnitY;
        radius = MathF.Max(MathF.Abs(radius), 0.005f);
        var radialExtents = new Vector3
        (
            radius * MathF.Sqrt(MathF.Max(0f, 1f - (axis.X * axis.X))),
            radius * MathF.Sqrt(MathF.Max(0f, 1f - (axis.Y * axis.Y))),
            radius * MathF.Sqrt(MathF.Max(0f, 1f - (axis.Z * axis.Z)))
        );
        var extents = Vector3.Abs(halfAxis) + radialExtents;
        return new() { Min = center - extents, Max = center + extents };
    }

    public static AABB CreateInstancePatchBounds
    (
        AABB                    localBounds,
        DraftSceneInstancePatch patch
    )
    {
        var transform = patch.WorldTransform.ToRuntime();
        var first     = CalculateTransformedBounds(localBounds, transform);
        if (patch.Kind != DraftSceneInstancePatchKind.Insert || patch.Count <= 1)
            return first;

        transform.Row3 += patch.Offset * (Math.Clamp(patch.Count, 1, 1024) - 1);
        var last = CalculateTransformedBounds(localBounds, transform);
        return new()
        {
            Min = Vector3.Min(first.Min, last.Min),
            Max = Vector3.Max(first.Max, last.Max)
        };
    }

    public static bool UsesYRotation
    (
        DraftSceneColliderInsertionKind kind
    ) =>
        kind is DraftSceneColliderInsertionKind.OrientedBox or
                DraftSceneColliderInsertionKind.Wall or
                DraftSceneColliderInsertionKind.Ramp;

    public static Vector3 RotateAroundY
    (
        Vector3 value,
        float   rotationDegrees
    )
    {
        var radians = rotationDegrees * (MathF.PI / 180f);
        var sin     = MathF.Sin(radians);
        var cos     = MathF.Cos(radians);
        return new((value.X * cos) + (value.Z * sin), value.Y, (-value.X * sin) + (value.Z * cos));
    }

    public static AABB CalculateTransformedBounds
    (
        AABB      localBounds,
        Matrix4x3 transform
    )
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = transform.Row0;
        var axisY       = transform.Row1;
        var axisZ       = transform.Row2;
        var center      = (axisX      * localCenter.X) + (axisY      * localCenter.Y) + (axisZ      * localCenter.Z) + transform.Row3;
        var extent      = (Abs(axisX) * localExtent.X) + (Abs(axisY) * localExtent.Y) + (Abs(axisZ) * localExtent.Z);
        return new() { Min = center                    - extent, Max = center         + extent };
    }

    public static bool TryUnionBounds
    (
        IEnumerable<AABB> bounds,
        out AABB          unionBounds
    )
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

    private static Vector3 Abs
    (
        Vector3 value
    ) =>
        new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));
}
