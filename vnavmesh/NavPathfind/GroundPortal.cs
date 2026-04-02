using System.Numerics;

namespace Navmesh;

internal readonly record struct GroundPortal(long FromRef, long ToRef, Vector3 Left, Vector3 Right, bool IsHardTransition)
{
    public Vector3 Midpoint => 0.5f * (Left + Right);
    public float Width => Vector2.Distance(new(Left.X, Left.Z), new(Right.X, Right.Z));
}
