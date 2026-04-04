using System.Numerics;

namespace vnavmesh.Movement.Planning;

internal abstract class MovementSegment
{
    public abstract MovementSegmentKind Kind                 { get; }
    public abstract bool                AllowVerticalControl { get; }
    public          float               CompletionTolerance  { get; init; }
    public          List<Vector3>       Waypoints            { get; init; } = [];
}
