using System.Numerics;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement.Planning;

internal abstract class MovementSegment
{
    public abstract MovementSegmentKind    Kind                 { get; }
    public abstract bool                   AllowVerticalControl { get; }
    public          MovementMode           MovementMode         { get; protected init; }
    public          Vector3                StartPosition        { get; init; }
    public          PathGeometryOwnership  GeometryOwnership    { get; init; }
    public          PathReachabilitySource ReachabilitySource   { get; init; }
    public          float                  CompletionTolerance  { get; init; }
    public          IReadOnlyList<Vector3> Waypoints            { get; init; } = [];
}
