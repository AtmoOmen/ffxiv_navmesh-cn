using System.Numerics;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Navigation.Planning;

internal sealed class PostprocessedPathSegment
{
    public required MovementMode           MovementMode         { get; init; }
    public required MovementSegmentKind    SegmentKind          { get; init; }
    public required bool                   AllowVerticalControl { get; init; }
    public required float                  CompletionTolerance  { get; init; }
    public required PathGeometryOwnership  GeometryOwnership    { get; init; }
    public required PathReachabilitySource ReachabilitySource   { get; init; }
    public          IReadOnlyList<Vector3> Waypoints            { get; init; } = [];
}
