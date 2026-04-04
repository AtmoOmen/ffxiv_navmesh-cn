using System.Numerics;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Navigation.Planning;

internal sealed class PostprocessedPath
{
    public required PathfindStatus                          Status               { get; init; }
    public required MovementMode                            RequestedMode        { get; init; }
    public required Vector3                                 RequestedDestination { get; init; }
    public required Vector3                                 FinalDestination     { get; init; }
    public required float                                   DestinationTolerance { get; init; }
    public          IReadOnlyList<PostprocessedPathSegment> Segments             { get; init; } = [];

    public bool Succeeded => Status != PathfindStatus.Failed;

    public List<Vector3> Waypoints => [.. Segments.SelectMany(segment => segment.Waypoints)];
}
