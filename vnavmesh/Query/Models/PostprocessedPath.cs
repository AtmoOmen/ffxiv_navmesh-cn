using System.Numerics;
using vnavmesh.Movement.Planning;
using vnavmesh.Query.Enums;

namespace vnavmesh.Query.Models;

internal sealed class PostprocessedPath
{
    public required PathfindStatus Status               { get; init; }
    public required MovementMode   RequestedMode        { get; init; }
    public required Vector3        RequestedDestination { get; init; }
    public required Vector3        FinalDestination     { get; init; }
    public required float          DestinationTolerance { get; init; }

    public IReadOnlyList<PostprocessedPathSegment> Segments { get; init; } = [];

    public bool Succeeded =>
        Status != PathfindStatus.Failed;

    public List<Vector3> Waypoints
    {
        get
        {
            List<Vector3> waypoints = [];

            foreach (var waypoint in Segments.SelectMany(segment => segment.Waypoints))
            {
                if (waypoints.Count                                  == 0 ||
                    Vector3.DistanceSquared(waypoints[^1], waypoint) > DuplicateWaypointDistanceSq)
                    waypoints.Add(waypoint);
            }

            return waypoints;
        }
    }

    #region 常量

    private const float DuplicateWaypointDistanceSq = 0.000001f;

    #endregion
}
