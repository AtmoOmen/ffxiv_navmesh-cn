using System.Numerics;
using Dalamud.Game.ClientState.Objects.SubKinds;
using vnavmesh.Configuration;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Execution;

internal sealed class MovementExecutionContext
{
    public required Config             Config                 { get; init; }
    public required IPlayerCharacter   Player                 { get; init; }
    public required MovementPlan       Plan                   { get; init; }
    public required int                SegmentIndex           { get; init; }
    public required MovementSegment    Segment                { get; init; }
    public required int                ActiveWaypointIndex    { get; init; }
    public required IReadOnlyList<int> SegmentWaypointIndices { get; init; }
    public required bool               MovementAllowed        { get; init; }
    public          Vector3?           PreviousPosition       { get; init; }

    public int      WaypointCount               => Segment.Waypoints.Count;
    public int      TraverseSegmentCount        => WaypointCount;
    public int      CurrentTraverseSegmentIndex => ActiveWaypointIndex;
    public bool     HasRemainingWaypoints       => ActiveWaypointIndex < WaypointCount;
    public Vector3? ActiveWaypoint              => HasRemainingWaypoints ? Segment.Waypoints[ActiveWaypointIndex] : null;

    public bool TryGetFirstRemainingWaypoint(int startSegmentIndex, out Vector3 waypoint)
    {
        for (var i = startSegmentIndex; i < Plan.Segments.Count; i++)
        {
            var segment       = Plan.Segments[i];
            var waypointIndex = SegmentWaypointIndices[i];

            if (waypointIndex < segment.Waypoints.Count)
            {
                waypoint = segment.Waypoints[waypointIndex];
                return true;
            }
        }

        waypoint = default;
        return false;
    }

    public bool TryGetCurrentTraverseSegment(int traverseSegmentIndex, out Vector3 start, out Vector3 end)
    {
        if (traverseSegmentIndex < 0 || traverseSegmentIndex >= TraverseSegmentCount)
        {
            start = default;
            end   = default;
            return false;
        }

        start = traverseSegmentIndex == 0 ? Segment.StartPosition : Segment.Waypoints[traverseSegmentIndex - 1];
        end   = Segment.Waypoints[traverseSegmentIndex];
        return true;
    }
}
