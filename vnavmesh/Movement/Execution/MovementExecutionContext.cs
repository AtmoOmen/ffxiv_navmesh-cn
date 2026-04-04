using System.Numerics;
using Dalamud.Game.ClientState.Objects.SubKinds;
using Dalamud.Plugin.Services;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Execution;

internal sealed class MovementExecutionContext
{
    public required Config           Config                { get; init; }
    public required IPlayerCharacter Player                { get; init; }
    public required MovementPlan     Plan                  { get; init; }
    public required int              SegmentIndex          { get; init; }
    public required MovementSegment  Segment               { get; init; }
    public required int              ActiveWaypointIndex   { get; init; }
    public required IReadOnlyList<int> SegmentWaypointIndices { get; init; }
    public required bool             MovementAllowed       { get; init; }
    public required float            PathTolerance         { get; init; }
    public          Vector3?         PreviousPosition      { get; init; }

    public int      WaypointCount        => Segment.Waypoints.Count;
    public bool     HasRemainingWaypoints => ActiveWaypointIndex < WaypointCount;
    public Vector3? ActiveWaypoint       => HasRemainingWaypoints ? Segment.Waypoints[ActiveWaypointIndex] : null;

    public bool TryGetFirstRemainingWaypoint(int startSegmentIndex, out Vector3 waypoint)
    {
        for (var i = startSegmentIndex; i < Plan.Segments.Count; i++)
        {
            var segment      = Plan.Segments[i];
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
}
