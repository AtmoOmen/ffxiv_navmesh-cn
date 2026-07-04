using System.Numerics;
using Dalamud.Game.ClientState.Objects.SubKinds;
using vnavmesh.Internal;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation;

namespace vnavmesh.Movement.Execution;

internal sealed class MovementExecutionContext
{
    public required PluginConfig       Config                 { get; init; }
    public required IPlayerCharacter   Player                 { get; init; }
    public required NavmeshQuery?      Query                  { get; init; }
    public required MovementPlan       Plan                   { get; init; }
    public required int                SegmentIndex           { get; init; }
    public required MovementSegment    Segment                { get; init; }
    public required int                ActiveWaypointIndex    { get; init; }
    public required IReadOnlyList<int> SegmentWaypointIndices { get; init; }
    public required bool               MovementAllowed        { get; init; }
    public          Vector3?           PreviousPosition       { get; init; }

    public int  WaypointCount         => Segment.Waypoints.Count;
    public bool HasRemainingWaypoints => ActiveWaypointIndex < WaypointCount;

    public Vector3? ActiveWaypoint => HasRemainingWaypoints ?
                                          Segment.Waypoints[ActiveWaypointIndex] :
                                          null;

    public bool TryGetFirstRemainingWaypoint(out Vector3 waypoint) =>
        TryFindRemainingWaypoint(static (_, _) => true, out waypoint);

    public bool TryGetFirstElevatedRemainingWaypoint(float minHeightDelta, out Vector3 waypoint) =>
        TryFindRemainingWaypoint(static (candidate, minimumY) => candidate.Y > minimumY, out waypoint, Player.Position.Y + minHeightDelta);

    private bool TryFindRemainingWaypoint(Func<Vector3, float, bool> predicate, out Vector3 waypoint, float threshold = 0)
    {
        for (var segmentIndex = SegmentIndex; segmentIndex < Plan.Segments.Count; segmentIndex++)
        {
            var segment = Plan.Segments[segmentIndex];
            var firstWaypointIndex = segmentIndex == SegmentIndex ?
                                         ActiveWaypointIndex :
                                         SegmentWaypointIndices[segmentIndex];

            for (var waypointIndex = firstWaypointIndex; waypointIndex < segment.Waypoints.Count; waypointIndex++)
            {
                var candidate = segment.Waypoints[waypointIndex];

                if (predicate(candidate, threshold))
                {
                    waypoint = candidate;
                    return true;
                }
            }
        }

        waypoint = default;
        return false;
    }

    public bool TryGetCurrentTraverseSegment(int traverseSegmentIndex, out Vector3 start, out Vector3 end)
    {
        if (traverseSegmentIndex < 0 || traverseSegmentIndex >= WaypointCount)
        {
            start = default;
            end   = default;
            return false;
        }

        start = traverseSegmentIndex == 0 ?
                    Segment.StartPosition :
                    Segment.Waypoints[traverseSegmentIndex - 1];
        end = Segment.Waypoints[traverseSegmentIndex];
        return true;
    }
}
