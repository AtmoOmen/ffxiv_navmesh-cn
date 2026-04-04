using System.Numerics;

namespace vnavmesh.Movement.Planning;

internal static class FlightWaypointNormalizer
{
    private const float AIRBORNE_START_DELTA_Y         = 0.001f;
    private const float DUPLICATE_WAYPOINT_DISTANCE_SQ = 0.000001f;

    public static List<Vector3> NormalizeForTakeoff(IReadOnlyList<Vector3> waypoints, Vector3 startPosition)
    {
        if (waypoints.Count == 0)
            return [];

        var firstDistinctIndex = 0;
        while (firstDistinctIndex < waypoints.Count && Vector3.DistanceSquared(waypoints[firstDistinctIndex], startPosition) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            firstDistinctIndex++;

        if (firstDistinctIndex >= waypoints.Count)
            return [waypoints[^1]];

        var firstAirborneIndex = -1;

        for (var i = firstDistinctIndex; i < waypoints.Count; i++)
            if (waypoints[i].Y > startPosition.Y + AIRBORNE_START_DELTA_Y)
            {
                firstAirborneIndex = i;
                break;
            }

        var startIndex = firstAirborneIndex >= 0 ? firstAirborneIndex : firstDistinctIndex;
        return [.. waypoints.Skip(startIndex)];
    }
}
