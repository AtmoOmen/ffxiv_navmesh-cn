using Dalamud.Game.ClientState.Conditions;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement.Planning;

internal sealed class MovementPlanBuilder
{
    public MovementPlan Build(PostprocessedPath path)
    {
        var segments                      = new List<MovementSegment>();
        var shouldNormalizeTakeoffSegment = path.RequestedMode == MovementMode.Flight && !IsAirborne;

        if (shouldNormalizeTakeoffSegment)
        {
            segments.Add
            (
                new()
                {
                    Kind                = MovementSegmentKind.Takeoff,
                    MovementMode        = MovementMode.Flight,
                    GeometryOwnership   = PathGeometryOwnership.None,
                    ReachabilitySource  = PathReachabilitySource.Volume,
                    CompletionTolerance = 0
                }
            );
        }

        segments.AddRange
        (
            path.Segments.Select
            (segment =>
                {
                    var resolvedSegment = shouldNormalizeTakeoffSegment && segment.SegmentKind == MovementSegmentKind.FlightTraverse
                                              ? NormalizeTakeoffSegment(segment)
                                              : segment;
                    if (segment.SegmentKind == MovementSegmentKind.FlightTraverse)
                        shouldNormalizeTakeoffSegment = false;
                    return BuildSegment(resolvedSegment);
                }
            )
        );

        return new()
        {
            RequestedMode        = path.RequestedMode,
            RequestedDestination = path.RequestedDestination,
            FinalDestination     = path.FinalDestination,
            DestinationTolerance = path.DestinationTolerance,
            Segments             = segments
        };
    }

    private static PostprocessedPathSegment NormalizeTakeoffSegment(PostprocessedPathSegment segment)
    {
        var normalizedWaypoints = FlightWaypointNormalizer.NormalizeForTakeoff(segment.Waypoints, segment.StartPosition);
        return new()
        {
            MovementMode         = segment.MovementMode,
            SegmentKind          = segment.SegmentKind,
            AllowVerticalControl = segment.AllowVerticalControl,
            StartPosition        = segment.StartPosition,
            CompletionTolerance  = segment.CompletionTolerance,
            GeometryOwnership    = segment.GeometryOwnership,
            ReachabilitySource   = segment.ReachabilitySource,
            Waypoints            = normalizedWaypoints,
            GroundCorridor       = segment.GroundCorridor
        };
    }

    private static MovementSegment BuildSegment(PostprocessedPathSegment segment) =>
        new()
        {
            Kind                = segment.SegmentKind,
            MovementMode        = segment.MovementMode,
            StartPosition       = segment.StartPosition,
            GeometryOwnership   = segment.GeometryOwnership,
            ReachabilitySource  = segment.ReachabilitySource,
            CompletionTolerance = segment.CompletionTolerance,
            Waypoints           = [.. segment.Waypoints],
            GroundCorridor      = segment.GroundCorridor
        };

    private static bool IsAirborne => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];
}
