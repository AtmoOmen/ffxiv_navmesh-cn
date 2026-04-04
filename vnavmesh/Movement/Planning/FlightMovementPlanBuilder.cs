using Dalamud.Game.ClientState.Conditions;
using vnavmesh.PathPostprocess;

namespace vnavmesh.Movement.Planning;

internal sealed class FlightMovementPlanBuilder : IMovementPlanBuilder
{
    public MovementPlan Build(PostprocessedPath path)
    {
        var segments = new List<MovementSegment>();

        if (!IsAirborne)
        {
            segments.Add
            (
                new TakeoffSegment
                {
                    CompletionTolerance = 0
                }
            );
        }

        segments.AddRange(path.Segments.Select(BuildSegment));

        return new()
        {
            RequestedMode        = path.RequestedMode,
            RequestedDestination = path.RequestedDestination,
            FinalDestination     = path.FinalDestination,
            DestinationTolerance = path.DestinationTolerance,
            Segments             = segments
        };
    }

    private static bool IsAirborne => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];

    private static MovementSegment BuildSegment(PostprocessedPathSegment segment) => new FlightTraverseSegment
    {
        CompletionTolerance = segment.CompletionTolerance,
        GeometryOwnership   = segment.GeometryOwnership,
        ReachabilitySource  = segment.ReachabilitySource,
        Waypoints           = [.. segment.Waypoints]
    };
}
