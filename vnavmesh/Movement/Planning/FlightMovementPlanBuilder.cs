using Dalamud.Game.ClientState.Conditions;
using vnavmesh.NavPathfind;

namespace vnavmesh.Movement.Planning;

internal sealed class FlightMovementPlanBuilder : IMovementPlanBuilder
{
    public MovementPlan Build(PathfindResult result, float destinationTolerance, float pathTolerance)
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

        segments.Add
        (
            new FlightTraverseSegment
            {
                CompletionTolerance = pathTolerance,
                Waypoints           = [.. result.Waypoints]
            }
        );

        return new()
        {
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = result.RequestedDestination,
            FinalDestination     = result.FinalDestination,
            DestinationTolerance = destinationTolerance,
            Segments             = segments
        };
    }

    private static bool IsAirborne => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];
}
