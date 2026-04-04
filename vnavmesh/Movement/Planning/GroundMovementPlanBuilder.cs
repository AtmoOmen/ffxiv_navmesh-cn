using vnavmesh.NavPathfind;

namespace vnavmesh.Movement.Planning;

internal sealed class GroundMovementPlanBuilder : IMovementPlanBuilder
{
    public MovementPlan Build(PathfindResult result, float destinationTolerance, float pathTolerance) =>
        new()
        {
            RequestedMode        = MovementMode.Ground,
            RequestedDestination = result.RequestedDestination,
            FinalDestination     = result.FinalDestination,
            DestinationTolerance = destinationTolerance,
            Segments =
            [
                new GroundTraverseSegment
                {
                    CompletionTolerance = pathTolerance,
                    Waypoints           = [.. result.Waypoints]
                }
            ]
        };
}