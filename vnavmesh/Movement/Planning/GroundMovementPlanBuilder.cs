using vnavmesh.PathPostprocess;

namespace vnavmesh.Movement.Planning;

internal sealed class GroundMovementPlanBuilder : IMovementPlanBuilder
{
    public MovementPlan Build(PostprocessedPath path) =>
        new()
        {
            RequestedMode        = path.RequestedMode,
            RequestedDestination = path.RequestedDestination,
            FinalDestination     = path.FinalDestination,
            DestinationTolerance = path.DestinationTolerance,
            Segments             = [.. path.Segments.Select(BuildSegment)]
        };

    private static MovementSegment BuildSegment(PostprocessedPathSegment segment) => new GroundTraverseSegment
    {
        CompletionTolerance = segment.CompletionTolerance,
        GeometryOwnership   = segment.GeometryOwnership,
        ReachabilitySource  = segment.ReachabilitySource,
        Waypoints           = [.. segment.Waypoints]
    };
}
