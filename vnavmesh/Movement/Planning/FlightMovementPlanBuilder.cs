using Dalamud.Game.ClientState.Conditions;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement.Planning;

internal sealed class FlightMovementPlanBuilder : IMovementPlanBuilder
{
    public MovementPlan Build(PostprocessedPath path)
    {
        var segments = new List<MovementSegment>();
        var shouldNormalizeTakeoffSegment = !IsAirborne;

        if (shouldNormalizeTakeoffSegment)
        {
            segments.Add
            (
                new TakeoffSegment
                {
                    CompletionTolerance = 0
                }
            );
        }

        segments.AddRange(path.Segments.Select(segment =>
        {
            var normalizedSegment = shouldNormalizeTakeoffSegment && segment.SegmentKind == MovementSegmentKind.FlightTraverse
                ? NormalizeTakeoffSegment(segment)
                : segment;
            if (segment.SegmentKind == MovementSegmentKind.FlightTraverse)
                shouldNormalizeTakeoffSegment = false;
            return BuildSegment(normalizedSegment);
        }));

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

    private static MovementSegment BuildSegment(PostprocessedPathSegment segment) => segment.SegmentKind switch
    {
        MovementSegmentKind.FlightTraverse => new FlightTraverseSegment
        {
            CompletionTolerance = segment.CompletionTolerance,
            StartPosition       = segment.StartPosition,
            GeometryOwnership   = segment.GeometryOwnership,
            ReachabilitySource  = segment.ReachabilitySource,
            Waypoints           = [.. segment.Waypoints],
            GroundCorridor      = segment.GroundCorridor
        },
        MovementSegmentKind.GroundTraverse => new GroundTraverseSegment
        {
            CompletionTolerance = segment.CompletionTolerance,
            StartPosition       = segment.StartPosition,
            GeometryOwnership   = segment.GeometryOwnership,
            ReachabilitySource  = segment.ReachabilitySource,
            Waypoints           = [.. segment.Waypoints],
            GroundCorridor      = segment.GroundCorridor
        },
        MovementSegmentKind.Takeoff => new TakeoffSegment
        {
            CompletionTolerance = segment.CompletionTolerance,
            StartPosition       = segment.StartPosition,
            GeometryOwnership   = segment.GeometryOwnership,
            ReachabilitySource  = segment.ReachabilitySource,
            Waypoints           = [.. segment.Waypoints],
            GroundCorridor      = segment.GroundCorridor
        },
        _ => throw new ArgumentOutOfRangeException(nameof(segment.SegmentKind), segment.SegmentKind, "未知移动阶段")
    };
}
