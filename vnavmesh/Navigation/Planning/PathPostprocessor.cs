using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Movement.Planning;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Planning;

internal sealed class PathPostprocessor
{
    private readonly DtNavMeshQuery _meshQuery;

    public PathPostprocessor(DtNavMeshQuery meshQuery) =>
        _meshQuery = meshQuery;

    public PostprocessedPath Process(PlannerResult result, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments)
        {
            var waypoints = BuildWaypoints(segment, useStringPulling, cancel);
            segments.Add
            (
                new()
                {
                    MovementMode         = segment.MovementMode,
                    SegmentKind          = segment.SegmentKind,
                    AllowVerticalControl = segment.AllowVerticalControl,
                    StartPosition        = segment.StartPosition,
                    CompletionTolerance  = 0,
                    GeometryOwnership    = PathGeometryOwnership.Postprocessor,
                    ReachabilitySource   = segment.ReachabilitySource,
                    Waypoints            = waypoints
                }
            );
        }

        return new()
        {
            Status               = result.Status,
            RequestedMode        = result.RequestedMode,
            RequestedDestination = result.RequestedDestination,
            FinalDestination     = result.FinalDestination,
            DestinationTolerance = result.DestinationTolerance,
            Segments             = segments
        };
    }

    public PostprocessedPath CreateExternalInputPath
        (List<Vector3> waypoints, MovementMode requestedMode, Vector3 requestedDestination, float destinationTolerance)
    {
        var segmentKind = requestedMode == MovementMode.Flight ? MovementSegmentKind.FlightTraverse : MovementSegmentKind.GroundTraverse;
        var normalized  = DeduplicateWaypoints(waypoints);

        return new()
        {
            Status               = PathfindStatus.Complete,
            RequestedMode        = requestedMode,
            RequestedDestination = requestedDestination,
            FinalDestination     = normalized.Count > 0 ? normalized[^1] : requestedDestination,
            DestinationTolerance = destinationTolerance,
            Segments =
            [
                new()
                {
                    MovementMode         = requestedMode,
                    SegmentKind          = segmentKind,
                    AllowVerticalControl = requestedMode == MovementMode.Flight,
                    StartPosition        = normalized.Count > 0 ? normalized[0] : requestedDestination,
                    CompletionTolerance  = 0,
                    GeometryOwnership    = PathGeometryOwnership.ExternalInput,
                    ReachabilitySource   = PathReachabilitySource.ExternalInput,
                    Waypoints            = normalized
                }
            ]
        };
    }

    private IReadOnlyList<Vector3> BuildWaypoints(PlannerPathSegment segment, bool useStringPulling, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        return segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => BuildMeshWaypoints(segment, useStringPulling),
            PlannerSegmentGeometryKind.DiscretePoints => DeduplicateWaypoints(segment.Points),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };
    }

    private IReadOnlyList<Vector3> BuildMeshWaypoints(PlannerPathSegment segment, bool useStringPulling)
    {
        if (segment.Corridor.Count == 0)
            return [];

        if (useStringPulling)
        {
            var straightPath = new List<DtStraightPath>();
            var straightStatus = _meshQuery.FindStraightPath
                (segment.StartPosition.SystemToRecast(), segment.EndPosition.SystemToRecast(), [.. segment.Corridor], ref straightPath, 1024, 0);
            if (straightStatus.Failed())
                throw new InvalidOperationException("地面路径后处理失败：无法生成 string-pulling 路径");

            return DeduplicateWaypoints(straightPath.Select(p => p.pos.RecastToSystem()));
        }

        return DeduplicateWaypoints(segment.Corridor.Select(r => _meshQuery.GetAttachedNavMesh().GetPolyCenter(r).RecastToSystem()).Append(segment.EndPosition));
    }

    private static List<Vector3> DeduplicateWaypoints(IEnumerable<Vector3> points)
    {
        List<Vector3> result = [];

        foreach (var point in points)
        {
            if (result.Count == 0 || Vector3.DistanceSquared(result[^1], point) > 0.000001f)
                result.Add(point);
        }

        return result;
    }
}
