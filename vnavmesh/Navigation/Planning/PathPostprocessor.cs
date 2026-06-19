using System.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Utilities;
using vnavmesh.Movement.Drivers;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Navigation.Planning;

internal sealed class PathPostprocessor
(
    Func<DtNavMeshQuery> getMeshQuery,
    Func<IDtQueryFilter> getGroundFilter
)
{
    private DtNavMeshQuery MeshQuery    => getMeshQuery();
    private IDtQueryFilter GroundFilter => getGroundFilter();

    public PostprocessedPath Process(PlannerResult result, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments)
            segments.Add(BuildSegment(segment, cancel));

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

    public PostprocessedPath ProcessStraightPath(PlannerResult result, CancellationToken cancel, int straightPathOptions = 0)
    {
        cancel.ThrowIfCancellationRequested();

        List<PostprocessedPathSegment> segments = new(result.Segments.Count);

        foreach (var segment in result.Segments)
            segments.Add(BuildStraightPathSegment(segment, straightPathOptions, cancel));

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

    private PostprocessedPathSegment BuildSegment(PlannerPathSegment segment, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        var groundCorridor = segment is { MovementMode: MovementMode.Ground, GeometryKind: PlannerSegmentGeometryKind.MeshCorridor }
                                 ? BuildGroundCorridor(segment)
                                 : segment.GroundCorridor;
        var (waypoints, flightPathDebug) = segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor   => (BuildMeshWaypoints(segment, groundCorridor), segment.FlightPathDebug),
            PlannerSegmentGeometryKind.DiscretePoints => BuildDiscreteWaypoints(segment),
            _                                         => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };

        return new()
        {
            MovementMode         = segment.MovementMode,
            SegmentKind          = segment.SegmentKind,
            AllowVerticalControl = segment.AllowVerticalControl,
            StartPosition        = segment.StartPosition,
            CompletionTolerance  = 0,
            Waypoints            = waypoints,
            GroundCorridor       = groundCorridor,
            FlightPathDebug      = flightPathDebug
        };
    }

    private PostprocessedPathSegment BuildStraightPathSegment(PlannerPathSegment segment, int straightPathOptions, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();

        var groundCorridor = segment is { MovementMode: MovementMode.Ground, GeometryKind: PlannerSegmentGeometryKind.MeshCorridor }
                                 ? BuildGroundCorridor(segment, straightPathOptions)
                                 : segment.GroundCorridor;
        var (waypoints, flightPathDebug) = segment.GeometryKind switch
        {
            PlannerSegmentGeometryKind.MeshCorridor => (BuildRawStraightPathWaypoints(segment, [.. segment.Corridor], straightPathOptions), segment.FlightPathDebug),
            PlannerSegmentGeometryKind.DiscretePoints => BuildRawDiscreteWaypoints(segment),
            _ => throw new ArgumentOutOfRangeException(nameof(segment.GeometryKind), segment.GeometryKind, "未知粗路径几何类型")
        };

        return new()
        {
            MovementMode         = segment.MovementMode,
            SegmentKind          = segment.SegmentKind,
            AllowVerticalControl = segment.AllowVerticalControl,
            StartPosition        = segment.StartPosition,
            CompletionTolerance  = 0,
            Waypoints            = waypoints,
            GroundCorridor       = groundCorridor,
            FlightPathDebug      = flightPathDebug
        };
    }

    private List<Vector3> BuildMeshWaypoints(PlannerPathSegment segment, GroundCorridorPayload? groundCorridor)
    {
        if (segment.Corridor.Count == 0)
            return [];

        return groundCorridor != null ? [.. groundCorridor.Corners.Select(c => c.Position)] : BuildStraightPathWaypoints(segment, [.. segment.Corridor]);
    }

    private List<Vector3> BuildStraightPathWaypoints(PlannerPathSegment segment, long[] corridor)
    {
        var (straightPath, straightPathCount) = QueryStraightPath(segment, corridor, DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS);
        return DeduplicateWaypoints(BuildAdjustedStraightPathPositions(segment, straightPath, straightPathCount, corridor, out _, out _));
    }

    private List<Vector3> BuildRawStraightPathWaypoints(PlannerPathSegment segment, long[] corridor, int straightPathOptions)
    {
        var (straightPath, straightPathCount) = QueryStraightPath(segment, corridor, straightPathOptions);
        return BuildAdjustedStraightPathPositions(segment, straightPath, straightPathCount, corridor, out _, out _);
    }

    private GroundCorridorPayload? BuildGroundCorridor(PlannerPathSegment segment)
        => BuildGroundCorridor(segment, DtStraightPathOptions.DT_STRAIGHTPATH_AREA_CROSSINGS);

    private GroundCorridorPayload? BuildGroundCorridor(PlannerPathSegment segment, int straightPathOptions)
    {
        if (segment.Corridor.Count == 0)
            return null;

        var corners = BuildGroundCorners(segment, [.. segment.Corridor], straightPathOptions, out var initialWaypointIndex);
        return new()
        {
            PolyRefs             = [.. segment.Corridor],
            Target               = segment.EndPosition,
            InitialWaypointIndex = initialWaypointIndex,
            InitialCornerIndex   = ResolveInitialCornerIndex(corners, initialWaypointIndex),
            Corners              = corners,
            LinkMarkers          = BuildGroundLinkMarkers(corners)
        };
    }

    private IReadOnlyList<GroundPathCorner> BuildGroundCorners
    (
        PlannerPathSegment segment,
        long[]             corridor,
        int                straightPathOptions,
        out int            initialWaypointIndex
    )
    {
        var (straightPath, straightPathCount) = QueryStraightPath(segment, corridor, straightPathOptions);
        var adjustedPositions = BuildAdjustedStraightPathPositions(segment, straightPath, straightPathCount, corridor, out var debugInfos, out initialWaypointIndex);

        List<GroundPathCorner> result = [];

        for (var i = 0; i < straightPathCount; i++)
        {
            var rawCorner = straightPath[i];
            var position  = adjustedPositions[i];
            var area      = ResolveArea(corridor, rawCorner.refs);
            var linkKind  = ResolveLinkKind(area, rawCorner.flags);
            var traversalProfile = ResolveTraversalProfile(rawCorner.refs, linkKind);
            if (linkKind is { } resolvedKind && debugInfos[i] is { } debugInfo)
                debugInfos[i] = debugInfo with
                {
                    TraversalProfile = traversalProfile,
                    TraversalCost    = NavmeshLinkTraversalProfiles.EstimateCost(position, ResolveLinkEndPosition(adjustedPositions, i), resolvedKind, traversalProfile)
                };
            var corner = new GroundPathCorner
            (
                position,
                rawCorner.refs,
                rawCorner.flags,
                area,
                linkKind,
                debugInfos[i]
            );

            if (result.Count == 0)
            {
                result.Add(corner);
                continue;
            }

            var previous = result[^1];
            if (Vector3.DistanceSquared(previous.Position, corner.Position) > DUPLICATE_WAYPOINT_DISTANCE_SQ ||
                previous.Area                                               != corner.Area                   ||
                previous.LinkKind                                           != corner.LinkKind               ||
                previous.StraightPathFlags                                  != corner.StraightPathFlags) result.Add(corner);
        }

        return result;
    }

    private List<Vector3> BuildAdjustedStraightPathPositions
    (
        PlannerPathSegment           segment,
        DtStraightPath[]             straightPath,
        int                          straightPathCount,
        long[]                       corridor,
        out GroundPathCornerDebug?[] debugInfos,
        out int                      initialWaypointIndex
    )
    {
        List<Vector3> result = new(straightPathCount);
        debugInfos           = new GroundPathCornerDebug?[straightPathCount];
        initialWaypointIndex = ResolveInitialWaypointIndex(segment, straightPath, straightPathCount);

        for (var i = 0; i < straightPathCount; ++i)
        {
            var adjusted = AdjustStraightPathPosition
            (
                straightPath,
                straightPathCount,
                corridor,
                i,
                initialWaypointIndex,
                segment.TraversalStartPosition,
                out var debugInfo
            );
            result.Add(adjusted);
            debugInfos[i] = debugInfo;
        }

        return result;
    }

    private Vector3 AdjustStraightPathPosition
    (
        DtStraightPath[]           straightPath,
        int                        straightPathCount,
        long[]                     corridor,
        int                        index,
        int                        initialWaypointIndex,
        Vector3                    traversalStartPosition,
        out GroundPathCornerDebug? debugInfo
    )
    {
        debugInfo = null;
        var point = straightPath[index].pos.RecastToSystem();
        if (straightPathCount < 2 || index < 0 || index >= straightPathCount)
            return point;

        var flags = straightPath[index].flags;
        if ((flags & DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0)
            return point;

        var isStartPoint        = index == 0;
        var isEndPoint          = index == straightPathCount - 1;
        var hasExecutionStart   = initialWaypointIndex >= 0 && initialWaypointIndex < straightPathCount;
        var isExecutionStart    = hasExecutionStart         && index                == initialWaypointIndex;
        var isInitiallyConsumed = hasExecutionStart         && index                < initialWaypointIndex;
        if (isEndPoint || (flags & DtStraightPathFlags.DT_STRAIGHTPATH_END) != 0)
            return point;

        if (!isStartPoint && straightPathCount < 3)
            return point;

        var polyRef = ResolveStraightPathPolyRef(straightPath, corridor, index);
        if (polyRef == 0)
            return point;

        var localPolyRefs = CollectLocalPolyRefs(straightPath, straightPathCount, corridor, index);
        var next          = straightPath[index + 1].pos.RecastToSystem();
        var previous = isExecutionStart
                           ? initialWaypointIndex == 0
                                 ? point - (next - point)
                                 : traversalStartPosition
                           : isStartPoint
                               ? point - (next - point)
                               : straightPath[index - 1].pos.RecastToSystem();
        var incoming = new Vector2(point.X - previous.X, point.Z - previous.Z);
        var outgoing = new Vector2(next.X  - point.X,    next.Z  - point.Z);
        if (incoming.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ || outgoing.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return point;

        incoming = Vector2.Normalize(incoming);
        outgoing = Vector2.Normalize(outgoing);

        var isStartAdjacent  = hasExecutionStart && index >= initialWaypointIndex && index <= initialWaypointIndex + 1;
        var isEndAdjacent    = index >= straightPathCount - 2;
        var endpointAdjacent = isStartAdjacent || isEndAdjacent;
        var cornerStrength   = Math.Clamp((1f - Vector2.Dot(incoming, outgoing)) * 0.5f, 0f, 1f);

        var bisector = incoming + outgoing;
        if (!isStartAdjacent && !isEndAdjacent && bisector.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return point;

        var travelDirection = isStartAdjacent
                                  ? outgoing
                                  : isEndAdjacent
                                      ? incoming
                                      : Vector2.Normalize(bisector);
        var preferredDirectionBias = isStartAdjacent
                                         ? outgoing
                                         : isEndAdjacent
                                             ? -incoming
                                             : cornerStrength <= WALL_STRAIGHT_PUSH_MAX_CORNER_STRENGTH
                                                 ? travelDirection
                                                 : -travelDirection;
        var scanOrigin = BuildScanOrigin(point, previous, next, polyRef, isStartAdjacent, isEndAdjacent);
        polyRef = ResolveScanPolyRef(localPolyRefs, polyRef, scanOrigin, WALL_SCAN_ORIGIN_REPROJECT_MAX_DISTANCE, out scanOrigin);

        var sample = SampleScanClearances(localPolyRefs, polyRef, scanOrigin, preferredDirectionBias);

        if (isStartAdjacent &&
            ShouldRescanStartAdjacent(sample.AverageClearance, sample.MinClearance, sample.WallPressure))
        {
            var recenteredOrigin = RecenterStartAdjacentScanOrigin(point, next, polyRef, outgoing, sample.WallPressure, sample.AverageClearance);
            var rescannedPolyRef = ResolveScanPolyRef(localPolyRefs, polyRef, recenteredOrigin, WALL_SCAN_ORIGIN_REPROJECT_MAX_DISTANCE, out recenteredOrigin);
            var rescannedSample  = SampleScanClearances(localPolyRefs, rescannedPolyRef, recenteredOrigin, preferredDirectionBias);

            if (rescannedSample.AverageClearance > sample.AverageClearance + 0.05f ||
                rescannedSample.MinClearance     > sample.MinClearance     + 0.01f)
            {
                scanOrigin = recenteredOrigin;
                polyRef    = rescannedPolyRef;
                sample     = rescannedSample;
            }
        }

        var minClearance       = sample.MinClearance;
        var maxClearance       = sample.MaxClearance;
        var totalClearance     = sample.TotalClearance;
        var averageClearance   = sample.AverageClearance;
        var preferredDirection = sample.PreferredDirection;
        var wallPressure       = sample.WallPressure;
        var samples            = sample.Samples;

        if (isInitiallyConsumed)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                true,
                false,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        if (minClearance >= WALL_PUSH_TARGET_CLEARANCE || totalClearance <= 0)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                Vector2.Zero,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                0f
            );
            return point;
        }

        if (TryAdjustNearStraightPosition
            (
                index,
                isExecutionStart,
                polyRef,
                localPolyRefs,
                point,
                scanOrigin,
                travelDirection,
                preferredDirection,
                wallPressure,
                cornerStrength,
                endpointAdjacent,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                out var straightAdjusted,
                out debugInfo
            )) return straightAdjusted;

        if (isStartAdjacent &&
            TryAdjustStartAdjacentByWallPressure
            (
                index,
                isExecutionStart,
                polyRef,
                localPolyRefs,
                point,
                scanOrigin,
                outgoing,
                preferredDirection,
                wallPressure,
                cornerStrength,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                out var startAdjusted,
                out debugInfo
            )) return startAdjusted;

        if (cornerStrength < WALL_PUSH_MIN_CORNER_STRENGTH)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        if (wallPressure.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        wallPressure = Vector2.Normalize(wallPressure);
        var lateralPush = preferredDirection + wallPressure * 1.35f;

        if (lateralPush.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        lateralPush = Vector2.Normalize(lateralPush);
        var forwardComponent = Vector2.Dot(lateralPush, outgoing);
        lateralPush -= outgoing * MathF.Max(0f, forwardComponent - WALL_PUSH_FORWARD_REJECTION);

        if (lateralPush.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        lateralPush = Vector2.Normalize(lateralPush);
        var asymmetry = (maxClearance - minClearance) / MathF.Max(maxClearance, 0.001f);

        if (asymmetry < WALL_PUSH_MIN_ASYMMETRY)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        var wallPressureMagnitude = WALL_SCAN_RADIUS - averageClearance;

        if (wallPressureMagnitude < WALL_PUSH_MIN_WALL_PRESSURE)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                0f,
                1f,
                0f,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        var clearanceAlongPush     = MeasureRaycastClearance(localPolyRefs, polyRef, scanOrigin, lateralPush, WALL_SCAN_RADIUS);
        var lateralWidth           = maxClearance + minClearance;
        var dynamicPushScale       = ComputeDynamicPushScale(lateralWidth);
        var dynamicPushMaxDistance = ComputeDynamicPushMaxDistance(lateralWidth);
        var rawPushDistance        = (WALL_PUSH_TARGET_CLEARANCE - minClearance) * Math.Clamp(asymmetry * (0.6f + cornerStrength), 0f, 1.5f);
        var desiredPush            = rawPushDistance                             * dynamicPushScale;
        var maxPushByClearance     = clearanceAlongPush                          * WALL_PUSH_CLEARANCE_FRACTION * dynamicPushScale;
        var pushDistance           = MathF.Min(dynamicPushMaxDistance, MathF.Min(desiredPush, maxPushByClearance));

        if (pushDistance < WALL_PUSH_MIN_DISTANCE)
        {
            debugInfo = BuildDebugInfo
            (
                index,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                sample.PreferredPolyRef,
                0,
                0,
                false,
                false,
                lateralWidth,
                dynamicPushScale,
                rawPushDistance,
                dynamicPushMaxDistance,
                0f,
                0f,
                false,
                false,
                point,
                scanOrigin,
                point,
                Vector2.Zero,
                preferredDirection,
                wallPressure,
                samples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return point;
        }

        var candidate = point + new Vector3(lateralPush.X * pushDistance, 0, lateralPush.Y * pushDistance);
        candidate = ResolveAdjustedPoint(localPolyRefs, polyRef, point, candidate, lateralPush);

        debugInfo = BuildDebugInfo
        (
            index,
            false,
            isExecutionStart,
            polyRef,
            localPolyRefs.Length,
            sample.PreferredPolyRef,
            0,
            0,
            false,
            false,
            lateralWidth,
            dynamicPushScale,
            rawPushDistance,
            dynamicPushMaxDistance,
            0f,
            0f,
            false,
            false,
            point,
            scanOrigin,
            candidate,
            Vector2.Zero,
            preferredDirection,
            wallPressure,
            samples,
            minClearance,
            maxClearance,
            averageClearance,
            cornerStrength
        );
        return candidate;
    }

    private static int ResolveInitialWaypointIndex(PlannerPathSegment segment, DtStraightPath[] straightPath, int straightPathCount)
    {
        if (segment.MovementMode != MovementMode.Ground || segment.SegmentKind != MovementSegmentKind.GroundTraverse || straightPathCount <= 0)
            return 0;

        List<Vector3> rawWaypoints = new(straightPathCount);
        for (var i = 0; i < straightPathCount; ++i)
            rawWaypoints.Add(straightPath[i].pos.RecastToSystem());

        return Math.Clamp
        (
            WaypointProgression.ConsumeGroundWaypoints
            (
                segment.TraversalStartPosition,
                segment.TraversalStartPosition,
                segment.StartPosition,
                rawWaypoints
            ),
            0,
            straightPathCount
        );
    }

    private long[] CollectLocalPolyRefs(DtStraightPath[] straightPath, int straightPathCount, long[] corridor, int index)
    {
        HashSet<long> uniqueRefs = [];
        List<long>    refs       = [];

        void TryAdd(long candidateRef)
        {
            if (candidateRef == 0 || !uniqueRefs.Add(candidateRef))
                return;

            refs.Add(candidateRef);
        }

        void TryAddNeighbours(long sourceRef)
        {
            if (sourceRef == 0)
                return;

            var status = MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRef(sourceRef, out var tile, out var poly);
            if (status.Failed() || tile == null || poly == null)
                return;

            for (var linkIndex = poly.firstLink; linkIndex != DtDetour.DT_NULL_LINK; linkIndex = tile.links[linkIndex].next)
            {
                var neighbourRef = tile.links[linkIndex].refs;
                if (neighbourRef == 0)
                    continue;

                if (!MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRef(neighbourRef, out var neighbourTile, out var neighbourPoly).Succeeded())
                    continue;

                if (neighbourPoly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
                    continue;

                if (!GroundFilter.PassFilter(neighbourRef, neighbourTile, neighbourPoly))
                    continue;

                TryAdd(neighbourRef);
            }
        }

        TryAdd(ResolveStraightPathPolyRef(straightPath, corridor, index));
        if (index > 0)
            TryAdd(ResolveStraightPathPolyRef(straightPath, corridor, index - 1));
        if (index + 1 < straightPathCount)
            TryAdd(ResolveStraightPathPolyRef(straightPath, corridor, index + 1));

        if (corridor.Length > 0)
        {
            var corridorIndex = Math.Clamp(index, 0, corridor.Length - 1);
            TryAdd(corridor[corridorIndex]);
            if (corridorIndex > 0)
                TryAdd(corridor[corridorIndex - 1]);
            if (corridorIndex + 1 < corridor.Length)
                TryAdd(corridor[corridorIndex + 1]);
        }

        foreach (var candidateRef in refs.ToArray())
            TryAddNeighbours(candidateRef);

        return [.. refs];
    }

    private Vector3 BuildScanOrigin(Vector3 point, Vector3 previous, Vector3 next, long polyRef, bool isStartAdjacent, bool isEndAdjacent)
    {
        var delta = isStartAdjacent
                        ? new Vector2(next.X - point.X, next.Z - point.Z)
                        : isEndAdjacent
                            ? new Vector2(previous.X                   - point.X, previous.Z                   - point.Z)
                            : new Vector2((previous.X + next.X) * 0.5f - point.X, (previous.Z + next.Z) * 0.5f - point.Z);
        var lengthSq = delta.LengthSquared();
        if (lengthSq <= WALL_SCAN_MIN_VECTOR_SQ)
            return point;

        var length = MathF.Sqrt(lengthSq);
        var endpointAdjacent = isStartAdjacent || isEndAdjacent;
        var insetScale = isStartAdjacent ? WALL_STARTPOINT_SCAN_ORIGIN_SCALE : endpointAdjacent ? WALL_ENDPOINT_ADJACENT_SCAN_ORIGIN_SCALE : 0.35f;
        var insetMin = isStartAdjacent ? WALL_STARTPOINT_SCAN_ORIGIN_MIN : WALL_SCAN_ORIGIN_INSET_MIN;
        var insetMax = isStartAdjacent ? WALL_STARTPOINT_SCAN_ORIGIN_MAX : endpointAdjacent ? WALL_ENDPOINT_ADJACENT_SCAN_ORIGIN_MAX : WALL_SCAN_ORIGIN_INSET_MAX;
        var inset = Math.Clamp(length * insetScale, insetMin, insetMax);
        var insetFactor = MathF.Min(1f, inset / length);
        var origin = point + new Vector3(delta.X * insetFactor, 0, delta.Y * insetFactor);

        if (MeshQuery.GetPolyHeight(polyRef, origin.SystemToRecast(), out var height).Succeeded())
            origin.Y = height;

        return origin;
    }

    private long ResolveScanPolyRef
        (long[] localPolyRefs, long fallbackPolyRef, Vector3 preferredPosition, float maxHorizontalDistance, out Vector3 projectedPosition)
    {
        projectedPosition = preferredPosition;
        LocalPolyProjection? bestProjection = null;

        foreach (var candidateRef in localPolyRefs)
        {
            if (!TryProjectToPoly(candidateRef, preferredPosition, out var candidateProjection))
                continue;

            if (candidateProjection.HorizontalDistanceSq > maxHorizontalDistance * maxHorizontalDistance)
                continue;

            if (bestProjection == null || IsBetterLocalProjection(candidateProjection, bestProjection.Value, fallbackPolyRef))
                bestProjection = candidateProjection;
        }

        if (bestProjection is not { } best)
            return fallbackPolyRef;

        projectedPosition = best.ProjectedPoint;
        return best.PolyRef;
    }

    private Vector3 ResolveAdjustedPoint(long[] localPolyRefs, long preferredPolyRef, Vector3 originalPoint, Vector3 candidate, Vector2 pushDirection)
    {
        if (MeshQuery.GetPolyHeight(preferredPolyRef, candidate.SystemToRecast(), out var height).Succeeded())
        {
            var directCandidate = candidate;
            directCandidate.Y = height;

            if (pushDirection.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
                return directCandidate;

            var normalizedPush = Vector2.Normalize(pushDirection);
            var directDelta    = new Vector2(directCandidate.X - originalPoint.X, directCandidate.Z - originalPoint.Z);
            var directProgress = Vector2.Dot(directDelta, normalizedPush);

            LocalPolyProjection? directBestProjectionCandidate = null;

            foreach (var candidateRef in localPolyRefs)
            {
                if (!TryProjectToPoly(candidateRef, candidate, out var projection))
                    continue;

                if (projection.HorizontalDistanceSq > WALL_ADJUSTED_POINT_REPROJECT_MAX_DISTANCE * WALL_ADJUSTED_POINT_REPROJECT_MAX_DISTANCE)
                    continue;

                if (directBestProjectionCandidate == null ||
                    IsBetterAdjustedProjection(projection, directBestProjectionCandidate.Value, preferredPolyRef, originalPoint, normalizedPush))
                    directBestProjectionCandidate = projection;
            }

            if (directBestProjectionCandidate is not { } directBestProjection)
                return directCandidate;

            var projectedDelta    = new Vector2(directBestProjection.ProjectedPoint.X - originalPoint.X, directBestProjection.ProjectedPoint.Z - originalPoint.Z);
            var projectedProgress = Vector2.Dot(projectedDelta, normalizedPush);
            if (projectedProgress > directProgress + WALL_ADJUSTED_POINT_DIRECT_ACCEPT_PROGRESS_EPSILON)
                return directBestProjection.ProjectedPoint;

            return directCandidate;
        }

        LocalPolyProjection? bestProjection = null;

        foreach (var candidateRef in localPolyRefs)
        {
            if (!TryProjectToPoly(candidateRef, candidate, out var projection))
                continue;

            if (projection.HorizontalDistanceSq > WALL_ADJUSTED_POINT_REPROJECT_MAX_DISTANCE * WALL_ADJUSTED_POINT_REPROJECT_MAX_DISTANCE)
                continue;

            if (bestProjection == null || IsBetterAdjustedProjection(projection, bestProjection.Value, preferredPolyRef, originalPoint, pushDirection))
                bestProjection = projection;
        }

        return bestProjection?.ProjectedPoint ?? candidate;
    }

    private bool TryProjectToPoly(long polyRef, Vector3 position, out LocalPolyProjection projection)
    {
        if (MeshQuery.ClosestPointOnPoly(polyRef, position.SystemToRecast(), out var closest, out var isOverPoly).Succeeded())
        {
            var projectedPoint       = closest.RecastToSystem();
            var horizontalDeltaX     = projectedPoint.X                    - position.X;
            var horizontalDeltaZ     = projectedPoint.Z                    - position.Z;
            var horizontalDistanceSq = horizontalDeltaX * horizontalDeltaX + horizontalDeltaZ * horizontalDeltaZ;
            projection = new(polyRef, projectedPoint, isOverPoly, horizontalDistanceSq, MathF.Abs(projectedPoint.Y - position.Y));
            return true;
        }

        projection = default;
        return false;
    }

    private static bool IsBetterLocalProjection(LocalPolyProjection candidate, LocalPolyProjection currentBest, long fallbackPolyRef)
    {
        if (candidate.IsOverPoly != currentBest.IsOverPoly)
            return candidate.IsOverPoly;

        if (MathF.Abs(candidate.HorizontalDistanceSq - currentBest.HorizontalDistanceSq) > 0.0001f)
            return candidate.HorizontalDistanceSq < currentBest.HorizontalDistanceSq;

        if (MathF.Abs(candidate.VerticalDistanceAbs - currentBest.VerticalDistanceAbs) > 0.0001f)
            return candidate.VerticalDistanceAbs < currentBest.VerticalDistanceAbs;

        if (candidate.PolyRef == fallbackPolyRef && currentBest.PolyRef != fallbackPolyRef)
            return true;

        return candidate.PolyRef < currentBest.PolyRef;
    }

    private static bool IsBetterAdjustedProjection
    (
        LocalPolyProjection candidate,
        LocalPolyProjection currentBest,
        long                fallbackPolyRef,
        Vector3             originalPoint,
        Vector2             pushDirection
    )
    {
        if (candidate.IsOverPoly != currentBest.IsOverPoly)
            return candidate.IsOverPoly;

        if (pushDirection.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ)
        {
            pushDirection = Vector2.Normalize(pushDirection);
            var candidateDelta    = new Vector2(candidate.ProjectedPoint.X   - originalPoint.X, candidate.ProjectedPoint.Z   - originalPoint.Z);
            var currentDelta      = new Vector2(currentBest.ProjectedPoint.X - originalPoint.X, currentBest.ProjectedPoint.Z - originalPoint.Z);
            var candidateProgress = Vector2.Dot(candidateDelta, pushDirection);
            var currentProgress   = Vector2.Dot(currentDelta,   pushDirection);

            if (MathF.Abs(candidateProgress - currentProgress) > 0.02f)
                return candidateProgress > currentProgress;

            var candidateOffAxis = candidateDelta - pushDirection * candidateProgress;
            var currentOffAxis   = currentDelta   - pushDirection * currentProgress;
            if (MathF.Abs(candidateOffAxis.LengthSquared() - currentOffAxis.LengthSquared()) > 0.0001f)
                return candidateOffAxis.LengthSquared() < currentOffAxis.LengthSquared();
        }

        if (MathF.Abs(candidate.HorizontalDistanceSq - currentBest.HorizontalDistanceSq) > 0.0001f)
            return candidate.HorizontalDistanceSq < currentBest.HorizontalDistanceSq;

        if (MathF.Abs(candidate.VerticalDistanceAbs - currentBest.VerticalDistanceAbs) > 0.0001f)
            return candidate.VerticalDistanceAbs < currentBest.VerticalDistanceAbs;

        if (candidate.PolyRef == fallbackPolyRef && currentBest.PolyRef != fallbackPolyRef)
            return true;

        return candidate.PolyRef < currentBest.PolyRef;
    }

    private ScanSampleResult SampleScanClearances(long[] localPolyRefs, long fallbackPolyRef, Vector3 scanOrigin, Vector2 preferredDirectionBias)
    {
        var                               minClearance            = float.MaxValue;
        var                               maxClearance            = 0f;
        var                               totalClearance          = 0f;
        var                               preferredDirection      = preferredDirectionBias;
        var                               preferredDirectionScore = float.MinValue;
        var                               preferredPolyRef        = fallbackPolyRef;
        var                               wallPressure            = Vector2.Zero;
        List<GroundPathCornerDebugSample> samples                 = new(WALL_SCAN_DIRECTION_COUNT);

        for (var sampleIndex = 0; sampleIndex < WALL_SCAN_DIRECTION_COUNT; ++sampleIndex)
        {
            var angle     = MathF.Tau * sampleIndex / WALL_SCAN_DIRECTION_COUNT;
            var direction = new Vector2(MathF.Cos(angle), MathF.Sin(angle));
            var clearance = MeasureRaycastClearance
                (localPolyRefs, fallbackPolyRef, scanOrigin, direction, WALL_SCAN_RADIUS, out var samplePolyRef, out var sampleStart);
            minClearance   =  MathF.Min(minClearance, clearance);
            maxClearance   =  MathF.Max(maxClearance, clearance);
            totalClearance += clearance;
            wallPressure   -= direction * (WALL_SCAN_RADIUS - clearance);

            var directionScore = clearance + Vector2.Dot(direction, preferredDirectionBias) * 0.35f;

            if (directionScore > preferredDirectionScore)
            {
                preferredDirectionScore = directionScore;
                preferredDirection      = direction;
                preferredPolyRef        = samplePolyRef;
            }

            samples.Add
            (
                new GroundPathCornerDebugSample
                (
                    sampleStart,
                    sampleStart + new Vector3(direction.X * clearance, 0, direction.Y * clearance),
                    clearance,
                    samplePolyRef
                )
            );
        }

        return new
            (minClearance, maxClearance, totalClearance, totalClearance / WALL_SCAN_DIRECTION_COUNT, preferredDirection, preferredPolyRef, wallPressure, samples);
    }

    private static bool ShouldRescanStartAdjacent(float averageClearance, float minClearance, Vector2 wallPressure) =>
        averageClearance             < WALL_START_ADJACENT_RESCAN_AVG_CLEARANCE_THRESHOLD &&
        minClearance                 <= 0.01f                                             &&
        wallPressure.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ;

    private static bool ShouldRescanNearStraight(float leftClearance, float rightClearance, float averageClearance)
    {
        if (leftClearance + rightClearance <= WALL_SCAN_MIN_VECTOR_SQ)
            return true;

        var nearClearance = MathF.Min(leftClearance, rightClearance);
        var farClearance  = MathF.Max(leftClearance, rightClearance);
        if (nearClearance <= WALL_STRAIGHT_RESCAN_ONE_SIDE_COLLAPSE_THRESHOLD &&
            farClearance  >= WALL_STRAIGHT_RESCAN_ONE_SIDE_OPEN_THRESHOLD)
            return true;

        if (nearClearance                <= WALL_STRAIGHT_RESCAN_EDGE_MIN_CLEARANCE_THRESHOLD &&
            farClearance - nearClearance >= WALL_STRAIGHT_RESCAN_EDGE_IMBALANCE_THRESHOLD)
            return true;

        return averageClearance < WALL_STRAIGHT_RESCAN_AVG_CLEARANCE_THRESHOLD;
    }

    private Vector3 RecenterStartAdjacentScanOrigin
    (
        Vector3 point,
        Vector3 next,
        long    polyRef,
        Vector2 outgoing,
        Vector2 wallPressure,
        float   averageClearance
    )
    {
        var forwardDistance = Vector3.Distance(point, next);
        var forwardInset = Math.Clamp
        (
            forwardDistance * WALL_START_ADJACENT_RESCAN_FORWARD_SCALE,
            WALL_START_ADJACENT_RESCAN_FORWARD_MIN,
            WALL_START_ADJACENT_RESCAN_FORWARD_MAX
        );

        var lateralEscape = Vector2.Normalize(wallPressure);
        lateralEscape -= outgoing * Vector2.Dot(lateralEscape, outgoing);
        lateralEscape =  lateralEscape.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ ? Vector2.Normalize(lateralEscape) : Vector2.Zero;

        var lateralInset = Math.Clamp
        (
            MathF.Max(0f, WALL_SCAN_RADIUS - averageClearance) * WALL_START_ADJACENT_RESCAN_LATERAL_SCALE,
            WALL_START_ADJACENT_RESCAN_LATERAL_MIN,
            WALL_START_ADJACENT_RESCAN_LATERAL_MAX
        );

        var origin = point                                                                          +
                     new Vector3(outgoing.X      * forwardInset, 0, outgoing.Y      * forwardInset) +
                     new Vector3(lateralEscape.X * lateralInset, 0, lateralEscape.Y * lateralInset);

        if (MeshQuery.GetPolyHeight(polyRef, origin.SystemToRecast(), out var height).Succeeded())
            origin.Y = height;

        return origin;
    }

    private static Vector3 RecenterNearStraightScanOrigin
    (
        Vector3 scanOrigin,
        Vector2 pathDirection,
        Vector2 lateralBias,
        float   averageClearance
    )
    {
        var forwardInset  = WALL_STRAIGHT_RESCAN_FORWARD_DISTANCE;
        var lateralEscape = BuildPathLateralDirection(lateralBias, pathDirection, Vector2.Zero);

        var lateralInset = Math.Clamp
        (
            MathF.Max(0f, WALL_SCAN_RADIUS - averageClearance) * WALL_STRAIGHT_RESCAN_LATERAL_SCALE,
            WALL_STRAIGHT_RESCAN_LATERAL_MIN,
            WALL_STRAIGHT_RESCAN_LATERAL_MAX
        );

        return scanOrigin                                                                     +
               new Vector3(pathDirection.X * forwardInset, 0, pathDirection.Y * forwardInset) +
               new Vector3(lateralEscape.X * lateralInset, 0, lateralEscape.Y * lateralInset);
    }

    private static long ResolveStraightPathPolyRef(DtStraightPath[] straightPath, long[] corridor, int index)
    {
        var polyRef = straightPath[index].refs;
        if (polyRef != 0)
            return polyRef;

        if (index > 0 && straightPath[index - 1].refs != 0)
            return straightPath[index - 1].refs;

        if (index + 1 < straightPath.Length && straightPath[index + 1].refs != 0)
            return straightPath[index + 1].refs;

        if (corridor.Length == 0)
            return 0;

        var corridorIndex = Math.Clamp(index, 0, corridor.Length - 1);
        return corridor[corridorIndex];
    }

    private static Vector2 BuildPathLateralDirection(Vector2 direction, Vector2 pathDirection, Vector2 preferredSide)
    {
        if (direction.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ || pathDirection.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return Vector2.Zero;

        var lateral = direction - pathDirection * Vector2.Dot(direction, pathDirection);
        if (lateral.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return Vector2.Zero;

        lateral = Vector2.Normalize(lateral);
        if (preferredSide.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ && Vector2.Dot(lateral, preferredSide) < 0f)
            lateral = -lateral;

        return lateral;
    }

    private bool TryBuildInteriorDirection
    (
        long[]      localPolyRefs,
        Vector3     point,
        Vector3     scanOrigin,
        Vector2     pathDirection,
        Vector2     preferredSide,
        out Vector2 interiorDirection
    )
    {
        interiorDirection = Vector2.Zero;
        if (pathDirection.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return false;

        var navmesh     = MeshQuery.GetAttachedNavMesh();
        var accumulated = Vector2.Zero;

        AccumulateFromOrigin(point,      1.0f);
        AccumulateFromOrigin(scanOrigin, 0.75f);

        if (accumulated.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return false;

        interiorDirection = Vector2.Normalize(accumulated);
        if (preferredSide.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ && Vector2.Dot(interiorDirection, preferredSide) < 0f)
            interiorDirection = preferredSide;

        return true;

        void AccumulateFromOrigin(Vector3 origin, float originWeight)
        {
            foreach (var candidateRef in localPolyRefs)
            {
                if (candidateRef == 0)
                    continue;

                var center = navmesh.GetPolyCenter(candidateRef).RecastToSystem();
                var lateral = BuildPathLateralDirection
                (
                    new Vector2(center.X - origin.X, center.Z - origin.Z),
                    pathDirection,
                    preferredSide
                );
                if (lateral.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
                    continue;

                var distance = Vector2.Distance(new(origin.X, origin.Z), new(center.X, center.Z));
                var weight   = originWeight / MathF.Max(0.35f, distance);
                accumulated += lateral * weight;
            }
        }
    }

    private static float ComputeDynamicPushWidthT(float lateralWidth)
    {
        return Math.Clamp
        (
            (lateralWidth - WALL_PUSH_DYNAMIC_WIDTH_NARROW) /
            MathF.Max(0.001f, WALL_PUSH_DYNAMIC_WIDTH_WIDE - WALL_PUSH_DYNAMIC_WIDTH_NARROW),
            0f,
            1f
        );
    }

    private static float ComputeDynamicPushScale(float lateralWidth)
    {
        var widthT = ComputeDynamicPushWidthT(lateralWidth);
        return WALL_PUSH_DYNAMIC_SCALE_MIN + (WALL_PUSH_DYNAMIC_SCALE_MAX - WALL_PUSH_DYNAMIC_SCALE_MIN) * widthT;
    }

    private static float ComputeDynamicPushMaxDistance(float lateralWidth)
    {
        var widthT = ComputeDynamicPushWidthT(lateralWidth);
        return WALL_PUSH_BASE_MAX_DISTANCE + (WALL_PUSH_DYNAMIC_MAX_DISTANCE_WIDE - WALL_PUSH_BASE_MAX_DISTANCE) * widthT;
    }

    private float MeasureRaycastClearance(long[] localPolyRefs, long fallbackPolyRef, Vector3 start, Vector2 direction, float maxDistance)
        => MeasureRaycastClearance(localPolyRefs, fallbackPolyRef, start, direction, maxDistance, out _, out _);

    private float MeasureRaycastClearance(long[] localPolyRefs, long fallbackPolyRef, Vector3 start, Vector2 direction, float maxDistance, out long bestPolyRef)
        => MeasureRaycastClearance(localPolyRefs, fallbackPolyRef, start, direction, maxDistance, out bestPolyRef, out _);

    private float MeasureRaycastClearance
    (
        long[]      localPolyRefs,
        long        fallbackPolyRef,
        Vector3     start,
        Vector2     direction,
        float       maxDistance,
        out long    bestPolyRef,
        out Vector3 bestProjectedStart
    )
    {
        var bestPolyRefLocal        = 0L;
        var bestProjectedStartLocal = start;

        if (direction.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
        {
            bestPolyRef        = bestPolyRefLocal;
            bestProjectedStart = bestProjectedStartLocal;
            return 0;
        }

        direction = Vector2.Normalize(direction);

        var end           = new Vector3(start.X + direction.X * maxDistance, start.Y, start.Z + direction.Y * maxDistance);
        var bestClearance = 0f;
        var hasCandidate  = false;

        TryMeasure(fallbackPolyRef);
        foreach (var candidateRef in localPolyRefs)
            TryMeasure(candidateRef);

        void TryMeasure(long candidateRef)
        {
            if (candidateRef == 0)
                return;

            if (!TryProjectToPoly(candidateRef, start, out var projection))
                return;

            var projectedStart = projection.ProjectedPoint;
            var status = MeshQuery.Raycast(candidateRef, projectedStart.SystemToRecast(), end.SystemToRecast(), GroundFilter, out var t, out _, [], out _, 0);
            if (status.Failed())
                return;

            var clearance = t == float.MaxValue ? maxDistance : Math.Clamp(t, 0f, 1f) * maxDistance;

            if (!hasCandidate || clearance > bestClearance)
            {
                bestClearance           = clearance;
                bestPolyRefLocal        = candidateRef;
                bestProjectedStartLocal = projectedStart;
                hasCandidate            = true;
            }
        }

        bestPolyRef        = bestPolyRefLocal;
        bestProjectedStart = bestProjectedStartLocal;
        return hasCandidate ? bestClearance : 0f;
    }

    private (DtStraightPath[] StraightPath, int Count) QueryStraightPath(PlannerPathSegment segment, long[] corridor, int straightPathOptions)
    {
        var straightPath = new DtStraightPath[MAX_SMOOTH_PATH_POINTS];
        var straightStatus = MeshQuery.FindStraightPath
        (
            segment.StartPosition.SystemToRecast(),
            segment.EndPosition.SystemToRecast(),
            corridor,
            corridor.Length,
            straightPath,
            out var straightPathCount,
            straightPath.Length,
            straightPathOptions
        );
        if (straightStatus.Failed())
            throw new InvalidOperationException("地面路径后处理失败：无法生成 straight path");

        return (straightPath, straightPathCount);
    }

    private GroundPathCornerDebug BuildDebugInfo
    (
        int                                        straightPathIndex,
        bool                                       initiallyConsumed,
        bool                                       isExecutionStart,
        long                                       scanPolyRef,
        int                                        localPolyCount,
        long                                       preferredPolyRef,
        long                                       leftPolyRef,
        long                                       rightPolyRef,
        bool                                       rescanned,
        bool                                       usedInteriorDirection,
        float                                      leftClearance,
        float                                      rightClearance,
        bool                                       straightBalanceSatisfied,
        bool                                       straightLowClearanceCase,
        Vector3                                    point,
        Vector3                                    scanOrigin,
        Vector3                                    adjustedPoint,
        Vector2                                    interiorDirection,
        Vector2                                    preferredDirection,
        Vector2                                    wallPressure,
        IReadOnlyList<GroundPathCornerDebugSample> samples,
        float                                      minClearance,
        float                                      maxClearance,
        float                                      averageClearance,
        float                                      cornerStrength
    )
        => BuildDebugInfo
        (
            straightPathIndex,
            initiallyConsumed,
            isExecutionStart,
            scanPolyRef,
            localPolyCount,
            preferredPolyRef,
            leftPolyRef,
            rightPolyRef,
            rescanned,
            usedInteriorDirection,
            0f,
            1f,
            0f,
            0f,
            leftClearance,
            rightClearance,
            straightBalanceSatisfied,
            straightLowClearanceCase,
            point,
            scanOrigin,
            adjustedPoint,
            interiorDirection,
            preferredDirection,
            wallPressure,
            samples,
            minClearance,
            maxClearance,
            averageClearance,
            cornerStrength
        );

    private GroundPathCornerDebug BuildDebugInfo
    (
        int                                        straightPathIndex,
        bool                                       initiallyConsumed,
        bool                                       isExecutionStart,
        long                                       scanPolyRef,
        int                                        localPolyCount,
        long                                       preferredPolyRef,
        long                                       leftPolyRef,
        long                                       rightPolyRef,
        bool                                       rescanned,
        bool                                       usedInteriorDirection,
        float                                      dynamicPushWidth,
        float                                      dynamicPushScale,
        float                                      rawPushDistance,
        float                                      leftClearance,
        float                                      rightClearance,
        bool                                       straightBalanceSatisfied,
        bool                                       straightLowClearanceCase,
        Vector3                                    point,
        Vector3                                    scanOrigin,
        Vector3                                    adjustedPoint,
        Vector2                                    interiorDirection,
        Vector2                                    preferredDirection,
        Vector2                                    wallPressure,
        IReadOnlyList<GroundPathCornerDebugSample> samples,
        float                                      minClearance,
        float                                      maxClearance,
        float                                      averageClearance,
        float                                      cornerStrength
    )
        => BuildDebugInfo
        (
            straightPathIndex,
            initiallyConsumed,
            isExecutionStart,
            scanPolyRef,
            localPolyCount,
            preferredPolyRef,
            leftPolyRef,
            rightPolyRef,
            rescanned,
            usedInteriorDirection,
            dynamicPushWidth,
            dynamicPushScale,
            rawPushDistance,
            0f,
            leftClearance,
            rightClearance,
            straightBalanceSatisfied,
            straightLowClearanceCase,
            point,
            scanOrigin,
            adjustedPoint,
            interiorDirection,
            preferredDirection,
            wallPressure,
            samples,
            minClearance,
            maxClearance,
            averageClearance,
            cornerStrength
        );

    private static GroundPathCornerDebug BuildDebugInfo
    (
        int                                        straightPathIndex,
        bool                                       initiallyConsumed,
        bool                                       isExecutionStart,
        long                                       scanPolyRef,
        int                                        localPolyCount,
        long                                       preferredPolyRef,
        long                                       leftPolyRef,
        long                                       rightPolyRef,
        bool                                       rescanned,
        bool                                       usedInteriorDirection,
        float                                      dynamicPushWidth,
        float                                      dynamicPushScale,
        float                                      rawPushDistance,
        float                                      dynamicPushMaxDistance,
        float                                      leftClearance,
        float                                      rightClearance,
        bool                                       straightBalanceSatisfied,
        bool                                       straightLowClearanceCase,
        Vector3                                    point,
        Vector3                                    scanOrigin,
        Vector3                                    adjustedPoint,
        Vector2                                    interiorDirection,
        Vector2                                    preferredDirection,
        Vector2                                    wallPressure,
        IReadOnlyList<GroundPathCornerDebugSample> samples,
        float                                      minClearance,
        float                                      maxClearance,
        float                                      averageClearance,
        float                                      cornerStrength
    )
    {
        var interiorEndpoint  = scanOrigin + new Vector3(interiorDirection.X  * WALL_SCAN_RADIUS, 0, interiorDirection.Y  * WALL_SCAN_RADIUS);
        var preferredEndpoint = scanOrigin + new Vector3(preferredDirection.X * WALL_SCAN_RADIUS, 0, preferredDirection.Y * WALL_SCAN_RADIUS);
        var pressureEndpoint  = scanOrigin + new Vector3(wallPressure.X       * WALL_SCAN_RADIUS, 0, wallPressure.Y       * WALL_SCAN_RADIUS);
        return new
        (
            straightPathIndex,
            initiallyConsumed,
            isExecutionStart,
            scanPolyRef,
            localPolyCount,
            preferredPolyRef,
            leftPolyRef,
            rightPolyRef,
            rescanned,
            usedInteriorDirection,
            dynamicPushWidth,
            dynamicPushScale,
            rawPushDistance,
            dynamicPushMaxDistance,
            leftClearance,
            rightClearance,
            straightBalanceSatisfied,
            straightLowClearanceCase,
            point,
            scanOrigin,
            adjustedPoint,
            interiorEndpoint,
            preferredEndpoint,
            pressureEndpoint,
            Vector3.Distance(point, adjustedPoint),
            minClearance,
            maxClearance,
            averageClearance,
            cornerStrength,
            samples
        );
    }

    private bool TryAdjustNearStraightPosition
    (
        int                                        straightPathIndex,
        bool                                       isExecutionStart,
        long                                       polyRef,
        long[]                                     localPolyRefs,
        Vector3                                    point,
        Vector3                                    scanOrigin,
        Vector2                                    travelDirection,
        Vector2                                    preferredDirection,
        Vector2                                    wallPressure,
        float                                      cornerStrength,
        bool                                       endpointAdjacent,
        IReadOnlyList<GroundPathCornerDebugSample> samples,
        float                                      minClearance,
        float                                      maxClearance,
        float                                      averageClearance,
        out Vector3                                adjustedPoint,
        out GroundPathCornerDebug?                 debugInfo
    )
    {
        adjustedPoint = point;
        debugInfo     = null;

        if (cornerStrength > WALL_STRAIGHT_PUSH_MAX_CORNER_STRENGTH || travelDirection.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return false;

        var pathDirection        = Vector2.Normalize(travelDirection);
        var leftNormal           = new Vector2(-pathDirection.Y, pathDirection.X);
        var rightNormal          = -leftNormal;
        var debugSamples         = samples;
        var preferredPolyRef     = polyRef;
        var rescanned            = false;
        var straightScanOrigin   = scanOrigin;
        var leftClearance        = MeasureRaycastClearance(localPolyRefs, polyRef, straightScanOrigin, leftNormal,  WALL_SCAN_RADIUS, out var leftPolyRef);
        var rightClearance       = MeasureRaycastClearance(localPolyRefs, polyRef, straightScanOrigin, rightNormal, WALL_SCAN_RADIUS, out var rightPolyRef);
        var lateralPreference    = rightClearance >= leftClearance ? rightNormal : leftNormal;
        var hasInteriorDirection = TryBuildInteriorDirection(localPolyRefs, point, straightScanOrigin, pathDirection, lateralPreference, out var interiorDirection);

        if (ShouldRescanNearStraight(leftClearance, rightClearance, averageClearance))
        {
            var pressureBias = BuildPathLateralDirection(wallPressure, pathDirection, lateralPreference);
            var rescanBias = hasInteriorDirection
                                 ? interiorDirection
                                 : pressureBias.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ
                                     ? pressureBias
                                     : lateralPreference;
            var rescannedOrigin  = RecenterNearStraightScanOrigin(straightScanOrigin, pathDirection, rescanBias, averageClearance);
            var rescannedPolyRef = ResolveScanPolyRef(localPolyRefs, polyRef, rescannedOrigin, WALL_SCAN_ORIGIN_REPROJECT_MAX_DISTANCE, out rescannedOrigin);
            var rescannedLeftClearance = MeasureRaycastClearance
                (localPolyRefs, rescannedPolyRef, rescannedOrigin, leftNormal, WALL_SCAN_RADIUS, out var rescannedLeftPolyRef);
            var rescannedRightClearance = MeasureRaycastClearance
                (localPolyRefs, rescannedPolyRef, rescannedOrigin, rightNormal, WALL_SCAN_RADIUS, out var rescannedRightPolyRef);

            if (rescannedLeftClearance + rescannedRightClearance > leftClearance + rightClearance + WALL_STRAIGHT_RESCAN_IMPROVEMENT_THRESHOLD ||
                MathF.Min
                    (rescannedLeftClearance, rescannedRightClearance) >
                MathF.Min(leftClearance, rightClearance) + WALL_STRAIGHT_RESCAN_MIN_CLEARANCE_IMPROVEMENT_THRESHOLD)
            {
                straightScanOrigin   = rescannedOrigin;
                scanOrigin           = rescannedOrigin;
                polyRef              = rescannedPolyRef;
                leftClearance        = rescannedLeftClearance;
                rightClearance       = rescannedRightClearance;
                leftPolyRef          = rescannedLeftPolyRef;
                rightPolyRef         = rescannedRightPolyRef;
                lateralPreference    = rightClearance >= leftClearance ? rightNormal : leftNormal;
                hasInteriorDirection = TryBuildInteriorDirection(localPolyRefs, point, straightScanOrigin, pathDirection, lateralPreference, out interiorDirection);
                rescanned            = true;

                var rescannedSample = SampleScanClearances(localPolyRefs, polyRef, straightScanOrigin, hasInteriorDirection ? interiorDirection : lateralPreference);
                minClearance       = rescannedSample.MinClearance;
                maxClearance       = rescannedSample.MaxClearance;
                averageClearance   = rescannedSample.AverageClearance;
                preferredDirection = rescannedSample.PreferredDirection;
                preferredPolyRef   = rescannedSample.PreferredPolyRef;
                wallPressure       = rescannedSample.WallPressure;
                debugSamples       = rescannedSample.Samples;
            }
        }

        var sideTotal = leftClearance + rightClearance;

        if (sideTotal <= WALL_SCAN_MIN_VECTOR_SQ)
        {
            debugInfo = BuildDebugInfo
            (
                straightPathIndex,
                false,
                isExecutionStart,
                polyRef,
                localPolyRefs.Length,
                preferredPolyRef,
                leftPolyRef,
                rightPolyRef,
                rescanned,
                hasInteriorDirection,
                leftClearance,
                rightClearance,
                false,
                false,
                point,
                straightScanOrigin,
                point,
                interiorDirection,
                pathDirection,
                wallPressure,
                debugSamples,
                minClearance,
                maxClearance,
                averageClearance,
                cornerStrength
            );
            return false;
        }

        var sideBalance         = rightClearance - leftClearance;
        var sideBalanceDistance = MathF.Abs(sideBalance);
        var sideBalanceRatio    = sideBalanceDistance / sideTotal;
        var minBalanceDistance = endpointAdjacent
                                     ? WALL_STRAIGHT_PUSH_MIN_BALANCE_DISTANCE * WALL_ENDPOINT_ADJACENT_BALANCE_DISTANCE_SCALE
                                     : WALL_STRAIGHT_PUSH_MIN_BALANCE_DISTANCE;
        var minBalanceRatio = endpointAdjacent
                                  ? WALL_STRAIGHT_PUSH_MIN_BALANCE_RATIO * WALL_ENDPOINT_ADJACENT_BALANCE_RATIO_SCALE
                                  : WALL_STRAIGHT_PUSH_MIN_BALANCE_RATIO;
        var nearClearance    = MathF.Min(leftClearance, rightClearance);
        var farClearance     = MathF.Max(leftClearance, rightClearance);
        var lateralDirection = sideBalance         > 0 ? rightNormal : leftNormal;
        var balancedEnough   = sideBalanceDistance >= minBalanceDistance && sideBalanceRatio >= minBalanceRatio;
        var lowClearanceCase = false;

        if (!balancedEnough)
        {
            lowClearanceCase =
                averageClearance             < WALL_STRAIGHT_LOW_CLEARANCE_AVG_THRESHOLD &&
                minClearance                 < WALL_STRAIGHT_LOW_CLEARANCE_MIN_THRESHOLD &&
                farClearance - nearClearance >= WALL_STRAIGHT_LOW_CLEARANCE_MIN_SIDE_GAP;
            if (!lowClearanceCase)
                return false;
        }

        var pushDirection = lateralDirection;
        if (hasInteriorDirection)
            pushDirection += interiorDirection * WALL_STRAIGHT_INTERIOR_PUSH_SCALE;

        var preferredLateral = BuildPathLateralDirection(preferredDirection, pathDirection, lateralDirection);
        if (preferredLateral.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ)
            pushDirection += preferredLateral * WALL_STRAIGHT_PREFERRED_PUSH_SCALE;

        var pressureLateral = BuildPathLateralDirection(wallPressure, pathDirection, lateralDirection);
        if (pressureLateral.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ)
            pushDirection += pressureLateral * WALL_STRAIGHT_PRESSURE_PUSH_SCALE;

        if (pushDirection.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            pushDirection = lateralDirection;

        pushDirection = Vector2.Normalize(pushDirection);
        var lateralWidth           = sideTotal;
        var dynamicPushScale       = ComputeDynamicPushScale(lateralWidth);
        var dynamicPushMaxDistance = ComputeDynamicPushMaxDistance(lateralWidth);
        var rawPushDistance        = MathF.Max(sideBalanceDistance * 0.5f, WALL_PUSH_TARGET_CLEARANCE - nearClearance);
        if (!balancedEnough)
            rawPushDistance = MathF.Max(rawPushDistance * WALL_STRAIGHT_LOW_CLEARANCE_PUSH_SCALE, WALL_PUSH_TARGET_CLEARANCE - nearClearance);
        var desiredPush        = rawPushDistance * dynamicPushScale;
        var maxPushByClearance = farClearance    * WALL_PUSH_CLEARANCE_FRACTION * dynamicPushScale;
        var pushDistance       = MathF.Min(dynamicPushMaxDistance, MathF.Min(desiredPush, maxPushByClearance));
        if (pushDistance < WALL_PUSH_MIN_DISTANCE)
            return false;

        adjustedPoint = point + new Vector3(pushDirection.X * pushDistance, 0, pushDirection.Y * pushDistance);
        adjustedPoint = ResolveAdjustedPoint(localPolyRefs, polyRef, point, adjustedPoint, pushDirection);

        debugInfo = BuildDebugInfo
        (
            straightPathIndex,
            false,
            isExecutionStart,
            polyRef,
            localPolyRefs.Length,
            preferredPolyRef,
            leftPolyRef,
            rightPolyRef,
            rescanned,
            hasInteriorDirection,
            lateralWidth,
            dynamicPushScale,
            rawPushDistance,
            dynamicPushMaxDistance,
            leftClearance,
            rightClearance,
            balancedEnough,
            lowClearanceCase,
            point,
            straightScanOrigin,
            adjustedPoint,
            interiorDirection,
            pushDirection,
            wallPressure,
            debugSamples,
            minClearance,
            maxClearance,
            averageClearance,
            cornerStrength
        );
        return true;
    }

    private bool TryAdjustStartAdjacentByWallPressure
    (
        int                                        straightPathIndex,
        bool                                       isExecutionStart,
        long                                       polyRef,
        long[]                                     localPolyRefs,
        Vector3                                    point,
        Vector3                                    scanOrigin,
        Vector2                                    outgoing,
        Vector2                                    preferredDirection,
        Vector2                                    wallPressure,
        float                                      cornerStrength,
        IReadOnlyList<GroundPathCornerDebugSample> samples,
        float                                      minClearance,
        float                                      maxClearance,
        float                                      averageClearance,
        out Vector3                                adjustedPoint,
        out GroundPathCornerDebug?                 debugInfo
    )
    {
        adjustedPoint = point;
        debugInfo     = null;

        if (wallPressure.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return false;

        var pressureEscape    = Vector2.Normalize(wallPressure);
        var escapeDirection   = preferredDirection + pressureEscape * 1.25f;
        var backwardComponent = Vector2.Dot(escapeDirection, -outgoing);
        if (backwardComponent > 0)
            escapeDirection += outgoing * backwardComponent;

        if (escapeDirection.LengthSquared() <= WALL_SCAN_MIN_VECTOR_SQ)
            return false;

        escapeDirection = Vector2.Normalize(escapeDirection);
        var clearanceAlongPush = MeasureRaycastClearance(localPolyRefs, polyRef, scanOrigin, escapeDirection, WALL_SCAN_RADIUS);
        if (clearanceAlongPush < WALL_PUSH_MIN_DISTANCE)
            return false;

        var pressureMagnitude      = MathF.Max(0f, WALL_SCAN_RADIUS - averageClearance);
        var lateralWidth           = maxClearance + minClearance;
        var dynamicPushScale       = ComputeDynamicPushScale(lateralWidth);
        var dynamicPushMaxDistance = ComputeDynamicPushMaxDistance(lateralWidth);
        var rawPushDistance        = MathF.Max(WALL_PUSH_TARGET_CLEARANCE - minClearance, pressureMagnitude * WALL_START_ADJACENT_PRESSURE_PUSH_SCALE);
        var desiredPush            = rawPushDistance    * dynamicPushScale;
        var maxPushByClearance     = clearanceAlongPush * WALL_PUSH_CLEARANCE_FRACTION * dynamicPushScale;
        var pushDistance           = MathF.Min(dynamicPushMaxDistance, MathF.Min(desiredPush, maxPushByClearance));
        if (pushDistance < WALL_PUSH_MIN_DISTANCE)
            return false;

        adjustedPoint = point + new Vector3(escapeDirection.X * pushDistance, 0, escapeDirection.Y * pushDistance);
        adjustedPoint = ResolveAdjustedPoint(localPolyRefs, polyRef, point, adjustedPoint, escapeDirection);

        debugInfo = BuildDebugInfo
        (
            straightPathIndex,
            false,
            isExecutionStart,
            polyRef,
            localPolyRefs.Length,
            preferredDirection.LengthSquared() > WALL_SCAN_MIN_VECTOR_SQ ? polyRef : 0,
            0,
            0,
            false,
            false,
            lateralWidth,
            dynamicPushScale,
            rawPushDistance,
            dynamicPushMaxDistance,
            0f,
            0f,
            false,
            false,
            point,
            scanOrigin,
            adjustedPoint,
            Vector2.Zero,
            preferredDirection,
            wallPressure,
            samples,
            minClearance,
            maxClearance,
            averageClearance,
            cornerStrength
        );
        return true;
    }

    private static int ResolveInitialCornerIndex(IReadOnlyList<GroundPathCorner> corners, int initialWaypointIndex)
    {
        if (corners.Count == 0)
            return 0;

        for (var i = 0; i < corners.Count; ++i)
        {
            var rawIndex = corners[i].Debug?.StraightPathIndex ?? i;
            if (rawIndex >= initialWaypointIndex)
                return i;
        }

        return corners.Count - 1;
    }

    private readonly record struct ScanSampleResult
    (
        float                                      MinClearance,
        float                                      MaxClearance,
        float                                      TotalClearance,
        float                                      AverageClearance,
        Vector2                                    PreferredDirection,
        long                                       PreferredPolyRef,
        Vector2                                    WallPressure,
        IReadOnlyList<GroundPathCornerDebugSample> Samples
    );

    private readonly record struct LocalPolyProjection
    (
        long    PolyRef,
        Vector3 ProjectedPoint,
        bool    IsOverPoly,
        float   HorizontalDistanceSq,
        float   VerticalDistanceAbs
    );

    private IReadOnlyList<GroundLinkMarker> BuildGroundLinkMarkers(IReadOnlyList<GroundPathCorner> corners)
    {
        List<GroundLinkMarker> markers = [];

        for (var i = 0; i < corners.Count; i++)
        {
            if (corners[i].LinkKind is not { } kind)
                continue;

            var traversalProfile = ResolveTraversalProfile(corners[i].PolyRef, kind);
            markers.Add
            (
                new
                (
                    i,
                    corners[i].Position,
                    corners[i].PolyRef,
                    kind,
                    traversalProfile,
                    NavmeshLinkTraversalProfiles.EstimateCost(corners[i].Position, ResolveLinkEndPosition(corners, i), kind, traversalProfile)
                )
            );
        }

        return markers;
    }

    private NavmeshArea ResolveArea(long[] corridor, long polyRef)
    {
        var resolvedRef = polyRef != 0 ? polyRef : corridor[^1];
        MeshQuery.GetAttachedNavMesh().GetTileAndPolyByRefUnsafe(resolvedRef, out _, out var poly);
        return (NavmeshArea)poly.GetArea();
    }

    private static NavmeshOffMeshKind? ResolveLinkKind(NavmeshArea area, byte straightPathFlags)
    {
        if ((straightPathFlags & DtStraightPathFlags.DT_STRAIGHTPATH_OFFMESH_CONNECTION) == 0)
            return null;

        return area switch
        {
            NavmeshArea.GeneratedClimbDown => NavmeshOffMeshKind.GeneratedClimbDown,
            NavmeshArea.GeneratedEdgeJump  => NavmeshOffMeshKind.GeneratedEdgeJump,
            NavmeshArea.ManualOffMesh      => NavmeshOffMeshKind.ManualOffMesh,
            NavmeshArea.Shortcut           => NavmeshOffMeshKind.Shortcut,
            NavmeshArea.Teleport           => NavmeshOffMeshKind.Teleport,
            NavmeshArea.ClientPath         => NavmeshOffMeshKind.ClientPath,
            _                              => null
        };
    }

    private NavmeshLinkTraversalProfile? ResolveTraversalProfile(long polyRef, NavmeshOffMeshKind? kind)
    {
        if (kind == null)
            return null;

        return MeshQuery.GetAttachedNavMesh() != null &&
               GroundFilter is NavmeshGroundQuery.GroundAreaCostFilter { } groundFilter &&
               groundFilter.TryGetRegisteredTraversalProfile(polyRef, out var traversalProfile)
            ? traversalProfile
            : null;
    }

    private static Vector3 ResolveLinkEndPosition(IReadOnlyList<GroundPathCorner> corners, int index) =>
        index + 1 < corners.Count ? corners[index + 1].Position : corners[index].Position;

    private static Vector3 ResolveLinkEndPosition(IReadOnlyList<Vector3> positions, int index) =>
        index + 1 < positions.Count ? positions[index + 1] : positions[index];

    private static (List<Vector3> Waypoints, FlightPathDebugPayload? Debug) BuildRawDiscreteWaypoints(PlannerPathSegment segment)
        => ([.. segment.Points], segment.FlightPathDebug);

    private static (List<Vector3> Waypoints, FlightPathDebugPayload? Debug) BuildDiscreteWaypoints(PlannerPathSegment segment)
    {
        if (segment.MovementMode != MovementMode.Flight)
            return (DeduplicateWaypoints(segment.Points), segment.FlightPathDebug);

        var           rawDebugLookup            = segment.FlightPathDebug?.Waypoints.ToDictionary(d => d.PathIndex);
        var           coarsePath                = segment.FlightPathDebug?.CoarsePath ?? [];
        var           proxyDebug                = segment.FlightPathDebug?.ProxyDebug;
        List<Vector3> deduplicated              = [];
        List<int>     deduplicatedSourceIndices = [];

        for (var i = 0; i < segment.Points.Count; ++i)
        {
            var point = segment.Points[i];
            if (deduplicated.Count > 0 && Vector3.DistanceSquared(deduplicated[^1], point) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
                continue;

            deduplicated.Add(point);
            deduplicatedSourceIndices.Add(i);
        }

        var deduplicatedDebugLookup = RemapFlightDebugLookup(rawDebugLookup, deduplicatedSourceIndices);
        var (simplifiedPoints, simplifiedDebugLookup) = SimplifyFlightWaypoints(deduplicated, deduplicatedDebugLookup);
        return (simplifiedPoints, BuildFlightPathDebugPayload(simplifiedDebugLookup, coarsePath, proxyDebug));
    }

    private static (List<Vector3> Waypoints, Dictionary<int, FlightPathWaypointDebug>? DebugLookup) SimplifyFlightWaypoints
    (
        List<Vector3>                             points,
        Dictionary<int, FlightPathWaypointDebug>? debugLookup
    )
    {
        if (points.Count <= 2)
            return ([.. points], RemapFlightDebugLookup(debugLookup, Enumerable.Range(0, points.Count)));

        List<Vector3> simplified      = [points[0]];
        List<int>     retainedIndices = [0];

        for (var i = 1; i < points.Count - 1; i++)
        {
            var previous = simplified[^1];
            var current  = points[i];
            var next     = points[i + 1];
            if (IsRedundantFlightWaypoint(previous, current, next))
                continue;

            simplified.Add(current);
            retainedIndices.Add(i);
        }

        simplified.Add(points[^1]);
        retainedIndices.Add(points.Count - 1);
        return (simplified, RemapFlightDebugLookup(debugLookup, retainedIndices));
    }

    private static Dictionary<int, FlightPathWaypointDebug>? RemapFlightDebugLookup
    (
        IReadOnlyDictionary<int, FlightPathWaypointDebug>? source,
        IEnumerable<int>                                   retainedIndices
    )
    {
        if (source == null || source.Count == 0)
            return null;

        Dictionary<int, FlightPathWaypointDebug> remapped      = [];
        var                                      nextPathIndex = 0;

        foreach (var sourceIndex in retainedIndices)
        {
            if (source.TryGetValue(sourceIndex, out var debug))
                remapped[nextPathIndex] = debug with { PathIndex = nextPathIndex };

            ++nextPathIndex;
        }

        return remapped.Count > 0 ? remapped : null;
    }

    private static FlightPathDebugPayload? BuildFlightPathDebugPayload
    (
        Dictionary<int, FlightPathWaypointDebug>?      debugLookup,
        IReadOnlyList<FlightCoarsePathDebugNode>?      coarsePath = null,
        FlightLongRangeProxyDebug?                     proxyDebug = null
    )
    {
        var resolvedCoarsePath = coarsePath ?? [];
        if ((debugLookup == null || debugLookup.Count == 0) && resolvedCoarsePath.Count == 0 && proxyDebug == null)
            return null;

        return new()
        {
            Waypoints  = debugLookup != null ? [.. debugLookup.OrderBy(pair => pair.Key).Select(pair => pair.Value)] : [],
            CoarsePath = resolvedCoarsePath,
            ProxyDebug = proxyDebug
        };
    }

    private static bool IsRedundantFlightWaypoint(Vector3 previous, Vector3 current, Vector3 next)
    {
        if (Vector3.DistanceSquared(previous, current) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;
        if (Vector3.DistanceSquared(current, next) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;
        if (Vector3.DistanceSquared(previous, next) <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return true;
        if (NeedsFlightVerticalPreservation(previous, next))
            return false;

        return DistanceToLineSegment(current, previous, next) <= COLLINEAR_WAYPOINT_TOLERANCE;
    }

    private static bool NeedsFlightVerticalPreservation(Vector3 previous, Vector3 next)
    {
        var verticalDelta = MathF.Abs(next.Y - previous.Y);
        if (verticalDelta <= FLIGHT_DESCENT_PRESERVE_MIN_DROP)
            return false;

        var horizontalDistance = HorizontalDistanceXZ(previous, next);
        if (horizontalDistance <= FLIGHT_DESCENT_PRESERVE_NEAR_VERTICAL_HORIZONTAL)
            return true;

        return verticalDelta / horizontalDistance >= FLIGHT_DESCENT_PRESERVE_MAX_SLOPE;
    }

    private static float DistanceToLineSegment(Vector3 value, Vector3 start, Vector3 end)
    {
        var segment       = end - start;
        var lengthSquared = segment.LengthSquared();
        if (lengthSquared <= DUPLICATE_WAYPOINT_DISTANCE_SQ)
            return Vector3.Distance(value, start);

        var progress  = Math.Clamp(Vector3.Dot(value - start, segment) / lengthSquared, 0f, 1f);
        var projected = start + progress * segment;
        return Vector3.Distance(value, projected);
    }

    private static float HorizontalDistanceXZ(Vector3 left, Vector3 right)
    {
        var dx = left.X           - right.X;
        var dz = left.Z           - right.Z;
        return MathF.Sqrt(dx * dx + dz * dz);
    }

    private static List<Vector3> DeduplicateWaypoints(IEnumerable<Vector3> points)
    {
        List<Vector3> result = [];

        foreach (var point in points)
        {
            if (result.Count == 0 || Vector3.DistanceSquared(result[^1], point) > DUPLICATE_WAYPOINT_DISTANCE_SQ)
                result.Add(point);
        }

        return result;
    }

    #region 常量

        private const float DUPLICATE_WAYPOINT_DISTANCE_SQ                           = 0.000001f;
    private const float COLLINEAR_WAYPOINT_TOLERANCE                             = 0.01f;
    private const int   MAX_SMOOTH_PATH_POINTS                                   = 102400;
    private const int   WALL_SCAN_DIRECTION_COUNT                                = 24;
    private const float WALL_SCAN_RADIUS                                         = 2.2f;
    private const float WALL_PUSH_TARGET_CLEARANCE                               = 0.90f;
    private const float WALL_PUSH_BASE_MAX_DISTANCE                              = 0.75f;
    private const float WALL_PUSH_MIN_DISTANCE                                   = 0.02f;
    private const float WALL_PUSH_MIN_ASYMMETRY                                  = 0.05f;
    private const float WALL_PUSH_CLEARANCE_FRACTION                             = 0.65f;
    private const float WALL_PUSH_DYNAMIC_WIDTH_NARROW                           = 0.70f;
    private const float WALL_PUSH_DYNAMIC_WIDTH_WIDE                             = 2.00f;
    private const float WALL_PUSH_DYNAMIC_SCALE_MIN                              = 0.30f;
    private const float WALL_PUSH_DYNAMIC_SCALE_MAX                              = 1.90f;
    private const float WALL_PUSH_DYNAMIC_MAX_DISTANCE_WIDE                      = 1.35f;
    private const float WALL_SCAN_MIN_VECTOR_SQ                                  = 0.0001f;
    private const float WALL_SCAN_ORIGIN_INSET_MIN                               = 0.05f;
    private const float WALL_SCAN_ORIGIN_INSET_MAX                               = 0.45f;
    private const float WALL_ENDPOINT_ADJACENT_SCAN_ORIGIN_SCALE                 = 0.12f;
    private const float WALL_ENDPOINT_ADJACENT_SCAN_ORIGIN_MAX                   = 0.12f;
    private const float WALL_STARTPOINT_SCAN_ORIGIN_SCALE                        = 0.45f;
    private const float WALL_STARTPOINT_SCAN_ORIGIN_MIN                          = 0.35f;
    private const float WALL_STARTPOINT_SCAN_ORIGIN_MAX                          = 0.90f;
    private const float WALL_SCAN_ORIGIN_REPROJECT_MAX_DISTANCE                  = 0.85f;
    private const float WALL_ADJUSTED_POINT_REPROJECT_MAX_DISTANCE               = 0.35f;
    private const float WALL_ADJUSTED_POINT_DIRECT_ACCEPT_PROGRESS_EPSILON       = 0.02f;
    private const float WALL_PUSH_FORWARD_REJECTION                              = 0.10f;
    private const float WALL_PUSH_MIN_CORNER_STRENGTH                            = 0.05f;
    private const float WALL_PUSH_MIN_WALL_PRESSURE                              = 0.03f;
    private const float WALL_STRAIGHT_PUSH_MAX_CORNER_STRENGTH                   = 0.20f;
    private const float WALL_STRAIGHT_PUSH_MIN_BALANCE_DISTANCE                  = 0.10f;
    private const float WALL_STRAIGHT_PUSH_MIN_BALANCE_RATIO                     = 0.10f;
    private const float WALL_STRAIGHT_LOW_CLEARANCE_AVG_THRESHOLD                = 0.55f;
    private const float WALL_STRAIGHT_LOW_CLEARANCE_MIN_THRESHOLD                = 0.20f;
    private const float WALL_STRAIGHT_LOW_CLEARANCE_PUSH_SCALE                   = 0.55f;
    private const float WALL_STRAIGHT_LOW_CLEARANCE_MIN_SIDE_GAP                 = 0.04f;
    private const float WALL_STRAIGHT_RESCAN_AVG_CLEARANCE_THRESHOLD             = 0.65f;
    private const float WALL_STRAIGHT_RESCAN_FORWARD_DISTANCE                    = 0.16f;
    private const float WALL_STRAIGHT_RESCAN_LATERAL_SCALE                       = 0.22f;
    private const float WALL_STRAIGHT_RESCAN_LATERAL_MIN                         = 0.08f;
    private const float WALL_STRAIGHT_RESCAN_LATERAL_MAX                         = 0.30f;
    private const float WALL_STRAIGHT_RESCAN_IMPROVEMENT_THRESHOLD               = 0.04f;
    private const float WALL_STRAIGHT_RESCAN_MIN_CLEARANCE_IMPROVEMENT_THRESHOLD = 0.03f;
    private const float WALL_STRAIGHT_RESCAN_ONE_SIDE_COLLAPSE_THRESHOLD         = 0.05f;
    private const float WALL_STRAIGHT_RESCAN_ONE_SIDE_OPEN_THRESHOLD             = 0.85f;
    private const float WALL_STRAIGHT_RESCAN_EDGE_MIN_CLEARANCE_THRESHOLD        = 0.28f;
    private const float WALL_STRAIGHT_RESCAN_EDGE_IMBALANCE_THRESHOLD            = 0.18f;
    private const float WALL_STRAIGHT_INTERIOR_PUSH_SCALE                        = 0.90f;
    private const float WALL_STRAIGHT_PREFERRED_PUSH_SCALE                       = 0.30f;
    private const float WALL_STRAIGHT_PRESSURE_PUSH_SCALE                        = 0.20f;
    private const float WALL_ENDPOINT_ADJACENT_BALANCE_DISTANCE_SCALE            = 0.60f;
    private const float WALL_ENDPOINT_ADJACENT_BALANCE_RATIO_SCALE               = 0.75f;
    private const float WALL_START_ADJACENT_PRESSURE_PUSH_SCALE                  = 0.90f;
    private const float WALL_START_ADJACENT_RESCAN_AVG_CLEARANCE_THRESHOLD       = 0.55f;
    private const float WALL_START_ADJACENT_RESCAN_FORWARD_SCALE                 = 0.60f;
    private const float WALL_START_ADJACENT_RESCAN_FORWARD_MIN                   = 0.45f;
    private const float WALL_START_ADJACENT_RESCAN_FORWARD_MAX                   = 1.25f;
    private const float WALL_START_ADJACENT_RESCAN_LATERAL_SCALE                 = 0.45f;
    private const float WALL_START_ADJACENT_RESCAN_LATERAL_MIN                   = 0.12f;
    private const float WALL_START_ADJACENT_RESCAN_LATERAL_MAX                   = 0.55f;
    private const float FLIGHT_DESCENT_PRESERVE_MIN_DROP                         = 1.00f;
    private const float FLIGHT_DESCENT_PRESERVE_NEAR_VERTICAL_HORIZONTAL         = 1.20f;
    private const float FLIGHT_DESCENT_PRESERVE_MAX_SLOPE                        = 0.90f;

    #endregion
}
