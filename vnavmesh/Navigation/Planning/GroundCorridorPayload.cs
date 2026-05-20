using System.Numerics;
using vnavmesh.Common.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Planning;

internal readonly record struct GroundPathCorner
(
    Vector3             Position,
    long                PolyRef,
    byte                StraightPathFlags,
    NavmeshArea         Area,
    NavmeshOffMeshKind? LinkKind,
    GroundPathCornerDebug? Debug
);

internal readonly record struct GroundLinkMarker
(
    int                CornerIndex,
    Vector3            Position,
    long               PolyRef,
    NavmeshOffMeshKind Kind,
    NavmeshLinkTraversalProfile? TraversalProfile = null,
    float               EstimatedTraversalCost = 0f
);

internal readonly record struct GroundPathCornerDebugSample
(
    Vector3 Start,
    Vector3 Endpoint,
    float   Clearance,
    long    PolyRef
);

internal readonly record struct GroundPathCornerDebug
(
    int                                         StraightPathIndex,
    bool                                        InitiallyConsumed,
    bool                                        IsExecutionStart,
    long                                        ScanPolyRef,
    int                                         LocalPolyCount,
    long                                        PreferredPolyRef,
    long                                        LeftPolyRef,
    long                                        RightPolyRef,
    bool                                        Rescanned,
    bool                                        UsedInteriorDirection,
    float                                       DynamicPushWidth,
    float                                       DynamicPushScale,
    float                                       RawPushDistance,
    float                                       DynamicPushMaxDistance,
    float                                       LeftClearance,
    float                                       RightClearance,
    bool                                        StraightBalanceSatisfied,
    bool                                        StraightLowClearanceCase,
    Vector3                                     OriginalPosition,
    Vector3                                     ScanOrigin,
    Vector3                                     AdjustedPosition,
    Vector3                                     InteriorDirectionEndpoint,
    Vector3                                     PreferredDirectionEndpoint,
    Vector3                                     WallPressureEndpoint,
    float                                       PushDistance,
    float                                       MinClearance,
    float                                       MaxClearance,
    float                                       AverageClearance,
    float                                       CornerStrength,
    IReadOnlyList<GroundPathCornerDebugSample>  Samples,
    NavmeshLinkTraversalProfile?                TraversalProfile = null,
    float                                       TraversalCost = 0f
)
{
    public bool PushApplied => PushDistance > 0.0001f;
}

internal sealed class GroundCorridorPayload
{
    public required IReadOnlyList<long>             PolyRefs    { get; init; }
    public required Vector3                         Target      { get; init; }
    public required int                             InitialWaypointIndex { get; init; }
    public required int                             InitialCornerIndex   { get; init; }
    public required IReadOnlyList<GroundPathCorner> Corners     { get; init; }
    public required IReadOnlyList<GroundLinkMarker> LinkMarkers { get; init; }
}
