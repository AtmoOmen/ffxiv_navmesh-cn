using System.Numerics;

namespace vnavmesh.Navigation.Planning;

internal enum FlightPathDebugSampleKind : byte
{
    Forward,
    Backward,
    Right,
    Left,
    ForwardRight,
    ForwardLeft,
    BackwardRight,
    BackwardLeft,
    Sweep,
    Up,
    Down
}

internal readonly record struct FlightPathDebugSample
(
    FlightPathDebugSampleKind Kind,
    Vector3                   Start,
    Vector3                   Endpoint,
    float                     Clearance
);

internal enum FlightPathVerticalMode : byte
{
    None,
    TunnelDescentAssist,
    UpwardBias,
    HeightCatchUp,
    DownwardBias,
    FloorAvoidance
}

internal enum FlightPathAdjustmentKind : byte
{
    None,
    UpwardCombined,
    UpwardVerticalOnly,
    UpwardVerticalBlend,
    UpwardHorizontalOnly,
    TunnelCombinedStrong,
    TunnelVerticalOnly,
    TunnelDirectionalFixedVertical,
    TunnelCombinedMedium,
    TunnelAfterVerticalDrop,
    TunnelCombined,
    TunnelHorizontalOnly,
    DownwardFixedVertical,
    DownwardDirectionalFixedVertical,
    DownwardAfterVerticalDrop,
    DownwardStrongBlend,
    DownwardCombined,
    DownwardMediumBlend,
    DownwardHorizontalOnly,
    DownwardVerticalOnly,
    NeutralCombined,
    NeutralHorizontalOnly,
    NeutralVerticalOnly
}

internal readonly record struct FlightPathWaypointDebug
(
    int                               PathIndex,
    ulong                             OriginalVoxel,
    ulong                             AdjustedVoxel,
    Vector3                           OriginalPosition,
    Vector3                           AdjustedPosition,
    Vector3                           HorizontalBiasEndpoint,
    Vector3                           VerticalBiasEndpoint,
    Vector3                           CombinedBiasEndpoint,
    float                             HorizontalPushDistance,
    float                             VerticalPushDistance,
    float                             PushDistance,
    float                             HorizontalImbalance,
    float                             VerticalImbalance,
    float                             ForwardClearance,
    float                             BackwardClearance,
    float                             LeftClearance,
    float                             RightClearance,
    float                             ForwardLeftClearance,
    float                             ForwardRightClearance,
    float                             BackwardLeftClearance,
    float                             BackwardRightClearance,
    float                             UpClearance,
    float                             DownClearance,
    float                             MaxClearance,
    FlightPathVerticalMode            VerticalMode,
    FlightPathAdjustmentKind          SelectedAdjustmentKind,
    bool                              GoalDescentApproach,
    bool                              DownhillTunnelTrend,
    bool                              ConstrainedTunnelDescent,
    bool                              TunnelDescentAssist,
    bool                              HeightCatchUpRequested,
    bool                              AllowDownwardPush,
    bool                              FinalRaiseApplied,
    float                             HeightMatchTarget,
    float                             PreferredMinHeight,
    Vector3                           BaseAdjustedPosition,
    IReadOnlyList<FlightPathDebugSample> Samples
)
{
    public bool PushApplied => PushDistance > 0.0001f;
}

internal sealed class FlightPathDebugPayload
{
    public required IReadOnlyList<FlightPathWaypointDebug> Waypoints { get; init; }
}
