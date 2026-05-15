namespace vnavmesh.Navigation.Volume.Pathfinding;

internal readonly record struct VolumeSearchTelemetry
(
    int                     VisitedNodes,
    int                     GeneratedNodes,
    int                     LineOfSightChecks,
    int                     LineOfSightHits,
    int                     PeakOpenListSize,
    VolumeSearchTermination Termination,
    int                     SearchAttempts,
    float                   HeuristicWeight
);
