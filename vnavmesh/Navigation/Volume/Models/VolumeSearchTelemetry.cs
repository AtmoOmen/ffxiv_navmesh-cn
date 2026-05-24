namespace vnavmesh.Navigation.Volume.Models;

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
