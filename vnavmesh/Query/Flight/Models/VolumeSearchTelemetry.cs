namespace vnavmesh.Query.Flight.Models;

internal readonly record struct VolumeSearchTelemetry
(
    int                     VisitedNodes,
    int                     GeneratedNodes,
    int                     LineOfSightChecks,
    int                     LineOfSightHits,
    int                     PeakOpenListSize,
    int                     CoarseExpandedNodes,
    VolumeSearchTermination Termination,
    int                     SearchAttempts,
    float                   HeuristicWeight
);
