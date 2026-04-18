namespace vnavmesh.Navigation.Volume;

internal readonly record struct VolumeSearchTelemetry
(
    int                    VisitedNodes,
    int                    GeneratedNodes,
    int                    LineOfSightChecks,
    int                    LineOfSightHits,
    int                    PeakOpenListSize,
    VolumeSearchTermination Termination,
    bool                   SearchRaycastEnabled,
    int                    SearchAttempts,
    float                  HeuristicWeight
);
