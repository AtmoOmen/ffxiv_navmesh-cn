namespace vnavmesh.Navigation.Volume;

internal readonly record struct VolumeVisibilityKey
(
    int                     FromNodeIndex,
    int                     FromNodeRevision,
    ulong                   ToVoxel,
    VolumePathCandidateKind CandidateKind
);
