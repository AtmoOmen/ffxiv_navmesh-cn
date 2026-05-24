namespace vnavmesh.Navigation.Volume.Models;

internal readonly record struct VolumeVisibilityKey
(
    int                     FromNodeIndex,
    int                     FromNodeRevision,
    ulong                   ToVoxel,
    VolumePathCandidateKind CandidateKind
);
