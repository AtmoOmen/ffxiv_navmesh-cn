using System.Numerics;

namespace vnavmesh.Navigation.Volume;

internal readonly record struct VolumeNeighbourEvaluation
(
    ulong   Voxel,
    int     BestParentIndex,
    Vector3 BestPosition,
    float   BestScore
);
