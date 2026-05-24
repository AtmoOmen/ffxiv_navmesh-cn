using System.Numerics;

namespace vnavmesh.Navigation.Volume.Models;

internal readonly record struct VolumeNeighbourEvaluation
(
    ulong   Voxel,
    int     BestParentIndex,
    Vector3 BestPosition,
    float   BestScore
);
