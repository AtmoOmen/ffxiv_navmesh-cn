using System.Numerics;

namespace vnavmesh.Navigation.Volume.Pathfinding;

internal readonly record struct VolumeNeighbourEvaluation
(
    ulong   Voxel,
    int     BestParentIndex,
    Vector3 BestPosition,
    float   BestScore
);
