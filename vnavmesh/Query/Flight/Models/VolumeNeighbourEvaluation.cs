using System.Numerics;

namespace vnavmesh.Query.Flight.Models;

internal readonly record struct VolumeNeighbourEvaluation
(
    ulong   Voxel,
    int     BestParentIndex,
    Vector3 BestPosition,
    float   BestScore
);
