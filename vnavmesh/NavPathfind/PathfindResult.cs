using System.Collections.Generic;
using System.Numerics;

namespace vnavmesh.NavPathfind;

internal readonly record struct PathfindResult(PathfindStatus Status, List<Vector3> Waypoints, Vector3 RequestedDestination, Vector3 FinalDestination)
{
    public bool Succeeded => Status != PathfindStatus.Failed;
}