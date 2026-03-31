using System.Collections.Generic;
using DotRecast.Core.Numerics;

namespace Navmesh;

public sealed class MeshCorridor(RcVec3f start, RcVec3f end, IReadOnlyList<long> polygons, IReadOnlyList<MeshPortal> portals)
{
    public RcVec3f Start { get; } = start;
    public RcVec3f End { get; } = end;
    public IReadOnlyList<long> Polygons { get; } = polygons;
    public IReadOnlyList<MeshPortal> Portals { get; } = portals;
}
