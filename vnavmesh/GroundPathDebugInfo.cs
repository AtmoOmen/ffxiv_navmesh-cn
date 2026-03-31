using System.Collections.Generic;
using System.Numerics;

namespace Navmesh;

public readonly record struct DebugLineSegment(Vector3 From, Vector3 To);
public readonly record struct DebugPortalSegment(Vector3 From, Vector3 To, bool IsNarrow, bool IsProtectedAnchor, float Width, float EffectiveClearance);

public sealed class GroundPathDebugInfo
{
    public List<Vector3> CorridorCenters { get; init; } = [];
    public List<DebugPortalSegment> RawPortals { get; init; } = [];
    public List<DebugPortalSegment> TrimmedPortals { get; init; } = [];
    public List<Vector3> Centerline { get; init; } = [];
    public List<Vector3> FinalPath { get; init; } = [];
    public List<int> ProtectedPointIndices { get; init; } = [];
    public bool IsPartial { get; set; }
    public Vector3 RequestedEnd { get; set; }
    public Vector3 ResolvedEnd { get; set; }
    public string PathStatusText { get; set; } = "";
    public long ReachedEndRef { get; set; }
}
