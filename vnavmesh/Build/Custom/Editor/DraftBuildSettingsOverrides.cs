using DotRecast.Recast;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Build.Ground.Enums;

namespace vnavmesh.Build.Custom.Editor;

public sealed class DraftBuildSettingsOverrides
{
    public float?         CellSize;
    public float?         CellHeight;
    public float?         AgentHeight;
    public float?         AgentRadius;
    public float?         AgentMaxClimb;
    public float?         AgentMaxSlopeDeg;
    public NavmeshFilter? Filtering;
    public float?         RegionMinSize;
    public float?         RegionMergeSize;
    public RcPartition?   Partitioning;
    public float?         PolyMaxEdgeLen;
    public float?         PolyMaxSimplificationError;
    public int?           PolyMaxVerts;
    public float?         DetailSampleDist;
    public float?         DetailMaxSampleError;
    public bool?          FastBuild;
    public bool?          GenerateEdgeClimbLinks;
    public bool?          GenerateEdgeJumpLinks;
    public float?         GroundTolerance;
    public float?         ClimbDownDistance;
    public float?         ClimbDownMaxHeight;
    public float?         ClimbDownMinHeight;
    public float?         EdgeJumpEndDistance;
    public float?         EdgeJumpHeight;
    public float?         EdgeJumpMaxDrop;
    public float?         EdgeJumpMinDrop;
    public float?         GroundTileSize;
    public int?           GroundTileCountMax;
    public float?         VolumeCellSize;
}
