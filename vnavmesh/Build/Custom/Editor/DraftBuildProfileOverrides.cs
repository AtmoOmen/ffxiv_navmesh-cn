using DotRecast.Recast;

namespace vnavmesh.Build.Custom.Editor;

public sealed class DraftBuildProfileOverrides
{
    public RcPartition? PartitioningOverride;
    public float?       CellSizeOverride;
    public float?       CellHeightOverride;
    public float?       RegionMinSizeOverride;
    public float?       RegionMergeSizeOverride;
    public float?       PolyMaxEdgeLenOverride;
    public float?       PolyMaxSimplificationErrorOverride;
    public float?       AgentRadiusOverride;
    public float?       VolumeLeafSizeOverride;
    public int?         VolumeMaxDepthOverride;
    public int[]?       VolumeLayerDepthsOverride;
    public float?       DetailSampleDistOverride;
    public bool?        GenerateEdgeClimbLinksOverride;
    public bool?        GenerateEdgeJumpLinksOverride;
}
