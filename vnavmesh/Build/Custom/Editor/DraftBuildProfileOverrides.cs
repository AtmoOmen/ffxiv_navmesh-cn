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
    public float?       VolumeCellSizeOverride;
    public float?       VolumeVerticalPaddingOverride;
    public float?       VolumeWallThickenNormalYThresholdOverride;
    public int?         VolumeWallThickenHorizontalRadiusOverride;
    public float?       VolumeThinWallStripNormalYThresholdOverride;
    public float?       VolumeThinWallStripMaxProjectedThicknessOverride;
    public float?       VolumeThinWallStripBaseRadiusOverride;
    public float?       VolumeThinWallStripExtraPaddingOverride;
    public float?       DetailSampleDistOverride;
    public bool?        GenerateEdgeClimbLinksOverride;
    public bool?        GenerateEdgeJumpLinksOverride;
}
