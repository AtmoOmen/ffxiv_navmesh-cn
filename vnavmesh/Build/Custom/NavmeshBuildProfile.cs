using DotRecast.Recast;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom;

public sealed class NavmeshBuildProfile
{
    public RcPartition? PartitioningOverride;
    public float?       CellSizeOverride;
    public float?       CellHeightOverride;
    public float?       RegionMinSizeOverride;
    public float?       RegionMergeSizeOverride;
    public float?       PolyMaxEdgeLenOverride;
    public float?       PolyMaxSimplificationErrorOverride;
    public float?       AgentRadiusOverride;
    public int[]?       VolumeTilesOverride;
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

    public void ApplyTo
    (
        NavmeshSettings settings
    )
    {
        if (PartitioningOverride is { } partitioning)
            settings.Partitioning = partitioning;
        if (CellSizeOverride is { } cellSize)
            settings.CellSize = cellSize;
        if (CellHeightOverride is { } cellHeight)
            settings.CellHeight = cellHeight;
        if (RegionMinSizeOverride is { } regionMinSize)
            settings.RegionMinSize = regionMinSize;
        if (RegionMergeSizeOverride is { } regionMergeSize)
            settings.RegionMergeSize = regionMergeSize;
        if (PolyMaxEdgeLenOverride is { } polyMaxEdgeLen)
            settings.PolyMaxEdgeLen = polyMaxEdgeLen;
        if (PolyMaxSimplificationErrorOverride is { } polyMaxSimplificationError)
            settings.PolyMaxSimplificationError = polyMaxSimplificationError;
        if (AgentRadiusOverride is { } agentRadius)
            settings.AgentRadius = agentRadius;
        if (VolumeTilesOverride is { } volumeTiles)
            settings.VolumeTiles = (int[])volumeTiles.Clone();
        if (VolumeVerticalPaddingOverride is { } volumeVerticalPadding)
            settings.VolumeVerticalPadding = volumeVerticalPadding;
        if (VolumeWallThickenNormalYThresholdOverride is { } volumeWallThickenNormalYThreshold)
            settings.VolumeWallThickenNormalYThreshold = volumeWallThickenNormalYThreshold;
        if (VolumeWallThickenHorizontalRadiusOverride is { } volumeWallThickenHorizontalRadius)
            settings.VolumeWallThickenHorizontalRadius = volumeWallThickenHorizontalRadius;
        if (VolumeThinWallStripNormalYThresholdOverride is { } volumeThinWallStripNormalYThreshold)
            settings.VolumeThinWallStripNormalYThreshold = volumeThinWallStripNormalYThreshold;
        if (VolumeThinWallStripMaxProjectedThicknessOverride is { } volumeThinWallStripMaxProjectedThickness)
            settings.VolumeThinWallStripMaxProjectedThickness = volumeThinWallStripMaxProjectedThickness;
        if (VolumeThinWallStripBaseRadiusOverride is { } volumeThinWallStripBaseRadius)
            settings.VolumeThinWallStripBaseRadius = volumeThinWallStripBaseRadius;
        if (VolumeThinWallStripExtraPaddingOverride is { } volumeThinWallStripExtraPadding)
            settings.VolumeThinWallStripExtraPadding = volumeThinWallStripExtraPadding;
        if (DetailSampleDistOverride is { } detailSampleDist)
            settings.DetailSampleDist = detailSampleDist;
        if (GenerateEdgeClimbLinksOverride is { } generateEdgeClimbLinks)
            settings.GenerateEdgeClimbLinks = generateEdgeClimbLinks;
        if (GenerateEdgeJumpLinksOverride is { } generateEdgeJumpLinks)
            settings.GenerateEdgeJumpLinks = generateEdgeJumpLinks;
    }
}
