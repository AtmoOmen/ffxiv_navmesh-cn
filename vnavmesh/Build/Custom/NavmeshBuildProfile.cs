using DotRecast.Recast;

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
    public float?       VolumeLeafSizeOverride;
    public int?         VolumeMaxDepthOverride;
    public int[]?       VolumeLayerDepthsOverride;
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
        if (VolumeLeafSizeOverride is { } volumeLeafSize)
            settings.VolumeLeafSize = volumeLeafSize;
        if (VolumeMaxDepthOverride is { } volumeMaxDepth)
            settings.VolumeMaxDepth = volumeMaxDepth;
        if (VolumeLayerDepthsOverride is { } volumeLayerDepths)
            settings.VolumeLayerDepths = (int[])volumeLayerDepths.Clone();
        if (DetailSampleDistOverride is { } detailSampleDist)
            settings.DetailSampleDist = detailSampleDist;
        if (GenerateEdgeClimbLinksOverride is { } generateEdgeClimbLinks)
            settings.GenerateEdgeClimbLinks = generateEdgeClimbLinks;
        if (GenerateEdgeJumpLinksOverride is { } generateEdgeJumpLinks)
            settings.GenerateEdgeJumpLinks = generateEdgeJumpLinks;
    }
}
