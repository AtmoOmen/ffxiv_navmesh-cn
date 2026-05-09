using vnavmesh.Common.Navigation.Mesh.Build;

namespace vnavmesh.Navigation.Mesh.Build;

public static class NavmeshBuildSettingsConversion
{
    public static NavmeshBuildSettings ToBuildSettings(this NavmeshSettings settings, bool flyable, int customizationVersion) =>
        new()
        {
            CellSize                   = settings.CellSize,
            CellHeight                 = settings.CellHeight,
            AgentHeight                = settings.AgentHeight,
            AgentRadius                = settings.AgentRadius,
            AgentMaxClimb              = settings.AgentMaxClimb,
            AgentMaxSlopeDeg           = settings.AgentMaxSlopeDeg,
            Filtering                  = (NavmeshBuildSettings.Filter)settings.Filtering,
            RegionMinSize              = settings.RegionMinSize,
            RegionMergeSize            = settings.RegionMergeSize,
            Partitioning               = settings.Partitioning,
            PolyMaxEdgeLen             = settings.PolyMaxEdgeLen,
            PolyMaxSimplificationError = settings.PolyMaxSimplificationError,
            PolyMaxVerts               = settings.PolyMaxVerts,
            DetailSampleDist           = settings.DetailSampleDist,
            DetailMaxSampleError       = settings.DetailMaxSampleError,
            FastBuild                  = settings.FastBuild,
            GenerateEdgeClimbLinks     = settings.GenerateEdgeClimbLinks,
            GenerateEdgeJumpLinks      = settings.GenerateEdgeJumpLinks,
            GroundTolerance            = settings.GroundTolerance,
            ClimbDownDistance          = settings.ClimbDownDistance,
            ClimbDownMaxHeight         = settings.ClimbDownMaxHeight,
            ClimbDownMinHeight         = settings.ClimbDownMinHeight,
            EdgeJumpEndDistance        = settings.EdgeJumpEndDistance,
            EdgeJumpHeight             = settings.EdgeJumpHeight,
            EdgeJumpMaxDrop            = settings.EdgeJumpMaxDrop,
            EdgeJumpMinDrop            = settings.EdgeJumpMinDrop,
            GroundTileSize             = settings.GroundTileSize,
            GroundTileCountMax         = settings.GroundTileCountMax,
            VolumeTiles                = (int[])settings.VolumeTiles.Clone(),
            BuildMaxCores              = 0,
            Flyable                    = flyable,
            CustomizationVersion       = customizationVersion
        };
}
