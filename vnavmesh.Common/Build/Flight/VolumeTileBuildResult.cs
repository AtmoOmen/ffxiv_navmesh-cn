namespace vnavmesh.Common.Build.Flight;

public sealed class VolumeTileBuildResult
{
    public required OctreeNodePool                Pool  { get; init; }
    public required int                           Depth { get; init; }
    public required List<(int YCube, int RootNode)> Roots { get; init; }
}
