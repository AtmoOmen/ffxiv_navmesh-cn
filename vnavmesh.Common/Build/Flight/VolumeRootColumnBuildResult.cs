namespace vnavmesh.Common.Build.Flight;

public sealed class VolumeRootColumnBuildResult
{
    public required ushort[]         Contents    { get; init; }
    public required List<VolumeTile> Subdivision { get; init; }
    public required List<(ulong Voxel, float TopY)> SurfaceTops { get; init; }
}
