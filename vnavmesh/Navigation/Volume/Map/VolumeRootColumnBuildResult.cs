namespace vnavmesh.Navigation.Volume;

public sealed class VolumeRootColumnBuildResult
{
    public required ushort[]        Contents    { get; init; }
    public required List<VolumeTile> Subdivision { get; init; }
}
