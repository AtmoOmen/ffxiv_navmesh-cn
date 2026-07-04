namespace vnavmesh.Navigation.Volume.Models;

internal readonly record struct VolumeVisibilityKey
(
    ulong FromVoxel,
    ulong ToVoxel
);
