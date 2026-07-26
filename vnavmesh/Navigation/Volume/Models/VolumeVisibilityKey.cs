using System.Numerics;

namespace vnavmesh.Navigation.Volume.Models;

internal readonly record struct VolumeVisibilityKey
{
    public ulong   FromVoxel    { get; }
    public ulong   ToVoxel      { get; }
    public Vector3 FromPosition { get; }
    public Vector3 ToPosition   { get; }

    public VolumeVisibilityKey(ulong fromVoxel, ulong toVoxel, Vector3 fromPosition, Vector3 toPosition)
    {
        if (CompareEndpoints(fromVoxel, fromPosition, toVoxel, toPosition) <= 0)
        {
            FromVoxel    = fromVoxel;
            ToVoxel      = toVoxel;
            FromPosition = fromPosition;
            ToPosition   = toPosition;
        }
        else
        {
            FromVoxel    = toVoxel;
            ToVoxel      = fromVoxel;
            FromPosition = toPosition;
            ToPosition   = fromPosition;
        }
    }

    private static int CompareEndpoints(ulong leftVoxel, Vector3 leftPosition, ulong rightVoxel, Vector3 rightPosition)
    {
        var voxelComparison = leftVoxel.CompareTo(rightVoxel);
        if (voxelComparison != 0)
            return voxelComparison;

        var xComparison = BitConverter.SingleToInt32Bits(leftPosition.X).CompareTo(BitConverter.SingleToInt32Bits(rightPosition.X));
        if (xComparison != 0)
            return xComparison;

        var yComparison = BitConverter.SingleToInt32Bits(leftPosition.Y).CompareTo(BitConverter.SingleToInt32Bits(rightPosition.Y));
        if (yComparison != 0)
            return yComparison;

        return BitConverter.SingleToInt32Bits(leftPosition.Z).CompareTo(BitConverter.SingleToInt32Bits(rightPosition.Z));
    }
}
