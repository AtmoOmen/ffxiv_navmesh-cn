using vnavmesh.Common.Navigation.Volume.Map;

namespace vnavmesh.Navigation.Volume.Utils;

public static class VoxelIndexUtil
{
    public static ushort ExtractL0Index(ulong voxel)
    {
        var temp = voxel;
        return VoxelMap.DecodeIndex(ref temp);
    }

    public static bool IsCoarseNeighbour(ulong voxel)
    {
        var temp = voxel;
        VoxelMap.DecodeIndex(ref temp);
        VoxelMap.DecodeIndex(ref temp);
        return VoxelMap.DecodeIndex(ref temp) == VoxelMap.INDEX_LEVEL_MASK;
    }

    public static bool TryExtractL1Parent(ulong voxel, out ulong l1Voxel)
    {
        var temp = voxel;
        var l0   = VoxelMap.DecodeIndex(ref temp);
        var l1   = VoxelMap.DecodeIndex(ref temp);

        if (l1 == VoxelMap.INDEX_LEVEL_MASK)
        {
            l1Voxel = VoxelMap.INVALID_VOXEL;
            return false;
        }

        l1Voxel = VoxelMap.EncodeIndex(l1);
        l1Voxel = VoxelMap.EncodeIndex(l0, l1Voxel);
        return true;
    }

    public static bool TryExtractL2IndexWithinL1(ulong voxel, ulong expectedL1, out ushort l2Index)
    {
        var temp    = voxel;
        var l0Index = VoxelMap.DecodeIndex(ref temp);
        var l1Index = VoxelMap.DecodeIndex(ref temp);
        l2Index = VoxelMap.DecodeIndex(ref temp);
        if (l2Index == VoxelMap.INDEX_LEVEL_MASK)
            return false;

        var l1Voxel = VoxelMap.EncodeIndex(l1Index);
        l1Voxel = VoxelMap.EncodeIndex(l0Index, l1Voxel);
        return l1Voxel == expectedL1;
    }

    public static ushort GetPackedReachableL1Faces(ulong packedConnectivity, int face)
        => (ushort)((packedConnectivity >> (face * 6)) & 0x3f);

    public static ulong SetPackedReachableL1Faces(ulong packedConnectivity, int face, ushort reachableFaces)
    {
        var shift = face * 6;
        packedConnectivity &= ~((ulong)0x3f << shift);
        packedConnectivity |= (ulong)(reachableFaces & 0x3f) << shift;
        return packedConnectivity;
    }

    public static bool CanTraverseMixedL1Cell(bool includeNonEmpty, ulong candidateL1, ulong goalL1)
        => includeNonEmpty && candidateL1 == goalL1;

    public static bool HasL1Face(ushort faceMask, int face)
        => (faceMask & (1 << face)) != 0;
}
