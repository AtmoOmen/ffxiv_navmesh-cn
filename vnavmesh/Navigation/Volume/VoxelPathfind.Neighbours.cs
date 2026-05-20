using System.Numerics;
using vnavmesh.Common.Navigation.Volume.Map;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private List<ulong> CollectNeighbours(ulong voxel)
    {
        neighbourScratch.Clear();

        var encodedVoxel = voxel;
        var l0Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l1Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l2Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l0Coords     = l0Desc.IndexToVoxel(l0Index);
        var l1Coords     = l1Index != VoxelMap.INDEX_LEVEL_MASK ? l1Desc.IndexToVoxel(l1Index) : default;
        var l2Coords     = l2Index != VoxelMap.INDEX_LEVEL_MASK ? l2Desc.IndexToVoxel(l2Index) : default;

        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  -1, 0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  +1, 0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, -1, 0,  0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, +1, 0,  0);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  0,  -1);
        CollectDirection(l0Index, l1Index, l2Index, l0Coords, l1Coords, l2Coords, 0,  0,  +1);

        return neighbourScratch;
    }

    private void CollectDirection
    (
        ushort                l0Index,
        ushort                l1Index,
        ushort                l2Index,
        (int x, int y, int z) l0Coords,
        (int x, int y, int z) l1Coords,
        (int x, int y, int z) l2Coords,
        int                   dx,
        int                   dy,
        int                   dz
    )
    {
        var coarseOnlyLongRangeSearch = ShouldRestrictLongRangeSearchToCoarseLevels();

        if (TryAddPreferredCoarseNeighbour(l0Index, l1Index, l0Coords, l1Coords, dx, dy, dz))
            return;

        if (allowCoarseL1Stepping && !coarseOnlyLongRangeSearch && l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);

            if (l1Desc.InBounds(l1Neighbour))
            {
                var l1NeighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                l1NeighbourVoxel = VoxelMap.EncodeIndex(l0Index, l1NeighbourVoxel);

                if (Volume.IsEmpty(l1NeighbourVoxel))
                    AddNeighbourIfEmpty(l1NeighbourVoxel);
            }
            else
            {
                var coarseL0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);

                if (l0Desc.InBounds(coarseL0Neighbour))
                {
                    var coarseL0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(coarseL0Neighbour));

                    if (Volume.IsEmpty(coarseL0NeighbourVoxel))
                        AddNeighbourIfEmpty(coarseL0NeighbourVoxel);
                    else
                    {
                        var l1X          = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
                        var l1Y          = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
                        var l1Z          = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
                        var crossL1Voxel = VoxelMap.EncodeSubIndex(coarseL0NeighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);

                        if (Volume.IsEmpty(crossL1Voxel))
                            AddNeighbourIfEmpty(crossL1Voxel);
                    }
                }
            }
        }

        if (!coarseOnlyLongRangeSearch && l2Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);

            if (l2Desc.InBounds(l2Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                AddNeighbourIfEmpty(neighbourVoxel);
                return;
            }
        }

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);

            if (l1Desc.InBounds(l1Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);

                if (Volume.IsEmpty(neighbourVoxel)) AddNeighbourIfEmpty(neighbourVoxel);
                else if (!coarseOnlyLongRangeSearch && l2Index != VoxelMap.INDEX_LEVEL_MASK)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    AddNeighbourIfEmpty(l2NeighbourVoxel);
                }
                else if (!coarseOnlyLongRangeSearch)
                    CollectBorder(neighbourVoxel, l2Desc, 2, dx, dy, dz);

                return;
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!l0Desc.InBounds(l0Neighbour))
            return;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));

        if (Volume.IsEmpty(l0NeighbourVoxel))
        {
            AddNeighbourIfEmpty(l0NeighbourVoxel);
            return;
        }

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1X              = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
            var l1Y              = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
            var l1Z              = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
            var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(l0NeighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);

            if (Volume.IsEmpty(l1NeighbourVoxel)) AddNeighbourIfEmpty(l1NeighbourVoxel);
            else if (!coarseOnlyLongRangeSearch && l2Index != VoxelMap.INDEX_LEVEL_MASK)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                AddNeighbourIfEmpty(l2NeighbourVoxel);
            }
            else if (!coarseOnlyLongRangeSearch)
                CollectBorder(l1NeighbourVoxel, l2Desc, 2, dx, dy, dz);

            return;
        }

        if (coarseOnlyLongRangeSearch) CollectBorderWithSubdivisionsL1Only(l0NeighbourVoxel, dx, dy, dz);
        else CollectBorderWithSubdivisions(l0NeighbourVoxel, dx, dy, dz);
    }

    private bool TryAddPreferredCoarseNeighbour
    (
        ushort                l0Index,
        ushort                l1Index,
        (int x, int y, int z) l0Coords,
        (int x, int y, int z) l1Coords,
        int                   dx,
        int                   dy,
        int                   dz
    )
    {
        if (!ShouldPreferCoarseDescentNeighbour(dx, dy, dz))
            return false;

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);

            if (l1Desc.InBounds(l1Neighbour))
            {
                var l1NeighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                l1NeighbourVoxel = VoxelMap.EncodeIndex(l0Index, l1NeighbourVoxel);

                if (CanUsePreferredCoarseNeighbour(l1NeighbourVoxel, dy))
                {
                    AddNeighbourIfEmpty(l1NeighbourVoxel);
                    return true;
                }
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!l0Desc.InBounds(l0Neighbour))
            return false;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));
        if (!CanUsePreferredCoarseNeighbour(l0NeighbourVoxel, dy))
            return false;

        AddNeighbourIfEmpty(l0NeighbourVoxel);
        return true;
    }

    private bool ShouldPreferCoarseDescentNeighbour(int dx, int dy, int dz)
    {
        if (!longRangeLateralBias.Enabled || !longRangeLateralBias.PreferDescending || dy > 0)
            return false;

        if (dy < 0)
            return true;

        var horizontal = new Vector2(dx, dz);
        if (!TryNormalize(horizontal, out var direction))
            return false;

        return Vector2.Dot(direction, longRangeLateralBias.Forward) >= LONG_RANGE_LATERAL_COARSE_PROMOTION_MIN_FORWARD_DOT;
    }

    private bool CanUsePreferredCoarseNeighbour(ulong voxel, int dy)
    {
        if (!Volume.IsEmpty(voxel))
            return false;

        return dy < 0 ? HasVerifiedTopEntry(voxel) : HasDownwardOpening(voxel);
    }

    private bool ShouldRestrictLongRangeSearchToCoarseLevels() => longRangeLateralBias.Enabled;

    private void CollectBorder(ulong voxel, VolumeLevel levelDesc, int level, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, levelDesc.NumCellsX - 1) : dx > 0 ? (0, 0) : (levelDesc.NumCellsX - 1, levelDesc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, levelDesc.NumCellsY - 1) : dy > 0 ? (0, 0) : (levelDesc.NumCellsY - 1, levelDesc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, levelDesc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (levelDesc.NumCellsZ - 1, levelDesc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
            AddNeighbourIfEmpty(VoxelMap.EncodeSubIndex(voxel, levelDesc.VoxelToIndex(x, y, z), level));
    }

    private void CollectBorderWithSubdivisions(ulong voxel, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (l1Desc.NumCellsX - 1, l1Desc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (l1Desc.NumCellsY - 1, l1Desc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (l1Desc.NumCellsZ - 1, l1Desc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel)) AddNeighbourIfEmpty(l1Voxel);
            else CollectBorder(l1Voxel, l2Desc, 2, dx, dy, dz);
        }
    }

    private void CollectBorderWithSubdivisionsL1Only(ulong voxel, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (l1Desc.NumCellsX - 1, l1Desc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (l1Desc.NumCellsY - 1, l1Desc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (l1Desc.NumCellsZ - 1, l1Desc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel))
                AddNeighbourIfEmpty(l1Voxel);
        }
    }

    private void AddNeighbourIfEmpty(ulong voxel)
    {
        if (l1CorridorDistance is not null)
        {
            if (!TryGetVoxelCorridorDistance(voxel, out var distance) || distance > currentL1CorridorRadius)
                return;
        }
        else if (!IsVoxelInsidePathConstraint(voxel))
            return;

        if (Volume.IsEmpty(voxel))
            neighbourScratch.Add(voxel);
    }
}
