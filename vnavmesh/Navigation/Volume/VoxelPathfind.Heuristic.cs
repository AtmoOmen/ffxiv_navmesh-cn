using System.Numerics;
using System.Runtime.CompilerServices;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Navigation.Volume.Search;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private static bool IsCoarseNeighbour(ulong voxel)
    {
        var temp = voxel;
        VoxelMap.DecodeIndex(ref temp);
        VoxelMap.DecodeIndex(ref temp);
        return VoxelMap.DecodeIndex(ref temp) == VoxelMap.INDEX_LEVEL_MASK;
    }

    private static bool TryExtractL1Parent(ulong voxel, out ulong l1Voxel)
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

    private bool HasTraversableL1FaceTransition(ulong currentL1, ulong neighbourL1, int dx, int dy, int dz)
    {
        var currentX   = dx > 0 ? l2Desc.NumCellsX     - 1 : 0;
        var currentY   = dy > 0 ? l2Desc.NumCellsY     - 1 : 0;
        var currentZ   = dz > 0 ? l2Desc.NumCellsZ     - 1 : 0;
        var neighbourX = dx > 0 ? 0 : l2Desc.NumCellsX - 1;
        var neighbourY = dy > 0 ? 0 : l2Desc.NumCellsY - 1;
        var neighbourZ = dz > 0 ? 0 : l2Desc.NumCellsZ - 1;

        var xMin = dx == 0 ? 0 : currentX;
        var xMax = dx == 0 ? l2Desc.NumCellsX - 1 : currentX;
        var yMin = dy == 0 ? 0 : currentY;
        var yMax = dy == 0 ? l2Desc.NumCellsY - 1 : currentY;
        var zMin = dz == 0 ? 0 : currentZ;
        var zMax = dz == 0 ? l2Desc.NumCellsZ - 1 : currentZ;

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var otherX = dx == 0 ? x : neighbourX;
            var otherY = dy == 0 ? y : neighbourY;
            var otherZ = dz == 0 ? z : neighbourZ;

            var currentLeaf   = VoxelMap.EncodeSubIndex(currentL1,   l2Desc.VoxelToIndex(x,      y,      z),      2);
            var neighbourLeaf = VoxelMap.EncodeSubIndex(neighbourL1, l2Desc.VoxelToIndex(otherX, otherY, otherZ), 2);
            if (Volume.IsEmpty(currentLeaf) && Volume.IsEmpty(neighbourLeaf))
                return true;
        }

        return false;
    }

    private static int OppositeFace(int face) => face ^ 1;

    private static ushort L1FaceBit(int face) => (ushort)(1 << face);

    private static bool HasL1Face(ushort faceMask, int face) => (faceMask & L1FaceBit(face)) != 0;

    private static bool CanTraverseMixedL1Cell(bool includeNonEmpty, ulong candidateL1, ulong goalL1)
        => includeNonEmpty && candidateL1 == goalL1;

    private static ushort GetPackedReachableL1Faces(ulong packedConnectivity, int face) => (ushort)((packedConnectivity >> (face * 6)) & L1_ALL_FACES_MASK);

    private static ulong SetPackedReachableL1Faces(ulong packedConnectivity, int face, ushort reachableFaces)
    {
        var shift = face * 6;
        packedConnectivity &= ~((ulong)L1_ALL_FACES_MASK << shift);
        packedConnectivity |= (ulong)(reachableFaces & L1_ALL_FACES_MASK) << shift;
        return packedConnectivity;
    }

    private ushort GetReachableL1FacesFromEntry(ulong l1Voxel, int entryFace)
    {
        if (entryFace is < 0 or > 5)
            return 0;
        if (Volume.IsEmpty(l1Voxel))
            return L1_ALL_FACES_MASK;

        var packedConnectivity = l1FaceConnectivityCache.GetOrAdd(l1Voxel, BuildPackedL1FaceConnectivity);
        return GetPackedReachableL1Faces(packedConnectivity, entryFace);
    }

    private ushort GetReachableL1FacesFromPoint(ulong l1Voxel, ulong actualVoxel, Vector3 point)
    {
        if (Volume.IsEmpty(l1Voxel))
            return L1_ALL_FACES_MASK;
        if (!TryResolveL1SeedIndex(l1Voxel, actualVoxel, point, out var seedIndex))
            return 0;

        return FloodFillL1ReachableFaces(l1Voxel, seedIndex);
    }

    private bool ArePointsConnectedWithinL1(ulong l1Voxel, ulong fromVoxel, Vector3 fromPoint, ulong toVoxel, Vector3 toPoint)
    {
        if (Volume.IsEmpty(l1Voxel))
            return true;
        if (!TryResolveL1SeedIndex(l1Voxel, fromVoxel, fromPoint, out var fromSeedIndex) ||
            !TryResolveL1SeedIndex(l1Voxel, toVoxel,   toPoint,   out var toSeedIndex))
            return false;
        if (fromSeedIndex == toSeedIndex)
            return true;

        return FloodFillL1SeedConnectivity(l1Voxel, fromSeedIndex, toSeedIndex);
    }

    private ulong BuildPackedL1FaceConnectivity(ulong l1Voxel)
    {
        if (Volume.IsEmpty(l1Voxel))
        {
            ulong packed = 0;
            for (var face = 0; face < 6; ++face)
                packed = SetPackedReachableL1Faces(packed, face, L1_ALL_FACES_MASK);
            return packed;
        }

        var   totalCellCount     = l2Desc.NumCellsTotal;
        var   visited            = new bool[totalCellCount];
        ulong packedConnectivity = 0;

        for (ushort seedIndex = 0; seedIndex < totalCellCount; ++seedIndex)
        {
            if (visited[seedIndex])
                continue;

            var leafVoxel = VoxelMap.EncodeSubIndex(l1Voxel, seedIndex, 2);

            if (!Volume.IsEmpty(leafVoxel))
            {
                visited[seedIndex] = true;
                continue;
            }

            var reachableFaces = FloodFillL1Component(l1Voxel, seedIndex, visited, null, out _);
            if (reachableFaces == 0)
                continue;

            for (var face = 0; face < 6; ++face)
            {
                if (!HasL1Face(reachableFaces, face))
                    continue;

                var combinedReachable = (ushort)(GetPackedReachableL1Faces(packedConnectivity, face) | reachableFaces);
                packedConnectivity = SetPackedReachableL1Faces(packedConnectivity, face, combinedReachable);
            }
        }

        return packedConnectivity;
    }

    private ushort FloodFillL1ReachableFaces(ulong l1Voxel, ushort seedIndex) => FloodFillL1Component(l1Voxel, seedIndex, null, null, out _);

    private bool FloodFillL1SeedConnectivity(ulong l1Voxel, ushort fromSeedIndex, ushort toSeedIndex)
    {
        FloodFillL1Component(l1Voxel, fromSeedIndex, null, toSeedIndex, out var reachedTarget);
        return reachedTarget;
    }

    private ushort FloodFillL1Component(ulong l1Voxel, ushort seedIndex, bool[]? visited, ushort? targetSeedIndex, out bool reachedTarget)
    {
        reachedTarget =   false;
        visited       ??= new bool[l2Desc.NumCellsTotal];

        if (visited[seedIndex])
        {
            reachedTarget = targetSeedIndex == seedIndex;
            return 0;
        }

        var seedLeaf = VoxelMap.EncodeSubIndex(l1Voxel, seedIndex, 2);

        if (!Volume.IsEmpty(seedLeaf))
        {
            visited[seedIndex] = true;
            return 0;
        }

        Queue<ushort> frontier = new();
        frontier.Enqueue(seedIndex);
        visited[seedIndex] = true;

        ushort reachableFaces = 0;

        while (frontier.TryDequeue(out var currentIndex))
        {
            if (targetSeedIndex == currentIndex)
                reachedTarget = true;

            var (x, y, z) = l2Desc.IndexToVoxel(currentIndex);
            if (y == 0)
                reachableFaces |= L1FaceBit(L1_FACE_NEG_Y);
            if (y == l2Desc.NumCellsY - 1)
                reachableFaces |= L1FaceBit(L1_FACE_POS_Y);
            if (x == 0)
                reachableFaces |= L1FaceBit(L1_FACE_NEG_X);
            if (x == l2Desc.NumCellsX - 1)
                reachableFaces |= L1FaceBit(L1_FACE_POS_X);
            if (z == 0)
                reachableFaces |= L1FaceBit(L1_FACE_NEG_Z);
            if (z == l2Desc.NumCellsZ - 1)
                reachableFaces |= L1FaceBit(L1_FACE_POS_Z);

            for (var dir = 0; dir < 6; ++dir)
            {
                var dx = dir switch
                {
                    L1_FACE_NEG_X => -1,
                    L1_FACE_POS_X => 1,
                    _             => 0
                };
                var dy = dir switch
                {
                    L1_FACE_NEG_Y => -1,
                    L1_FACE_POS_Y => 1,
                    _             => 0
                };
                var dz = dir switch
                {
                    L1_FACE_NEG_Z => -1,
                    L1_FACE_POS_Z => 1,
                    _             => 0
                };
                var nx = x + dx;
                var ny = y + dy;
                var nz = z + dz;
                if (!l2Desc.InBounds(nx, ny, nz))
                    continue;

                var neighbourIndex = l2Desc.VoxelToIndex(nx, ny, nz);
                if (visited[neighbourIndex])
                    continue;

                var neighbourLeaf = VoxelMap.EncodeSubIndex(l1Voxel, neighbourIndex, 2);

                if (!Volume.IsEmpty(neighbourLeaf))
                {
                    visited[neighbourIndex] = true;
                    continue;
                }

                visited[neighbourIndex] = true;
                frontier.Enqueue(neighbourIndex);
            }
        }

        return reachableFaces;
    }

    private bool TryResolveL1SeedIndex(ulong l1Voxel, ulong actualVoxel, Vector3 point, out ushort seedIndex)
    {
        seedIndex = 0;

        if (TryExtractL2IndexWithinL1(actualVoxel, l1Voxel, out seedIndex))
            return true;

        seedIndex = ResolvePointL2IndexWithinL1(l1Voxel, point);
        if (Volume.IsEmpty(VoxelMap.EncodeSubIndex(l1Voxel, seedIndex, 2)))
            return true;

        return TryFindNearestEmptyL1SeedIndex(l1Voxel, point, out seedIndex);
    }

    private static bool TryExtractL2IndexWithinL1(ulong voxel, ulong expectedL1, out ushort l2Index)
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

    private ushort ResolvePointL2IndexWithinL1(ulong l1Voxel, Vector3 point)
    {
        var bounds   = Volume.VoxelBounds(l1Voxel, 0);
        var span     = bounds.max - bounds.min;
        var clamped  = Vector3.Clamp(point, bounds.min, bounds.max - new Vector3(SCORE_EPSILON));
        var relative = clamped - bounds.min;
        var x        = span.X > SCORE_EPSILON ? Math.Clamp((int)(relative.X / span.X * l2Desc.NumCellsX), 0, l2Desc.NumCellsX - 1) : 0;
        var y        = span.Y > SCORE_EPSILON ? Math.Clamp((int)(relative.Y / span.Y * l2Desc.NumCellsY), 0, l2Desc.NumCellsY - 1) : 0;
        var z        = span.Z > SCORE_EPSILON ? Math.Clamp((int)(relative.Z / span.Z * l2Desc.NumCellsZ), 0, l2Desc.NumCellsZ - 1) : 0;
        return l2Desc.VoxelToIndex(x, y, z);
    }

    private bool TryFindNearestEmptyL1SeedIndex(ulong l1Voxel, Vector3 point, out ushort seedIndex)
    {
        seedIndex = 0;
        var bestDistance = float.MaxValue;
        var found        = false;

        for (ushort currentIndex = 0; currentIndex < l2Desc.NumCellsTotal; ++currentIndex)
        {
            var leafVoxel = VoxelMap.EncodeSubIndex(l1Voxel, currentIndex, 2);
            if (!Volume.IsEmpty(leafVoxel))
                continue;

            var center          = ResolveVoxelCenter(leafVoxel);
            var distanceSquared = Vector3.DistanceSquared(center, point);
            if (distanceSquared + SCORE_EPSILON >= bestDistance)
                continue;

            bestDistance = distanceSquared;
            seedIndex    = currentIndex;
            found        = true;
        }

        return found;
    }

    private ulong ResolveRepresentativeL1Voxel(ulong voxel, Vector3 referencePoint)
    {
        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1Voxel;

        var l0Index  = ExtractL0Index(voxel);
        var bounds   = Volume.VoxelBounds(voxel, 0);
        var span     = bounds.max - bounds.min;
        var clamped  = Vector3.Clamp(referencePoint, bounds.min, bounds.max - new Vector3(SCORE_EPSILON));
        var relative = clamped - bounds.min;
        var l1x      = span.X > SCORE_EPSILON ? Math.Clamp((int)(relative.X / span.X * l1Desc.NumCellsX), 0, l1Desc.NumCellsX - 1) : 0;
        var l1y      = span.Y > SCORE_EPSILON ? Math.Clamp((int)(relative.Y / span.Y * l1Desc.NumCellsY), 0, l1Desc.NumCellsY - 1) : 0;
        var l1z      = span.Z > SCORE_EPSILON ? Math.Clamp((int)(relative.Z / span.Z * l1Desc.NumCellsZ), 0, l1Desc.NumCellsZ - 1) : 0;
        return VoxelMap.EncodeIndex(l0Index, VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1x, l1y, l1z)));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsVoxelInsidePathConstraint(ulong voxel)
    {
        if (l1PathSet == null)
            return true;

        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1PathSet.Contains(l1Voxel);

        return l0PathSet?.Contains(ExtractL0Index(voxel)) ?? false;
    }

    private bool TryGetVoxelCorridorDistance(ulong voxel, out int distance)
    {
        distance = int.MaxValue;
        if (l1CorridorDistance == null)
            return false;

        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1CorridorDistance.TryGetValue(l1Voxel, out distance);

        return l0CorridorDistance?.TryGetValue(ExtractL0Index(voxel), out distance) ?? false;
    }

    private bool TryGetVoxelL1DistanceFloor(ulong voxel, out float distance)
    {
        distance = float.MaxValue;
        if (l1DistanceField == null)
            return false;

        if (TryExtractL1Parent(voxel, out var l1Voxel))
            return l1DistanceField.TryGetValue(l1Voxel, out distance);

        return l0DistanceField?.TryGetValue(ExtractL0Index(voxel), out distance) ?? false;
    }

    private bool TryCreateGuidedCorridor(Vector3 fromPos, Vector3 toPos, out GuidedSearchCorridor corridor)
    {
        corridor = default;

        var horizontalDelta  = new Vector2(toPos.X - fromPos.X, toPos.Z - fromPos.Z);
        var horizontalLength = horizontalDelta.Length();
        if (horizontalLength < GUIDED_CORRIDOR_MIN_HORIZONTAL_DISTANCE)
            return false;

        var horizontalDirection = horizontalDelta / horizontalLength;
        var leafHorizontalSize  = MathF.Max(l2Desc.CellSize.X, l2Desc.CellSize.Z);
        var leafVerticalSize    = l2Desc.CellSize.Y;
        var verticalDelta       = toPos.Y - fromPos.Y;
        var horizontalRadius = Math.Clamp
        (
            horizontalLength   * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_SCALE,
            leafHorizontalSize * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MIN_LEAF_CELLS,
            MathF.Max(leafHorizontalSize * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_LEAF_CELLS, horizontalLength * GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_DISTANCE_SCALE)
        );
        var upwardAllowance = Math.Max
        (
            MathF.Abs(verticalDelta) + (leafVerticalSize * GUIDED_CORRIDOR_UPWARD_ALLOWANCE_MIN_LEAF_CELLS),
            horizontalLength * GUIDED_CORRIDOR_UPWARD_ALLOWANCE_DISTANCE_SCALE
        );
        var downwardAllowance = Math.Max
        (
            MathF.Abs(verticalDelta) + (leafVerticalSize * GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_MIN_LEAF_CELLS),
            horizontalLength * GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_DISTANCE_SCALE
        );
        var endpointSlack = Math.Max
            (horizontalRadius * GUIDED_CORRIDOR_ENDPOINT_SLACK_RADIUS_SCALE, leafHorizontalSize * GUIDED_CORRIDOR_ENDPOINT_SLACK_MIN_LEAF_CELLS);

        corridor = new
        (
            fromPos,
            horizontalDirection,
            horizontalLength,
            verticalDelta,
            horizontalRadius,
            upwardAllowance,
            downwardAllowance,
            endpointSlack
        );
        return true;
    }
    
    private float CalculateCorridorOverflowPenalty(Vector3 point)
    {
        var relative           = point - guidedCorridor.Start;
        var relativeHorizontal = new Vector2(relative.X, relative.Z);
        var advance            = Vector2.Dot(relativeHorizontal, guidedCorridor.HorizontalDirection);
        var clampedAdvance     = Math.Clamp(advance, 0f, guidedCorridor.HorizontalLength);

        var advanceOverflow = 0f;
        if (advance < -guidedCorridor.EndpointSlack)
            advanceOverflow = -guidedCorridor.EndpointSlack - advance;
        else if (advance > guidedCorridor.HorizontalLength + guidedCorridor.EndpointSlack)
            advanceOverflow = advance - guidedCorridor.HorizontalLength - guidedCorridor.EndpointSlack;

        var lateral         = relativeHorizontal - (guidedCorridor.HorizontalDirection * clampedAdvance);
        var lateralDist     = lateral.Length();
        var lateralOverflow = MathF.Max(lateralDist - guidedCorridor.HorizontalRadius, 0f);

        var t              = guidedCorridor.HorizontalLength > SCORE_EPSILON ? Math.Clamp(advance / guidedCorridor.HorizontalLength, 0f, 1f) : 0f;
        var baselineY      = guidedCorridor.Start.Y + (guidedCorridor.VerticalDelta * t);
        var verticalOffset = point.Y                - baselineY;
        var verticalOverflow = verticalOffset >= 0f
                                   ? MathF.Max(verticalOffset  - guidedCorridor.UpwardAllowance,   0f)
                                   : MathF.Max(-verticalOffset - guidedCorridor.DownwardAllowance, 0f);

        var totalOverflow = advanceOverflow + lateralOverflow + verticalOverflow;
        if (totalOverflow <= SCORE_EPSILON)
            return 0f;

        var hardCutoff = guidedCorridor.HorizontalRadius * GUIDED_CORRIDOR_HARD_CUTOFF_MULTIPLIER;
        if (totalOverflow > hardCutoff)
            return -1f;

        var normalizedOverflow = totalOverflow / guidedCorridor.HorizontalRadius;
        return normalizedOverflow * normalizedOverflow * GUIDED_CORRIDOR_OVERFLOW_PENALTY_SCALE;
    }

    private Vector3 ResolveSearchCandidatePosition(ulong voxel, Vector3 point)
    {
        point = Volume.ClampPointToVoxel(voxel, point);
        return ApplyWallAwareInset(voxel, point);
    }

    private Vector3 ResolveVoxelCenter(ulong voxel)
    {
        var (min, max) = Volume.VoxelBounds(voxel, 0);
        return (min + max) * 0.5f;
    }

    private Vector3 ApplyWallAwareInset(ulong voxel, Vector3 point)
    {
        var wallMask = GetVoxelWallMask(voxel);
        if (wallMask == 0)
            return point;

        var (min, max) = Volume.VoxelBounds(voxel, 0);
        var inset = ResolveSearchVoxelInset(voxel);

        if ((wallMask & SEARCH_WALL_NEG_X) != 0)
            point.X = MathF.Max(point.X, min.X + inset);
        if ((wallMask & SEARCH_WALL_POS_X) != 0)
            point.X = MathF.Min(point.X, max.X - inset);
        if ((wallMask & SEARCH_WALL_NEG_Y) != 0)
            point.Y = MathF.Max(point.Y, min.Y + inset);
        if ((wallMask & SEARCH_WALL_POS_Y) != 0)
            point.Y = MathF.Min(point.Y, max.Y - inset);
        if ((wallMask & SEARCH_WALL_NEG_Z) != 0)
            point.Z = MathF.Max(point.Z, min.Z + inset);
        if ((wallMask & SEARCH_WALL_POS_Z) != 0)
            point.Z = MathF.Min(point.Z, max.Z - inset);

        return point;
    }

    private float CalculateWallProximityPenalty(ulong voxel, Vector3 position)
    {
        if (voxel == goalVoxel)
            return 0f;

        var wallMask = GetVoxelWallMask(voxel);
        if (wallMask == 0)
            return 0f;

        var (min, max) = Volume.VoxelBounds(voxel, 0);
        var voxelSize = max - min;
        var minExtent = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
        var preferredClearance = Math.Clamp
        (
            minExtent * SEARCH_PATH_WALL_PREFERRED_CLEARANCE_RATIO,
            SEARCH_PATH_WALL_PREFERRED_CLEARANCE_MIN,
            minExtent * SEARCH_PATH_WALL_PREFERRED_CLEARANCE_MAX_FRACTION
        );
        if (preferredClearance <= SCORE_EPSILON)
            return 0f;

        var pressure = 0f;
        if ((wallMask & SEARCH_WALL_NEG_X) != 0)
            pressure += WallPressure(position.X - min.X, preferredClearance);
        if ((wallMask & SEARCH_WALL_POS_X) != 0)
            pressure += WallPressure(max.X - position.X, preferredClearance);
        if ((wallMask & SEARCH_WALL_NEG_Y) != 0)
            pressure += WallPressure(position.Y - min.Y, preferredClearance);
        if ((wallMask & SEARCH_WALL_POS_Y) != 0)
            pressure += WallPressure(max.Y - position.Y, preferredClearance);
        if ((wallMask & SEARCH_WALL_NEG_Z) != 0)
            pressure += WallPressure(position.Z - min.Z, preferredClearance);
        if ((wallMask & SEARCH_WALL_POS_Z) != 0)
            pressure += WallPressure(max.Z - position.Z, preferredClearance);

        return pressure * minExtent * SEARCH_PATH_WALL_PENALTY_SCALE;
    }

    private static float WallPressure(float clearance, float preferredClearance)
    {
        if (preferredClearance <= SCORE_EPSILON)
            return 0f;

        return Math.Clamp((preferredClearance - clearance) / preferredClearance, 0f, 1f);
    }

    private float ResolveSearchVoxelInset(ulong voxel)
    {
        var voxelSize   = GetVoxelSize(voxel);
        var minExtent   = MathF.Min(voxelSize.X, MathF.Min(voxelSize.Y, voxelSize.Z));
        var scaledInset = minExtent * SEARCH_PATH_WALL_INSET_RATIO;
        var maxInset    = minExtent * SEARCH_PATH_WALL_INSET_MAX_FRACTION;
        var minInset    = MathF.Min(SEARCH_PATH_WALL_INSET_MIN, maxInset);
        return Math.Clamp(scaledInset, minInset, maxInset);
    }

    private byte GetVoxelWallMask(ulong voxel)
    {
        if (voxelWallMaskCache.TryGetValue(voxel, out var cached))
            return cached;

        byte wallMask = 0;
        if (!HasEmptyNeighbourInDirection(voxel, -1, 0, 0))
            wallMask |= SEARCH_WALL_NEG_X;
        if (!HasEmptyNeighbourInDirection(voxel, +1, 0, 0))
            wallMask |= SEARCH_WALL_POS_X;
        if (!HasEmptyNeighbourInDirection(voxel, 0, -1, 0))
            wallMask |= SEARCH_WALL_NEG_Y;
        if (!HasEmptyNeighbourInDirection(voxel, 0, +1, 0))
            wallMask |= SEARCH_WALL_POS_Y;
        if (!HasEmptyNeighbourInDirection(voxel, 0, 0, -1))
            wallMask |= SEARCH_WALL_NEG_Z;
        if (!HasEmptyNeighbourInDirection(voxel, 0, 0, +1))
            wallMask |= SEARCH_WALL_POS_Z;

        if (!voxelWallMaskCache.TryAdd(voxel, wallMask))
            return voxelWallMaskCache[voxel];

        return wallMask;
    }

    private bool HasEmptyNeighbourInDirection(ulong voxel, int dx, int dy, int dz)
    {
        var encodedVoxel = voxel;
        var l0Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l1Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l2Index      = VoxelMap.DecodeIndex(ref encodedVoxel);
        var l0Coords     = l0Desc.IndexToVoxel(l0Index);
        var l1Coords     = l1Index != VoxelMap.INDEX_LEVEL_MASK ? l1Desc.IndexToVoxel(l1Index) : default;
        var l2Coords     = l2Index != VoxelMap.INDEX_LEVEL_MASK ? l2Desc.IndexToVoxel(l2Index) : default;

        if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l2Neighbour = (l2Coords.x + dx, l2Coords.y + dy, l2Coords.z + dz);

            if (l2Desc.InBounds(l2Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l2Desc.VoxelToIndex(l2Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l1Index, neighbourVoxel);
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                return Volume.IsEmpty(neighbourVoxel);
            }
        }

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1Neighbour = (l1Coords.x + dx, l1Coords.y + dy, l1Coords.z + dz);

            if (l1Desc.InBounds(l1Neighbour))
            {
                var neighbourVoxel = VoxelMap.EncodeIndex(l1Desc.VoxelToIndex(l1Neighbour));
                neighbourVoxel = VoxelMap.EncodeIndex(l0Index, neighbourVoxel);
                if (Volume.IsEmpty(neighbourVoxel))
                    return true;

                if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
                {
                    var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                    var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                    var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                    var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(neighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                    return Volume.IsEmpty(l2NeighbourVoxel);
                }

                return HasEmptyOnBorder(neighbourVoxel, l2Desc, 2, dx, dy, dz);
            }
        }

        var l0Neighbour = (l0Coords.x + dx, l0Coords.y + dy, l0Coords.z + dz);
        if (!l0Desc.InBounds(l0Neighbour))
            return false;

        var l0NeighbourVoxel = VoxelMap.EncodeIndex(l0Desc.VoxelToIndex(l0Neighbour));
        if (Volume.IsEmpty(l0NeighbourVoxel))
            return true;

        if (l1Index != VoxelMap.INDEX_LEVEL_MASK)
        {
            var l1X              = dx == 0 ? l1Coords.x : dx > 0 ? 0 : l1Desc.NumCellsX - 1;
            var l1Y              = dy == 0 ? l1Coords.y : dy > 0 ? 0 : l1Desc.NumCellsY - 1;
            var l1Z              = dz == 0 ? l1Coords.z : dz > 0 ? 0 : l1Desc.NumCellsZ - 1;
            var l1NeighbourVoxel = VoxelMap.EncodeSubIndex(l0NeighbourVoxel, l1Desc.VoxelToIndex(l1X, l1Y, l1Z), 1);

            if (Volume.IsEmpty(l1NeighbourVoxel))
                return true;

            if (l2Index != VoxelMap.INDEX_LEVEL_MASK)
            {
                var l2X              = dx == 0 ? l2Coords.x : dx > 0 ? 0 : l2Desc.NumCellsX - 1;
                var l2Y              = dy == 0 ? l2Coords.y : dy > 0 ? 0 : l2Desc.NumCellsY - 1;
                var l2Z              = dz == 0 ? l2Coords.z : dz > 0 ? 0 : l2Desc.NumCellsZ - 1;
                var l2NeighbourVoxel = VoxelMap.EncodeSubIndex(l1NeighbourVoxel, l2Desc.VoxelToIndex(l2X, l2Y, l2Z), 2);
                return Volume.IsEmpty(l2NeighbourVoxel);
            }

            return HasEmptyOnBorder(l1NeighbourVoxel, l2Desc, 2, dx, dy, dz);
        }

        return HasEmptyBorderWithSubdivisions(l0NeighbourVoxel, dx, dy, dz);
    }

    private bool HasEmptyOnBorder(ulong voxel, VolumeLevel levelDesc, int level, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, levelDesc.NumCellsX - 1) : dx > 0 ? (0, 0) : (levelDesc.NumCellsX - 1, levelDesc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, levelDesc.NumCellsY - 1) : dy > 0 ? (0, 0) : (levelDesc.NumCellsY - 1, levelDesc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, levelDesc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (levelDesc.NumCellsZ - 1, levelDesc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
            if (Volume.IsEmpty(VoxelMap.EncodeSubIndex(voxel, levelDesc.VoxelToIndex(x, y, z), level)))
                return true;

        return false;
    }

    private bool HasEmptyBorderWithSubdivisions(ulong voxel, int dx, int dy, int dz)
    {
        var (xMin, xMax) = dx == 0 ? (0, l1Desc.NumCellsX - 1) : dx > 0 ? (0, 0) : (l1Desc.NumCellsX - 1, l1Desc.NumCellsX - 1);
        var (yMin, yMax) = dy == 0 ? (0, l1Desc.NumCellsY - 1) : dy > 0 ? (0, 0) : (l1Desc.NumCellsY - 1, l1Desc.NumCellsY - 1);
        var (zMin, zMax) = dz == 0 ? (0, l1Desc.NumCellsZ - 1) : dz > 0 ? (0, 0) : (l1Desc.NumCellsZ - 1, l1Desc.NumCellsZ - 1);

        for (var z = zMin; z <= zMax; ++z)
        for (var x = xMin; x <= xMax; ++x)
        for (var y = yMin; y <= yMax; ++y)
        {
            var l1Voxel = VoxelMap.EncodeSubIndex(voxel, l1Desc.VoxelToIndex(x, y, z), 1);
            if (Volume.IsEmpty(l1Voxel) || HasEmptyOnBorder(l1Voxel, l2Desc, 2, dx, dy, dz))
                return true;
        }

        return false;
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private float HeuristicDistance(Vector3 position, ulong voxel)
    {
        var h = longRangeLateralBias.Enabled ? ComputeLongRangeLateralHeuristic(position, voxel) : Vector3.Distance(position, goalPos);
        if (TryGetVoxelL1DistanceFloor(voxel, out var l1Dist))
            h = MathF.Max(h, l1Dist);
        return h;
    }

    private float ComputeLongRangeLateralHeuristic(Vector3 position, ulong voxel)
        => ComputeLongRangeLateralHeuristic(position, voxel, longRangeLateralBias, goalPos);

    private float ComputeLongRangeLateralHeuristic(Vector3 position, ulong voxel, LongRangeLateralBias bias, Vector3 goal)
    {
        var horizontalGoalDist  = HorizontalDistanceXZ(position, goal);
        var aboveGoal           = MathF.Max(position.Y - goal.Y,     0f);
        var belowGoal           = MathF.Max(goal.Y     - position.Y, 0f);
        var relativeFromStart   = position - bias.Start;
        var horizontalFromStart = new Vector2(relativeFromStart.X, relativeFromStart.Z);
        var forwardProgress     = Vector2.Dot(horizontalFromStart, bias.Forward);
        var lateralOffset       = MathF.Abs(Vector2.Dot(horizontalFromStart, bias.Right));
        var forwardRemaining    = MathF.Max(0f, bias.HorizontalDistance - forwardProgress);
        var horizontalTerm = (horizontalGoalDist * LONG_RANGE_LATERAL_GOAL_DISTANCE_HEURISTIC_WEIGHT)     +
                             (forwardRemaining   * LONG_RANGE_LATERAL_FORWARD_REMAINING_HEURISTIC_WEIGHT) +
                             (lateralOffset      * LONG_RANGE_LATERAL_SIDE_OFFSET_HEURISTIC_WEIGHT);

        float verticalTerm;

        if (bias.PreferDescending)
        {
            verticalTerm = (aboveGoal * (LONG_RANGE_LATERAL_DESCENT_ABOVE_GOAL_HEURISTIC_WEIGHT * bias.HeightPriority)) +
                           (belowGoal * LONG_RANGE_LATERAL_DESCENT_BELOW_GOAL_HEURISTIC_WEIGHT);
        }
        else
        {
            verticalTerm = (belowGoal * LONG_RANGE_LATERAL_ASCENT_BELOW_GOAL_HEURISTIC_WEIGHT) +
                           (aboveGoal * LONG_RANGE_LATERAL_ASCENT_ABOVE_GOAL_HEURISTIC_WEIGHT);
        }

        if (bias.PreferDescending                                                                          &&
            aboveGoal >= l2Desc.CellSize.Y * LONG_RANGE_LATERAL_DOWNWARD_OPENING_MIN_ABOVE_GOAL_LEAF_CELLS &&
            HasDownwardOpening(voxel))
        {
            horizontalTerm *= LONG_RANGE_LATERAL_DOWNWARD_OPENING_HORIZONTAL_HEURISTIC_SCALE;
            verticalTerm   *= LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_HEURISTIC_SCALE;
        }

        return horizontalTerm + verticalTerm;
    }

    private float CalculateLongRangeLateralTraversalPenalty(ulong parentVoxel, Vector3 parentPosition, ulong destinationVoxel, Vector3 destination)
        => CalculateLongRangeLateralTraversalPenalty(parentVoxel, parentPosition, destinationVoxel, destination, longRangeLateralBias, goalPos);

    private float CalculateLongRangeLateralTraversalPenalty
    (
        ulong                parentVoxel,
        Vector3              parentPosition,
        ulong                destinationVoxel,
        Vector3              destination,
        LongRangeLateralBias bias,
        Vector3              goal
    )
    {
        if (!bias.Enabled)
            return 0f;

        var step                 = destination - parentPosition;
        var horizontalStep       = new Vector2(step.X, step.Z);
        var horizontalStepLength = horizontalStep.Length();
        var forwardStep          = Vector2.Dot(horizontalStep, bias.Forward);
        var lateralStep          = MathF.Abs(Vector2.Dot(horizontalStep, bias.Right));
        var penalty              = 0f;

        if (forwardStep < -SCORE_EPSILON)
            penalty += -forwardStep * LONG_RANGE_LATERAL_REVERSE_STEP_PENALTY * bias.DirectionalPenaltyScale;

        if (bias.PreferDescending)
        {
            var climbStep            = MathF.Max(step.Y,                                  0f);
            var descentStep          = MathF.Max(-step.Y,                                 0f);
            var parentAboveGoal      = MathF.Max(parentPosition.Y - goal.Y,               0f);
            var destinationAboveGoal = MathF.Max(destination.Y    - goal.Y,               0f);
            var descentTowardGoal    = MathF.Max(parentAboveGoal  - destinationAboveGoal, 0f);
            var progressRoom = (MathF.Max(forwardStep, 0f) * LONG_RANGE_LATERAL_FORWARD_PROGRESS_CREDIT) +
                               (descentStep                * LONG_RANGE_LATERAL_DESCENT_PROGRESS_CREDIT);

            if (climbStep > SCORE_EPSILON)
                penalty += climbStep * LONG_RANGE_LATERAL_CLIMB_STEP_PENALTY * bias.HeightPriority;
            if (lateralStep > progressRoom + SCORE_EPSILON)
                penalty += (lateralStep - progressRoom) * LONG_RANGE_LATERAL_LATERAL_STALL_PENALTY * bias.DirectionalPenaltyScale;

            if (parentAboveGoal >= l2Desc.CellSize.Y * LONG_RANGE_LATERAL_DOWNWARD_OPENING_MIN_ABOVE_GOAL_LEAF_CELLS)
            {
                var parentHasDownwardOpening      = HasDownwardOpening(parentVoxel);
                var destinationHasDownwardOpening = HasDownwardOpening(destinationVoxel);

                if (destinationHasDownwardOpening)
                    penalty *= LONG_RANGE_LATERAL_DOWNWARD_OPENING_DESTINATION_PENALTY_SCALE;

                if (parentHasDownwardOpening)
                {
                    var requiredDescent = MathF.Min
                    (
                        parentAboveGoal,
                        l2Desc.CellSize.Y * LONG_RANGE_LATERAL_DOWNWARD_OPENING_REQUIRED_DESCENT_LEAF_CELLS
                    );

                    if (descentTowardGoal + SCORE_EPSILON < requiredDescent)
                    {
                        var missedDescent = requiredDescent - descentTowardGoal;
                        penalty += horizontalStepLength * LONG_RANGE_LATERAL_DOWNWARD_OPENING_MISSED_DESCENT_PENALTY * bias.HeightPriority;
                        penalty += missedDescent        * LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_MISS_PENALTY  * bias.HeightPriority;
                    }
                }
            }
        }
        else
        {
            var progressRoom = MathF.Max(forwardStep, 0f) * LONG_RANGE_LATERAL_FORWARD_PROGRESS_CREDIT;
            if (lateralStep > progressRoom + SCORE_EPSILON)
                penalty += (lateralStep - progressRoom) * LONG_RANGE_LATERAL_LATERAL_STALL_PENALTY * LONG_RANGE_LATERAL_NON_DESCENT_STALL_SCALE;
        }

        return penalty;
    }

    private bool HasDownwardOpening(ulong voxel)
    {
        if (verifiedDownwardOpeningCache.TryGetValue(voxel, out var cached))
            return cached == 1;

        var open = HasVerifiedVerticalAccessThroughFace(voxel, true);
        verifiedDownwardOpeningCache.TryAdd(voxel, open ? (byte)1 : (byte)2);
        return open;
    }

    private bool HasVerifiedTopEntry(ulong voxel)
    {
        if (verifiedTopEntryCache.TryGetValue(voxel, out var cached))
            return cached == 1;

        var open = HasVerifiedVerticalAccessThroughFace(voxel, false);
        verifiedTopEntryCache.TryAdd(voxel, open ? (byte)1 : (byte)2);
        return open;
    }

    private bool HasVerifiedVerticalAccessThroughFace(ulong voxel, bool throughLowerFace)
    {
        var wallMask = GetVoxelWallMask(voxel);

        if (throughLowerFace)
        {
            if ((wallMask & SEARCH_WALL_NEG_Y) != 0)
                return false;
        }
        else if ((wallMask & SEARCH_WALL_POS_Y) != 0)
            return false;

        var (min, max) = Volume.VoxelBounds(voxel, 0);
        var size     = max - min;
        var rootMinY = Volume.RootTile.BoundsMin.Y;
        var rootMaxY = Volume.RootTile.BoundsMax.Y;
        var epsX     = MathF.Max(SCORE_EPSILON, MathF.Min(size.X * 0.18f, l2Desc.CellSize.X * 1.25f));
        var epsY     = MathF.Max(SCORE_EPSILON, MathF.Min(size.Y * 0.18f, l2Desc.CellSize.Y * 1.25f));
        var epsZ     = MathF.Max(SCORE_EPSILON, MathF.Min(size.Z * 0.18f, l2Desc.CellSize.Z * 1.25f));
        var minX     = min.X + epsX;
        var maxX     = max.X - epsX;
        var minZ     = min.Z + epsZ;
        var maxZ     = max.Z - epsZ;
        var probeDepth = MathF.Max
            (l2Desc.CellSize.Y * LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_LEAF_CELLS, size.Y * LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_HEIGHT_SCALE);
        var insideY = throughLowerFace
                          ? min.Y + Math.Clamp(size.Y * 0.60f, epsY, MathF.Max(epsY, size.Y - epsY))
                          : max.Y - Math.Clamp(size.Y * 0.25f, epsY, MathF.Max(epsY, size.Y - epsY));
        var outsideY = throughLowerFace
                           ? MathF.Max(min.Y - probeDepth, rootMinY + epsY)
                           : MathF.Min(max.Y + probeDepth, rootMaxY - epsY);

        if (throughLowerFace)
        {
            if (outsideY >= insideY - SCORE_EPSILON)
                return false;
        }
        else if (outsideY <= insideY + SCORE_EPSILON)
            return false;

        Span<Vector2> offsets = stackalloc Vector2[5];
        offsets[0] = Vector2.Zero;
        var offsetX = MathF.Max(0f, (maxX - minX) * 0.35f);
        var offsetZ = MathF.Max(0f, (maxZ - minZ) * 0.35f);
        offsets[1] = new(+offsetX, 0f);
        offsets[2] = new(-offsetX, 0f);
        offsets[3] = new(0f, +offsetZ);
        offsets[4] = new(0f, -offsetZ);

        var centerX = (minX + maxX) * 0.5f;
        var centerZ = (minZ + maxZ) * 0.5f;

        for (var i = 0; i < offsets.Length; ++i)
        {
            var x         = Math.Clamp(centerX + offsets[i].X, minX, maxX);
            var z         = Math.Clamp(centerZ + offsets[i].Y, minZ, maxZ);
            var inside    = new Vector3(x, insideY,  z);
            var outside   = new Vector3(x, outsideY, z);
            var from      = throughLowerFace ? inside : outside;
            var to        = throughLowerFace ? outside : inside;
            var startLeaf = Volume.FindLeafVoxel(from);
            var endLeaf   = Volume.FindLeafVoxel(to);
            if (!startLeaf.empty || !endLeaf.empty)
                continue;
            if (VoxelSearch.LineOfSight(Volume, startLeaf.voxel, endLeaf.voxel, from, to))
                return true;
        }

        return false;
    }
}
