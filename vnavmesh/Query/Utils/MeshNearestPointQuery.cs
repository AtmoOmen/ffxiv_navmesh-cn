using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using vnavmesh.Common.Extensions;

namespace vnavmesh.Query.Utils;

internal sealed class MeshNearestPointQuery
{
    private readonly DtNavMesh mesh;
    private readonly int       minTileX = int.MaxValue;
    private readonly int       minTileZ = int.MaxValue;
    private readonly int       maxTileX = int.MinValue;
    private readonly int       maxTileZ = int.MinValue;

    internal MeshNearestPointQuery(DtNavMesh mesh)
    {
        this.mesh = mesh;

        for (var tileIndex = 0; tileIndex < mesh.GetMaxTiles(); ++tileIndex)
        {
            var tile = mesh.GetTile(tileIndex);
            if (tile?.data?.header is not { } header)
                continue;

            minTileX = Math.Min(minTileX, header.x);
            minTileZ = Math.Min(minTileZ, header.y);
            maxTileX = Math.Max(maxTileX, header.x);
            maxTileZ = Math.Max(maxTileZ, header.y);
        }
    }

    internal long FindNearest
    (
        Vector3        point,
        float          halfExtentXZ,
        float          halfExtentY,
        IDtQueryFilter filter,
        out Vector3    nearestPoint,
        out bool       isOverPoly
    )
    {
        nearestPoint = point;
        isOverPoly   = false;
        var center = point.ToRecast();
        var extent = new RcVec3f(halfExtentXZ, halfExtentY, halfExtentXZ);
        if (!center.IsFinite() || !extent.IsFinite() || halfExtentXZ < 0 || halfExtentY < 0 || minTileX > maxTileX)
            return 0;

        var boundsMin = center - extent;
        var boundsMax = center + extent;
        if (!boundsMin.IsFinite() || !boundsMax.IsFinite())
            return 0;

        ref readonly var parameters = ref mesh.GetParams();
        var              meshMinX   = parameters.orig.X + (minTileX       * parameters.tileWidth);
        var              meshMinZ   = parameters.orig.Z + (minTileZ       * parameters.tileHeight);
        var              meshMaxX   = parameters.orig.X + ((maxTileX + 1) * parameters.tileWidth);
        var              meshMaxZ   = parameters.orig.Z + ((maxTileZ + 1) * parameters.tileHeight);
        if (boundsMax.X < meshMinX || boundsMax.Z < meshMinZ || boundsMin.X > meshMaxX || boundsMin.Z > meshMaxZ)
            return 0;

        mesh.CalcTileLoc(new(Math.Clamp(boundsMin.X, meshMinX, meshMaxX), 0, Math.Clamp(boundsMin.Z, meshMinZ, meshMaxZ)), out var firstX, out var firstZ);
        mesh.CalcTileLoc(new(Math.Clamp(boundsMax.X, meshMinX, meshMaxX), 0, Math.Clamp(boundsMax.Z, meshMinZ, meshMaxZ)), out var lastX,  out var lastZ);
        firstX = Math.Clamp(firstX, minTileX, maxTileX);
        firstZ = Math.Clamp(firstZ, minTileZ, maxTileZ);
        lastX  = Math.Clamp(lastX,  minTileX, maxTileX);
        lastZ  = Math.Clamp(lastZ,  minTileZ, maxTileZ);
        mesh.CalcTileLoc(new(Math.Clamp(center.X, meshMinX, meshMaxX), 0, Math.Clamp(center.Z, meshMinZ, meshMaxZ)), out var centerX, out var centerZ);
        centerX = Math.Clamp(centerX, firstX, lastX);
        centerZ = Math.Clamp(centerZ, firstZ, lastZ);

        var state   = new SearchState(center, boundsMin, boundsMax, filter);
        var maxRing = Math.Max(Math.Max(centerX - firstX, lastX - centerX), Math.Max(centerZ - firstZ, lastZ - centerZ));

        for (var ring = 0; ring <= maxRing; ++ring)
        {
            var left   = Math.Max(firstX, centerX - ring);
            var right  = Math.Min(lastX, centerX  + ring);
            var top    = Math.Max(firstZ, centerZ - ring);
            var bottom = Math.Min(lastZ, centerZ  + ring);

            for (var tileZ = top; tileZ <= bottom; ++tileZ)
            {
                if (tileZ == centerZ - ring || tileZ == centerZ + ring)
                {
                    for (var tileX = left; tileX <= right; ++tileX)
                        SearchTile(tileX, tileZ, ref state);
                }
                else
                {
                    if (left == centerX - ring)
                        SearchTile(left, tileZ, ref state);
                    if (right != left && right == centerX + ring)
                        SearchTile(right, tileZ, ref state);
                }
            }

            var remainingDistance = float.PositiveInfinity;
            if (left > firstX)
                remainingDistance = MathF.Min(remainingDistance, center.X - (parameters.orig.X + (left * parameters.tileWidth)));
            if (right < lastX)
                remainingDistance = MathF.Min(remainingDistance, parameters.orig.X + ((right + 1) * parameters.tileWidth) - center.X);
            if (top > firstZ)
                remainingDistance = MathF.Min(remainingDistance, center.Z - (parameters.orig.Z + (top * parameters.tileHeight)));
            if (bottom < lastZ)
                remainingDistance = MathF.Min(remainingDistance, parameters.orig.Z + ((bottom + 1) * parameters.tileHeight) - center.Z);
            remainingDistance = MathF.Max(0, remainingDistance - 0.001f);
            if (remainingDistance * remainingDistance > state.DistanceSquared)
                break;
        }

        nearestPoint = state.Point.ToSystem();
        isOverPoly   = state.OverPoly;
        return state.Reference;
    }

    private void SearchTile(int tileX, int tileZ, ref SearchState state)
    {
        Span<int> tiles = stackalloc int[32];
        var       count = mesh.GetTilesAt(tileX, tileZ, tiles, tiles.Length);

        for (var tileOrder = 0; tileOrder < count; ++tileOrder)
        {
            var tile          = mesh.GetTile(tiles[tileOrder]);
            var data          = tile.data;
            var header        = data.header;
            var baseReference = mesh.GetPolyRefBase(tile);

            if (data.bvTree != null)
            {
                var factor = header.bvQuantFactor;
                var queryMin = new RcVec3i
                {
                    X = (int)(factor * (Math.Clamp(state.Min.X, header.bmin.X, header.bmax.X) - header.bmin.X)) & 0x7ffffffe,
                    Y = (int)(factor * (Math.Clamp(state.Min.Y, header.bmin.Y, header.bmax.Y) - header.bmin.Y)) & 0x7ffffffe,
                    Z = (int)(factor * (Math.Clamp(state.Min.Z, header.bmin.Z, header.bmax.Z) - header.bmin.Z)) & 0x7ffffffe
                };
                var queryMax = new RcVec3i
                {
                    X = (int)((factor * (Math.Clamp(state.Max.X, header.bmin.X, header.bmax.X) - header.bmin.X)) + 1) | 1,
                    Y = (int)((factor * (Math.Clamp(state.Max.Y, header.bmin.Y, header.bmax.Y) - header.bmin.Y)) + 1) | 1,
                    Z = (int)((factor * (Math.Clamp(state.Max.Z, header.bmin.Z, header.bmax.Z) - header.bmin.Z)) + 1) | 1
                };
                var inverseFactor = 1f / factor;
                var nodeIndex     = 0;

                while (nodeIndex < header.bvNodeCount)
                {
                    var node    = data.bvTree[nodeIndex];
                    var overlap = DtUtils.OverlapQuantBounds(queryMin, queryMax, node.bmin, node.bmax);

                    if (overlap)
                    {
                        var nodeMin = new RcVec3f
                        (
                            header.bmin.X                                       + ((node.bmin.X - 1) * inverseFactor),
                            header.bmin.Y + ((node.bmin.Y - 1) * inverseFactor) - header.walkableClimb,
                            header.bmin.Z                                       + ((node.bmin.Z - 1) * inverseFactor)
                        );
                        var nodeMax = new RcVec3f
                        (
                            header.bmin.X + ((node.bmax.X + 1) * inverseFactor),
                            header.bmin.Y + ((node.bmax.Y + 1) * inverseFactor) + header.walkableClimb,
                            header.bmin.Z + ((node.bmax.Z + 1) * inverseFactor)
                        );
                        overlap = DistanceSquaredToBounds(state.Center, nodeMin, nodeMax) <= state.DistanceSquared;
                    }

                    if (node.i >= 0 && overlap)
                        Evaluate(tile, baseReference | (uint)node.i, data.polys[node.i], tileOrder, nodeIndex, ref state);

                    nodeIndex += overlap || node.i >= 0 ?
                                     1 :
                                     -node.i;
                }
            }
            else
            {
                for (var polyIndex = 0; polyIndex < header.polyCount; ++polyIndex)
                {
                    var poly = data.polys[polyIndex];
                    if (poly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
                        continue;

                    var boundsMin = data.verts.ToVec3(poly.verts[0] * 3);
                    var boundsMax = boundsMin;

                    for (var vertex = 1; vertex < poly.vertCount; ++vertex)
                    {
                        var position = data.verts.ToVec3(poly.verts[vertex] * 3);
                        boundsMin = RcVec3f.Min(boundsMin, position);
                        boundsMax = RcVec3f.Max(boundsMax, position);
                    }

                    if (DtUtils.OverlapBounds(state.Min, state.Max, boundsMin, boundsMax))
                        Evaluate(tile, baseReference | (uint)polyIndex, poly, tileOrder, polyIndex, ref state);
                }
            }
        }
    }

    private void Evaluate(DtMeshTile tile, long reference, DtPoly poly, int tileOrder, int polyOrder, ref SearchState state)
    {
        if (!state.Filter.PassFilter(reference, tile, poly))
            return;

        mesh.ClosestPointOnPoly(reference, state.Center, out var point, out var overPoly);
        var difference = state.Center - point;
        var distance   = difference.LengthSquared();

        if (overPoly)
        {
            var height = MathF.Max(0, MathF.Abs(difference.Y) - tile.data.header.walkableClimb);
            distance = height * height;
        }

        if (distance > state.DistanceSquared)
            return;

        if (distance.CompareTo(state.DistanceSquared) == 0)
        {
            var order = tile.data.header.y.CompareTo(state.TileZ);
            if (order == 0)
                order = tile.data.header.x.CompareTo(state.TileX);
            if (order == 0)
                order = tileOrder.CompareTo(state.TileOrder);
            if (order == 0)
                order = polyOrder.CompareTo(state.PolyOrder);
            if (order >= 0)
                return;
        }

        state.DistanceSquared = distance;
        state.Reference       = reference;
        state.Point           = point;
        state.OverPoly        = overPoly;
        state.TileX           = tile.data.header.x;
        state.TileZ           = tile.data.header.y;
        state.TileOrder       = tileOrder;
        state.PolyOrder       = polyOrder;
    }

    private static float DistanceSquaredToBounds(RcVec3f point, RcVec3f min, RcVec3f max)
    {
        var deltaX = MathF.Max(0, MathF.Max(min.X - point.X, point.X - max.X) - 0.001f);
        var deltaY = MathF.Max(0, MathF.Max(min.Y - point.Y, point.Y - max.Y) - 0.001f);
        var deltaZ = MathF.Max(0, MathF.Max(min.Z - point.Z, point.Z - max.Z) - 0.001f);
        return (deltaX * deltaX) + (deltaY * deltaY) + (deltaZ * deltaZ);
    }

    private struct SearchState
    (
        RcVec3f        center,
        RcVec3f        min,
        RcVec3f        max,
        IDtQueryFilter filter
    )
    {
        internal readonly RcVec3f        Center          = center;
        internal readonly RcVec3f        Min             = min;
        internal readonly RcVec3f        Max             = max;
        internal readonly IDtQueryFilter Filter          = filter;
        internal          float          DistanceSquared = float.MaxValue;
        internal          long           Reference;
        internal          RcVec3f        Point = center;
        internal          bool           OverPoly;
        internal          int            TileX;
        internal          int            TileZ;
        internal          int            TileOrder;
        internal          int            PolyOrder;
    }
}
