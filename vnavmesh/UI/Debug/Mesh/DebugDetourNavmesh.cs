using System.Numerics;
using Dalamud.Bindings.ImGui;
using DotRecast.Detour;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Utilities;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Debug.Recast;
using vnavmesh.UI.Rendering;

namespace vnavmesh.UI.Debug.Mesh;

using static DotRecast.Detour.DtDetour;

public class DebugDetourNavmesh : DebugRecast
{
    private struct PerTile
    {
        public EffectMesh.Data? VisuRough; // non-detail mesh
        public EffectMesh.Data? VisuDetail;
    }

    private DtNavMesh       _navmesh;
    private DtNavMeshQuery? _query;
    private UITree          _tree;
    private DebugDrawer     _dd;
    private PerTile[]       _perTile;
    private List<long>      _path;
    private float           _closedListRenderHorizontalDistance = 50f;
    private float           _closedListRenderVerticalDistance   = 10f;

    private static Vector4 _colAreaNull      = new(0.00f, 0.00f, 0.00f, 0.25f);
    private static Vector4 _colAreaGround    = new(0.00f, 0.75f, 1.00f, 0.50f);
    private static Vector4 _colAreaClimb     = new(0.18f, 0.80f, 0.44f, 0.60f);
    private static Vector4 _colAreaJump      = new(0.20f, 0.60f, 0.86f, 0.60f);
    private static Vector4 _colAreaManual    = new(0.95f, 0.61f, 0.07f, 0.60f);
    private static Vector4 _colAreaTeleport   = new(0.91f, 0.30f, 0.24f, 0.60f);
    private static Vector4 _colAreaClientPath = new(0.65f, 0.36f, 0.95f, 0.60f);
    private static Vector4 _colAreaUnreachable = new(0.50f, 0.50f, 0.50f, 0.65f);
    private static Vector4 _colClosedList     = new(1.00f, 0.75f, 1.00f, 0.50f);

    private enum InstanceID
    {
        Tile,
        AreaNull,
        AreaGround,
        AreaClimb,
        AreaJump,
        AreaManual,
        AreaTeleport,
        AreaClientPath,
        AreaUnreachable,
        ClosedList,
        Count
    }

    public DebugDetourNavmesh(DtNavMesh navmesh, DtNavMeshQuery? query, List<long> queryPath, UITree tree, DebugDrawer dd)
    {
        _navmesh = navmesh;
        _query   = query;
        _path    = queryPath;
        _tree    = tree;
        _dd      = dd;
        _perTile = new PerTile[navmesh.GetParams().maxTiles];
    }

    public override void Dispose()
    {
        foreach (var perTile in _perTile)
        {
            perTile.VisuRough?.Dispose();
            perTile.VisuDetail?.Dispose();
        }
    }

    public void Draw()
    {
        DrawMesh();
        DrawQuery();
    }

    private void DrawMesh()
    {
        using var nr = _tree.Node("Detour 导航网格");
        if (!nr.Opened)
            return;

        ref readonly var param = ref _navmesh.GetParams();
        _tree.LeafNode($"原点：{param.orig}");
        _tree.LeafNode($"区块大小：{param.tileWidth:f3}x{param.tileHeight:f3} (每区块最多 {param.maxPolys} 个多边形)");
        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("ClosedList 水平渲染距离###closedListRenderHorizontalDistance", ref _closedListRenderHorizontalDistance, 2f, 500f, "%.0f");
        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("ClosedList 垂直渲染距离###closedListRenderVerticalDistance", ref _closedListRenderVerticalDistance, 1f, 100f, "%.0f");

        using var nt = _tree.Node($"区块 (最多 {param.maxTiles} 个)###tiles");
        if (nt.SelectedOrHovered)
            VisualizeWithClosedList();

        if (nt.Opened)
        {
            for (var i = 0; i < param.maxTiles; ++i)
            {
                var tile = _navmesh.GetTile(i);
                if (tile.data == null)
                    continue;
                using var ntile = _tree.Node
                (
                    $"区块 {i} (位于 {tile.data.header.x}x{tile.data.header.y}x{tile.data.header.layer})：标志={tile.flags:X}，Salt={tile.salt}，基础多边形引用={_navmesh.GetPolyRefBase(tile):X}###{i}"
                );
                if (!ntile.Opened)
                    continue;

                _tree.LeafNode($"头部：Magic={tile.data.header.magic:X}，版本={tile.data.header.version}，用户 ID={tile.data.header.userId}");
                _tree.LeafNode($"边界：[{tile.data.header.bmin}]-[{tile.data.header.bmax}] (量化因子={tile.data.header.bvQuantFactor})");

                using (var np = _tree.Node($"多边形 ({tile.data.header.polyCount})"))
                {
                    if (np.SelectedOrHovered)
                        VisualizeRoughPolygons(tile, true);

                    if (np.Opened)
                    {
                        for (var j = 0; j < tile.data.header.polyCount; ++j)
                        {
                            var       p    = tile.data.polys[j];
                            var       polyRef  = EncodePolyId(tile.salt, tile.index, p.index);
                            var       flagText = TryGetPolyFlags(polyRef, out var polyFlags) ? $"{polyFlags:X}" : "<err>";
                            using var ntri = _tree.Node($"{p.index} (0x{p.index:X})：{p.vertCount} 个顶点，标志={p.flags:X} / 实际标志={flagText}，面积类型={p.GetArea()}，多边形类型={p.GetPolyType()}");
                            if (ntri.SelectedOrHovered)
                                VisualizeRoughPolygon(tile, p, true);

                            if (ntri.Opened)
                            {
                                for (var k = 0; k < p.vertCount; ++k)
                                    if (_tree.LeafNode($"{p.verts[k]} ({GetVertex(tile, p.verts[k])})，邻接={p.neis[k]:X}").SelectedOrHovered)
                                        VisualizeVertex(GetVertex(tile, p.verts[k]));

                                for (var k = p.firstLink; k != DT_NULL_LINK; k = tile.links[k].next)
                                {
                                    var link = tile.links[k];
                                    if (_tree.LeafNode($"链接 {k}：引用={link.refs:X}，边缘={link.edge}，侧面={link.side}，bmin={link.bmin}，bmax={link.bmax}").SelectedOrHovered)
                                        VisualizeRoughPolygon(link.refs, true);
                                }
                            }
                        }
                    }
                }

                using (var nv = _tree.Node($"顶点 ({tile.data.header.vertCount})"))
                {
                    if (nv.Opened)
                    {
                        for (var j = 0; j < tile.data.header.vertCount; ++j)
                            if (_tree.LeafNode($"{j}: {GetVertex(tile, j):f3}").SelectedOrHovered)
                                VisualizeVertex(GetVertex(tile, j));
                    }
                }

                using (var nd = _tree.Node
                           ($"细节网格 ({tile.data.header.detailMeshCount} 个子网格，共 {tile.data.header.detailVertCount} 个顶点，共 {tile.data.header.detailTriCount} 个图元)"))
                {
                    if (nd.SelectedOrHovered)
                        VisualizeDetailPolygons(tile, true);

                    if (nd.Opened)
                    {
                        for (var j = 0; j < tile.data.header.detailMeshCount; ++j)
                        {
                            var       poly = tile.data.polys[j];
                            ref var   sub  = ref tile.data.detailMeshes[j];
                            using var nsub = _tree.Node($"{j}：基础顶点={poly.vertCount}");
                            if (nsub.SelectedOrHovered)
                                VisualizeDetailSubmesh(tile, j, true);
                            if (!nsub.Opened)
                                continue;

                            using (var np = _tree.Node($"三角形 ({sub.triCount} 个三角形，起始于 {sub.triBase})"))
                            {
                                if (np.Opened)
                                {
                                    for (var k = 0; k < sub.triCount; ++k)
                                    {
                                        var offset = (sub.triBase + k) * 4;
                                        var v1i    = tile.data.detailTris[offset];
                                        var v2i    = tile.data.detailTris[offset + 1];
                                        var v3i    = tile.data.detailTris[offset + 2];
                                        var flags  = tile.data.detailTris[offset + 3];
                                        var v1     = GetDetailVertex(tile, poly, v1i);
                                        var v2     = GetDetailVertex(tile, poly, v2i);
                                        var v3     = GetDetailVertex(tile, poly, v3i);
                                        if (_tree.LeafNode($"{k}: {v1i}x{v2i}x{v3i} ({v1:f3}x{v2:f3}x{v3:f3})，标志={flags:X}").SelectedOrHovered)
                                            VisualizeTriangle(v1, v2, v3, 0xff000000, 2);
                                    }
                                }
                            }

                            using (var nv = _tree.Node($"顶点 ({sub.vertCount} 个顶点，起始于 {sub.vertBase})"))
                            {
                                if (nv.Opened)
                                {
                                    for (var k = 0; k < sub.vertCount; ++k)
                                    {
                                        var v = GetDetailVertex(tile, sub.vertBase + k);
                                        if (_tree.LeafNode($"{k}: {v:f3}").SelectedOrHovered)
                                            VisualizeVertex(v);
                                    }
                                }
                            }
                        }
                    }
                }

                _tree.LeafNode($"链接 ({tile.data.header.maxLinkCount} 个上限)");
                _tree.LeafNode($"包围体 ({tile.data.header.bvNodeCount} 个节点)");
                _tree.LeafNode($"离网链接 ({tile.data.header.offMeshConCount} 个，起始于图元 #{tile.data.header.offMeshBase})");
            }
        }
    }

    private void DrawQuery()
    {
        using var nr = _tree.Node("Detour 查询", _query == null || _query.GetNodePool().GetNodeCount() == 0);
        if (!nr.Opened)
            return;

        var i = 1;

        foreach (var n in _query!.GetNodePool().AsEnumerable())
        {
            var queried = _path.Any(p => p == n.id);
            var node = _tree.LeafNode
                ($"{i++}: {n.id:X}, 父节点={n.pidx}, 进入={n.pos}, 消耗={n.cost}, 总计={n.total}, 状态={n.state}, 标志={n.flags}", queried ? 0xff808000 : 0xffffffff);

            if (node.SelectedOrHovered || queried)
            {
                VisualizeRoughPolygon(n.id, true, node.SelectedOrHovered);
                VisualizeVertex(n.pos.RecastToSystem());
            }
        }
    }

    private EffectMesh.Data GetOrInitVisualizerRough(int tileIndex)
    {
        ref var perTile = ref _perTile[tileIndex];

        if (perTile.VisuRough == null)
        {
            var primsPerPoly = _navmesh.GetMaxVertsPerPoly() - 2;
            var tile         = _navmesh.GetTile(tileIndex);
            perTile.VisuRough = new(_dd.RenderContext, tile.data.header.vertCount, tile.data.header.polyCount * primsPerPoly, (int)InstanceID.Count, false);
            using var builder = perTile.VisuRough.Map(_dd.RenderContext);

            var timer = StopWatchTimer.Create();

            // instances differ only by color
            builder.AddInstance(new(Matrix4x3.Identity, IntColor(tileIndex, 0.75f)));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaNull));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaGround));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaClimb));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaJump));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaManual));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaTeleport));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaClientPath));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaUnreachable));
            builder.AddInstance(new(Matrix4x3.Identity, _colClosedList));

            for (var i = 0; i < tile.data.header.vertCount; ++i)
                builder.AddVertex(GetVertex(tile, i));

            // one 'mesh' per polygon; by default, assign color based on tile index
            var startingPrimitive = 0;

            for (var i = 0; i < tile.data.header.polyCount; ++i)
            {
                var p = tile.data.polys[i];

                if (p.GetPolyType() != DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION && p.vertCount >= 3)
                {
                    for (var j = 2; j < p.vertCount; ++j)
                        builder.AddTriangle(p.verts[0], p.verts[j], p.verts[j - 1]); // flipped for dx order
                    builder.AddMesh(0, startingPrimitive, p.vertCount - 2, 0, 1);
                    startingPrimitive += p.vertCount - 2;
                }
                else
                {
                    // add empty mesh to ensure indices are matching
                    builder.AddMesh(0, startingPrimitive, 0, 0, 0);
                }
            }

            Service.Log.Debug($"导航网格粗略可视化区块 #{tileIndex} 构建耗时：{timer.Value().TotalMilliseconds:f3}ms");
        }

        return perTile.VisuRough;
    }

    private EffectMesh.Data GetOrInitVisualizerDetail(int tileIndex)
    {
        ref var perTile = ref _perTile[tileIndex];

        if (perTile.VisuDetail == null)
        {
            var tile = _navmesh.GetTile(tileIndex);
            perTile.VisuDetail = new
                (_dd.RenderContext, tile.data.header.vertCount + tile.data.header.detailVertCount, tile.data.header.detailTriCount, (int)InstanceID.Count, false);
            using var builder = perTile.VisuDetail.Map(_dd.RenderContext);

            var timer = StopWatchTimer.Create();

            // instances differ only by color
            builder.AddInstance(new(Matrix4x3.Identity, IntColor(tileIndex, 0.75f)));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaNull));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaGround));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaClimb));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaJump));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaManual));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaTeleport));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaClientPath));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaUnreachable));
            builder.AddInstance(new(Matrix4x3.Identity, _colClosedList));

            for (var i = 0; i < tile.data.header.vertCount; ++i)
                builder.AddVertex(GetVertex(tile, i));
            for (var i = 0; i < tile.data.header.detailVertCount; ++i)
                builder.AddVertex(GetDetailVertex(tile, i));

            // one 'mesh' per polygon; by default, assign color based on tile index
            var startingPrimitive = 0;

            for (var i = 0; i < tile.data.header.detailMeshCount; ++i)
            {
                var     poly = tile.data.polys[i];
                ref var sub  = ref tile.data.detailMeshes[i];

                for (var j = 0; j < sub.triCount; ++j)
                {
                    var offset = (sub.triBase + j) * 4;
                    var v1i    = tile.data.detailTris[offset];
                    var v2i    = tile.data.detailTris[offset + 1];
                    var v3i    = tile.data.detailTris[offset + 2];
                    var v1 = v1i < poly.vertCount
                                 ? poly.verts[v1i]
                                 : tile.data.header.vertCount + tile.data.detailMeshes[poly.index].vertBase + v1i - poly.vertCount;
                    var v2 = v2i < poly.vertCount
                                 ? poly.verts[v2i]
                                 : tile.data.header.vertCount + tile.data.detailMeshes[poly.index].vertBase + v2i - poly.vertCount;
                    var v3 = v3i < poly.vertCount
                                 ? poly.verts[v3i]
                                 : tile.data.header.vertCount + tile.data.detailMeshes[poly.index].vertBase + v3i - poly.vertCount;
                    builder.AddTriangle(v1, v3, v2); // flipped for dx order
                }

                builder.AddMesh(0, startingPrimitive, sub.triCount, 0, 1);
                startingPrimitive += sub.triCount;
            }

            Service.Log.Debug($"导航网格细节可视化区块 #{tileIndex} 构建耗时：{timer.Value().TotalMilliseconds:f3}ms");
        }

        return perTile.VisuDetail;
    }

    private void VisualizeWithClosedList()
    {
        var playerPosition        = Service.ObjectTable.LocalPlayer?.Position;
        var maxHorizontalDistance = _closedListRenderHorizontalDistance;
        var maxVerticalDistance   = _closedListRenderVerticalDistance;

        for (var i = 0; i < _perTile.Length; ++i)
        {
            var tile = _navmesh.GetTile(i);
            if (tile.data != null)
                VisualizeDetailPolygons(tile, true, playerPosition, maxHorizontalDistance, maxVerticalDistance);
        }
    }

    private void VisualizeRoughPolygons(DtMeshTile tile, bool colorByArea)
    {
        if (_dd.EffectMesh == null)
            return;
        var visu = GetOrInitVisualizerRough(tile.index);
        _dd.EffectMesh.Bind(_dd.RenderContext, false, false);
        visu.Bind(_dd.RenderContext);
        for (var i = 0; i < tile.data.header.polyCount; ++i)
            VisualizeRoughPolygon(tile, visu, tile.data.polys[i], colorByArea, false);
    }

    private void VisualizeRoughPolygon(DtMeshTile tile, DtPoly poly, bool colorByArea, bool highlight = false)
    {
        if (_dd.EffectMesh == null)
            return;
        var visu = GetOrInitVisualizerRough(tile.index);
        _dd.EffectMesh.Bind(_dd.RenderContext, false, false);
        visu.Bind(_dd.RenderContext);
        VisualizeRoughPolygon(tile, visu, poly, colorByArea, highlight);
    }

    private void VisualizeRoughPolygon(long refs, bool colorByArea, bool highlight = false)
    {
        if (_navmesh.GetTileAndPolyByRef(refs, out var tile, out var poly).Succeeded())
            VisualizeRoughPolygon(tile, poly, colorByArea, highlight);
    }

    // effect + data are expected to be already bound
    private void VisualizeRoughPolygon(DtMeshTile tile, EffectMesh.Data visu, DtPoly poly, bool colorByArea, bool highlight)
    {
        if (poly.GetPolyType() != DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
        {
            if (poly.vertCount < 3)
                return;
            // triangles
            var polyRef   = EncodePolyId(tile.salt, tile.index, poly.index);
            var instance = _query != null && _query.IsInClosedList(polyRef) ? InstanceID.ClosedList :
                           !colorByArea ? InstanceID.Tile :
                           InstanceForPoly(polyRef, (NavmeshArea)poly.GetArea());
            var mesh = visu.Meshes[poly.index] with { FirstInstance = (int)instance };
            visu.DrawManual(_dd.RenderContext, mesh);

            // edges
            var from = GetVertex(tile, poly.verts[0]);

            for (var i = 0; i < poly.vertCount; ++i)
            {
                var to    = GetVertex(tile, poly.verts[i == poly.vertCount - 1 ? 0 : i + 1]);
                var inner = poly.neis[i] != 0;
                var color = 0xd8403000;

                if (inner)
                {
                    color = 0x20403000;

                    if ((poly.neis[i] & DT_EXT_LINK) != 0)
                    {
                        var con = false;

                        for (var k = poly.firstLink; k != DT_NULL_LINK; k = tile.links[k].next)
                            if (tile.links[k].edge == i)
                            {
                                con = true;
                                break;
                            }

                        color = con ? 0x30ffffffu : 0x30000000u;
                    }
                }

                if (highlight)
                    color |= 0xff000000;
                _dd.DrawWorldLine(from, to, color, highlight ? 3 : inner ? 1 : 2);
                from = to;
            }

            // vertices
            for (var i = 0; i < poly.vertCount; ++i)
                _dd.DrawWorldPoint(GetVertex(tile, poly.verts[i]), 3, 0xff000000);
        }
        // TODO: ...
    }

    private void VisualizeDetailPolygons
    (
        DtMeshTile tile,
        bool       colorByArea,
        Vector3?   playerPosition        = null,
        float      maxHorizontalDistance = float.PositiveInfinity,
        float      maxVerticalDistance   = float.PositiveInfinity
    )
    {
        if (_dd.EffectMesh == null)
            return;
        var visu = GetOrInitVisualizerDetail(tile.index);
        _dd.EffectMesh.Bind(_dd.RenderContext, false, false);
        visu.Bind(_dd.RenderContext);
        for (var i = 0; i < tile.data.header.detailMeshCount; ++i)
        {
            var poly = tile.data.polys[i];
            if (ShouldVisualizeDetailSubmesh(tile, poly, playerPosition, maxHorizontalDistance, maxVerticalDistance))
                VisualizeDetailSubmeshWithEdges(tile, visu, poly, colorByArea, false);
        }

        if (playerPosition == null)
        {
            for (var i = 0; i < tile.data.header.vertCount; ++i)
                _dd.DrawWorldPointFilled(GetVertex(tile, i), 3, 0xff00ff00);
            for (var i = 0; i < tile.data.header.detailVertCount; ++i)
                _dd.DrawWorldPointFilled(GetDetailVertex(tile, i), 2, 0xff0000ff);
        }
    }

    private void VisualizeDetailSubmesh(DtMeshTile tile, int index, bool colorByArea)
    {
        if (_dd.EffectMesh == null)
            return;
        var     poly = tile.data.polys[index];
        ref var sub  = ref tile.data.detailMeshes[poly.index];

        var visu = GetOrInitVisualizerDetail(tile.index);
        _dd.EffectMesh.Bind(_dd.RenderContext, false, false);
        visu.Bind(_dd.RenderContext);
        VisualizeDetailSubmeshWithEdges(tile, visu, poly, colorByArea, true);

        // vertices
        for (var i = 0; i < poly.vertCount; ++i)
            _dd.DrawWorldPointFilled(GetVertex(tile, poly.verts[i]), 3, 0xff00ff00);
        for (var i = 0; i < sub.vertCount; ++i)
            _dd.DrawWorldPointFilled(GetDetailVertex(tile, sub.vertBase + i), 2, 0xff0000ff);
    }

    private void VisualizeDetailSubmeshWithEdges(DtMeshTile tile, EffectMesh.Data visu, DtPoly poly, bool colorByArea, bool highlight)
    {
        // triangles
        var polyRef   = EncodePolyId(tile.salt, tile.index, poly.index);
        var instance = _query != null && _query.IsInClosedList(polyRef) ? InstanceID.ClosedList :
                       !colorByArea ? InstanceID.Tile :
                       InstanceForPoly(polyRef, (NavmeshArea)poly.GetArea());
        var mesh = visu.Meshes[poly.index] with { FirstInstance = (int)instance };
        visu.DrawManual(_dd.RenderContext, mesh);

        // edges
        ref var sub   = ref tile.data.detailMeshes[poly.index];
        var     color = highlight ? 0xff000000 : 0x80000000;

        for (var i = 0; i < sub.triCount; ++i)
        {
            var offset = (sub.triBase + i) * 4;
            var v1i    = tile.data.detailTris[offset];
            var v2i    = tile.data.detailTris[offset + 1];
            var v3i    = tile.data.detailTris[offset + 2];
            var flags  = tile.data.detailTris[offset + 3];
            var v1     = GetDetailVertex(tile, poly, v1i);
            var v2     = GetDetailVertex(tile, poly, v2i);
            var v3     = GetDetailVertex(tile, poly, v3i);
            _dd.DrawWorldLine(v1, v2, color, (GetDetailTriEdgeFlags(flags, 0) & DtDetailTriEdgeFlags.DT_DETAIL_EDGE_BOUNDARY) != 0 ? 2 : 1);
            _dd.DrawWorldLine(v2, v3, color, (GetDetailTriEdgeFlags(flags, 1) & DtDetailTriEdgeFlags.DT_DETAIL_EDGE_BOUNDARY) != 0 ? 2 : 1);
            _dd.DrawWorldLine(v3, v1, color, (GetDetailTriEdgeFlags(flags, 2) & DtDetailTriEdgeFlags.DT_DETAIL_EDGE_BOUNDARY) != 0 ? 2 : 1);
        }
    }

    private bool ShouldVisualizeDetailSubmesh(DtMeshTile tile, DtPoly poly, Vector3? playerPosition, float maxHorizontalDistance, float maxVerticalDistance)
    {
        if (playerPosition == null)
            return true;

        ref var sub = ref tile.data.detailMeshes[poly.index];
        for (var i = 0; i < sub.triCount; ++i)
        {
            var offset = (sub.triBase + i) * 4;
            var v1     = GetDetailVertex(tile, poly, tile.data.detailTris[offset]);
            var v2     = GetDetailVertex(tile, poly, tile.data.detailTris[offset + 1]);
            var v3     = GetDetailVertex(tile, poly, tile.data.detailTris[offset + 2]);
            if (IsTriangleWithinClosedListRenderDistance(playerPosition.Value, v1, v2, v3, maxHorizontalDistance, maxVerticalDistance))
                return true;
        }

        return false;
    }

    private static bool IsTriangleWithinClosedListRenderDistance
    (
        Vector3 playerPosition,
        Vector3 v1,
        Vector3 v2,
        Vector3 v3,
        float   maxHorizontalDistance,
        float   maxVerticalDistance
    )
    {
        if (IsVertexWithinClosedListRenderDistance(playerPosition, v1, maxHorizontalDistance, maxVerticalDistance) ||
            IsVertexWithinClosedListRenderDistance(playerPosition, v2, maxHorizontalDistance, maxVerticalDistance) ||
            IsVertexWithinClosedListRenderDistance(playerPosition, v3, maxHorizontalDistance, maxVerticalDistance))
            return true;

        var minY = MathF.Min(v1.Y, MathF.Min(v2.Y, v3.Y));
        var maxY = MathF.Max(v1.Y, MathF.Max(v2.Y, v3.Y));
        if (playerPosition.Y < minY - maxVerticalDistance || playerPosition.Y > maxY + maxVerticalDistance)
            return false;

        var playerXZ = new Vector2(playerPosition.X, playerPosition.Z);
        var v1XZ     = new Vector2(v1.X, v1.Z);
        var v2XZ     = new Vector2(v2.X, v2.Z);
        var v3XZ     = new Vector2(v3.X, v3.Z);
        var closest  = ClosestPointOnTriangleXZ(playerXZ, v1XZ, v2XZ, v3XZ);
        return Vector2.DistanceSquared(playerXZ, closest) <= maxHorizontalDistance * maxHorizontalDistance;
    }

    private static bool IsVertexWithinClosedListRenderDistance(Vector3 playerPosition, Vector3 vertex, float maxHorizontalDistance, float maxVerticalDistance)
    {
        var horizontalDelta = new Vector2(playerPosition.X - vertex.X, playerPosition.Z - vertex.Z);
        var verticalDelta   = MathF.Abs(playerPosition.Y - vertex.Y);
        return horizontalDelta.LengthSquared() <= maxHorizontalDistance * maxHorizontalDistance && verticalDelta <= maxVerticalDistance;
    }

    private static Vector2 ClosestPointOnTriangleXZ(Vector2 point, Vector2 a, Vector2 b, Vector2 c)
    {
        if (TryProjectPointInsideTriangleXZ(point, a, b, c, out var projected))
            return projected;

        var ab = ClosestPointOnSegmentXZ(point, a, b);
        var bc = ClosestPointOnSegmentXZ(point, b, c);
        var ca = ClosestPointOnSegmentXZ(point, c, a);

        var abDist = Vector2.DistanceSquared(point, ab);
        var bcDist = Vector2.DistanceSquared(point, bc);
        var caDist = Vector2.DistanceSquared(point, ca);

        if (abDist <= bcDist && abDist <= caDist)
            return ab;
        return bcDist <= caDist ? bc : ca;
    }

    private static bool TryProjectPointInsideTriangleXZ(Vector2 point, Vector2 a, Vector2 b, Vector2 c, out Vector2 projected)
    {
        var v0  = b - a;
        var v1  = c - a;
        var v2  = point - a;
        var d00 = Vector2.Dot(v0, v0);
        var d01 = Vector2.Dot(v0, v1);
        var d11 = Vector2.Dot(v1, v1);
        var d20 = Vector2.Dot(v2, v0);
        var d21 = Vector2.Dot(v2, v1);
        var den = d00 * d11 - d01 * d01;

        if (MathF.Abs(den) <= float.Epsilon)
        {
            projected = default;
            return false;
        }

        var v = (d11 * d20 - d01 * d21) / den;
        var w = (d00 * d21 - d01 * d20) / den;
        var u = 1 - v - w;

        if (u < 0 || v < 0 || w < 0)
        {
            projected = default;
            return false;
        }

        projected = u * a + v * b + w * c;
        return true;
    }

    private static Vector2 ClosestPointOnSegmentXZ(Vector2 point, Vector2 a, Vector2 b)
    {
        var ab = b - a;
        var lengthSq = ab.LengthSquared();
        if (lengthSq <= float.Epsilon)
            return a;

        var progress = Math.Clamp(Vector2.Dot(point - a, ab) / lengthSq, 0f, 1f);
        return a + progress * ab;
    }

    private void VisualizeTriangle(Vector3 v1, Vector3 v2, Vector3 v3, uint color, int thickness)
    {
        _dd.DrawWorldLine(v1, v2, color, thickness);
        _dd.DrawWorldLine(v2, v3, color, thickness);
        _dd.DrawWorldLine(v3, v1, color, thickness);
    }

    private void VisualizeVertex(Vector3 v) => _dd.DrawWorldPoint(v, 5, 0xff0000ff, 2);

    private InstanceID InstanceForPoly(long polyRef, NavmeshArea area)
    {
        if (TryGetPolyFlags(polyRef, out var flags) && (flags & (int)NavmeshPolyFlags.Unreachable) != 0)
            return InstanceID.AreaUnreachable;

        return InstanceForArea(area);
    }

    private bool TryGetPolyFlags(long polyRef, out int flags) => _navmesh.GetPolyFlags(polyRef, out flags).Succeeded();

    private static InstanceID InstanceForArea(NavmeshArea area) => area switch
    {
        NavmeshArea.Null               => InstanceID.AreaNull,
        NavmeshArea.Ground             => InstanceID.AreaGround,
        NavmeshArea.GeneratedClimbDown => InstanceID.AreaClimb,
        NavmeshArea.GeneratedEdgeJump  => InstanceID.AreaJump,
        NavmeshArea.ManualOffMesh      => InstanceID.AreaManual,
        NavmeshArea.Teleport           => InstanceID.AreaTeleport,
        NavmeshArea.ClientPath         => InstanceID.AreaClientPath,
        _                              => InstanceID.AreaGround
    };

    private Vector3 GetVertex(DtMeshTile tile, int i)
    {
        try
        {
            return new(tile.data.verts[i * 3], tile.data.verts[i * 3 + 1], tile.data.verts[i * 3 + 2]);
        }
        catch (IndexOutOfRangeException)
        {
            Service.Log.Debug($"vertex {i} is out of bounds for tile");
            throw;
        }
    }

    private Vector3 GetDetailVertex(DtMeshTile tile, int i) => new(tile.data.detailVerts[i * 3], tile.data.detailVerts[i * 3 + 1], tile.data.detailVerts[i * 3 + 2]);

    private Vector3 GetDetailVertex(DtMeshTile tile, DtPoly poly, int localIndex) => localIndex < poly.vertCount
                                                                                         ? GetVertex(tile, poly.verts[localIndex])
                                                                                         : GetDetailVertex
                                                                                         (
                                                                                             tile,
                                                                                             tile.data.detailMeshes[poly.index].vertBase +
                                                                                             localIndex -
                                                                                             poly.vertCount
                                                                                         );
}
