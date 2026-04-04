using System;
using System.Numerics;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Interface.Debug.Components;
using vnavmesh.Interface.Render;
using vnavmesh.Utils;

namespace vnavmesh.Interface.Debug;

public class DebugPolyMesh : DebugRecast
{
    private RcPolyMesh _mesh;
    private UITree _tree;
    private DebugDrawer _dd;
    private EffectMesh.Data? _visu;

    private static int _heightOffset = 0;
    private static Vector4 _colAreaNull = new(0, 0, 0, 0.25f);
    private static Vector4 _colAreaWalkable = new(0, 0.75f, 1.0f, 0.25f);

    public DebugPolyMesh(RcPolyMesh mesh, UITree tree, DebugDrawer dd)
    {
        _mesh = mesh;
        _tree = tree;
        _dd = dd;
    }

    public override void Dispose()
    {
        _visu?.Dispose();
    }

    public void Draw()
    {
        using var nr = _tree.Node("多边形网格 (Poly Mesh)");
        if (!nr.Opened)
            return;

        DrawBaseInfo(_tree, _mesh.bmin, _mesh.bmax, _mesh.cs, _mesh.ch);
        _tree.LeafNode($"杂项：边界大小={_mesh.borderSize}，最大边缘误差={_mesh.maxEdgeError}，最大顶点数/多边形={_mesh.nvp}");

        using (var nv = _tree.Node($"顶点 ({_mesh.nverts})###verts", _mesh.nverts == 0))
        {
            if (nv.Opened)
            {
                for (int i = 0; i < _mesh.nverts; ++i)
                    if (_tree.LeafNode($"{i}: {_mesh.verts[3 * i]}x{_mesh.verts[3 * i + 1]}x{_mesh.verts[3 * i + 2]}").SelectedOrHovered)
                        VisualizeVertex(i);
            }
        }

        using (var np = _tree.Node($"多边形 ({_mesh.npolys})###polys", _mesh.npolys == 0))
        {
            if (np.SelectedOrHovered)
                VisualizeMesh();
            if (np.Opened)
            {
                for (int i = 0; i < _mesh.npolys; ++i)
                {
                    using var npoly = _tree.Node($"{i}: 区域={_mesh.regs[i]}, 标志={_mesh.flags[i]:X}, 面积={_mesh.areas[i]}");
                    if (npoly.SelectedOrHovered)
                        VisualizePolygon(i);
                    if (npoly.Opened)
                    {
                        for (int j = 0; j < _mesh.nvp; ++j)
                        {
                            var vertex = _mesh.polys[i * 2 * _mesh.nvp + j];
                            if (vertex == RcConstants.RC_MESH_NULL_IDX)
                                break;
                            if (_tree.LeafNode($"顶点 {j}：#{vertex} = {_mesh.verts[3 * vertex]}x{_mesh.verts[3 * vertex + 1]}x{_mesh.verts[3 * vertex + 2]}").SelectedOrHovered)
                                VisualizeVertex(vertex);
                        }
                        for (int j = 0; j < _mesh.nvp; ++j)
                        {
                            var adj = _mesh.polys[i * 2 * _mesh.nvp + _mesh.nvp + j];
                            if (adj == RcConstants.RC_MESH_NULL_IDX)
                                break;
                            if (_tree.LeafNode($"邻接 {j}：{adj}").SelectedOrHovered)
                                VisualizePolygon(adj);
                        }
                    }
                }
            }
        }
    }

    private EffectMesh.Data GetOrInitVisualizer()
    {
        if (_visu == null)
        {
            var primsPerPoly = _mesh.nvp - 2;
            _visu = new EffectMesh.Data(_dd.RenderContext, _mesh.nverts, _mesh.npolys * primsPerPoly, 2, false);
            using var builder = _visu.Map(_dd.RenderContext);

            var timer = StopWatchTimer.Create();

            // one 'instance' per area
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaNull));
            builder.AddInstance(new(Matrix4x3.Identity, _colAreaWalkable));

            for (int i = 0; i < _mesh.nverts; ++i)
                builder.AddVertex(GetVertex(i));

            // one 'mesh' per polygon, each polygon that has <max vertices is padded
            int startingPrimitive = 0;
            for (int i = 0; i < _mesh.npolys; ++i)
            {
                var offset = i * _mesh.nvp * 2;
                for (int j = 2; j < _mesh.nvp; ++j)
                    builder.AddTriangle(_mesh.polys[offset], _mesh.polys[offset + j], _mesh.polys[offset + j - 1]); // flipped for dx order
                var numTriangles = _mesh.polys.AsSpan(offset, _mesh.nvp).IndexOf(RcConstants.RC_MESH_NULL_IDX);
                if (numTriangles < 0)
                    numTriangles = _mesh.nvp;
                numTriangles = Math.Max(numTriangles - 2, 0);

                builder.AddMesh(0, startingPrimitive, numTriangles, _mesh.areas[i] == 0 ? 0 : 1, 1);
                startingPrimitive += primsPerPoly;
            }
            Service.Log.Debug($"polymesh visualization build time: {timer.Value().TotalMilliseconds:f3}ms");
        }
        return _visu;
    }

    public void VisualizeMesh()
    {
        _dd.EffectMesh?.Draw(_dd.RenderContext, GetOrInitVisualizer());
        for (int i = 0; i < _mesh.npolys; ++i)
            VisualizeEdges(i);
    }

    private void VisualizePolygon(int index)
    {
        _dd.EffectMesh?.DrawSingle(_dd.RenderContext, GetOrInitVisualizer(), index);
        VisualizeEdges(index);
    }

    private void VisualizeEdges(int index)
    {
        var offset = index * _mesh.nvp * 2;
        if (_mesh.polys[offset] != RcConstants.RC_MESH_NULL_IDX)
        {
            var from = GetVertex(_mesh.polys[offset]);
            var adj = _mesh.polys[offset + _mesh.nvp];
            for (int i = 1; i < _mesh.nvp; ++i)
            {
                var v = _mesh.polys[offset + i];
                if (v == RcConstants.RC_MESH_NULL_IDX)
                    break;
                var to = GetVertex(v);
                VisualizeEdge(from, to, adj);
                from = to;
                adj = _mesh.polys[offset + _mesh.nvp + i];
            }
            VisualizeEdge(from, GetVertex(_mesh.polys[offset]), adj);
        }
    }

    private void VisualizeEdge(Vector3 from, Vector3 to, int adj) => _dd.DrawWorldLine(from, to, adj == RcConstants.RC_MESH_NULL_IDX ? 0xd8403000 : 0x80403000, adj == RcConstants.RC_MESH_NULL_IDX ? 2 : 1);
    private void VisualizeVertex(int index) => _dd.DrawWorldPoint(GetVertex(index), 5, 0xff0000ff);

    private Vector3 GetVertex(int index) => _mesh.bmin.RecastToSystem() + new Vector3(_mesh.cs, _mesh.ch, _mesh.cs) * new Vector3(_mesh.verts[3 * index], _mesh.verts[3 * index + 1] + _heightOffset, _mesh.verts[3 * index + 2]);
}
