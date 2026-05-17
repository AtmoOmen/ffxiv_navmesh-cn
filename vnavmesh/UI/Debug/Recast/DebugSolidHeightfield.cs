using System.Numerics;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Utilities;
using vnavmesh.Shared.Utilities;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Rendering;

namespace vnavmesh.UI.Debug.Recast;

public class DebugSolidHeightfield : DebugRecast
{
    private RcHeightfield    _hf;
    private UITree           _tree;
    private DebugDrawer      _dd;
    private int              _numNullSpans;
    private int              _numWalkableSpans;
    private int[,]           _spanCellOffsets;
    private EffectMesh.Data? _visu;

    private static Vector4 _colAreaNull     = new(0.25f, 0.25f, 0.25f, 0.7f);
    private static Vector4 _colAreaWalkable = new(0.25f, 0.5f, 0.63f, 0.7f);

    private static Vector4 AreaColor(int area) => area == 0 ? _colAreaNull : _colAreaWalkable; // TODO: other colors for other areas

    public DebugSolidHeightfield(RcHeightfield hf, UITree tree, DebugDrawer dd)
    {
        _hf   = hf;
        _tree = tree;
        _dd   = dd;

        _spanCellOffsets = new int[hf.width, hf.height];
        var icell = 0;

        for (var z = 0; z < hf.height; ++z)
        for (var x = 0; x < hf.width; ++x)
        {
            _spanCellOffsets[x, z] = _numNullSpans + _numWalkableSpans;
            var span = hf.spans[icell++];

            while (span != 0)
            {
                ref var spanRef = ref hf.Span(span);
                if (spanRef.area == 0)
                    ++_numNullSpans;
                else
                    ++_numWalkableSpans;
                span = spanRef.next;
            }
        }
    }

    public override void Dispose() =>
        _visu?.Dispose();

    public void Draw()
    {
        using var nr = _tree.Node("实体高度场 (Solid Heightfield)");
        if (!nr.Opened)
            return;

        DrawBaseInfo(_tree, _hf.width, _hf.height, _hf.bmin, _hf.bmax, _hf.cs, _hf.ch);
        _tree.LeafNode($"边界大小：{_hf.borderSize}");

        using var nc = _tree.Node("单元格");
        if (nc.SelectedOrHovered)
            Visualize();
        if (!nc.Opened)
            return;

        for (var z = 0; z < _hf.height; ++z)
        {
            UITree.NodeRaii? nz = null;

            for (var x = 0; x < _hf.width; ++x)
            {
                var span = _hf.spans[z * _hf.width + x];
                if (span == 0)
                    continue;

                nz ??= _tree.Node($"[*x{z}]");
                if (!nz.Value.Opened)
                    break;

                using var nx = _tree.Node($"[{x}x{z}]");
                if (nx.SelectedOrHovered)
                    VisualizeCell(x, z);

                if (nx.Opened)
                {
                    var ispan = 0;

                    while (span != 0)
                    {
                        ref var spanRef = ref _hf.Span(span);
                        if (_tree.LeafNode($"{ispan}: y={spanRef.smin}-{spanRef.smax}, area={spanRef.area:X}").SelectedOrHovered)
                            VisualizeSpan(_spanCellOffsets[x, z] + ispan);
                        span = spanRef.next;
                        ++ispan;
                    }
                }
            }

            nz?.Dispose();
        }
    }

    private EffectMesh.Data GetOrInitVisualizer()
    {
        if (_visu == null)
        {
            _visu = new(_dd.RenderContext, 8, 12, _numNullSpans + _numWalkableSpans, false);
            using var builder = _visu.Map(_dd.RenderContext);
            var       box     = new AnalyticMeshBox(builder);

            var timer = StopWatchTimer.Create();
            // TODO: one thing i don't like about current visualization is the lack of edges and/or any depth cues
            var       icell = 0;
            var       icnt  = 0;
            Matrix4x3 world = new() { M11 = _hf.cs * 0.5f, M33 = _hf.cs * 0.5f }; // x/z scale never changes
            world.M43 = _hf.bmin.Z + _hf.cs * 0.5f;
            var x0  = _hf.bmin.X + _hf.cs * 0.5f;
            var chh = _hf.ch * 0.5f;

            for (var z = 0; z < _hf.height; ++z)
            {
                world.M41 = x0;

                for (var x = 0; x < _hf.width; ++x)
                {
                    var span = _hf.spans[icell++];

                    while (span != 0)
                    {
                        ref var spanRef = ref _hf.Span(span);
                        world.M22 = (spanRef.smax - spanRef.smin) * chh;
                        world.M42 = _hf.bmin.Y + (spanRef.smin + spanRef.smax) * chh;
                        builder.AddInstance(new(world, AreaColor(spanRef.area)));
                        builder.AddMesh(box.FirstVertex, box.FirstPrimitive, box.NumPrimitives, icnt++, 1);
                        span = spanRef.next;
                    }

                    world.M41 += _hf.cs;
                }

                world.M43 += _hf.cs;
            }

            Service.Log.Debug($"hf visualization build time: {timer.Value().TotalMilliseconds:f3}ms");
        }

        return _visu;
    }

    private void Visualize() =>
        _dd.EffectMesh?.Draw(_dd.RenderContext, GetOrInitVisualizer());

    private void VisualizeCell(int x, int z)
    {
        var numSpans = 0;
        var span     = _hf.spans[z * _hf.width + x];

        while (span != 0)
        {
            ++numSpans;
            span = _hf.Span(span).next;
        }

        if (numSpans > 0)
            _dd.EffectMesh?.DrawSubset(_dd.RenderContext, GetOrInitVisualizer(), _spanCellOffsets[x, z], numSpans);
    }

    private void VisualizeSpan(int spanIndex) =>
        _dd.EffectMesh?.DrawSubset(_dd.RenderContext, GetOrInitVisualizer(), spanIndex, 1);
}
