using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;
using vnavmesh.Navigation;
using vnavmesh.Navigation.Volume;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Rendering;

namespace vnavmesh.UI.Debug.Volume;

public class DebugVoxelMap : IDisposable
{
    private const float DefaultVoxelRenderHorizontalDistance = 50f;
    private const float DefaultVoxelRenderVerticalDistance   = 10f;
    private const float DefaultQueryRenderHorizontalDistance = 80f;
    private const float DefaultQueryRenderVerticalDistance   = 20f;
    private const int   DefaultQueryRenderBudget             = 12000;
    private const int   DefaultQueryListBudget               = 2000;
    private const int   AutoAggregateQueryNodeThreshold      = 30000;

    private enum QueryVisualizationMode
    {
        Auto,
        AggregatedPoints,
        AggregatedBoxes,
        RawWireframe
    }

    private struct QueryBucketAccumulator
    {
        public int Count;
        public int ClosedCount;
    }

    private readonly record struct QueryBucketRenderEntry
    (
        ulong   Voxel,
        Vector3 Min,
        Vector3 Max,
        int     Count,
        int     ClosedCount
    );

    private VoxelMap                     _vm;
    private VoxelPathfind?               _query;
    private NavmeshQuery?                _navQuery;
    private UITree                       _tree;
    private DebugDrawer                  _dd;
    private int[]                        _numSubdivPerLevel;
    private int[]                        _numLeavesPerLevel;
    private float                        _renderHorizontalDistance      = DefaultVoxelRenderHorizontalDistance;
    private float                        _renderVerticalDistance        = DefaultVoxelRenderVerticalDistance;
    private float                        _queryRenderHorizontalDistance = DefaultQueryRenderHorizontalDistance;
    private float                        _queryRenderVerticalDistance   = DefaultQueryRenderVerticalDistance;
    private bool                         _statsInitialized;
    private QueryVisualizationMode       _queryVisualizationMode = QueryVisualizationMode.Auto;
    private int                          _queryAggregationLevel;
    private int                          _queryRenderBudget = DefaultQueryRenderBudget;
    private int                          _queryListBudget   = DefaultQueryListBudget;
    private List<QueryBucketRenderEntry> _queryBuckets      = [];
    private int                          _queryBucketsMaxCount;
    private int                          _cachedQueryNodeCount   = -1;
    private int                          _cachedVisitedNodes     = -1;
    private int                          _cachedAggregationLevel = -1;
    private long                         _nextQueryCacheRefreshAt;

    public DebugVoxelMap(VoxelMap vm, VoxelPathfind? query, NavmeshQuery? navQuery, UITree tree, DebugDrawer dd)
    {
        _vm       = vm;
        _query    = query;
        _navQuery = navQuery;
        _tree     = tree;
        _dd       = dd;

        _numSubdivPerLevel = new int[vm.Levels.Length];
        _numLeavesPerLevel = new int[vm.Levels.Length];
    }

    public void Dispose() { }

    public void Draw()
    {
        using var nr = _tree.Node("体素图 (Voxel Map)");
        if (!nr.Opened)
            return;

        EnsureTileStatsInitialized();
        var player = Service.ObjectTable.LocalPlayer;

        if (player != null)
        {
            var playerPosition = player.Position;
            var playerVoxel    = _vm.FindLeafVoxel(playerPosition);
            if (_tree.LeafNode($"玩家原始叶体素：{playerVoxel.voxel:X} (是否为空={playerVoxel.empty})").SelectedOrHovered && playerVoxel.voxel != VoxelMap.INVALID_VOXEL)
                VisualizeVoxel(playerVoxel.voxel);

            if (_navQuery != null)
            {
                var resolved = _navQuery.FindNearestVolumeVoxelSurfaceAware(playerPosition);
                if (_tree.LeafNode
                             ($"玩家飞行定位体素：{resolved.Voxel:X} (地表锚定={resolved.UsedSurfaceAnchor}，搜索点={resolved.SearchPoint:f3}，安全点={resolved.SafePoint:f3})")
                         .SelectedOrHovered &&
                    resolved.Voxel != VoxelMap.INVALID_VOXEL)
                    VisualizeVoxel(resolved.Voxel);
            }
        }

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("Voxel 水平渲染距离###voxelRenderHorizontalDistance", ref _renderHorizontalDistance, 2f, 500f, "%.0f");
        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("Voxel 垂直渲染距离###voxelRenderVerticalDistance", ref _renderVerticalDistance, 1f, 100f, "%.0f");

        for (var level = 0; level < _vm.Levels.Length; ++level)
        {
            var l = _vm.Levels[level];
            _tree.LeafNode
                ($"层级 {level}：{_numSubdivPerLevel[level]} 个细分节点，{_numLeavesPerLevel[level]} 个叶节点，大小={l.CellSize:f3}，数量={l.NumCellsX}x{l.NumCellsY}x{l.NumCellsZ}");
        }

        DrawTile(_vm.RootTile, "根瓦片 (Root tile)");

        using (var nv = _tree.Node($"查询节点 ({_query?.NodeSpan.Length})###query", _query == null || _query.NodeSpan.Length == 0))
        {
            DrawQueryVisualizationSettings();

            if (nv.SelectedOrHovered)
                VisualizeQuery();

            if (nv.Opened && _query != null)
            {
                var ns        = _query.NodeSpan;
                var listLimit = Math.Clamp(_queryListBudget, 100, 20000);
                if (ns.Length > listLimit)
                    _tree.LeafNode($"节点列表已限流：显示前 {listLimit} / {ns.Length} 个，避免界面卡顿");

                for (var i = 0; i < Math.Min(ns.Length, listLimit); ++i)
                {
                    ref var n      = ref ns[i];
                    var     bounds = _vm.VoxelBounds(n.Voxel, 0);

                    if (_tree.LeafNode
                            ($"[{i}] {n.Voxel:X} ({bounds.min:f3}-{bounds.max:f3}), 父节点={n.ParentIndex}, 消耗={n.GScore:f4}, 总计={n.HScore:f4}").SelectedOrHovered)
                    {
                        VisualizeVoxel(n.Voxel);
                        ref var parent = ref ns[Math.Clamp(n.ParentIndex, 0, ns.Length - 1)];
                        _dd.DrawWorldLine(parent.Position, n.Position, 0xff00ffff);
                        _dd.DrawWorldPointFilled(parent.Position, 2, 0xff00ffff);
                        _dd.DrawWorldPointFilled(n.Position,      2, 0xff0000ff);
                    }
                }
            }
        }
    }

    public void VisualizeVoxel(ulong voxel) => VisualizeCell(_vm.VoxelBounds(voxel, 0), 0xff0080ff);

    private void InitTile(VolumeTile tile)
    {
        for (var i = 0; i < tile.CellCount; ++i)
        {
            var t = tile.GetCell(i);
            if ((t & VoxelMap.VOXEL_ID_MASK) == VoxelMap.VOXEL_ID_MASK)
                ++_numLeavesPerLevel[tile.Level];
        }

        _numSubdivPerLevel[tile.Level] += tile.SubdivisionCount;
        foreach (ref readonly var sub in tile.Subdivisions)
            InitTile(sub);
    }

    private void EnsureTileStatsInitialized()
    {
        if (_statsInitialized)
            return;

        Array.Clear(_numSubdivPerLevel);
        Array.Clear(_numLeavesPerLevel);
        InitTile(_vm.RootTile);
        _statsInitialized = true;
    }

    private void DrawTile(VolumeTile tile, string name)
    {
        using var nr = _tree.Node($"{name}: {tile.BoundsMin:f3} - {tile.BoundsMax:f3} ({tile.SubdivisionCount} subtiles)");
        if (nr.SelectedOrHovered)
            VisualizeTile(tile);
        if (!nr.Opened)
            return;

        for (var i = 0; i < tile.CellCount; i++)
            if ((tile.GetCell(i) & VoxelMap.VOXEL_OCCUPIED_BIT) != 0)
            {
                var v  = IndexToVoxel(tile, i);
                var cn = $"{v.x}x{v.y}x{v.z}";
                var id = tile.GetCell(i) & VoxelMap.VOXEL_ID_MASK;

                if (id == VoxelMap.VOXEL_ID_MASK)
                {
                    // fully solid
                    if (_tree.LeafNode($"{v.x}x{v.y}x{v.z}").SelectedOrHovered)
                        VisualizeCell(tile.CalculateSubdivisionBounds(v), 0xff0080ff);
                }
                else
                {
                    // subdivided
                    DrawTile(tile.GetSubdivision(id), $"{cn} -> #{id}");
                }
            }
    }

    private void VisualizeTile(VolumeTile tile)
    {
        var playerPosition = Service.ObjectTable.LocalPlayer?.Position;

        if (playerPosition == null)
        {
            VisualizeAllTileCells(tile);
            return;
        }

        VisualizeTileCulled(tile, playerPosition.Value, _renderHorizontalDistance, _renderVerticalDistance);
    }

    private void VisualizeQuery()
    {
        if (_query == null)
            return;

        var ns = _query.NodeSpan;
        if (ns.Length == 0)
            return;

        var effectiveMode         = ResolveEffectiveQueryVisualizationMode(ns.Length);
        var renderBudget          = Math.Clamp(_queryRenderBudget, 100, 50000);
        var playerPosition        = Service.ObjectTable.LocalPlayer?.Position;
        var maxHorizontalDistance = _queryRenderHorizontalDistance;
        var maxVerticalDistance   = _queryRenderVerticalDistance;

        if (effectiveMode == QueryVisualizationMode.RawWireframe)
        {
            var stride   = Math.Max(1, ns.Length / renderBudget);
            var rendered = 0;

            for (var i = 0; i < ns.Length && rendered < renderBudget; i += stride)
            {
                ref var node   = ref ns[i];
                var     bounds = _vm.VoxelBounds(node.Voxel, 0);
                if (playerPosition != null &&
                    !IsBoundsWithinRenderDistance(bounds.min, bounds.max, playerPosition.Value, maxHorizontalDistance, maxVerticalDistance))
                    continue;

                VisualizeCell
                (
                    bounds,
                    node.Closed ?
                        0x88FFB300u :
                        0x8800BCD4u,
                    node.Closed ?
                        1 :
                        2
                );
                rendered++;
            }

            return;
        }

        var aggregationLevel = ResolveQueryAggregationLevel();
        EnsureQueryBucketCache(aggregationLevel);
        if (_queryBuckets.Count == 0)
            return;

        var renderedBuckets = 0;

        foreach (var bucket in _queryBuckets)
        {
            if (renderedBuckets >= renderBudget)
                break;
            if (playerPosition != null && !IsBoundsWithinRenderDistance(bucket.Min, bucket.Max, playerPosition.Value, maxHorizontalDistance, maxVerticalDistance))
                continue;

            var color = QueryBucketColor(bucket.Count, _queryBucketsMaxCount, bucket.ClosedCount);

            if (effectiveMode == QueryVisualizationMode.AggregatedBoxes)
            {
                var density = QueryBucketDensity(bucket.Count, _queryBucketsMaxCount);
                VisualizeCell
                (
                    (bucket.Min, bucket.Max),
                    color,
                    density >= 0.65f ?
                        2 :
                        1
                );
            }
            else
            {
                var density   = QueryBucketDensity(bucket.Count, _queryBucketsMaxCount);
                var levelBias = Math.Max(0, _vm.Levels.Length - aggregationLevel - 1) * 0.75f;
                var radius    = 3f + density * 5f + levelBias;
                _dd.DrawWorldPointFilled((bucket.Min + bucket.Max) * 0.5f, radius, color);
            }

            renderedBuckets++;
        }
    }

    private void VisualizeAllTileCells(VolumeTile tile)
    {
        for (var i = 0; i < tile.CellCount; i++)
        {
            var cell = tile.GetCell(i);
            if ((cell & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                continue;

            var id = cell & VoxelMap.VOXEL_ID_MASK;

            if (id == VoxelMap.VOXEL_ID_MASK)
            {
                var bounds = tile.CalculateSubdivisionBounds(IndexToVoxel(tile, i));
                VisualizeCell(bounds, 0xff0080ff);
            }
            else
            {
                VisualizeAllTileCells(tile.GetSubdivision(id));
            }
        }
    }

    private void VisualizeTileCulled(VolumeTile tile, Vector3 playerPosition, float maxHorizontalDistance, float maxVerticalDistance)
    {
        if (!IsBoundsWithinRenderDistance(tile.BoundsMin, tile.BoundsMax, playerPosition, maxHorizontalDistance, maxVerticalDistance))
            return;

        for (var i = 0; i < tile.CellCount; i++)
        {
            var cell = tile.GetCell(i);
            if ((cell & VoxelMap.VOXEL_OCCUPIED_BIT) == 0)
                continue;

            var id = cell & VoxelMap.VOXEL_ID_MASK;

            if (id == VoxelMap.VOXEL_ID_MASK)
            {
                var bounds = tile.CalculateSubdivisionBounds(IndexToVoxel(tile, i));
                if (IsBoundsWithinRenderDistance(bounds.min, bounds.max, playerPosition, maxHorizontalDistance, maxVerticalDistance))
                    VisualizeCell(bounds, 0xff0080ff);
            }
            else
            {
                VisualizeTileCulled(tile.GetSubdivision(id), playerPosition, maxHorizontalDistance, maxVerticalDistance);
            }
        }
    }

    private static (int x, int y, int z) IndexToVoxel(VolumeTile tile, int index)
    {
        var level = tile.LevelDesc;
        var y     = index & (level.NumCellsY - 1);
        var xz    = index >> level.ShiftYX;
        var x     = xz & (level.NumCellsX - 1);
        var z     = xz >> level.ShiftXZ;
        return (x, y, z);
    }

    private static bool IsBoundsWithinRenderDistance
    (
        Vector3 min,
        Vector3 max,
        Vector3 playerPosition,
        float   maxHorizontalDistance,
        float   maxVerticalDistance
    )
    {
        if (playerPosition.Y < min.Y - maxVerticalDistance || playerPosition.Y > max.Y + maxVerticalDistance)
            return false;

        var clampedX = Math.Clamp(playerPosition.X, min.X, max.X);
        var clampedZ = Math.Clamp(playerPosition.Z, min.Z, max.Z);
        var dx       = playerPosition.X - clampedX;
        var dz       = playerPosition.Z - clampedZ;
        return dx * dx + dz * dz <= maxHorizontalDistance * maxHorizontalDistance;
    }

    private void DrawQueryVisualizationSettings()
    {
        if (_query == null)
            return;

        var ns = _query.NodeSpan;
        if (ns.Length == 0)
            return;

        var effectiveMode = ResolveEffectiveQueryVisualizationMode(ns.Length);

        if (ImGui.BeginCombo("查询可视化", QueryVisualizationModeLabel(_queryVisualizationMode)))
        {
            foreach (var mode in Enum.GetValues<QueryVisualizationMode>())
            {
                var selected = mode == _queryVisualizationMode;
                if (ImGui.Selectable(QueryVisualizationModeLabel(mode), selected))
                    _queryVisualizationMode = mode;
                if (selected)
                    ImGui.SetItemDefaultFocus();
            }

            ImGui.EndCombo();
        }

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        if (ImGui.SliderInt("查询聚合层级###queryAggregationLevel", ref _queryAggregationLevel, 0, _vm.Levels.Length - 1))
            InvalidateQueryBucketCache();

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("查询节点 水平裁切距离###queryRenderHorizontalDistance", ref _queryRenderHorizontalDistance, 2f, 500f, "%.0f");

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("查询节点 垂直裁切距离###queryRenderVerticalDistance", ref _queryRenderVerticalDistance, 1f, 100f, "%.0f");

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderInt("查询渲染预算###queryRenderBudget", ref _queryRenderBudget, 100, 50000, "%d");

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderInt("查询列表显示上限###queryListBudget", ref _queryListBudget, 100, 20000, "%d");

        _tree.LeafNode($"查询渲染实际模式：{QueryVisualizationModeLabel(effectiveMode)}");
        _tree.LeafNode($"查询渲染参数：节点={ns.Length}，已访问={_query.LastTelemetry.VisitedNodes}，聚合层级=L{ResolveQueryAggregationLevel()}");

        if (effectiveMode != QueryVisualizationMode.RawWireframe)
        {
            EnsureQueryBucketCache(ResolveQueryAggregationLevel());
            _tree.LeafNode($"查询聚合结果：{_queryBuckets.Count} 个桶，最密集桶 {_queryBucketsMaxCount} 节点");
        }
    }

    private QueryVisualizationMode ResolveEffectiveQueryVisualizationMode(int nodeCount) =>
        _queryVisualizationMode switch
        {
            QueryVisualizationMode.Auto when nodeCount > AutoAggregateQueryNodeThreshold => QueryVisualizationMode.AggregatedPoints,
            QueryVisualizationMode.Auto                                                  => QueryVisualizationMode.RawWireframe,
            _                                                                            => _queryVisualizationMode
        };

    private int ResolveQueryAggregationLevel() => Math.Clamp(_queryAggregationLevel, 0, _vm.Levels.Length - 1);

    private void EnsureQueryBucketCache(int aggregationLevel)
    {
        if (_query == null)
            return;

        var now          = Environment.TickCount64;
        var nodeCount    = _query.NodeSpan.Length;
        var visitedNodes = _query.LastTelemetry.VisitedNodes;
        var stale = aggregationLevel != _cachedAggregationLevel ||
                    nodeCount        != _cachedQueryNodeCount   ||
                    visitedNodes     != _cachedVisitedNodes;

        if (!stale || now < _nextQueryCacheRefreshAt)
            return;

        _nextQueryCacheRefreshAt = now + 250;
        _cachedAggregationLevel  = aggregationLevel;
        _cachedQueryNodeCount    = nodeCount;
        _cachedVisitedNodes      = visitedNodes;
        _queryBucketsMaxCount    = 0;
        _queryBuckets.Clear();

        var                                       ns       = _query.NodeSpan;
        var                                       capacity = Math.Clamp(nodeCount / 16, 256, 262144);
        Dictionary<ulong, QueryBucketAccumulator> buckets  = new(capacity);

        for (var i = 0; i < ns.Length; ++i)
        {
            var     bucketVoxel = ReduceVoxelToLevel(ns[i].Voxel, aggregationLevel);
            ref var bucket      = ref CollectionsMarshal.GetValueRefOrAddDefault(buckets, bucketVoxel, out _);
            bucket.Count++;
            if (ns[i].Closed)
                bucket.ClosedCount++;
        }

        foreach (var (bucketVoxel, bucket) in buckets)
        {
            var bounds = _vm.VoxelBounds(bucketVoxel, 0);
            _queryBuckets.Add(new(bucketVoxel, bounds.min, bounds.max, bucket.Count, bucket.ClosedCount));
            if (bucket.Count > _queryBucketsMaxCount)
                _queryBucketsMaxCount = bucket.Count;
        }

        _queryBuckets.Sort
        (static (left, right) =>
            {
                var byCount = right.Count.CompareTo(left.Count);
                return byCount != 0 ?
                           byCount :
                           right.ClosedCount.CompareTo(left.ClosedCount);
            }
        );
    }

    private void InvalidateQueryBucketCache()
    {
        _cachedAggregationLevel  = -1;
        _cachedQueryNodeCount    = -1;
        _cachedVisitedNodes      = -1;
        _nextQueryCacheRefreshAt = 0;
        _queryBuckets.Clear();
        _queryBucketsMaxCount = 0;
    }

    private static ulong ReduceVoxelToLevel(ulong voxel, int level)
    {
        if (voxel == VoxelMap.INVALID_VOXEL)
            return voxel;

        Span<ushort> indices = stackalloc ushort[8];
        var          temp    = voxel;
        var          count   = 0;

        for (var i = 0; i <= level && i < indices.Length; ++i)
        {
            var index = VoxelMap.DecodeIndex(ref temp);
            if (index == VoxelMap.INDEX_LEVEL_MASK)
                break;

            indices[count++] = index;
        }

        if (count == 0)
            return VoxelMap.INVALID_VOXEL;

        ulong result = VoxelMap.INVALID_VOXEL;
        for (var i = count - 1; i >= 0; --i)
            result = VoxelMap.EncodeIndex(indices[i], result);
        return result;
    }

    private static float QueryBucketDensity(int count, int maxCount)
    {
        if (count <= 0 || maxCount <= 0)
            return 0f;

        var numerator   = MathF.Log(count + 1);
        var denominator = MathF.Max(MathF.Log(maxCount + 1), 0.0001f);
        return Math.Clamp(numerator / denominator, 0f, 1f);
    }

    private static uint QueryBucketColor(int count, int maxCount, int closedCount)
    {
        var density = QueryBucketDensity(count, maxCount);
        var closedRatio = count > 0 ?
                              Math.Clamp((float)closedCount / count, 0f, 1f) :
                              0f;
        var frontier  = new Vector3(0x33, 0xD1, 0xFF);
        var explored  = new Vector3(0xFF, 0xB3, 0x00);
        var saturated = new Vector3(0xF4, 0x43, 0x36);
        var rgb       = Vector3.Lerp(frontier, explored, closedRatio);
        rgb = Vector3.Lerp(rgb, saturated, density);
        var alpha = 72f + 156f * MathF.Max(density, closedRatio * 0.65f);
        return PackColor(rgb, alpha);
    }

    private static uint PackColor(Vector3 rgb, float alpha)
    {
        var r = (uint)Math.Clamp((int)MathF.Round(rgb.X), 0, 255);
        var g = (uint)Math.Clamp((int)MathF.Round(rgb.Y), 0, 255);
        var b = (uint)Math.Clamp((int)MathF.Round(rgb.Z), 0, 255);
        var a = (uint)Math.Clamp((int)MathF.Round(alpha), 0, 255);
        return a << 24 | b << 16 | g << 8 | r;
    }

    private static string QueryVisualizationModeLabel(QueryVisualizationMode mode) => mode switch
    {
        QueryVisualizationMode.Auto             => "自动",
        QueryVisualizationMode.AggregatedPoints => "聚合点云",
        QueryVisualizationMode.AggregatedBoxes  => "聚合盒子",
        QueryVisualizationMode.RawWireframe     => "原始线框",
        _                                       => mode.ToString()
    };

    private void VisualizeCell((Vector3 min, Vector3 max) bounds, uint color, int thickness = 1) => _dd.DrawWorldAABB
        ((bounds.min + bounds.max) * 0.5f, (bounds.max - bounds.min) * 0.5f, color, thickness);
}
