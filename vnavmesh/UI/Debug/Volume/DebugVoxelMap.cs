using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using vnavmesh.Common.Build.Flight;
using vnavmesh.Query;
using vnavmesh.Query.Flight;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;

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

    private SparseVoxelOctree        _vm;
    private VoxelPathfinder?         _query;
    private NavmeshQuery?            _navQuery;
    private UITree                   _tree;
    private DebugDrawer              _dd;
    private int[]                    _numNodesPerLevel;
    private int[]                    _numSolidLeavesPerLevel;
    private float                    _renderHorizontalDistance      = DefaultVoxelRenderHorizontalDistance;
    private float                    _renderVerticalDistance        = DefaultVoxelRenderVerticalDistance;
    private float                    _queryRenderHorizontalDistance = DefaultQueryRenderHorizontalDistance;
    private float                    _queryRenderVerticalDistance   = DefaultQueryRenderVerticalDistance;
    private bool                     _statsInitialized;
    private QueryVisualizationMode   _queryVisualizationMode = QueryVisualizationMode.Auto;
    private int                      _queryAggregationLevel;
    private int                      _queryRenderBudget = DefaultQueryRenderBudget;
    private int                      _queryListBudget   = DefaultQueryListBudget;
    private List<QueryBucketRenderEntry> _queryBuckets = [];
    private int                      _queryBucketsMaxCount;
    private int                      _cachedQueryNodeCount   = -1;
    private int                      _cachedVisitedNodes     = -1;
    private int                      _cachedAggregationLevel = -1;
    private long                     _nextQueryCacheRefreshAt;

    public DebugVoxelMap
    (
        SparseVoxelOctree vm,
        VoxelPathfinder?  query,
        NavmeshQuery?     navQuery,
        UITree            tree,
        DebugDrawer       dd
    )
    {
        _vm       = vm;
        _query    = query;
        _navQuery = navQuery;
        _tree     = tree;
        _dd       = dd;

        _numNodesPerLevel      = new int[vm.MaxDepth + 1];
        _numSolidLeavesPerLevel = new int[vm.MaxDepth + 1];
    }

    public void Dispose()
    {
    }

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

            if (_tree.LeafNode($"玩家原始叶体素：{playerVoxel.voxel:X} (是否为空={playerVoxel.empty})").SelectedOrHovered && playerVoxel.voxel != SparseVoxelOctree.INVALID_VOXEL)
                VisualizeVoxel(playerVoxel.voxel);

            if (_navQuery != null)
            {
                var resolved = _navQuery.FindNearestVolumeVoxelSurfaceAware(playerPosition);

                if (_tree.LeafNode
                             ($"玩家飞行定位体素：{resolved.Voxel:X} (地表锚定={resolved.UsedSurfaceAnchor}，搜索点={resolved.SearchPoint:f3}，安全点={resolved.SafePoint:f3})")
                         .SelectedOrHovered &&
                    resolved.Voxel != SparseVoxelOctree.INVALID_VOXEL)
                    VisualizeVoxel(resolved.Voxel);
            }
        }

        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("Voxel 水平渲染距离###voxelRenderHorizontalDistance", ref _renderHorizontalDistance, 2f, 500f, "%.0f");
        ImGui.SetNextItemWidth(220 * ImGui.GetIO().FontGlobalScale);
        ImGui.SliderFloat("Voxel 垂直渲染距离###voxelRenderVerticalDistance", ref _renderVerticalDistance, 1f, 100f, "%.0f");

        for (var level = 0; level <= _vm.MaxDepth; ++level)
        {
            var size = _vm.LeafSize * (1 << (_vm.MaxDepth - level));
            _tree.LeafNode
                ($"深度 {level}：{_numNodesPerLevel[level]} 个存储节点，{_numSolidLeavesPerLevel[level]} 个实心叶，大小={size:f3}");
        }

        DrawNode(_vm.RootNodeIndex, 0, 0, 0, 0, "根节点 (Root)");

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

    public void VisualizeVoxel
    (
        ulong voxel
    ) => VisualizeCell(_vm.VoxelBounds(voxel, 0), 0xff0080ff);

    private void EnsureTileStatsInitialized()
    {
        if (_statsInitialized)
            return;

        Array.Clear(_numNodesPerLevel);
        Array.Clear(_numSolidLeavesPerLevel);
        InitNode(_vm.RootNodeIndex, 0);
        _statsInitialized = true;
    }

    private void InitNode
    (
        int node,
        int depth
    )
    {
        var state = _vm.GetNodeState(node);

        if (state == SparseVoxelOctree.NODE_EMPTY)
            return;

        ++_numNodesPerLevel[depth];

        if (state == SparseVoxelOctree.NODE_SOLID_LEAF)
        {
            ++_numSolidLeavesPerLevel[depth];
            return;
        }

        for (var c = 0; c < 8; ++c)
        {
            var child = _vm.GetChildNode(node, c);

            if (child != 0)
                InitNode(child - 1, depth + 1);
        }
    }

    private void DrawNode
    (
        int    node,
        int    depth,
        int    x,
        int    y,
        int    z,
        string name
    )
    {
        var (min, max) = BoundsOfNode(depth, x, y, z);
        var state = _vm.GetNodeState(node);

        if (state == SparseVoxelOctree.NODE_SOLID_LEAF)
        {
            if (_tree.LeafNode($"{name}: 实心叶 {min:f3} - {max:f3}").SelectedOrHovered)
                VisualizeCell((min, max), 0xff0080ff);
            return;
        }

        using var nr = _tree.Node($"{name}: {min:f3} - {max:f3}");

        if (nr.SelectedOrHovered)
            VisualizeCell((min, max), 0x2200a0ff);

        if (!nr.Opened)
            return;

        for (var c = 0; c < 8; ++c)
        {
            var child = _vm.GetChildNode(node, c);
            var cx = (x << 1) | (c & 1);
            var cy = (y << 1) | ((c >> 1) & 1);
            var cz = (z << 1) | ((c >> 2) & 1);

            if (child == 0)
                continue;

            DrawNode(child - 1, depth + 1, cx, cy, cz, $"{name} -> 子 {c}");
        }
    }

    private (Vector3 min, Vector3 max) BoundsOfNode
    (
        int depth,
        int x,
        int y,
        int z
    )
    {
        var size = _vm.LeafSize * (1 << (_vm.MaxDepth - depth));
        var min  = _vm.BoundsMin + new Vector3(x * size, y * size, z * size);
        return (min, min + new Vector3(size));
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
                var levelBias = Math.Max(0, _vm.MaxDepth - aggregationLevel) * 0.75f;
                var radius    = 3f + (density * 5f) + levelBias;
                _dd.DrawWorldPointFilled((bucket.Min + bucket.Max) * 0.5f, radius, color);
            }

            renderedBuckets++;
        }
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
        return (dx * dx) + (dz * dz) <= maxHorizontalDistance * maxHorizontalDistance;
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

        if (ImGui.SliderInt("查询聚合层级###queryAggregationLevel", ref _queryAggregationLevel, 0, _vm.MaxDepth))
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

    private QueryVisualizationMode ResolveEffectiveQueryVisualizationMode
    (
        int nodeCount
    ) =>
        _queryVisualizationMode switch
        {
            QueryVisualizationMode.Auto when nodeCount > AutoAggregateQueryNodeThreshold => QueryVisualizationMode.AggregatedPoints,
            QueryVisualizationMode.Auto                                                  => QueryVisualizationMode.RawWireframe,
            _                                                                             => _queryVisualizationMode
        };

    private int ResolveQueryAggregationLevel() => Math.Clamp(_queryAggregationLevel, 0, _vm.MaxDepth);

    private void EnsureQueryBucketCache
    (
        int aggregationLevel
    )
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
        (
            static (left, right) =>
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

    private static ulong ReduceVoxelToLevel
    (
        ulong voxel,
        int   level
    )
    {
        if (voxel == SparseVoxelOctree.INVALID_VOXEL)
            return voxel;

        var depth = SparseVoxelOctree.DepthOf(voxel);

        if (depth <= level)
            return voxel;

        var (x, y, z) = SparseVoxelOctree.CoordAtDepth(voxel, level);
        return SparseVoxelOctree.EncodeCoord(level, x, y, z);
    }

    private static float QueryBucketDensity
    (
        int count,
        int maxCount
    )
    {
        if (count <= 0 || maxCount <= 0)
            return 0f;

        var numerator   = MathF.Log(count + 1);
        var denominator = MathF.Max(MathF.Log(maxCount + 1), 0.0001f);
        return Math.Clamp(numerator / denominator, 0f, 1f);
    }

    private static uint QueryBucketColor
    (
        int count,
        int maxCount,
        int closedCount
    )
    {
        var density     = QueryBucketDensity(count, maxCount);
        var closedRatio = count > 0 ?
                              Math.Clamp((float)closedCount / count, 0f, 1f) :
                              0f;
        var frontier  = new Vector3(0x33, 0xD1, 0xFF);
        var explored  = new Vector3(0xFF, 0xB3, 0x00);
        var saturated = new Vector3(0xF4, 0x43, 0x36);
        var rgb       = Vector3.Lerp(frontier, explored, closedRatio);
        rgb = Vector3.Lerp(rgb, saturated, density);
        var alpha = 72f + (156f * MathF.Max(density, closedRatio * 0.65f));
        return PackColor(rgb, alpha);
    }

    private static uint PackColor
    (
        Vector3 rgb,
        float   alpha
    )
    {
        var r = (uint)Math.Clamp((int)MathF.Round(rgb.X), 0, 255);
        var g = (uint)Math.Clamp((int)MathF.Round(rgb.Y), 0, 255);
        var b = (uint)Math.Clamp((int)MathF.Round(rgb.Z), 0, 255);
        var a = (uint)Math.Clamp((int)MathF.Round(alpha), 0, 255);
        return (a << 24) | (b << 16) | (g << 8) | r;
    }

    private static string QueryVisualizationModeLabel
    (
        QueryVisualizationMode mode
    ) => mode switch
    {
        QueryVisualizationMode.Auto             => "自动",
        QueryVisualizationMode.AggregatedPoints => "聚合点云",
        QueryVisualizationMode.AggregatedBoxes  => "聚合盒子",
        QueryVisualizationMode.RawWireframe     => "原始线框",
        _                                       => mode.ToString()
    };

    private void VisualizeCell
    (
        (Vector3 min, Vector3 max) bounds,
        uint                       color,
        int                        thickness = 1
    ) => _dd.DrawWorldAABB
        ((bounds.min + bounds.max) * 0.5f, (bounds.max - bounds.min) * 0.5f, color, thickness);
}
