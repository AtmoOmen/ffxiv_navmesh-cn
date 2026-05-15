using System.Numerics;
using Dalamud.Bindings.ImGui;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;
using vnavmesh.Navigation.Volume;
using vnavmesh.Navigation.Volume.Pathfinding;
using vnavmesh.Shared.Utilities;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Rendering;

namespace vnavmesh.UI.Debug.Volume;

public class DebugVoxelMap : IDisposable
{
    private const float DefaultVoxelRenderHorizontalDistance = 50f;
    private const float DefaultVoxelRenderVerticalDistance   = 10f;

    private VoxelMap                                                _vm;
    private VoxelPathfind?                                          _query;
    private UITree                                                  _tree;
    private DebugDrawer                                             _dd;
    private int[]                                                   _numSubdivPerLevel;
    private int[]                                                   _numLeavesPerLevel;
    private float                                                   _renderHorizontalDistance = DefaultVoxelRenderHorizontalDistance;
    private float                                                   _renderVerticalDistance   = DefaultVoxelRenderVerticalDistance;
    private bool                                                    _statsInitialized;

    public DebugVoxelMap(VoxelMap vm, VoxelPathfind? query, UITree tree, DebugDrawer dd)
    {
        _vm    = vm;
        _query = query;
        _tree  = tree;
        _dd    = dd;

        _numSubdivPerLevel = new int[vm.Levels.Length];
        _numLeavesPerLevel = new int[vm.Levels.Length];
    }

    public void Dispose() { }

    public void Draw()
    {
        using var nr = _tree.Node("体素图 (Voxel Map)");
        if (!nr.Opened)
            return;

        var playerVoxel = _vm.FindLeafVoxel(Service.ObjectTable.LocalPlayer?.Position ?? default);
        EnsureTileStatsInitialized();
        _tree.LeafNode($"玩家所在体素：{playerVoxel.voxel:X} (是否为空={playerVoxel.empty})");
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
            if (nv.SelectedOrHovered)
                VisualizeQuery();

            if (nv.Opened && _query != null)
            {
                var ns = _query.NodeSpan;

                for (var i = 0; i < ns.Length; ++i)
                {
                    ref var n      = ref ns[i];
                    var     bounds = _vm.VoxelBounds(n.Voxel, 0);

                    if (_tree.LeafNode
                            ($"[{i}] {n.Voxel:X} ({bounds.min:f3}-{bounds.max:f3}), 父节点={n.ParentIndex}, 消耗={n.GScore:f4}, 总计={n.HScore:f4}").SelectedOrHovered)
                    {
                        VisualizeVoxel(n.Voxel);
                        ref var parent = ref ns[n.ParentIndex];
                        _dd.DrawWorldLine(parent.Position, n.Position, 0xff00ffff);
                        _dd.DrawWorldPointFilled(parent.Position, 2, 0xff00ffff);
                        _dd.DrawWorldPointFilled(n.Position,      2, 0xff0000ff);
                    }
                }
            }
        }
    }

    public void VisualizeVoxel(ulong voxel) => VisualizeCell(_vm.VoxelBounds(voxel, 0));

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
                        VisualizeCell(tile.CalculateSubdivisionBounds(v));
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
        if (_query != null)
        {
            var ns = _query.NodeSpan;
            for (var i = 0; i < ns.Length; ++i) VisualizeVoxel(ns[i].Voxel);
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
                VisualizeCell(bounds);
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
                    VisualizeCell(bounds);
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

    private void VisualizeCell((Vector3 min, Vector3 max) bounds) => _dd.DrawWorldAABB
        ((bounds.min + bounds.max) * 0.5f, (bounds.max - bounds.min) * 0.5f, 0xff0080ff);
}
