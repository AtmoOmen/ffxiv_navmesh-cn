using Dalamud.Bindings.ImGui;
using Navmesh.Movement;
using Navmesh.NavVolume;
using System;
using System.Numerics;

namespace Navmesh.Debug;

class DebugNavmeshManager : IDisposable
{
    private NavmeshManager _manager;
    private FollowPath _path;
    private AsyncMoveRequest _asyncMove;
    private DTRProvider _dtr;
    private UITree _tree = new();
    private DebugDrawer _dd;
    private DebugGameCollision _coll;
    private Vector3 _target;

    private DebugDetourNavmesh? _drawNavmesh;
    private DebugVoxelMap? _debugVoxelMap;

    public DebugNavmeshManager(DebugDrawer dd, DebugGameCollision coll, NavmeshManager manager, FollowPath path, AsyncMoveRequest move, DTRProvider dtr)
    {
        _manager = manager;
        _path = path;
        _asyncMove = move;
        _dtr = dtr;
        _dd = dd;
        _coll = coll;
        _manager.OnNavmeshChanged += OnNavmeshChanged;
    }

    public void Dispose()
    {
        _manager.OnNavmeshChanged -= OnNavmeshChanged;
        _drawNavmesh?.Dispose();
        _debugVoxelMap?.Dispose();
    }

    public void Draw()
    {
        var progress = _manager.LoadTaskProgress;
        if (progress >= 0)
        {
            ImGui.ProgressBar(progress, new Vector2(200, 0));
        }
        else
        {
            ImGui.SetNextItemWidth(100);
            if (ImGui.Button("重新加载"))
                _manager.Reload(true);
            ImGui.SameLine();
            if (ImGui.Button("重建"))
                _manager.Reload(false);
        }
        ImGui.SameLine();
        ImGui.TextUnformatted(_manager.CurrentKey);
        ImGui.TextUnformatted($"寻路任务数: {(_manager.PathfindInProgress ? 1 : 0)} 进行中 {_manager.NumQueuedPathfindRequests} 队列中");

        if (_manager.Navmesh == null || _manager.Query == null)
            return;

        var player = Service.ObjectTable.LocalPlayer;
        var playerPos = player?.Position ?? default;
        ImGui.TextUnformatted($"玩家位置: {playerPos}");
        if (ImGui.Button("设置目标为当前位置"))
            _target = player?.Position ?? default;
        ImGui.SameLine();
        if (ImGui.Button("设置目标为当前目标位置"))
            _target = player?.TargetObject?.Position ?? default;
        ImGui.SameLine();
        if (ImGui.Button("设置目标为标记位置"))
            _target = MapUtils.FlagToPoint(_manager.Query) ?? default;
        ImGui.SameLine();
        ImGui.TextUnformatted($"当前目标: {_target}");

        if (ImGui.Button("导出位图"))
            ExportBitmap(_manager.Navmesh, _manager.Query, playerPos);

        ImGui.Checkbox("允许移动", ref _path.MovementAllowed);
        ImGui.Checkbox("使用射线检测", ref _manager.UseRaycasts);
        ImGui.Checkbox("使用拉绳算法", ref _manager.UseStringPulling);
        if (ImGui.Button("地面寻路至目标"))
            _asyncMove.MoveTo(_target, false);
        ImGui.SameLine();
        if (ImGui.Button("空间寻路至目标"))
            _asyncMove.MoveTo(_target, true);

        DrawPosition("玩家", playerPos);
        DrawPosition("目标", _target);
        DrawPosition("标记", MapUtils.FlagToPoint(_manager.Query) ?? default);
        DrawPosition("地面", _manager.Query.FindPointOnFloor(playerPos) ?? default);

        _drawNavmesh ??= new(_manager.Navmesh.Mesh, _manager.Query.MeshQuery, _manager.Query.LastPath, _tree, _dd);
        _drawNavmesh.Draw();
        if (_manager.Navmesh.Volume != null)
        {
            _debugVoxelMap ??= new(_manager.Navmesh.Volume, _manager.Query.VolumeQuery, _tree, _dd);
            _debugVoxelMap.Draw();
        }
    }

    private void DrawPosition(string tag, Vector3 position)
    {
        _manager.Navmesh!.Mesh.CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        _tree.LeafNode($"{tag} position: {position:f3}, tile: {tileX}x{tileZ}, poly: {_manager.Query!.FindNearestMeshPoly(position):X}");
        var voxel = _manager.Query.FindNearestVolumeVoxel(position);
        if (_tree.LeafNode($"{tag} voxel: {voxel:X}###{tag}voxel").SelectedOrHovered && voxel != VoxelMap.InvalidVoxel)
            _debugVoxelMap?.VisualizeVoxel(voxel);
    }

    private void ExportBitmap(Navmesh navmesh, NavmeshQuery query, Vector3 startingPos)
    {
        _manager.BuildBitmap(startingPos, "D:\\navmesh.bmp", 0.5f);
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        _drawNavmesh?.Dispose();
        _drawNavmesh = null;
        _debugVoxelMap?.Dispose();
        _debugVoxelMap = null;
    }
}
