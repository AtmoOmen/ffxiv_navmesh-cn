using System;
using System.Numerics;
using Dalamud.Bindings.ImGui;
using vnavmesh.Interface.Debug.Components;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;
using vnavmesh.NavVolume;
using vnavmesh.Utils;

namespace vnavmesh.Interface.Debug;

class DebugNavmeshManager : IDisposable
{
	private NavmeshManager _manager;
	private MovementPlanExecutor _movementExecutor;
	private AsyncMoveRequest _asyncMove;
	private DTRProvider _dtr;
	private UITree _tree = new();
	private DebugDrawer _dd;
	private DebugGameCollision _coll;
	private Vector3 _target;

	private DebugDetourNavmesh? _drawNavmesh;
	private DebugVoxelMap? _debugVoxelMap;
	private DebugLinks? _debugLinks;

	public DebugNavmeshManager(DebugDrawer dd, DebugGameCollision coll, NavmeshManager manager, MovementPlanExecutor movementExecutor, AsyncMoveRequest move, DTRProvider dtr)
	{
		_manager = manager;
		_movementExecutor = movementExecutor;
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
			if (ImGui.Button("重新构建"))
				_manager.Reload(false);
		}
		ImGui.SameLine();
		ImGui.TextUnformatted(_manager.CurrentKey);
		ImGui.TextUnformatted($"寻路任务数量：{(_manager.PathfindInProgress ? 1 : 0)} 正在进行，{_manager.NumQueuedPathfindRequests} 已入队");

		if (_manager.Navmesh == null || _manager.Query == null)
			return;

		var player = Service.ObjectTable.LocalPlayer;
		var playerPos = player?.Position ?? default;
		ImGui.TextUnformatted($"玩家位置：{playerPos}");
		if (ImGui.Button("设为当前位置"))
			_target = player?.Position ?? default;
		ImGui.SameLine();
		if (ImGui.Button("设为目标位置"))
			_target = player?.TargetObject?.Position ?? default;
		ImGui.SameLine();
		if (ImGui.Button("设为标点位置"))
			_target = MapUtil.FlagToPoint(_manager.Query) ?? default;
		ImGui.SameLine();
		ImGui.TextUnformatted($"当前目标：{_target}");

		if (ImGui.Button("导出位图"))
			ExportBitmap(_manager.Navmesh, _manager.Query, playerPos);

		ImGui.Checkbox("允许移动", ref _movementExecutor.MovementAllowed);
		ImGui.Checkbox("使用射线检测", ref _manager.UseRaycasts);
		ImGui.Checkbox("使用拉绳算法", ref _manager.UseStringPulling);
		if (ImGui.Button("使用导航网格寻路至目标"))
			_asyncMove.MoveTo(_target, false);
		ImGui.SameLine();
		if (ImGui.Button("使用体素寻路至目标"))
			_asyncMove.MoveTo(_target, true);

		DrawPosition("玩家", playerPos);
		DrawPosition("目标", _target);
		DrawPosition("旗帜", MapUtil.FlagToPoint(_manager.Query) ?? default);
		DrawPosition("地面", _manager.Query.FindPointOnFloor(playerPos) ?? default);

		_drawNavmesh ??= new(_manager.Navmesh.Mesh, _manager.Query.MeshQuery, _manager.Query.LastPath, _tree, _dd);
		_drawNavmesh.Draw();
		if (_manager.Navmesh.Volume != null)
		{
			_debugVoxelMap ??= new(_manager.Navmesh.Volume, _manager.Query.VolumeQuery, _tree, _dd);
			_debugVoxelMap.Draw();
		}

		_debugLinks ??= new(_manager.Navmesh, _dd);
		_debugLinks.Draw();
	}

	private void DrawPosition(string tag, Vector3 position)
	{
		_manager.Navmesh!.Mesh.CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
		_tree.LeafNode($"{tag}位置：{position:f3}，区块 (Tile)：{tileX}x{tileZ}，多边形 (Poly)：{_manager.Query!.FindNearestMeshPoly(position):X}");
		var voxel = _manager.Query.FindNearestVolumeVoxel(position);
		if (_tree.LeafNode($"{tag}体素：{voxel:X}###{tag}voxel").SelectedOrHovered && voxel != VoxelMap.InvalidVoxel)
			_debugVoxelMap?.VisualizeVoxel(voxel);
	}

	private void ExportBitmap(Navmesh.Navmesh navmesh, NavmeshQuery query, Vector3 startingPos)
	{
		_manager.BuildBitmap(startingPos, "D:\\navmesh.bmp", 0.5f);
	}

	private void OnNavmeshChanged(Navmesh.Navmesh? navmesh, NavmeshQuery? query)
	{
		_drawNavmesh?.Dispose();
		_drawNavmesh = null;
		_debugVoxelMap?.Dispose();
		_debugVoxelMap = null;
	}
}
