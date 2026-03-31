using Dalamud.Bindings.ImGui;
using System.Collections.Generic;
using System.Linq;
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
	private DebugLinks? _debugLinks;

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
			_target = MapUtils.FlagToPoint(_manager.Query) ?? default;
		ImGui.SameLine();
		ImGui.TextUnformatted($"当前目标：{_target}");

		if (ImGui.Button("导出位图"))
			ExportBitmap(_manager.Navmesh, _manager.Query, playerPos);

		ImGui.Checkbox("允许移动", ref _path.MovementAllowed);
		ImGui.Checkbox("使用射线检测", ref _manager.UseRaycasts);
		ImGui.Checkbox("使用拉绳算法", ref _manager.UseStringPulling);
		if (ImGui.Button("使用导航网格寻路至目标"))
			_asyncMove.MoveTo(_target, false);
		ImGui.SameLine();
		if (ImGui.Button("使用体素寻路至目标"))
			_asyncMove.MoveTo(_target, true);

		DrawPosition("玩家", playerPos);
		DrawPosition("目标", _target);
		DrawPosition("旗帜", MapUtils.FlagToPoint(_manager.Query) ?? default);
		DrawPosition("地面", _manager.Query.FindPointOnFloor(playerPos) ?? default);
		DrawGroundPathDebug(_manager.Query.LastGroundPath);

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

	private void ExportBitmap(Navmesh navmesh, NavmeshQuery query, Vector3 startingPos)
	{
		_manager.BuildBitmap(startingPos, "D:\\navmesh.bmp", 0.5f);
	}

	private void DrawGroundPathDebug(GroundPathDebugInfo? debug)
	{
		if (debug == null)
		{
			_tree.LeafNode("地面路径调试：暂无数据");
			return;
		}

		using (var node = _tree.Node($"地面路径调试：走廊 {debug.CorridorCenters.Count}，门户 {debug.RawPortals.Count}，输出点 {debug.FinalPath.Count}"))
		{
			if (!node.Opened)
				return;

			DrawDebugPoint("原始目标点", debug.RequestedEnd, 0xFFFFFF40);
			DrawDebugPoint("实际可达终点", debug.ResolvedEnd, 0xFF40FFFF);
			var statusLeaf = _tree.LeafNode($"状态：{debug.PathStatusText}，Partial={debug.IsPartial}，到达 Poly={debug.ReachedEndRef:X}");
			if (debug.IsPartial && statusLeaf.SelectedOrHovered)
			{
				_dd.DrawWorldLine(debug.ResolvedEnd, debug.RequestedEnd, 0xFFFF4080, 3);
				_dd.DrawWorldPointFilled(debug.RequestedEnd, 4, 0xFFFFFF40);
				_dd.DrawWorldPointFilled(debug.ResolvedEnd, 4, 0xFF40FFFF);
			}

			DrawDebugPolyline("原始多边形走廊中心线", debug.CorridorCenters, 0xFF40A0FF);
			DrawDebugPortals("原始门户", debug.RawPortals, false);
			DrawDebugPortals("收缩后门户", debug.TrimmedPortals, true);
			DrawDebugPolyline("居中中心线", debug.Centerline, 0xFF40FF40);
			DrawDebugPolyline("最终输出路径", debug.FinalPath, 0xFFFFFF40, 3, debug.ProtectedPointIndices);
		}
	}

	private void DrawDebugPolyline(string label, IReadOnlyList<Vector3> points, uint color, int thickness = 2, IReadOnlyCollection<int>? protectedPointIndices = null)
	{
		var leaf = _tree.LeafNode($"{label}：{points.Count} 个点");
		if (!leaf.SelectedOrHovered || points.Count < 2)
			return;

		for (var i = 0; i < points.Count - 1; i++)
			_dd.DrawWorldLine(points[i], points[i + 1], color, thickness);
		for (var i = 0; i < points.Count; i++)
		{
			var pointColor = protectedPointIndices?.Contains(i) == true ? 0xFFFF4040 : color;
			_dd.DrawWorldPointFilled(points[i], 3, pointColor);
		}
	}

	private void DrawDebugPortals(string label, IReadOnlyList<DebugPortalSegment> segments, bool includeMetrics)
	{
		var leaf = _tree.LeafNode($"{label}：{segments.Count} 条");
		if (!leaf.SelectedOrHovered)
			return;

		foreach (var segment in segments)
		{
			var color = segment.IsProtectedAnchor ? 0xFFFF4040u : segment.IsNarrow ? 0xFFFFA040u : 0xFF40D0FFu;
			_dd.DrawWorldLine(segment.From, segment.To, color, 2);
			_dd.DrawWorldPointFilled(segment.From, 3, color);
			_dd.DrawWorldPointFilled(segment.To, 3, color);
			if (includeMetrics)
			{
				var center = (segment.From + segment.To) * 0.5f;
				_tree.LeafNode($"门户宽度 {segment.Width:f3}，有效边距 {segment.EffectiveClearance:f3}，窄口={segment.IsNarrow}，保护={segment.IsProtectedAnchor} @ {center:f3}");
			}
		}
	}

	private void DrawDebugPoint(string label, Vector3 point, uint color)
	{
		var leaf = _tree.LeafNode($"{label}：{point:f3}");
		if (!leaf.SelectedOrHovered)
			return;

		_dd.DrawWorldPointFilled(point, 4, color);
	}

	private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
	{
		_drawNavmesh?.Dispose();
		_drawNavmesh = null;
		_debugVoxelMap?.Dispose();
		_debugVoxelMap = null;
	}
}
