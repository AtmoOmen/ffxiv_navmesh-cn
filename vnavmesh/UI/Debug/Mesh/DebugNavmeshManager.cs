using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using Dalamud.Interface.Utility.Raii;
using FFXIVClientStructs.FFXIV.Client.UI.Agent;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navigation;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Debug.Volume;

namespace vnavmesh.UI.Debug.Mesh;

internal class DebugNavmeshManager : IDisposable
{
    private const float DuplicateRenderedPointDistanceSq = 0.000001f;
    private const uint  PathRawGroundLineColor           = 0xFF5C6BC0u;
    private const uint  PathRawGroundPointColor          = 0xFF3949ABu;
    private const uint  PathRequestedStartLinkColor      = 0xFFEC407Au;
    private const uint  PathActualStartColor             = 0xFF8E24AAu;
    private const uint  PathConsumedPrefixColor          = 0xFF757575u;

    private sealed record RenderedPath
    (
        Vector3           RequestStart,
        PostprocessedPath Result,
        uint              LineColor,
        uint              PointColor,
        uint              StartColor,
        uint              EndColor,
        string            Label
    );

    private NavmeshManager       manager;
    private MovementPlanExecutor movementExecutor;
    private AsyncMoveRequest     asyncMove;
    private UITree               tree = new();
    private DebugDrawer          dd;

    private DebugDetourNavmesh? drawNavmesh;
    private DebugVoxelMap?      debugVoxelMap;
    private DebugLinks?         debugLinks;

    private Vector3                  target;
    private Task<PostprocessedPath>? renderPathTask;
    private CancellationTokenSource? renderPathCancelSource;
    private List<RenderedPath>       renderedPaths = [];
    private bool                     renderPathFlyMode;
    private Vector3                  renderPathRequestStart;
    private bool                     renderStraightPathMode;
    private bool                     showRawGroundPath = true;

    public DebugNavmeshManager
    (
        DebugDrawer          dd,
        NavmeshManager       manager,
        MovementPlanExecutor movementExecutor,
        AsyncMoveRequest     move
    )
    {
        this.manager                  =  manager;
        this.movementExecutor         =  movementExecutor;
        asyncMove                     =  move;
        this.dd                       =  dd;
        this.manager.OnNavmeshChanged += OnNavmeshChanged;
    }

    public void Dispose()
    {
        CancelRenderPathTask();
        manager.OnNavmeshChanged -= OnNavmeshChanged;
        drawNavmesh?.Dispose();
        debugVoxelMap?.Dispose();
    }

    public void Draw()
    {
        if (ImGui.CollapsingHeader("导航路网", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.TextUnformatted($"场景键: {manager.CurrentKey}");

            ImGui.Spacing();

            var progress = manager.LoadTaskProgress;

            if (progress >= 0)
            {
                ImGui.ProgressBar(progress, ImGuiHelpers.ScaledVector2(200, 0));

                ImGui.SameLine();
                ImGui.TextUnformatted("构建进度");
            }
            else
            {
                if (ImGui.Button("重新加载"))
                    manager.Reload(true);

                ImGui.SameLine();
                if (ImGui.Button("重新构建"))
                    manager.Reload(false);
            }
        }

        if (manager.Navmesh == null || manager.Query == null)
            return;

        TryCollectRenderedPathResult();

        if (ImGui.CollapsingHeader("寻路", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.TextUnformatted($"正在执行: {(manager.PathfindInProgress ? 1 : 0)}\t正在等待: {manager.NumQueuedPathfindRequests}");
            if (renderPathTask != null)
                ImGui.TextUnformatted($"渲染算路执行中: {(renderPathFlyMode ? "空间" : "地面")} / {(renderStraightPathMode ? "StraightPath" : "普通")}");
            ImGui.TextUnformatted($"已缓存渲染路径: {renderedPaths.Count}");
            ImGui.Checkbox("显示原始漏斗路径对照", ref showRawGroundPath);

            ImGui.Checkbox("允许移动", ref movementExecutor.MovementAllowed);

            ImGui.NewLine();

            ImGui.TextUnformatted($"目标位置: {target}");

            var player = Service.ObjectTable.LocalPlayer;

            using (ImRaii.Disabled(player == null))
            {
                if (ImGui.Button("当前位置"))
                    target = player?.Position ?? default;
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(player?.TargetObject == null))
            {
                if (ImGui.Button("目标位置"))
                    target = player?.TargetObject?.Position ?? default;
            }

            unsafe
            {
                ImGui.SameLine();

                using (ImRaii.Disabled(AgentMap.Instance()->FlagMarkerCount == 0))
                {
                    if (ImGui.Button("地图标记位置"))
                        target = MapUtil.FlagToPoint(manager.Query) ?? default;
                }
            }

            using (ImRaii.Disabled(target == Vector3.Zero))
            {
                if (ImGui.Button("地面寻路"))
                    asyncMove.MoveTo(target, false);

                ImGui.SameLine();
                if (ImGui.Button("空间寻路"))
                    asyncMove.MoveTo(target, true);

                ImGui.SameLine(0, ImGui.GetStyle().ItemSpacing.X * ImGuiHelpers.GlobalScale);
                if (ImGui.Button("取消寻路"))
                    asyncMove.Stop();
            }

            using (ImRaii.Disabled(player == null || target == Vector3.Zero || renderPathTask != null))
            {
                if (ImGui.Button("发起寻路(渲染)"))
                    StartRenderedPathQuery(player!.Position, target, false, false);

                ImGui.SameLine();
                if (ImGui.Button("发起直线路径(渲染)"))
                    StartRenderedPathQuery(player!.Position, target, false, true);

                ImGui.SameLine();
                if (ImGui.Button("发起空间寻路(渲染)"))
                    StartRenderedPathQuery(player!.Position, target, true, false);
            }

            ImGui.SameLine();

            using (ImRaii.Disabled(renderedPaths.Count == 0 && renderPathTask == null))
            {
                if (ImGui.Button("清除渲染结果"))
                {
                    CancelRenderPathTask();
                    renderedPaths.Clear();
                }
            }

            ImGui.NewLine();

            if (ImGui.Button("导出位图 (玩家中心)"))
                ExportBitmap(player?.Position ?? default);

            if (player != null)
            {
                DrawPosition("玩家", player.Position);
                var floor = manager.Query.FindPointOnFloor(player.Position);
                if (floor != null)
                    DrawPosition("地面", floor.Value);
            }

            if (target != Vector3.Zero)
                DrawPosition("目标", target);
            var flag = MapUtil.FlagToPoint(manager.Query);
            if (flag != null)
                DrawPosition("标点", flag.Value);
        }

        if (ImGui.CollapsingHeader("统计", ImGuiTreeNodeFlags.DefaultOpen))
        {
            using (var nd = tree.Node("地面寻路"))
            {
                if (nd.Opened)
                {
                    var diagnostics = manager.Query.GetGroundDiagnostics();
                    var partialRate = diagnostics.GroundQueries > 0 ?
                                          diagnostics.PartialQueries / (double)diagnostics.GroundQueries :
                                          0;
                    tree.LeafNode($"总查询次数：{diagnostics.GroundQueries}");
                    tree.LeafNode($"失败次数：{diagnostics.FailedQueries}");
                    tree.LeafNode($"Partial 次数：{diagnostics.PartialQueries}，占比 {partialRate:P1}");
                    tree.LeafNode($"容差命中次数：{diagnostics.ReachedWithinRangeQueries}");
                    tree.LeafNode($"自动向下攀爬链接：{diagnostics.GeneratedClimbLinksAccepted}");
                    tree.LeafNode($"自动边缘跳跃链接：{diagnostics.GeneratedJumpLinksAccepted}");
                }
            }

            drawNavmesh ??= new(manager.Navmesh.Mesh, manager.Query.MeshQuery, manager.Query.LastPath, tree, dd);
            drawNavmesh.Draw();

            if (manager.Navmesh.Volume != null)
            {
                debugVoxelMap ??= new(manager.Navmesh.Volume, manager.Query.VolumeQuery, manager.Query, tree, dd);
                debugVoxelMap.Draw();
            }

            debugLinks ??= new(manager.Navmesh, dd);
            debugLinks.Draw();
        }
    }

    private void DrawPosition(string tag, Vector3 position)
    {
        if (position == Vector3.Zero)
            return;

        manager.Navmesh!.Mesh.CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        var nearestAll            = manager.Query!.FindNearestMeshPoly(position);
        var nearestReachable      = manager.Query.FindNearestMeshPoly(position, allowUnreachable: false);
        var nearestPointAll       = manager.Query.FindNearestPointOnMesh(position);
        var nearestPointReachable = manager.Query.FindNearestPointOnMesh(position, allowUnreachable: false);
        var floorAll              = manager.Query.FindPointOnFloor(position);
        var floorReachable        = manager.Query.FindPointOnFloor(position, allowUnreachable: false);

        tree.LeafNode
        (
            $"{tag}位置：{position:f3}，区块 (Tile)：{tileX}x{tileZ}，最近多边形 All/Reachable：{FormatPolyRef(nearestAll)} / {FormatPolyRef(nearestReachable)}"
        );
        tree.LeafNode($"{tag}最近点 All：{FormatVector(nearestPointAll)}");
        tree.LeafNode($"{tag}最近点 Reachable：{FormatVector(nearestPointReachable)}");
        tree.LeafNode($"{tag}地板投影 All：{FormatVector(floorAll)}");
        tree.LeafNode($"{tag}地板投影 Reachable：{FormatVector(floorReachable)}");
        var voxel = manager.Query.FindNearestVolumeVoxel(position);
        if (tree.LeafNode($"{tag}体素：{voxel:X}###{tag}voxel").SelectedOrHovered && voxel != VoxelMap.INVALID_VOXEL)
            debugVoxelMap?.VisualizeVoxel(voxel);
        var voxelSurface = manager.Query.FindNearestVolumeVoxelSurfaceAware(position);
        if (tree.LeafNode
            (
                $"{tag}体素 Flight：{voxelSurface.Voxel:X}，地表锚定={(voxelSurface.UsedSurfaceAnchor ? "是" : "否")}，搜索点={voxelSurface.SearchPoint:f3}，安全点={voxelSurface.SafePoint:f3}###{tag}voxelFlight"
            ).SelectedOrHovered &&
            voxelSurface.Voxel != VoxelMap.INVALID_VOXEL)
            debugVoxelMap?.VisualizeVoxel(voxelSurface.Voxel);
    }

    private static string FormatPolyRef(long polyRef) => polyRef != 0 ?
                                                             polyRef.ToString("X") :
                                                             "<none>";

    private static string FormatVector(Vector3? value) => value is { } point ?
                                                              point.ToString("f3") :
                                                              "<none>";

    private void ExportBitmap(Vector3 startingPos) =>
        manager.BuildBitmap([startingPos], "D:\\navmesh.bmp", 0.5f);

    public void DrawRenderedPaths()
    {
        foreach (var renderedPath in renderedPaths)
        {
            DrawRenderedPath(renderedPath);

            if (showRawGroundPath)
                DrawRawGroundPath(renderedPath);
        }
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        CancelRenderPathTask();
        renderedPaths.Clear();
        drawNavmesh?.Dispose();
        drawNavmesh = null;
        debugVoxelMap?.Dispose();
        debugVoxelMap = null;
    }

    private void StartRenderedPathQuery(Vector3 from, Vector3 to, bool fly, bool straightPath)
    {
        CancelRenderPathTask();
        renderPathFlyMode      = fly;
        renderStraightPathMode = straightPath;
        renderPathRequestStart = from;
        renderPathCancelSource = new();
        renderPathTask = straightPath ?
                             manager.QueryStraightPathDetailed(from, to, fly, externalCancel: renderPathCancelSource.Token) :
                             manager.QueryPathDetailed(from, to, fly, externalCancel: renderPathCancelSource.Token);
    }

    private void TryCollectRenderedPathResult()
    {
        if (renderPathTask is not { IsCompleted: true })
            return;

        try
        {
            var result = renderPathTask.Result;

            if (result.Succeeded)
            {
                var renderedPath = renderStraightPathMode ?
                                       new RenderedPath(renderPathRequestStart, result, 0xFF00BCD4u, 0xFF0097A7u, 0xFF1E88E5u, 0xFFE53935u, "StraightPath") :
                                       new RenderedPath
                                       (
                                           renderPathRequestStart,
                                           result,
                                           renderPathFlyMode ?
                                               0xFF2ECC71u :
                                               0xFFF39C12u,
                                           renderPathFlyMode ?
                                               0xFF27AE60u :
                                               0xFFD35400u,
                                           0xFF3498DBu,
                                           0xFFE74C3Cu,
                                           renderPathFlyMode ?
                                               "Flight" :
                                               "Ground"
                                       );
                renderedPaths.Add(renderedPath);
            }
        }
        catch (OperationCanceledException)
        {
        }
        catch (Exception ex)
        {
            Plugin.DuoLog(ex, "渲染算路失败");
        }
        finally
        {
            renderPathTask.Dispose();
            renderPathTask = null;
            renderPathCancelSource?.Dispose();
            renderPathCancelSource = null;
        }
    }

    private void CancelRenderPathTask()
    {
        renderPathCancelSource?.Cancel();

        if (renderPathTask is { IsCompleted: true })
        {
            renderPathTask.Dispose();
            renderPathTask = null;
        }

        renderPathCancelSource?.Dispose();
        renderPathCancelSource = null;
    }

    private void DrawRenderedPath(RenderedPath renderedPath)
    {
        List<Vector3> points       = [];
        var           firstSegment = renderedPath.Result.Segments.FirstOrDefault();
        var actualStart = firstSegment?.StartPosition ??
                          (renderedPath.Result.Waypoints.Count > 0 ?
                               renderedPath.Result.Waypoints[0] :
                               renderedPath.Result.FinalDestination);
        var initialWaypointIndex = firstSegment?.GroundCorridor?.InitialCornerIndex ?? 0;

        DrawConsumedPrefix(firstSegment, actualStart, initialWaypointIndex);

        points.Add
        (
            initialWaypointIndex > 0 ?
                renderedPath.RequestStart :
                actualStart
        );

        {
            var firstWaypointSkipped = false;

            foreach (var segment in renderedPath.Result.Segments)
            {
                var waypointStart = !firstWaypointSkipped ?
                                        Math.Clamp(initialWaypointIndex, 0, segment.Waypoints.Count) :
                                        0;
                firstWaypointSkipped = true;

                for (var i = waypointStart; i < segment.Waypoints.Count; ++i)
                {
                    var waypoint = segment.Waypoints[i];
                    if (Vector3.DistanceSquared(points[^1], waypoint) > DuplicateRenderedPointDistanceSq)
                        points.Add(waypoint);
                }
            }

            if (Vector3.DistanceSquared(points[^1], renderedPath.Result.FinalDestination) > DuplicateRenderedPointDistanceSq)
                points.Add(renderedPath.Result.FinalDestination);

            for (var i = 1; i < points.Count; ++i)
            {
                dd.DrawWorldLine(points[i - 1], points[i], renderedPath.LineColor, 2);
                dd.DrawWorldPointFilled(points[i], 3, renderedPath.PointColor);
            }
        }

        dd.DrawWorldPointFilled(renderedPath.RequestStart,            4, renderedPath.StartColor);
        dd.DrawWorldPointFilled(actualStart,                          4, PathActualStartColor);
        dd.DrawWorldPointFilled(renderedPath.Result.FinalDestination, 4, renderedPath.EndColor);

        if (Vector3.DistanceSquared(renderedPath.RequestStart, actualStart) > DuplicateRenderedPointDistanceSq)
            dd.DrawWorldLine(renderedPath.RequestStart, actualStart, PathRequestedStartLinkColor, 2);
    }

    private void DrawRawGroundPath(RenderedPath renderedPath)
    {
        foreach (var segment in renderedPath.Result.Segments)
        {
            if (segment.GroundCorridor is not { RawCorners.Count: > 0 } groundCorridor)
                continue;

            for (var i = 0; i < groundCorridor.RawCorners.Count; ++i)
            {
                var point = groundCorridor.RawCorners[i].Position;
                if (i > 0)
                    dd.DrawWorldLine(groundCorridor.RawCorners[i - 1].Position, point, PathRawGroundLineColor, 1);

                dd.DrawWorldPointFilled(point, 2, PathRawGroundPointColor);
            }
        }
    }

    private void DrawConsumedPrefix(PostprocessedPathSegment? firstSegment, Vector3 actualStart, int initialWaypointIndex)
    {
        if (firstSegment == null || initialWaypointIndex <= 0 || firstSegment.Waypoints.Count == 0)
            return;

        List<Vector3> prefix = [actualStart];

        for (var i = 0; i < Math.Min(initialWaypointIndex, firstSegment.Waypoints.Count); ++i)
        {
            var waypoint = firstSegment.Waypoints[i];
            if (Vector3.DistanceSquared(prefix[^1], waypoint) > DuplicateRenderedPointDistanceSq)
                prefix.Add(waypoint);
        }

        for (var i = 1; i < prefix.Count; ++i)
        {
            dd.DrawWorldLine(prefix[i - 1], prefix[i], PathConsumedPrefixColor, 1);
            dd.DrawWorldPointFilled(prefix[i], 2, PathConsumedPrefixColor);
        }
    }

}
