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
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Mesh.Runtime;
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
    private const uint  PathCornerScanColor              = 0x669B59B6u;
    private const uint  PathCornerOriginalColor          = 0xFFB71C1Cu;
    private const uint  PathCornerScanOriginColor        = 0xFF26A69Au;
    private const uint  PathCornerAdjustedColor          = 0xFF66BB6Au;
    private const uint  PathCornerInteriorColor          = 0xFF26C6DAu;
    private const uint  PathCornerPreferredColor         = 0xFF42A5F5u;
    private const uint  PathCornerPressureColor          = 0xFFFFA726u;
    private const uint  PathCornerMoveColor              = 0xFFFFFF00u;
    private const uint  PathCornerSkippedColor           = 0xFF9E9E9Eu;
    private const uint  PathRequestedStartLinkColor      = 0xFFEC407Au;
    private const uint  PathActualStartColor             = 0xFF8E24AAu;
    private const uint  PathConsumedPrefixColor          = 0xFF757575u;
    private const uint  PathFlightOriginalColor          = 0xFF8E24AAu;
    private const uint  PathFlightHorizontalColor        = 0xFF42A5F5u;
    private const uint  PathFlightVerticalColor          = 0xFF26A69Au;
    private const uint  PathFlightCombinedColor          = 0xFFFFB300u;
    private const uint  PathFlightCoarseLineColor        = 0xFF7CFC00u;
    private const uint  PathFlightCoarsePointColor       = 0xFFB2FF59u;
    private const uint  PathFlightCoarseBoxColor         = 0x442EE6A6u;
    private const uint  PathFlightProxyPointColor        = 0xFF00E5FFu;
    private const uint  PathFlightProxyLineColor         = 0xFF00BCD4u;
    private const uint  PathFlightProxyTailDirectColor   = 0xFF00E676u;
    private const uint  PathFlightProxyTailRelayColor    = 0xFFFFC107u;

    private sealed record RenderedPath(Vector3 RequestStart, PostprocessedPath Result, uint LineColor, uint PointColor, uint StartColor, uint EndColor, string Label);

    private NavmeshManager       manager;
    private MovementPlanExecutor movementExecutor;
    private AsyncMoveRequest     asyncMove;
    private UITree               tree = new();
    private DebugDrawer          dd;
    
    private DebugDetourNavmesh? drawNavmesh;
    private DebugVoxelMap?      debugVoxelMap;
    private DebugLinks?         debugLinks;

    private Vector3 target;
    private Task<PostprocessedPath>?      renderPathTask;
    private CancellationTokenSource?      renderPathCancelSource;
    private List<RenderedPath>            renderedPaths = [];
    private bool                          renderPathFlyMode;
    private Vector3                       renderPathRequestStart;
    private bool                          renderStraightPathMode;
    private bool                          showCornerPushDebug = true;

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
            ImGui.Checkbox("显示角点扫描/推出调试", ref showCornerPushDebug);

            ImGui.Checkbox("允许移动", ref movementExecutor.MovementAllowed);
            
            ImGui.NewLine();
            
            ImGui.TextUnformatted($"目标位置: {target}");
            
            var player    = Service.ObjectTable.LocalPlayer;
            
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
                if (ImGui.Button("停止寻路"))
                    movementExecutor.Stop();
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
            
            DrawPosition("玩家", player?.Position ?? default);
            DrawPosition("目标", target);
            DrawPosition("标点", MapUtil.FlagToPoint(manager.Query)                          ?? default);
            DrawPosition("地面", manager.Query.FindPointOnFloor(player?.Position ?? default) ?? default);
        }

        if (ImGui.CollapsingHeader("统计", ImGuiTreeNodeFlags.DefaultOpen))
        {
            using (var nd = tree.Node("地面寻路"))
            {
                if (nd.Opened)
                {
                    var diagnostics = manager.Query.GetGroundDiagnostics();
                    var partialRate = diagnostics.GroundQueries > 0 ? diagnostics.PartialQueries / (double)diagnostics.GroundQueries : 0;
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
        manager.Navmesh!.Mesh.CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        var nearestAll       = manager.Query!.FindNearestMeshPoly(position);
        var nearestReachable = manager.Query.FindNearestMeshPoly(position, allowUnreachable: false);
        var nearestPointAll  = manager.Query.FindNearestPointOnMesh(position);
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
        if (tree.LeafNode($"{tag}体素 Flight：{voxelSurface.Voxel:X}，地表锚定={(voxelSurface.UsedSurfaceAnchor ? "是" : "否")}，搜索点={voxelSurface.SearchPoint:f3}，安全点={voxelSurface.SafePoint:f3}###{tag}voxelFlight").SelectedOrHovered &&
            voxelSurface.Voxel != VoxelMap.INVALID_VOXEL)
            debugVoxelMap?.VisualizeVoxel(voxelSurface.Voxel);
    }

    private static string FormatPolyRef(long polyRef) => polyRef != 0 ? polyRef.ToString("X") : "<none>";

    private static string FormatVector(Vector3? value) => value is { } point ? point.ToString("f3") : "<none>";

    private void ExportBitmap(Vector3 startingPos) =>
        manager.BuildBitmap(startingPos, "D:\\navmesh.bmp", 0.5f);

    public void DrawRenderedPaths()
    {
        foreach (var renderedPath in renderedPaths)
        {
            DrawRenderedPath(renderedPath);
            if (showCornerPushDebug)
            {
                DrawRenderedPathCornerDebug(renderedPath);
                DrawRenderedPathFlightDebug(renderedPath);
            }
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
        renderPathTask        = straightPath
                                    ? manager.QueryStraightPathDetailed(from, to, fly, externalCancel: renderPathCancelSource.Token)
                                    : manager.QueryPathDetailed(from, to, fly, externalCancel: renderPathCancelSource.Token);
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
                var renderedPath = renderStraightPathMode
                                       ? new RenderedPath(renderPathRequestStart, result, 0xFF00BCD4u, 0xFF0097A7u, 0xFF1E88E5u, 0xFFE53935u, "StraightPath")
                                       : new RenderedPath
                                       (
                                           renderPathRequestStart,
                                           result,
                                           renderPathFlyMode ? 0xFF2ECC71u : 0xFFF39C12u,
                                           renderPathFlyMode ? 0xFF27AE60u : 0xFFD35400u,
                                           0xFF3498DBu,
                                           0xFFE74C3Cu,
                                           renderPathFlyMode ? "Flight" : "Ground"
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
            renderPathTask         = null;
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
        var coarseDebugOnly = HasCoarseOnlyFlightDebug(renderedPath.Result);
        List<Vector3> points = [];
        var firstSegment = renderedPath.Result.Segments.FirstOrDefault();
        var actualStart = firstSegment?.StartPosition
                       ?? (renderedPath.Result.Waypoints.Count > 0 ? renderedPath.Result.Waypoints[0] : renderedPath.Result.FinalDestination);
        var initialWaypointIndex = firstSegment?.GroundCorridor?.InitialWaypointIndex ?? 0;

        if (!coarseDebugOnly)
            DrawConsumedPrefix(firstSegment, actualStart, initialWaypointIndex);

        if (!coarseDebugOnly)
            points.Add(initialWaypointIndex > 0 ? renderedPath.RequestStart : actualStart);

        if (!coarseDebugOnly)
        {
            var firstWaypointSkipped = false;
            foreach (var segment in renderedPath.Result.Segments)
            {
                var waypointStart = !firstWaypointSkipped ? Math.Clamp(initialWaypointIndex, 0, segment.Waypoints.Count) : 0;
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

        dd.DrawWorldPointFilled(renderedPath.RequestStart, 4, renderedPath.StartColor);
        dd.DrawWorldPointFilled(actualStart, 4, PathActualStartColor);
        dd.DrawWorldPointFilled(renderedPath.Result.FinalDestination, 4, renderedPath.EndColor);

        if (!coarseDebugOnly && Vector3.DistanceSquared(renderedPath.RequestStart, actualStart) > DuplicateRenderedPointDistanceSq)
            dd.DrawWorldLine(renderedPath.RequestStart, actualStart, PathRequestedStartLinkColor, 2);
    }

    private void DrawRenderedPathCornerDebug(RenderedPath renderedPath)
    {
        foreach (var segment in renderedPath.Result.Segments)
        {
            if (segment.GroundCorridor == null)
                continue;

            for (var cornerIndex = 0; cornerIndex < segment.GroundCorridor.Corners.Count; ++cornerIndex)
            {
                var corner = segment.GroundCorridor.Corners[cornerIndex];
                if (corner.Debug is not { } debug)
                    continue;

                foreach (var sample in debug.Samples)
                    dd.DrawWorldLine(sample.Start, sample.Endpoint, ColorForClearance(sample.Clearance, debug.MaxClearance), 1);

                dd.DrawWorldLine(debug.OriginalPosition, debug.ScanOrigin, PathCornerScanColor, 1);
                dd.DrawWorldLine(debug.ScanOrigin, debug.InteriorDirectionEndpoint, PathCornerInteriorColor, 2);
                dd.DrawWorldLine(debug.ScanOrigin, debug.PreferredDirectionEndpoint, PathCornerPreferredColor, 2);
                dd.DrawWorldLine(debug.ScanOrigin, debug.WallPressureEndpoint, PathCornerPressureColor, 2);

                if (debug.PushApplied)
                    dd.DrawWorldLine(debug.OriginalPosition, debug.AdjustedPosition, PathCornerMoveColor, 3);

                var pointColor = debug.InitiallyConsumed ? PathCornerSkippedColor : debug.PushApplied ? PathCornerAdjustedColor : renderedPath.PointColor;
                if (cornerIndex == segment.GroundCorridor.InitialCornerIndex)
                    dd.DrawWorldPointFilled(debug.AdjustedPosition, 6, 0x33FFFFFFu);

                dd.DrawWorldPointFilled(debug.OriginalPosition, 3, debug.InitiallyConsumed ? PathCornerSkippedColor : PathCornerOriginalColor);
                dd.DrawWorldPointFilled(debug.ScanOrigin, 3, PathCornerScanOriginColor);
                dd.DrawWorldPointFilled(debug.AdjustedPosition, 4, pointColor);

                var labelAnchor = debug.AdjustedPosition + new Vector3(0, 0.12f, 0);
                dd.DrawWorldText
                (
                    labelAnchor,
                    $"idx={debug.StraightPathIndex} skip={(debug.InitiallyConsumed ? 1 : 0)} exec={(debug.IsExecutionStart ? 1 : 0)} push={debug.PushDistance:F2} raw={debug.RawPushDistance:F2} cap={debug.DynamicPushMaxDistance:F2} w={debug.DynamicPushWidth:F2} sc={debug.DynamicPushScale:F2} min={debug.MinClearance:F2} avg={debug.AverageClearance:F2} corner={debug.CornerStrength:F2} L={debug.LeftClearance:F2}@{debug.LeftPolyRef:X} R={debug.RightClearance:F2}@{debug.RightPolyRef:X} bal={(debug.StraightBalanceSatisfied ? 1 : 0)} low={(debug.StraightLowClearanceCase ? 1 : 0)} re={(debug.Rescanned ? 1 : 0)} in={(debug.UsedInteriorDirection ? 1 : 0)} cand={debug.LocalPolyCount} scan={debug.ScanPolyRef:X} pref={debug.PreferredPolyRef:X}",
                    0xFFFFFFFFu
                );
            }
        }
    }

    private void DrawRenderedPathFlightDebug(RenderedPath renderedPath)
    {
        foreach (var segment in renderedPath.Result.Segments)
        {
            if (segment.FlightPathDebug == null)
                continue;

            if (segment.FlightPathDebug.CoarsePath.Count > 0)
                DrawFlightCoarsePath(segment.FlightPathDebug.CoarsePath);
            if (segment.FlightPathDebug.ProxyDebug is { } proxyDebug)
                DrawFlightProxyDebug(proxyDebug);

            foreach (var debug in segment.FlightPathDebug.Waypoints)
            {
                foreach (var sample in debug.Samples)
                    dd.DrawWorldLine(sample.Start, sample.Endpoint, ColorForClearance(sample.Clearance, debug.MaxClearance), 1);

                if (Vector3.DistanceSquared(debug.OriginalPosition, debug.HorizontalBiasEndpoint) > DuplicateRenderedPointDistanceSq)
                    dd.DrawWorldLine(debug.OriginalPosition, debug.HorizontalBiasEndpoint, PathFlightHorizontalColor, 2);
                if (Vector3.DistanceSquared(debug.OriginalPosition, debug.VerticalBiasEndpoint) > DuplicateRenderedPointDistanceSq)
                    dd.DrawWorldLine(debug.OriginalPosition, debug.VerticalBiasEndpoint, PathFlightVerticalColor, 2);
                if (Vector3.DistanceSquared(debug.OriginalPosition, debug.CombinedBiasEndpoint) > DuplicateRenderedPointDistanceSq)
                    dd.DrawWorldLine(debug.OriginalPosition, debug.CombinedBiasEndpoint, PathFlightCombinedColor, 1);
                if (debug.PushApplied)
                    dd.DrawWorldLine(debug.OriginalPosition, debug.AdjustedPosition, PathCornerMoveColor, 3);

                dd.DrawWorldPointFilled(debug.OriginalPosition, 3, PathFlightOriginalColor);
                dd.DrawWorldPointFilled(debug.AdjustedPosition, 4, debug.PushApplied ? PathCornerAdjustedColor : renderedPath.PointColor);

                dd.DrawWorldText
                (
                    debug.AdjustedPosition + new Vector3(0, 0.12f, 0),
                    $"idx={debug.PathIndex} push={debug.PushDistance:F2} h={debug.HorizontalPushDistance:F2} v={debug.VerticalPushDistance:F2} hi={debug.HorizontalImbalance:F2} vi={debug.VerticalImbalance:F2} F={debug.ForwardClearance:F2} B={debug.BackwardClearance:F2} L={debug.LeftClearance:F2} R={debug.RightClearance:F2} FL={debug.ForwardLeftClearance:F2} FR={debug.ForwardRightClearance:F2} BL={debug.BackwardLeftClearance:F2} BR={debug.BackwardRightClearance:F2} U={debug.UpClearance:F2} D={debug.DownClearance:F2} vm={debug.VerticalMode} sel={debug.SelectedAdjustmentKind} gd={(debug.GoalDescentApproach ? 1 : 0)} dt={(debug.DownhillTunnelTrend ? 1 : 0)} ct={(debug.ConstrainedTunnelDescent ? 1 : 0)} td={(debug.TunnelDescentAssist ? 1 : 0)} cu={(debug.HeightCatchUpRequested ? 1 : 0)} ad={(debug.AllowDownwardPush ? 1 : 0)} raise={(debug.FinalRaiseApplied ? 1 : 0)} hm={debug.HeightMatchTarget:F2} pm={debug.PreferredMinHeight:F2} baseY={debug.BaseAdjustedPosition.Y:F2} vox={debug.OriginalVoxel:X}->{debug.AdjustedVoxel:X}",
                    0xFFFFFFFFu
                );
            }
        }
    }

    private void DrawFlightCoarsePath(IReadOnlyList<FlightCoarsePathDebugNode> coarsePath)
    {
        var volume = manager.Navmesh?.Volume;
        for (var i = 0; i < coarsePath.Count; ++i)
        {
            var node = coarsePath[i];
            if (volume != null && node.Voxel != VoxelMap.INVALID_VOXEL)
            {
                var bounds = volume.VoxelBounds(node.Voxel, 0);
                dd.DrawWorldAABB((bounds.min + bounds.max) * 0.5f, (bounds.max - bounds.min) * 0.5f, PathFlightCoarseBoxColor, 1);
            }

            dd.DrawWorldPointFilled(node.Position, 4, PathFlightCoarsePointColor);
            dd.DrawWorldText(node.Position + new Vector3(0, 0.12f, 0), $"L1[{node.PathIndex}] {node.Voxel:X}", 0xFFFFFFFFu);

            if (i > 0)
                dd.DrawWorldLine(coarsePath[i - 1].Position, node.Position, PathFlightCoarseLineColor, 2);
        }
    }

    private void DrawFlightProxyDebug(FlightLongRangeProxyDebug proxyDebug)
    {
        var volume = manager.Navmesh?.Volume;
        if (volume != null && proxyDebug.ProxyVoxel != VoxelMap.INVALID_VOXEL)
        {
            var bounds = volume.VoxelBounds(proxyDebug.ProxyVoxel, 0);
            dd.DrawWorldAABB((bounds.min + bounds.max) * 0.5f, (bounds.max - bounds.min) * 0.5f, PathFlightProxyLineColor, 1);
        }

        dd.DrawWorldPointFilled(proxyDebug.ProxyPosition, 5, PathFlightProxyPointColor);
        dd.DrawWorldLine(proxyDebug.ProxyPosition, proxyDebug.TailStartPosition, PathFlightProxyLineColor, 2);

        var tailColor = proxyDebug.TailKind switch
        {
            FlightLongRangeTailKind.DirectToGoal       => PathFlightProxyTailDirectColor,
            FlightLongRangeTailKind.ShortRangeRelay    => PathFlightProxyTailRelayColor,
            FlightLongRangeTailKind.ShortRangeRelayPartial => PathFlightProxyTailRelayColor,
            _                                          => PathFlightProxyLineColor
        };

        if (Vector3.DistanceSquared(proxyDebug.TailStartPosition, proxyDebug.TailTargetPosition) > DuplicateRenderedPointDistanceSq)
            dd.DrawWorldLine(proxyDebug.TailStartPosition, proxyDebug.TailTargetPosition, tailColor, 2);

        dd.DrawWorldPointFilled(proxyDebug.TailStartPosition, 4, tailColor);
        dd.DrawWorldText
        (
            proxyDebug.ProxyPosition + new Vector3(0, 0.16f, 0),
            $"proxy {proxyDebug.ProxyVoxel:X} tail={proxyDebug.TailKind}",
            0xFFFFFFFFu
        );
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

    private static uint ColorForClearance(float clearance, float maxClearance)
    {
        var normalized = maxClearance > 0.0001f ? Math.Clamp(clearance / maxClearance, 0f, 1f) : 0f;
        var red        = (byte)(255 * (1f - normalized));
        var green      = (byte)(255 * normalized);
        return 0x66000000u | red | ((uint)green << 8);
    }

    private static bool HasCoarseOnlyFlightDebug(PostprocessedPath result)
        => result.Segments.Any(segment => segment is { Waypoints.Count: 0, FlightPathDebug.CoarsePath.Count: > 0 });
}
