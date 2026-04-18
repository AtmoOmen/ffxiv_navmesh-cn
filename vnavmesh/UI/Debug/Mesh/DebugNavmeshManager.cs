using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using Dalamud.Interface.Utility.Raii;
using FFXIVClientStructs.FFXIV.Client.UI.Agent;
using vnavmesh.Bootstrap;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Requests;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Debug.Volume;

namespace vnavmesh.UI.Debug.Mesh;

internal class DebugNavmeshManager : IDisposable
{
    private NavmeshManager       manager;
    private MovementPlanExecutor movementExecutor;
    private AsyncMoveRequest     asyncMove;
    private UITree               tree = new();
    private DebugDrawer          dd;
    
    private DebugDetourNavmesh? drawNavmesh;
    private DebugVoxelMap?      debugVoxelMap;
    private DebugLinks?         debugLinks;

    private Vector3 target;

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

        if (ImGui.CollapsingHeader("寻路", ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.TextUnformatted($"正在执行: {(manager.PathfindInProgress ? 1 : 0)}\t正在等待: {manager.NumQueuedPathfindRequests}");
            
            ImGui.Checkbox("允许移动",     ref movementExecutor.MovementAllowed);
            ImGui.Checkbox("使用射线检测", ref manager.UseRaycasts);
            ImGui.Checkbox("使用拉绳算法", ref manager.UseStringPulling);
            
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
                    tree.LeafNode($"Partial 次数：{diagnostics.PartialQueries}，占比 {partialRate:P1}");
                    tree.LeafNode($"疑似接缝截断：{diagnostics.SuspectedTileSeamCutoffs}");
                    tree.LeafNode($"any-angle 选中：{diagnostics.AnyAnglePreferred}");
                    tree.LeafNode($"普通 A* 回退：{diagnostics.ClassicFallbacks}");
                    tree.LeafNode($"起点重选：{diagnostics.StartReplacements}");
                    tree.LeafNode($"终点重选：{diagnostics.EndReplacements}");
                    tree.LeafNode($"自动向下攀爬链接：{diagnostics.GeneratedClimbLinksAccepted}");
                    tree.LeafNode($"自动边缘跳跃链接：{diagnostics.GeneratedJumpLinksAccepted}");
                }
            }

            drawNavmesh ??= new(manager.Navmesh.Mesh, manager.Query.MeshQuery, manager.Query.LastPath, tree, dd);
            drawNavmesh.Draw();

            if (manager.Navmesh.Volume != null)
            {
                debugVoxelMap ??= new(manager.Navmesh.Volume, manager.Query.VolumeQuery, tree, dd);
                debugVoxelMap.Draw();
            }

            debugLinks ??= new(manager.Navmesh, dd);
            debugLinks.Draw();
        }
    }

    private void DrawPosition(string tag, Vector3 position)
    {
        manager.Navmesh!.Mesh.CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        tree.LeafNode($"{tag}位置：{position:f3}，区块 (Tile)：{tileX}x{tileZ}，多边形 (Poly)：{manager.Query!.FindNearestMeshPoly(position):X}");
        var voxel = manager.Query.FindNearestVolumeVoxel(position);
        if (tree.LeafNode($"{tag}体素：{voxel:X}###{tag}voxel").SelectedOrHovered && voxel != VoxelMap.INVALID_VOXEL)
            debugVoxelMap?.VisualizeVoxel(voxel);
    }

    private void ExportBitmap(Vector3 startingPos) =>
        manager.BuildBitmap(startingPos, "D:\\navmesh.bmp", 0.5f);

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        drawNavmesh?.Dispose();
        drawNavmesh = null;
        debugVoxelMap?.Dispose();
        debugVoxelMap = null;
    }
}
