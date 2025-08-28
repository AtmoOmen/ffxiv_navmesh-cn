using System;
using System.Numerics;
using Dalamud.Interface.Utility.Raii;
using ImGuiNET;
using Navmesh.Movement;
using Navmesh.NavVolume;

namespace Navmesh.Debug;

internal class DebugNavmeshManager : IDisposable
{
    private bool IsMeshQueryReady => NavmeshManager is { Navmesh: not null, Query: not null };
    
    private readonly UITree UITree = new();
    
    private readonly NavmeshManager   NavmeshManager;
    private readonly FollowPath       FollowPath;
    private readonly AsyncMoveRequest AsyncMove;
    private readonly DebugDrawer      DebugDrawer;
    
    private DebugDetourNavmesh? DrawNavmesh;
    private DebugVoxelMap?      DrawVoxelMap;
    
    private Vector3 TargetPos;

    public DebugNavmeshManager(DebugDrawer dd, NavmeshManager manager, FollowPath path, AsyncMoveRequest move)
    {
        NavmeshManager = manager;
        FollowPath     = path;
        AsyncMove      = move;
        DebugDrawer    = dd;
        
        NavmeshManager.OnNavmeshChanged += OnNavmeshChanged;
    }

    public void Dispose()
    {
        NavmeshManager.OnNavmeshChanged -= OnNavmeshChanged;
        DrawNavmesh?.Dispose();
        DrawVoxelMap?.Dispose();
    }

    public void Draw()
    {
        var player    = Service.ClientState.LocalPlayer;
        var playerPos = player?.Position ?? default;
        
        ImGui.Text($"当前区域: {NavmeshManager.CurrentKey}");

        using (ImRaii.PushIndent())
        {
            var progress = NavmeshManager.LoadTaskProgress;
            if (progress >= 0)
                ImGui.ProgressBar(progress, new(200, 0));
            else
            {
                if (ImGui.Button("重载"))
                    NavmeshManager.Reload(true);

                ImGui.SameLine();
                if (ImGui.Button("重构"))
                    NavmeshManager.Reload(false);

                if (IsMeshQueryReady)
                {
                    ImGui.SameLine();
                    if (ImGui.Button("导出位图"))
                        ExportBitmap(NavmeshManager.Navmesh!, NavmeshManager.Query!, playerPos);
                }
            }
        }
        
        ImGui.NewLine();
        
        ImGui.Text("寻路任务");

        using (ImRaii.PushIndent())
        {
            ImGui.Text($"计算中: {(NavmeshManager.PathfindInProgress ? 1 : 0)}");
            ImGui.Text($"排队中: {NavmeshManager.NumQueuedPathfindRequests}");
        }

        if (!IsMeshQueryReady)
            return;
        
        ImGui.NewLine();
        
        ImGui.Text("手动寻路");

        using (ImRaii.PushIndent())
        {
            ImGui.Text($"玩家位置: {playerPos:F1}");
            ImGui.Text($"目的地: {TargetPos:F1}");
            
            ImGui.AlignTextToFramePadding();
            ImGui.Text("设定:");
            
            ImGui.SameLine();
            if (ImGui.Button("当前位置"))
                TargetPos = player?.Position ?? default;

            ImGui.SameLine();
            if (ImGui.Button("选中目标位置"))
                TargetPos = player?.TargetObject?.Position ?? default;

            ImGui.SameLine();
            if (ImGui.Button("地图标点位置"))
                TargetPos = MapUtils.FlagToPoint(NavmeshManager.Query!) ?? default;
            
            ImGui.AlignTextToFramePadding();
            ImGui.Text("移动:");
            
            ImGui.SameLine();
            if (ImGui.Button("步行"))
                AsyncMove.MoveTo(TargetPos, false);

            ImGui.SameLine();
            if (ImGui.Button("飞行"))
                AsyncMove.MoveTo(TargetPos, true);
            
            ImGui.SameLine();
            ImGui.TextDisabled("|");

            ImGui.SameLine();
            if (ImGui.Button("停止算路"))
                AsyncMove.CancelPathfinding();
            
            ImGui.SameLine();
            if (ImGui.Button("停止寻路"))
                FollowPath.Stop();
        }
        
        ImGui.NewLine();
        
        ImGui.Text("临时设置");

        using (ImRaii.PushIndent())
        {
            var isMovementAllowed = FollowPath.MovementAllowed;
            if (ImGui.Checkbox("允许导航过程中手动干预移动", ref isMovementAllowed))
                FollowPath.MovementAllowed = isMovementAllowed;
            ImGui.Checkbox("使用光线投射算法",      ref NavmeshManager.UseRaycasts);
            ImGui.Checkbox("使用拉绳算法",        ref NavmeshManager.UseStringPulling);
        }

        ImGui.NewLine();

        DrawPosition("玩家", playerPos);
        DrawPosition("目标", TargetPos);
        DrawPosition("标点", MapUtils.FlagToPoint(NavmeshManager.Query!)       ?? default);
        DrawPosition("地面", NavmeshManager.Query!.FindPointOnFloor(playerPos) ?? default);

        DrawNavmesh ??= 
            new(NavmeshManager.Navmesh!.Mesh, NavmeshManager.Query.MeshQuery, NavmeshManager.Query.LastPath, UITree, DebugDrawer);
        DrawNavmesh.Draw();
        
        if (NavmeshManager.Navmesh!.Volume != null)
        {
            DrawVoxelMap ??= new(NavmeshManager.Navmesh.Volume, NavmeshManager.Query.VolumeQuery, UITree, DebugDrawer);
            DrawVoxelMap.Draw();
        }
    }

    private void DrawPosition(string tag, Vector3 position)
    {
        NavmeshManager.Navmesh!.Mesh.CalcTileLoc(position.SystemToRecast(), out var tileX, out var tileZ);
        UITree.LeafNode($"{tag} 位置: {position:f3}, 格: {tileX}x{tileZ}, poly: {NavmeshManager.Query!.FindNearestMeshPoly(position):X}");
        
        var voxel = NavmeshManager.Query.FindNearestVolumeVoxel(position);
        if (UITree.LeafNode($"{tag} 体素: {voxel:X}###{tag}voxel").SelectedOrHovered && voxel != VoxelMap.InvalidVoxel)
            DrawVoxelMap?.VisualizeVoxel(voxel);
    }

    private void ExportBitmap(Navmesh navmesh, NavmeshQuery query, Vector3 startingPos) => 
        NavmeshManager.BuildBitmap(startingPos, "D:\\navmesh.bmp", 0.5f);

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        DrawNavmesh?.Dispose();
        DrawNavmesh = null;
        
        DrawVoxelMap?.Dispose();
        DrawVoxelMap = null;
    }
}
