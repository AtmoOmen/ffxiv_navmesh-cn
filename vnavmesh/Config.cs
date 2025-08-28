using Newtonsoft.Json.Linq;
using System;
using System.IO;
using Dalamud.Interface.Utility;
using ImGuiNET;

namespace Navmesh;

public class Config
{
    private const int version = 1;

    public bool AutoLoadNavmesh = true;
    public bool EnableDTR       = true;
    public bool AlignCameraToMovement;
    public bool ShowWaypoints;
    public bool ForceShowGameCollision;
    public bool CancelMoveOnUserInput;
    
    public float VoxelPathfindRandomFactor = 0.5f;
    
    // 体素路径查找性能优化参数
    public float VoxelPathfindMaxStepsBaseFactor       = 2.0f;    // 基础步数倍数因子
    public float VoxelPathfindEarlyTerminationDistance = 2.0f;    // 早期终止距离阈值
    public int   VoxelPathfindMinSteps                 = 5000;    // 最小步数保证
    public float VoxelPathfindMaxStepsMultiplier       = 1000.0f; // 距离步数乘数
    
    // 地面Mesh寻路自动重算配置
    public bool  EnableAutoRecalculateGroundPath;  // 是否启用自动重算
    public float AutoRecalculateIntervalMs = 500f; // 自动重算时间间隔(ms)

    public event Action? Modified;

    public void NotifyModified() => Modified?.Invoke();

    public void Draw()
    {
        ImGui.Spacing();
        
        ImGui.Text("一般");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("切换区域时, 自动加载/构建区域导航数据", ref AutoLoadNavmesh))
            NotifyModified();
        
        if (ImGui.Checkbox("在服务器状态栏显示导航信息", ref EnableDTR))
            NotifyModified();

        ImGui.NewLine();
        
        ImGui.Text("操控");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("将镜头面向对齐前进方向", ref AlignCameraToMovement))
            NotifyModified();
        
        if (ImGui.Checkbox("当尝试操控游戏角色时, 自动取消寻路任务", ref CancelMoveOnUserInput))
            NotifyModified();
        
        ImGui.NewLine();
        
        ImGui.Text("显示");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("显示导航路径点", ref ShowWaypoints))
            NotifyModified();
        
        if (ImGui.Checkbox("强制显示游戏内碰撞体", ref ForceShowGameCollision))
            NotifyModified();
        
        ImGui.NewLine();
        
        ImGui.Text("体素导航 (飞行)");
        
        ImGui.Separator();
        ImGui.Spacing();

        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("路线随机性", ref VoxelPathfindRandomFactor, 0.1f, 1f, "%.1f"))
            NotifyModified();
        
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("最大步数基础因子", ref VoxelPathfindMaxStepsBaseFactor, 1.0f, 5.0f, "%.1f"))
            NotifyModified();
            
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("早期终止距离", ref VoxelPathfindEarlyTerminationDistance, 0.5f, 10.0f, "%.1f"))
            NotifyModified();
            
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderInt("最小保证步数", ref VoxelPathfindMinSteps, 1000, 20000, "%d"))
            NotifyModified();
            
        ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
        if (ImGui.SliderFloat("距离步数乘数", ref VoxelPathfindMaxStepsMultiplier, 100.0f, 5000.0f, "%.0f"))
            NotifyModified();

        ImGui.NewLine();
        
        ImGui.Text("路网导航 (地面)");
        
        ImGui.Separator();
        ImGui.Spacing();
        
        if (ImGui.Checkbox("启用自动重算", ref EnableAutoRecalculateGroundPath))
            NotifyModified();
        
        if (EnableAutoRecalculateGroundPath)
        {
            ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
            if (ImGui.SliderFloat("重算间隔 (毫秒)", ref AutoRecalculateIntervalMs, 10f, 2000f, "%.0f"))
                NotifyModified();
            if (ImGui.IsItemHovered())
                ImGui.SetTooltip("地面寻路过程中, 每隔固定时间发送一次重算请求, 这会导致已有的卡寻路检测完全失效并和某些插件的兼容性下降, 但在大部分时候会有相对更佳的表现");
        }
    }

    public void Save(FileInfo file)
    {
        try
        {
            JObject jContents = new()
            {
                { "Version", version },
                { "Payload", JObject.FromObject(this) }
            };
            File.WriteAllText(file.FullName, jContents.ToString());
        }
        catch (Exception e)
        {
            Service.Log.Error($"保存配置文件至 {file.FullName} 时失败: {e}");
        }
    }

    public void Load(FileInfo file)
    {
        try
        {
            var contents       = File.ReadAllText(file.FullName);
            var json           = JObject.Parse(contents);
            var currentVersion = (int?)json["Version"] ?? 0;
            if (json["Payload"] is JObject payload)
            {
                payload = ConvertConfig(payload, currentVersion);
                var thisType = GetType();
                foreach (var (f, data) in payload)
                {
                    var thisField = thisType.GetField(f);
                    if (thisField != null)
                    {
                        var value = data?.ToObject(thisField.FieldType);
                        if (value != null)
                        {
                            thisField.SetValue(this, value);
                        }
                    }
                }
            }
        }
        catch (Exception e)
        {
            Service.Log.Error($"无法从 {file.FullName} 加载配置内容: {e}");
        }
    }

    private static JObject ConvertConfig(JObject payload, int toVersion) => payload;
}
