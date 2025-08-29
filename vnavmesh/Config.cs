using Newtonsoft.Json.Linq;
using System;
using System.IO;
using Dalamud.Interface.Utility;
using Dalamud.Interface.Utility.Raii;

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
    public int   MeshFilterType = 1;
    
    public int   PullStringType;
    
    public float PullStringDefaultImprovedBasedSafeMargin = 1f;
    public float PullStringDefaultImprovedSafeMargin      = 2f;

    public event Action? Modified;

    public void NotifyModified() => Modified?.Invoke();

    public void Draw()
    {
        ImGui.Spacing();

        using var tab = ImRaii.TabBar("###Config");
        if (!tab) return;

        using (var generalTab = ImRaii.TabItem("一般"))
        {
            if (generalTab)
            {
                ImGui.Spacing();
                
                if (ImGui.Checkbox("切换区域时, 自动加载/构建区域导航数据", ref AutoLoadNavmesh))
                    NotifyModified();
        
                if (ImGui.Checkbox("在服务器状态栏显示导航信息", ref EnableDTR))
                    NotifyModified();
            }
        }
        
        using (var controlTab = ImRaii.TabItem("控制"))
        {
            if (controlTab)
            {
                ImGui.Spacing();
                
                if (ImGui.Checkbox("寻路时, 将镜头面向对齐前进方向", ref AlignCameraToMovement))
                    NotifyModified();
                
                if (ImGui.Checkbox("寻路时, 当尝试操控角色时, 自动取消当前寻路", ref CancelMoveOnUserInput))
                    NotifyModified();
            }
        }
        
        using (var displayTab = ImRaii.TabItem("显示"))
        {
            if (displayTab)
            {
                ImGui.Spacing();
                
                if (ImGui.Checkbox("寻路过程中, 显示导航路径点", ref ShowWaypoints))
                    NotifyModified();
                
                if (ImGui.Checkbox("强制显示游戏内碰撞体", ref ForceShowGameCollision))
                    NotifyModified();
            }
        }
        
        using (var voxelTab = ImRaii.TabItem("体素导航 (飞行)"))
        {
            if (voxelTab)
            {
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
            }
        }
        
        using (var meshTab = ImRaii.TabItem("路网导航 (地面)"))
        {
            if (meshTab)
            {
                ImGui.Spacing();
                
                if (ImGui.Checkbox("自动重算", ref EnableAutoRecalculateGroundPath))
                    NotifyModified();
                ImGuiOm.TooltipHover("每隔固定时间发送一次重算请求, 这会导致已有的卡寻路检测完全失效并和某些插件的兼容性下降, 但在大部分时候会有相对更佳的表现, 如果你平时寻路时间就已经较长, 请勿开启本项");
                
                if (EnableAutoRecalculateGroundPath)
                {
                    ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                    if (ImGui.SliderFloat("重算间隔 (毫秒)", ref AutoRecalculateIntervalMs, 10f, 10_000f, "%.0f"))
                        NotifyModified();
                    ImGuiOm.TooltipHover("按住 Ctrl 单击可以直接填写数值");
                }
                
                ImGui.NewLine();
                
                ImGui.Text("计算类型");

                using (ImRaii.PushId("MeshCalculationType"))
                using (ImRaii.PushIndent())
                {
                    if (ImGui.RadioButton("默认", ref MeshFilterType, 0))
                        NotifyModified();

                    ImGui.SameLine();
                    if (ImGui.RadioButton("障碍物临近惩罚", ref MeshFilterType, 1))
                        NotifyModified();
                    ImGuiOm.TooltipHover("为每个非障碍物的网格计算它到最近障碍物的距离, 形成危险系数, 在计算过程中除了加上基础移动成本，还要乘以或加上该网格的危险系数, 使最终路径更加倾向远离障碍物, 而非贴边");
                }
                
                ImGui.NewLine();
                
                ImGui.Text("拉绳算法类型 (后处理)");

                using (ImRaii.PushId("MeshPullStringType"))
                using (ImRaii.PushIndent())
                {
                    if (ImGui.RadioButton("默认", ref PullStringType, 0))
                        NotifyModified();
                    ImGuiOm.TooltipHover("路径拉直效果最好, 但会让路径更加倾向紧贴障碍物");

                    ImGui.SameLine();
                    if (ImGui.RadioButton("保持障碍距离", ref PullStringType, 1))
                        NotifyModified();
                    ImGuiOm.TooltipHover("路径拉直效果较差, 水平距离抖动可能较多, 但会尝试让路径保持与障碍物之间的距离, 建议同时开启自动重算功能");
                    
                    ImGui.SameLine();
                    if (ImGui.RadioButton("默认 (简化 / 自适应)", ref PullStringType, 2))
                        NotifyModified();
                    ImGuiOm.TooltipHover("在默认基础上增加的障碍物检测, 拉直效果较好, 自适应检测安全通道宽度");
                    
                    if (ImGui.RadioButton("默认 (简化 / 手动)", ref PullStringType, 3))
                        NotifyModified();
                    ImGuiOm.TooltipHover("在默认基础上增加的障碍物检测, 拉直效果较好, 需要自行指定安全通道宽度");

                    switch (PullStringType)
                    {
                        case 2:
                            ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                            if (ImGui.SliderFloat("基础安全宽度", ref PullStringDefaultImprovedBasedSafeMargin, 0.1f, 10f, "%.2f"))
                                NotifyModified();
                            ImGuiOm.TooltipHover("按住 Ctrl 单击可以直接填写数值");
                            break;
                        case 3:
                            ImGui.SetNextItemWidth(200f * ImGuiHelpers.GlobalScale);
                            if (ImGui.SliderFloat("安全宽度", ref PullStringDefaultImprovedSafeMargin, 0.1f, 10f, "%.2f"))
                                NotifyModified();
                            ImGuiOm.TooltipHover("按住 Ctrl 单击可以直接填写数值");
                            break;
                    }
                }
            }
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
