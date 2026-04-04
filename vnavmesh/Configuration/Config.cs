using Dalamud.Bindings.ImGui;
using Dalamud.Configuration;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using vnavmesh.Bootstrap;

namespace vnavmesh.Configuration;

public class Config : IPluginConfiguration
{
    public int Version { get; set; } = 1;

    public bool  AutoLoadNavmesh      = true;
    public bool  EnableDTR            = true;
    public bool  ShowQueryStatusInDTR = true;
    public bool  AlignCameraToMovement;
    public float AlignCameraHeight = -15;
    public bool  ShowWaypoints;
    public bool  ForceShowGameCollision;
    public bool  CancelMoveOnUserInput;
    public bool  StopOnStuck;
    public float StuckTolerance       = 0.05f;
    public int   StuckTimeoutMs       = 500;
    public bool  RetryOnStuck         = true;
    public float PathTolerance        = 0.05f;
    public float RandomnessMultiplier = 1f;
    public int   BuildMaxCores        = 1;

    private const float              CONFIG_VALUE_WIDTH    = 260f;
    private const ImGuiTreeNodeFlags DEFAULT_SECTION_FLAGS = ImGuiTreeNodeFlags.DefaultOpen;

    public void Save() =>
        Service.PluginInterface.SavePluginConfig(this);

    public void Draw()
    {
        DrawSection("导航与显示", "控制导航数据加载、可视化以及 DTR 信息栏展示。", DrawNavigationAndDisplaySection);
        DrawSection("移动与镜头", "调整角色跟随行为、镜头联动与寻路细节。",       DrawMovementAndCameraSection);
        DrawSection("卡住处理",  "配置角色疑似卡住时的判定条件与恢复策略。",      DrawStuckHandlingSection);
        DrawSection("性能与构建", "控制导航网格构建时的资源占用。",           DrawPerformanceSection);
    }

    private void DrawNavigationAndDisplaySection()
    {
        DrawCheckbox("切换区域时自动加载导航数据", ref AutoLoadNavmesh,        "进入新区域后自动尝试载入对应的导航数据。");
        DrawCheckbox("显示路径点",         ref ShowWaypoints,          "在世界中绘制当前路径的关键点。");
        DrawCheckbox("始终显示游戏碰撞体积",    ref ForceShowGameCollision, "用于排查地形碰撞与导航结果之间的差异。");
        DrawCheckbox("启用 DTR 信息栏",    ref EnableDTR,              "在界面上方的信息栏显示插件状态。");

        using var disabled = ImRaii.Disabled(!EnableDTR);
        DrawCheckbox("在 DTR 信息栏中显示详细查询状态", ref ShowQueryStatusInDTR, "显示更细的查询状态变化，便于观察插件当前行为。");
    }

    private void DrawMovementAndCameraSection()
    {
        DrawCheckbox("玩家产生移动输入时取消当前路径", ref CancelMoveOnUserInput, "手动接管角色时立即停止自动移动。");
        DrawSliderFloat("终点容差",   ref PathTolerance,        0.01f, 1f, "%.2f", "角色进入目标点附近该范围后，视为已经到达最终目的地。");
        DrawSliderFloat("寻路随机性",  ref RandomnessMultiplier, 0f,    1f, "%.2f", "数值越高，路径越分散；数值越低，路径越稳定。");

        ImGui.Spacing();
        ImGui.TextDisabled("镜头联动");
        DrawCheckbox("镜头跟随移动方向", ref AlignCameraToMovement, "自动将镜头转向角色当前的移动方向。");

        using var disabled = ImRaii.Disabled(!AlignCameraToMovement);
        DrawSliderFloat("相机高度角", ref AlignCameraHeight, -75, 75, "%.0f", "数值越高越偏向俯视，数值越低越偏向平视。");
    }

    private void DrawStuckHandlingSection()
    {
        DrawCheckbox("卡住时停止寻路", ref StopOnStuck, "检测到角色长时间几乎不移动时，停止当前寻路。");

        if (!StopOnStuck)
        {
            ImGui.TextDisabled("启用后可进一步配置卡住判定与自动重试逻辑。");
            return;
        }

        ImGui.Spacing();
        using var indent = ImRaii.PushIndent();

        DrawSliderFloat("卡住判定阈值 (米/秒)", ref StuckTolerance, 0.5f, 3f, "%.2f", "每帧最小移动距离低于此值时，视为没有明显移动。");
        DrawSliderInt("卡住超时时间 (毫秒)", ref StuckTimeoutMs, 100, 10_000, "%d", "持续低于阈值达到该时长后，判定为卡住。");
        DrawCheckbox("停止后自动重新寻路", ref RetryOnStuck, "启用后会在卡住停止后尝试重新规划路径。");
    }

    private void DrawPerformanceSection()
    {
        DrawSliderInt("路网构建最大使用核心数", ref BuildMaxCores, -8, Environment.ProcessorCount, "%d", "0 表示使用全部核心，正数表示指定数量，负数表示保留对应数量的核心。");
        ImGui.TextDisabled($"当前可用逻辑核心数： {Environment.ProcessorCount}");
    }

    private static void DrawSection(string title, string description, Action drawContent)
    {
        if (!ImGui.CollapsingHeader(title, DEFAULT_SECTION_FLAGS))
            return;

        using var id = ImRaii.PushId(title);

        ImGui.TextDisabled(description);

        ImGui.Spacing();

        using var indent = ImRaii.PushIndent();
        drawContent();

        ImGui.Spacing();
    }

    private void DrawCheckbox(string label, ref bool value, string? help = null)
    {
        if (ImGui.Checkbox(label, ref value))
            Save();

        DrawHelp(help);
    }

    private void DrawSliderFloat(string label, ref float value, float min, float max, string format, string? help = null)
    {
        ImGui.SetNextItemWidth(CONFIG_VALUE_WIDTH);
        if (ImGui.SliderFloat(label, ref value, min, max, format))
            Save();

        DrawHelp(help);
    }

    private void DrawSliderInt(string label, ref int value, int min, int max, string format, string? help = null)
    {
        ImGui.SetNextItemWidth(CONFIG_VALUE_WIDTH);
        if (ImGui.SliderInt(label, ref value, min, max, format))
            Save();

        DrawHelp(help);
    }

    private static void DrawHelp(string? help)
    {
        if (string.IsNullOrWhiteSpace(help))
            return;

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
    }
}
