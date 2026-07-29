using Dalamud.Bindings.ImGui;
using Dalamud.Configuration;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;

namespace vnavmesh.Internal;

public class PluginConfig : IPluginConfiguration
{
    public int Version { get; set; } = 1;

    public bool  AutoLoadNavmesh      = true;
    public bool  EnableDTR            = true;
    public bool  ShowQueryStatusInDTR = true;
    public bool  AlignCameraToMovement;
    public float AlignCameraHeight = -15;
    public bool  ShowWaypoints;
    public bool  ForceShowGameCollision;
    public bool  RenderWhenGameUiHidden;
    public bool  CancelMoveOnUserInput;
    public bool  StopOnStuck;
    public float UnstuckDetectionSeconds = 2f;
    public float UnstuckCooldownSeconds  = 3f;
    public float PathTolerance           = 0.05f;

    private const float              CONFIG_VALUE_WIDTH    = 260f;
    private const ImGuiTreeNodeFlags DEFAULT_SECTION_FLAGS = ImGuiTreeNodeFlags.DefaultOpen;

    public void Save() =>
        Service.PluginInterface.SavePluginConfig(this);

    public void Draw()
    {
        DrawSection("导航与显示", "控制导航数据加载、可视化以及 DTR 信息栏展示。", DrawNavigationAndDisplaySection);
        DrawSection("移动与镜头", "调整角色跟随行为、镜头联动与寻路细节。",       DrawMovementAndCameraSection);
        DrawSection("卡住处理",  "配置角色疑似卡住时的判定条件与恢复策略。",      DrawStuckHandlingSection);
    }

    private void DrawNavigationAndDisplaySection()
    {
        DrawCheckbox("切换区域时自动加载导航数据", ref AutoLoadNavmesh,        "进入新区域后自动尝试载入对应的导航数据。");
        DrawCheckbox("显示路径点",         ref ShowWaypoints,          "在世界中绘制当前路径的关键点。");
        DrawCheckbox("始终显示游戏碰撞体积",    ref ForceShowGameCollision, "用于排查地形碰撞与导航结果之间的差异。");
        DrawCheckbox("游戏隐藏界面时仍然渲染",   ref RenderWhenGameUiHidden, "即使使用游戏内隐藏界面功能，插件的调试渲染与窗口仍继续绘制。");
        DrawCheckbox("启用 DTR 信息栏",    ref EnableDTR,              "在界面上方的信息栏显示插件状态。");

        using var disabled = ImRaii.Disabled(!EnableDTR);
        DrawCheckbox("在 DTR 信息栏中显示详细查询状态", ref ShowQueryStatusInDTR, "显示更细的查询状态变化，便于观察插件当前行为。");
    }

    private void DrawMovementAndCameraSection()
    {
        DrawCheckbox("玩家产生移动输入时取消当前路径", ref CancelMoveOnUserInput, "手动接管角色时立即停止自动移动。");
        DrawSliderFloat("终点容差", ref PathTolerance, 0.01f, 1f, "%.2f", "角色进入目标点附近该范围后，视为已经到达最终目的地。");

        ImGui.Spacing();
        ImGui.TextDisabled("镜头联动");
        DrawCheckbox("镜头跟随移动方向", ref AlignCameraToMovement, "自动将镜头转向角色当前的移动方向。");

        using var disabled = ImRaii.Disabled(!AlignCameraToMovement);
        DrawSliderFloat("相机高度角", ref AlignCameraHeight, -75, 75, "%.0f", "数值越高越偏向俯视，数值越低越偏向平视。");
    }

    private void DrawStuckHandlingSection()
    {
        DrawCheckbox("启用自动防卡", ref StopOnStuck, "检测到角色长时间无法推进路径时，自动尝试跳跃或短时脱困。");

        if (!StopOnStuck)
        {
            ImGui.TextDisabled("启用后可进一步配置卡住判定时长与冷却期。");
            return;
        }

        ImGui.Spacing();
        using var indent = ImRaii.PushIndent();

        DrawSliderFloat("卡住判定时长 (秒)", ref UnstuckDetectionSeconds, 0.5f, 10f, "%.1f", "持续几乎没有实际位移达到该时长后，开始尝试脱困。");
        DrawSliderFloat("脱困冷却期 (秒)",  ref UnstuckCooldownSeconds,  0.5f, 10f, "%.1f", "每次随机位移脱困结束后，暂停重新判定卡住的时间。");
    }

    private static void DrawSection
    (
        string title,
        string description,
        Action drawContent
    )
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

    private void DrawCheckbox
    (
        string   label,
        ref bool value,
        string?  help = null
    )
    {
        if (ImGui.Checkbox(label, ref value))
            Save();

        DrawHelp(help);
    }

    private void DrawSliderFloat
    (
        string    label,
        ref float value,
        float     min,
        float     max,
        string    format,
        string?   help = null
    )
    {
        ImGui.SetNextItemWidth(CONFIG_VALUE_WIDTH);
        if (ImGui.SliderFloat(label, ref value, min, max, format))
            Save();

        DrawHelp(help);
    }

    private void DrawSliderInt
    (
        string  label,
        ref int value,
        int     min,
        int     max,
        string  format,
        string? help = null
    )
    {
        ImGui.SetNextItemWidth(CONFIG_VALUE_WIDTH);
        if (ImGui.SliderInt(label, ref value, min, max, format))
            Save();

        DrawHelp(help);
    }

    private static void DrawHelp
    (
        string? help
    )
    {
        if (string.IsNullOrWhiteSpace(help))
            return;

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
    }
}
