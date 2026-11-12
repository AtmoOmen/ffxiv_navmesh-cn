using Dalamud.Bindings.ImGui;
using Dalamud.Configuration;
using Dalamud.Interface.Utility.Raii;
using Lumina.Data;

namespace vnavmesh.Internal;

public class PluginConfig : IPluginConfiguration
{
    public int Version { get; set; } = 1;

    public Language UILanguage = Service.ClientLanguage;

    public bool  AutoLoadNavmesh      = true;
    public int   CacheRetentionDays   = 7;
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
        var text = new Localization(UILanguage);

        ImGui.SetNextItemWidth(CONFIG_VALUE_WIDTH);
        var languageIndex = text.LanguageIndex;

        if (ImGui.Combo($"{text.LanguageLabel}###UILanguage", ref languageIndex, Localization.LanguageNames, Localization.LanguageNames.Length))
        {
            UILanguage = Localization.Languages[languageIndex];
            Save();
            text = new Localization(UILanguage);
        }

        ImGui.Spacing();
        DrawSection(text.NavigationAndDisplay, DrawNavigationAndDisplaySection);
        DrawSection(text.MovementAndCamera,    DrawMovementAndCameraSection);
    }

    private void DrawNavigationAndDisplaySection()
    {
        var text = new Localization(UILanguage);

        DrawCheckbox(text.AutoLoadNavmesh, ref AutoLoadNavmesh, text.AutoLoadNavmeshHelp);
        DrawSliderInt(text.CacheRetentionDays, ref CacheRetentionDays, 0, 90, "%d", text.CacheRetentionDaysHelp);
        DrawCheckbox(text.ShowWaypoints,          ref ShowWaypoints,          text.ShowWaypointsHelp);
        DrawCheckbox(text.ForceShowGameCollision, ref ForceShowGameCollision, text.ForceShowGameCollisionHelp);
        DrawCheckbox(text.RenderWhenGameUIHidden, ref RenderWhenGameUiHidden, text.RenderWhenGameUIHiddenHelp);
        DrawCheckbox(text.EnableDTR,              ref EnableDTR,              text.EnableDTRHelp);

        using var disabled = ImRaii.Disabled(!EnableDTR);
        DrawCheckbox(text.ShowQueryStatusInDTR, ref ShowQueryStatusInDTR, text.ShowQueryStatusInDTRHelp);
    }

    private void DrawMovementAndCameraSection()
    {
        var text = new Localization(UILanguage);

        DrawCheckbox(text.CancelMoveOnUserInput, ref CancelMoveOnUserInput, text.CancelMoveOnUserInputHelp);
        DrawSliderFloat(text.PathTolerance, ref PathTolerance, 0.01f, 1f, "%.2f", text.PathToleranceHelp);

        ImGui.Spacing();
        ImGui.TextDisabled(text.CameraTracking);
        DrawCheckbox(text.AlignCameraToMovement, ref AlignCameraToMovement, text.AlignCameraToMovementHelp);

        using (ImRaii.Disabled(!AlignCameraToMovement))
            DrawSliderFloat(text.AlignCameraHeight, ref AlignCameraHeight, -75, 75, "%.0f", text.AlignCameraHeightHelp);
        
        DrawCheckbox(text.StopOnStuck, ref StopOnStuck, text.StopOnStuckHelp);

        if (!StopOnStuck)
        {
            ImGui.TextDisabled(text.StuckHandlingDisabled);
            return;
        }

        ImGui.Spacing();
        using var indent = ImRaii.PushIndent();

        DrawSliderFloat(text.UnstuckDetectionSeconds, ref UnstuckDetectionSeconds, 0.5f, 10f, "%.1f", text.UnstuckDetectionSecondsHelp);
        DrawSliderFloat(text.UnstuckCooldownSeconds,  ref UnstuckCooldownSeconds,  0.5f, 10f, "%.1f", text.UnstuckCooldownSecondsHelp);
    }

    private static void DrawSection
    (
        string title,
        Action drawContent
    )
    {
        if (!ImGui.CollapsingHeader(title, DEFAULT_SECTION_FLAGS))
            return;

        using var id = ImRaii.PushId(title);
        
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

        using (ImRaii.PushColor(ImGuiCol.Text, ImGui.GetColorU32(ImGuiCol.TextDisabled)))
            ImGui.TextWrapped(help);
    }
}
