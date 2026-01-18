using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using Newtonsoft.Json.Linq;
using System;
using System.IO;

namespace Navmesh;

public class Config
{
    private const int _version = 1;

    public bool AutoLoadNavmesh = true;
    public bool EnableDTR = true;
    public bool ShowQueryStatusInDTR = true;
    public bool AlignCameraToMovement;
    public float AlignCameraHeight = -15;
    public float CameraLookAheadDistance = 50f;
    public bool CameraSmoothingEnabled = true;
    public float CameraSmoothTimeH = 1.5f;
    public float CameraSmoothTimeV = 1.5f;
    public float CameraTurnSpeedH = 240f;
    public float CameraTurnSpeedV = 240f;
    public bool CameraPitchFollowEnabled = true;
    public float CameraPitchFollowStrength = 0.75f;
    public float CameraPitchMaxOffsetDeg = 30f;
    public float CameraPitchDeadzoneDeg = 2f;
    public bool ShowWaypoints;
    public bool ForceShowGameCollision;
    public bool CancelMoveOnUserInput;
    public bool StopOnStuck = false;
    public float StuckTolerance = 0.05f;
    public int StuckTimeoutMs = 500;
    public bool RetryOnStuck = true;
    public float RandomnessMultiplier = 1f;
    public int BuildMaxCores = 1;

    private static readonly int realMaxCores = Environment.ProcessorCount;

    public event Action? Modified;

    public void NotifyModified() => Modified?.Invoke();

    public void Draw()
    {
        if (ImGui.Checkbox("切换区域时自动加载导航数据", ref AutoLoadNavmesh))
            NotifyModified();
        if (ImGui.Checkbox("启用服务器信息栏", ref EnableDTR))
            NotifyModified();
        if (ImGui.Checkbox("在服务器信息栏中显示详细查询状态", ref ShowQueryStatusInDTR))
            NotifyModified();
        if (ImGui.Checkbox("镜头跟随移动方向", ref AlignCameraToMovement))
            NotifyModified();
        using (ImRaii.Disabled(!AlignCameraToMovement))
        {
            ImGui.SetNextItemWidth(200);
            if (ImGui.SliderFloat("相机高度 (角度)", ref AlignCameraHeight, -75, 75))
                NotifyModified();

            ImGui.SetNextItemWidth(200);
            if (ImGui.SliderFloat("镜头提前量 (米)", ref CameraLookAheadDistance, 0f, 30f, "%.1f"))
                NotifyModified();

            if (ImGui.Checkbox("平滑镜头运动", ref CameraSmoothingEnabled))
                NotifyModified();

            using (ImRaii.Disabled(!CameraSmoothingEnabled))
            {
                ImGui.SetNextItemWidth(200);
                if (ImGui.SliderFloat("水平平滑时间 (秒)", ref CameraSmoothTimeH, 0.02f, 1.5f, "%.2f"))
                    NotifyModified();

                ImGui.SetNextItemWidth(200);
                if (ImGui.SliderFloat("俯仰平滑时间 (秒)", ref CameraSmoothTimeV, 0.02f, 1.5f, "%.2f"))
                    NotifyModified();
            }

            ImGui.SetNextItemWidth(200);
            if (ImGui.SliderFloat("水平转向速度 (度/秒)", ref CameraTurnSpeedH, 30f, 720f, "%.0f"))
                NotifyModified();

            ImGui.SetNextItemWidth(200);
            if (ImGui.SliderFloat("俯仰转向速度 (度/秒)", ref CameraTurnSpeedV, 30f, 720f, "%.0f"))
                NotifyModified();

            if (ImGui.Checkbox("飞行时镜头俯仰跟随爬升/俯冲", ref CameraPitchFollowEnabled))
                NotifyModified();

            using (ImRaii.Disabled(!CameraPitchFollowEnabled))
            {
                ImGui.SetNextItemWidth(200);
                if (ImGui.SliderFloat("俯仰跟随强度", ref CameraPitchFollowStrength, 0f, 1.5f, "%.2f"))
                    NotifyModified();

                ImGui.SetNextItemWidth(200);
                if (ImGui.SliderFloat("俯仰最大偏移 (角度)", ref CameraPitchMaxOffsetDeg, 0f, 60f, "%.0f"))
                    NotifyModified();

                ImGui.SetNextItemWidth(200);
                if (ImGui.SliderFloat("俯仰死区 (角度)", ref CameraPitchDeadzoneDeg, 0f, 10f, "%.1f"))
                    NotifyModified();
            }
        }
        if (ImGui.Checkbox("显示路径点", ref ShowWaypoints))
            NotifyModified();
        if (ImGui.Checkbox("始终显示游戏碰撞体积", ref ForceShowGameCollision))
            NotifyModified();
        if (ImGui.Checkbox("玩家移动输入时取消当前路径", ref CancelMoveOnUserInput))
            NotifyModified();
        if (ImGui.Checkbox("卡住时停止寻路", ref StopOnStuck))
            NotifyModified();

        ImGui.SetNextItemWidth(200);
        if (ImGui.SliderInt("路网构建最大使用核心数", ref BuildMaxCores, -8, realMaxCores))
            NotifyModified();
        ImGuiComponents.HelpMarker("0 为使用全部核心\n正数为使用指定数量核心\n负数为保留指定数量核心");

        if (StopOnStuck)
        {
            if (ImGui.SliderFloat("卡寻路阈值 (米/秒)", ref StuckTolerance, 0.5f, 3f))
                NotifyModified();
            if (ImGui.IsItemHovered())
                ImGui.SetTooltip("每帧最小移动距离 低于此值视为卡寻路");

            if (ImGui.SliderInt("卡寻路超时时间 (毫秒)", ref StuckTimeoutMs, 100, 10_000))
                NotifyModified();
            if (ImGui.IsItemHovered())
                ImGui.SetTooltip("低于卡寻路阈值持续多久后停止");

            if (ImGui.Checkbox("停止后重新寻路", ref RetryOnStuck))
                NotifyModified();
            if (ImGui.IsItemHovered())
                ImGui.SetTooltip("启用后 被视为卡寻路时会尝试重新寻路");
        }

        ImGui.SetNextItemWidth(200);
        if (ImGui.SliderFloat("随机性", ref RandomnessMultiplier, 0f, 1.0f, "%.2f"))
            NotifyModified();
    }

    public void Save(FileInfo file)
    {
        try
        {
            JObject jContents = new()
            {
                { "Version", _version },
                { "Payload", JObject.FromObject(this) }
            };
            File.WriteAllText(file.FullName, jContents.ToString());
        }
        catch (Exception e)
        {
            Service.Log.Error($"Failed to save config to {file.FullName}: {e}");
        }
    }

    public void Load(FileInfo file)
    {
        try
        {
            var contents = File.ReadAllText(file.FullName);
            var json = JObject.Parse(contents);
            var version = (int?)json["Version"] ?? 0;
            if (json["Payload"] is JObject payload)
            {
                payload = ConvertConfig(payload, version);
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
            Service.Log.Error($"Failed to load config from {file.FullName}: {e}");
        }
    }

    private static JObject ConvertConfig(JObject payload, int version)
    {
        return payload;
    }
}
