using System.Globalization;
using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Editor;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.UI.Editor;

internal static class CustomizationEditorWidgets
{
    public static string FormatEnumDisplayName<T>(T value) where T : struct, Enum =>
        GetEnumDisplayName(value);

    public static string FormatPreviewStateDisplayName(CustomizationPreviewBuilder.State value) =>
        value switch
        {
            CustomizationPreviewBuilder.State.NotBuilt   => "未构建",
            CustomizationPreviewBuilder.State.InProgress => "构建中",
            CustomizationPreviewBuilder.State.Failed     => "失败",
            CustomizationPreviewBuilder.State.Ready      => "就绪",
            _                                            => value.ToString()
        };

    private static string GetFlagsDisplayName<T>(T value) where T : struct, Enum
    {
        var raw = Convert.ToUInt64(value);
        if (raw == 0)
            return GetEnumDisplayName(value);

        var names = new List<string>();

        foreach (var candidate in Enum.GetValues<T>())
        {
            var candidateRaw = Convert.ToUInt64(candidate);
            if (candidateRaw == 0 || (candidateRaw & (candidateRaw - 1)) != 0)
                continue;

            if ((raw & candidateRaw) == candidateRaw)
                names.Add(GetEnumDisplayName(candidate));
        }

        return names.Count > 0 ?
                   string.Join(" | ", names) :
                   value.ToString();
    }

    private static string GetEnumDisplayName<T>(T value) where T : struct, Enum =>
        value switch
        {
            DraftSceneInstancePatchKind.ClearInstances    => "清空实例",
            DraftSceneInstancePatchKind.RemoveInstance    => "删除实例",
            DraftSceneInstancePatchKind.Transform         => "变换",
            DraftSceneInstancePatchKind.SetFlags          => "设置标记",
            DraftScenePartPatchKind.Vertex                => "顶点",
            DraftScenePartPatchKind.PrimitiveFlags        => "三角标记",
            DraftScenePartPatchKind.PrimitiveEdit         => "三角编辑",
            DraftSceneColliderInsertionKind.Aabb          => "AABB",
            DraftSceneColliderInsertionKind.Cylinder      => "圆柱",
            DraftMeshLinkKind.Points                      => "两点直连",
            DraftMeshLinkKind.Shortcut                    => "捷径",
            DraftMeshLinkKind.ClientPath                  => "客户端路径",
            SceneExtractor.PrimitiveFlags.None            => "无",
            SceneExtractor.PrimitiveFlags.ForceUnwalkable => "强制不可行走",
            SceneExtractor.PrimitiveFlags.FlyThrough      => "可穿飞",
            SceneExtractor.PrimitiveFlags.Unlandable      => "不可降落",
            SceneExtractor.PrimitiveFlags.ForceWalkable   => "强制可行走",
            SceneExtractor.PrimitiveFlags.Fishable        => "可钓鱼",
            NavmeshArea.Null                              => "空",
            NavmeshArea.Ground                            => "地面",
            NavmeshArea.GeneratedClimbDown                => "自动跳落",
            NavmeshArea.GeneratedEdgeJump                 => "自动边缘跳跃",
            NavmeshArea.ManualOffMesh                     => "手动离网连接",
            NavmeshArea.Shortcut                          => "捷径",
            NavmeshArea.Teleport                          => "传送",
            NavmeshArea.ClientPath                        => "客户端路径",
            NavmeshPolyFlags.None                         => "无",
            NavmeshPolyFlags.Ground                       => "地面",
            NavmeshPolyFlags.GeneratedClimbDown           => "自动跳落",
            NavmeshPolyFlags.GeneratedEdgeJump            => "自动边缘跳跃",
            NavmeshPolyFlags.ManualOffMesh                => "手动离网连接",
            NavmeshPolyFlags.Shortcut                     => "捷径",
            NavmeshPolyFlags.Teleport                     => "传送",
            NavmeshPolyFlags.ClientPath                   => "客户端路径",
            NavmeshPolyFlags.Unreachable                  => "不可达",
            NavmeshPolyFlags.AllTraversable               => "全部可通行",
            NavmeshOffMeshKind.GeneratedClimbDown         => "自动跳落",
            NavmeshOffMeshKind.GeneratedEdgeJump          => "自动边缘跳跃",
            NavmeshOffMeshKind.ManualOffMesh              => "手动离网连接",
            NavmeshOffMeshKind.Shortcut                   => "捷径",
            NavmeshOffMeshKind.Teleport                   => "传送",
            NavmeshOffMeshKind.ClientPath                 => "客户端路径",
            _                                             => value.ToString()
        };

    public static bool DrawBool(string label, ref bool value) =>
        ImGui.Checkbox(label, ref value);

    public static bool DrawString(string label, ref string value)
    {
        var next = value;
        if (!ImGui.InputText(label, ref next))
            return false;

        value = next;
        return true;
    }

    public static bool DrawInt(string label, ref int value) =>
        ImGui.InputInt(label, ref value);

    public static bool DrawUInt64(string label, ref ulong value)
    {
        var text = value.ToString("X", CultureInfo.InvariantCulture);
        if (!ImGui.InputText(label, ref text))
            return false;

        if (string.IsNullOrWhiteSpace(text))
            return false;

        if (ulong.TryParse
            (
                text.StartsWith
                (
                    "0x",
                    StringComparison.OrdinalIgnoreCase
                ) ?
                    text[2..] :
                    text,
                NumberStyles.HexNumber,
                CultureInfo.InvariantCulture,
                out var parsed
            ))
        {
            value = parsed;
            return true;
        }

        return false;
    }

    public static bool DrawFloat(string label, ref float value, float speed, float min = -10000f, float max = 10000f) =>
        ImGui.DragFloat(label, ref value, speed, min, max, "%.3f");

    public static bool DrawVector3(string label, ref Vector3 value)
    {
        var changed = false;

        if (ImGui.TreeNodeEx(label, ImGuiTreeNodeFlags.DefaultOpen))
        {
            changed |= ImGui.DragFloat("X 轴", ref value.X, 0.1f, -100000, 100000, "%.3f");
            changed |= ImGui.DragFloat("Y 轴", ref value.Y, 0.1f, -100000, 100000, "%.3f");
            changed |= ImGui.DragFloat("Z 轴", ref value.Z, 0.1f, -100000, 100000, "%.3f");

            ImGui.TreePop();
        }

        return changed;
    }

    public static bool DrawBoundsEditor(string label, ref Vector3 min, ref Vector3 max)
    {
        var changed = false;
        if (!ImGui.TreeNodeEx(label, ImGuiTreeNodeFlags.DefaultOpen))
            return false;

        var center = (min + max) * 0.5f;
        var size   = max - min;
        changed |= DrawVector3("中心", ref center);
        changed |= DrawVector3("尺寸", ref size);

        if (changed)
        {
            size = Vector3.Max(size, new(0.01f));
            min  = center - (size * 0.5f);
            max  = center + (size * 0.5f);
        }

        if (ImGui.TreeNodeEx("原始最小值 / 最大值"))
        {
            changed |= DrawVector3("最小值", ref min);
            changed |= DrawVector3("最大值", ref max);
            ImGui.TreePop();
        }

        ImGui.TreePop();
        return changed;
    }

    public static bool DrawMatrix(string label, ref DraftMatrix4x3 matrix)
    {
        var changed = false;
        if (!ImGui.TreeNodeEx(label, ImGuiTreeNodeFlags.DefaultOpen))
            return false;

        var translation = matrix.Translation;
        var scale       = matrix.GetScale();
        changed |= DrawVector3("平移", ref translation);
        changed |= DrawVector3("缩放", ref scale);
        if (changed)
            matrix.SetTranslationScale(translation, scale);

        if (ImGui.TreeNodeEx("原始 Matrix4x3"))
        {
            changed |= DrawVector3("第 0 行", ref matrix.Row0);
            changed |= DrawVector3("第 1 行", ref matrix.Row1);
            changed |= DrawVector3("第 2 行", ref matrix.Row2);
            changed |= DrawVector3("第 3 行", ref matrix.Row3);
            ImGui.TreePop();
        }

        ImGui.TreePop();
        return changed;
    }

    public static bool DrawNullableFloat(string label, ref float? value, float fallback, string help = "")
    {
        var enabled = value.HasValue;

        if (ImGui.Checkbox($"启用##{label}", ref enabled))
        {
            value = enabled ?
                        fallback :
                        null;
            return true;
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (!string.IsNullOrEmpty(help))
            {
                ImGui.SameLine();
                ImGuiComponents.HelpMarker(help);
            }

            return false;
        }

        var current = value!.Value;

        if (ImGui.DragFloat($"##{label}_value", ref current, 0.1f, -100000, 100000, "%.3f"))
        {
            value = current;
            return true;
        }

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
        return false;
    }

    public static bool DrawNullableInt(string label, ref int? value, int fallback, string help = "")
    {
        var enabled = value.HasValue;

        if (ImGui.Checkbox($"启用##{label}", ref enabled))
        {
            value = enabled ?
                        fallback :
                        null;
            return true;
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (!string.IsNullOrEmpty(help))
            {
                ImGui.SameLine();
                ImGuiComponents.HelpMarker(help);
            }

            return false;
        }

        var current = value!.Value;

        if (ImGui.InputInt($"##{label}_value", ref current))
        {
            value = current;
            return true;
        }

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
        return false;
    }

    public static bool DrawNullableBool(string label, ref bool? value, bool fallback, string help = "")
    {
        var enabled = value.HasValue;

        if (ImGui.Checkbox($"启用##{label}", ref enabled))
        {
            value = enabled ?
                        fallback :
                        null;
            return true;
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (!string.IsNullOrEmpty(help))
            {
                ImGui.SameLine();
                ImGuiComponents.HelpMarker(help);
            }

            return false;
        }

        var current = value!.Value;

        if (ImGui.Checkbox($"##{label}_value", ref current))
        {
            value = current;
            return true;
        }

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
        return false;
    }

    public static bool DrawNullableEnum<T>(string label, ref T? value, T? fallback, string help = "") where T : struct, Enum
    {
        var enabled = value.HasValue;

        if (ImGui.Checkbox($"启用##{label}", ref enabled))
        {
            value = enabled ?
                        fallback.GetValueOrDefault() :
                        null;
            return true;
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (!string.IsNullOrEmpty(help))
            {
                ImGui.SameLine();
                ImGuiComponents.HelpMarker(help);
            }

            return false;
        }

        var current = value!.Value;

        if (DrawEnumCombo($"##{label}_value", ref current))
        {
            value = current;
            return true;
        }

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
        return false;
    }

    public static bool DrawNullableIntArray(string label, ref int[]? value, int[] fallback, string help = "")
    {
        var enabled = value != null;

        if (ImGui.Checkbox($"启用##{label}", ref enabled))
        {
            value = enabled ?
                        (int[])fallback.Clone() :
                        null;
            return true;
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (!string.IsNullOrEmpty(help))
            {
                ImGui.SameLine();
                ImGuiComponents.HelpMarker(help);
            }

            return false;
        }

        var current = (int[])value!.Clone();

        switch (current.Length)
        {
            case > 0 when ImGui.InputInt($"##{label}_0", ref current[0]):
                value = current;
                return true;
            case > 1:
            {
                ImGui.SameLine();

                if (ImGui.InputInt($"##{label}_1", ref current[1]))
                {
                    value = current;
                    return true;
                }

                break;
            }
        }

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
        return false;
    }

    public static bool DrawNullableFlags<T>(string label, ref T? value, T? fallback, string help = "") where T : struct, Enum
    {
        var enabled = value.HasValue;

        if (ImGui.Checkbox($"启用##{label}", ref enabled))
        {
            value = enabled ?
                        fallback.GetValueOrDefault() :
                        null;
            return true;
        }

        ImGui.SameLine();
        ImGui.TextUnformatted(label);

        if (!enabled)
        {
            if (!string.IsNullOrEmpty(help))
            {
                ImGui.SameLine();
                ImGuiComponents.HelpMarker(help);
            }

            return false;
        }

        var current = value!.Value;

        if (DrawFlags($"##{label}_value", ref current))
        {
            value = current;
            return true;
        }

        ImGui.SameLine();
        ImGuiComponents.HelpMarker(help);
        return false;
    }

    public static bool DrawEnumCombo<T>(string label, ref T value) where T : struct, Enum
    {
        var       changed = false;
        using var combo   = ImRaii.Combo(label, GetEnumDisplayName(value));
        if (!combo)
            return false;

        foreach (var candidate in Enum.GetValues<T>())
        {
            var isSelected = EqualityComparer<T>.Default.Equals(candidate, value);

            if (ImGui.Selectable(GetEnumDisplayName(candidate), isSelected) && !isSelected)
            {
                value   = candidate;
                changed = true;
            }

            if (isSelected)
                ImGui.SetItemDefaultFocus();
        }

        return changed;
    }

    public static bool DrawEnumCombo(string label, ref int value, string[] options)
    {
        var       changed = false;
        using var combo   = ImRaii.Combo(label, options[Math.Clamp(value, 0, options.Length - 1)]);
        if (!combo)
            return false;

        for (var i = 0; i < options.Length; ++i)
        {
            var isSelected = i == value;

            if (ImGui.Selectable(options[i], isSelected) && !isSelected)
            {
                value   = i;
                changed = true;
            }

            if (i == value)
                ImGui.SetItemDefaultFocus();
        }

        return changed;
    }

    public static bool DrawFlags<T>(string label, ref T value) where T : struct, Enum
    {
        using var combo = ImRaii.Combo(label, GetFlagsDisplayName(value));
        if (!combo)
            return false;

        var changed = false;
        var current = Convert.ToUInt64(value);

        foreach (var candidate in Enum.GetValues<T>())
        {
            var raw = Convert.ToUInt64(candidate);
            if (raw == 0 || (raw & (raw - 1)) != 0)
                continue;

            var enabled = (current & raw) == raw;

            if (ImGui.Checkbox(GetEnumDisplayName(candidate), ref enabled))
            {
                current = enabled ?
                              current | raw :
                              current & ~raw;
                changed = true;
            }
        }

        if (changed)
            value = (T)Enum.ToObject(typeof(T), current);

        return changed;
    }
}
