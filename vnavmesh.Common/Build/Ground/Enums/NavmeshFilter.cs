namespace vnavmesh.Common.Build.Ground.Enums;

/// <summary>
///     导航网格构建时的过滤标记，控制烘焙过程中丢弃哪些 span。
/// </summary>
/// <remarks>
///     位标记组合，用于 Recast 烘焙阶段剔除不适合的几何体，
///     影响最终生成的多边形集合。
/// </remarks>
[Flags]
public enum NavmeshFilter
{
    /// <summary>不过滤，保留所有 span。</summary>
    None = 0,

    /// <summary>剔除低矮障碍物上方的 span（如头顶有阻碍物的区域），角色通过时会卡住。</summary>
    LowHangingObstacles = 1 << 0,

    /// <summary>剔除悬崖边缘的 span，防止路径边界过于贴近不可行走边缘。</summary>
    LedgeSpans = 1 << 1,

    /// <summary>标记而非剔除低矮可行走 span，用于特殊处理而非直接丢弃。</summary>
    WalkableLowHeightSpans = 1 << 2,

    /// <summary>剔除室内区域的 span（如洞穴内部），室外寻路时忽略之。</summary>
    Interiors = 1 << 3
}
