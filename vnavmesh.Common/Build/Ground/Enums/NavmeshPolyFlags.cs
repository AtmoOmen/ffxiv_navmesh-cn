namespace vnavmesh.Common.Build.Ground.Enums;

/// <summary>
///     导航网格多边形可穿越性标记，按位组合表示多边形上允许的移动方式。
/// </summary>
/// <remarks>
///     单个多边形可同时拥有多种穿越方式（如既可行走也可跳落），
///     位标记使路径搜索时能高效判断某条边是否可通过。
/// </remarks>
[Flags]
public enum NavmeshPolyFlags
{
    /// <summary>无任何穿越能力，多边形不可用于寻路。</summary>
    None = 0,

    /// <summary>常规地面，角色可正常行走通过。</summary>
    Ground = 1 << 0,

    /// <summary>自动生成的攀爬/跳落连接，用于从高处跳下至低处。</summary>
    GeneratedClimbDown = 1 << 1,

    /// <summary>自动生成的边缘跳跃连接，用于跨越小缝隙或断崖。</summary>
    GeneratedEdgeJump = 1 << 2,

    /// <summary>手工放置的离网连接（如桥梁、跳台），由关卡设计师手动标记。</summary>
    ManualOffMesh = 1 << 3,

    /// <summary>传送连接（如以太之光瞬移），瞬间从一点到达另一点。</summary>
    Teleport = 1 << 4,

    /// <summary>客户端侧轨迹路径，由客户端自行插值计算移动轨迹。</summary>
    ClientPath = 1 << 5,

    /// <summary>普通移动捷径，用于连接 Recast 未识别但角色可正常通过的位置。</summary>
    Shortcut = 1 << 6,

    /// <summary>不可达区域，路径搜索时直接跳过。</summary>
    Unreachable = 1 << 8,

    /// <summary>所有可穿越方式的组合，用于允许任意移动类型的查询。</summary>
    AllTraversable = Ground | GeneratedClimbDown | GeneratedEdgeJump | ManualOffMesh | Shortcut | Teleport | ClientPath
}
