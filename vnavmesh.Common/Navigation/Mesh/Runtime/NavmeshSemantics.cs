using System.Numerics;

namespace vnavmesh.Common.Navigation.Mesh.Runtime;

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

    /// <summary>不可达区域，路径搜索时直接跳过。</summary>
    Unreachable = 1 << 8,

    /// <summary>所有可穿越方式的组合，用于允许任意移动类型的查询。</summary>
    AllTraversable = Ground | GeneratedClimbDown | GeneratedEdgeJump | ManualOffMesh | Teleport | ClientPath
}

/// <summary>
///     导航网格区域类型，对应各多边形的语义用途。
/// </summary>
/// <remarks>
///     与 <see cref="NavmeshPolyFlags" /> 不同，Area 是互斥的单一类型标识，
///     一个多边形只能属于一种区域。
/// </remarks>
public enum NavmeshArea
{
    /// <summary>无效区域，未初始化或已被剔除的多边形。</summary>
    Null = 0,

    /// <summary>常规可行走地面。</summary>
    Ground = 1,

    /// <summary>自动生成的高差跳落区域，角色从此处跳下至更低平台。</summary>
    GeneratedClimbDown = 2,

    /// <summary>自动生成的边缘跳跃区域，覆盖缝隙两侧的起跳与落地。</summary>
    GeneratedEdgeJump = 3,

    /// <summary>手工定义的离网连接区域，用于需要精确控制的非标准移动路径。</summary>
    ManualOffMesh = 4,

    /// <summary>传送区域（以太之光、水晶等），角色触发后瞬时转移。</summary>
    Teleport = 5,

    /// <summary>客户端插值路径区域，服务器不参与该段的移动验证。</summary>
    ClientPath = 6
}

/// <summary>
///     离网连接的细分种类，描述非行走移动方式的具体类型。
/// </summary>
/// <remarks>
///     用于 <see cref="NavmeshLink" /> 等结构体中区分连接属性，
///     决定路径跟随时的动作表现（跳落/跳跃/传送等）。
/// </remarks>
public enum NavmeshOffMeshKind
{
    /// <summary>自动生成的高差跳落连接。</summary>
    GeneratedClimbDown,

    /// <summary>自动生成的边缘跳跃连接。</summary>
    GeneratedEdgeJump,

    /// <summary>手工放置的离网连接。</summary>
    ManualOffMesh,

    /// <summary>传送瞬移连接。</summary>
    Teleport,

    /// <summary>客户端插值路径连接。</summary>
    ClientPath
}

/// <summary>
///     导航网格中的一条离网连接，定义两个空间点之间的非行走移动关系。
/// </summary>
/// <param name="Start">连接起点坐标。</param>
/// <param name="End">连接终点坐标。</param>
/// <param name="Kind">连接类型（跳落/跳跃/传送等）。</param>
/// <param name="Bidirectional">是否可双向穿越，<see langword="false" /> 表示仅单向。</param>
/// <param name="UserId">用户自定义 ID，用于与外部数据关联。</param>
public readonly record struct NavmeshLink
(
    Vector3            Start,
    Vector3            End,
    NavmeshOffMeshKind Kind,
    bool               Bidirectional,
    int                UserId
);

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
