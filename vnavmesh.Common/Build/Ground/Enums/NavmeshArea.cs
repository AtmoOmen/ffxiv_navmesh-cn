namespace vnavmesh.Common.Build.Ground.Enums;

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
    ClientPath = 6,

    /// <summary>普通移动捷径，作为可用但不应过度偏好的人工连接。</summary>
    Shortcut = 7
}
