using vnavmesh.Common.Build.Ground.Models;

namespace vnavmesh.Common.Build.Ground.Enums;

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
    ClientPath,

    /// <summary>普通移动捷径。</summary>
    Shortcut
}
