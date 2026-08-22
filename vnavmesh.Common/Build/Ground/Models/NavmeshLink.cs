using System.Numerics;
using vnavmesh.Common.Build.Ground.Enums;

namespace vnavmesh.Common.Build.Ground.Models;

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
    Vector3                      Start,
    Vector3                      End,
    NavmeshOffMeshKind           Kind,
    bool                         Bidirectional,
    int                          UserId,
    NavmeshLinkTraversalProfile? TraversalProfile = null
);
