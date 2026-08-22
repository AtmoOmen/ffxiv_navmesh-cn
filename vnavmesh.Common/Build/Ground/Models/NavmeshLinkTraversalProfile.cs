namespace vnavmesh.Common.Build.Ground.Models;

/// <summary>
///     单条离网连接的穿越代价配置。
/// </summary>
/// <param name="DistanceScale">几何距离缩放系数。</param>
/// <param name="FixedPenalty">固定附加代价。</param>
public readonly record struct NavmeshLinkTraversalProfile
(
    float DistanceScale,
    float FixedPenalty
);
