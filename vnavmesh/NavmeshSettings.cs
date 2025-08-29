using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using DotRecast.Recast;
using System;
using ImGuiNET;

namespace Navmesh;

public class NavmeshSettings
{
    [Flags]
    public enum Filter
    {
        None = 0,
        LowHangingObstacles = 1 << 0,
        LedgeSpans = 1 << 1,
        WalkableLowHeightSpans = 1 << 2,
        Interiors = 1 << 3,
    }

    public float CellSize = 0.25f;
    public float CellHeight = 0.25f;
    public float AgentHeight = 2.0f;
    public float AgentRadius = 0.5f;
    public float AgentMaxClimb = 0.5f;
    public float AgentMaxSlopeDeg = 55f;
    public Filter Filtering = Filter.LowHangingObstacles | Filter.LedgeSpans | Filter.WalkableLowHeightSpans;
    public float RegionMinSize = 8;
    public float RegionMergeSize = 20;
    public RcPartition Partitioning = RcPartition.WATERSHED;
    public float PolyMaxEdgeLen = 12f;
    public float PolyMaxSimplificationError = 1.5f;
    public int PolyMaxVerts = 6;
    public float DetailSampleDist = 6f;
    public float DetailMaxSampleError = 1f;

    // we assume that bounds are constant -1024 to 1024 along each axis (since that's the quantization range of position in some packets)
    // there is some code that relies on tiling being power-of-2
    // current values mean 128x128x128 L1 tiles -> 16x16x16 L2 tiles -> 2x2x2 voxels
    public int[] NumTiles = [16, 8, 8];


    public void Draw()
    {
        DrawConfigFloat(ref CellSize, 0.1f, 1.0f, 0.01f, "光栅化：单元格大小 (#cs)", """
            用于场地的 xz 平面单元格大小。[限制：> 0] [单位：世界坐标]

            体素化单元格大小 #cs 定义了地面平面两个轴向（Recast 中的 x 和 z）的体素尺寸。
            此值通常基于角色半径 `r` 计算。建议的 #cs 初始值为 `r/2` 或 `r/3`。
            较小的 #cs 值会增加光栅化分辨率和导航网格细节，但总生成时间会指数级增长。
            在户外环境中，`r/2` 通常足够好。对于有狭窄空间的室内场景，你可能需要额外的精度，
            此时 `r/3` 或更小的值可能会带来更好的结果。

            最初的本能是将此值减少到接近零来最大化生成导航网格的细节。
            然而，这很快就会出现收益递减的情况。超过某个点后，生成的导航网格通常没有太大的
            可感知差异，但生成时间会大幅增加。这会阻碍你快速迭代关卡设计的能力，收益甚微。
            这里的一般建议是使用你能接受的尽可能大的 #cs 值。

            #cs 和 #ch 定义体素/网格/单元格大小。因此它们的值对所有以体素单位定义的参数
            都有显著的副作用。

            此参数的最小值取决于平台的浮点精度，实际最小值通常在 0.05 左右。
            """);
        DrawConfigFloat(ref CellHeight, 0.1f, 1.0f, 0.01f, "光栅化：单元格高度 (#ch)", """
            用于场地的 y 轴单元格大小。[限制：> 0] [单位：世界坐标]

            体素化单元格高度 #ch 单独定义，以便在高度测试中允许更高的精度。
            #ch 的良好起始点是 #cs 值的一半。较小的 #ch 值确保导航网格正确连接
            仅被小路缘或沟渠分隔的区域。如果在高度不连续的地方（例如楼梯或路缘）
            在导航网格中产生小洞，你可能需要减小单元格高度值以提高 Recast 的
            垂直光栅化精度。

            #cs 和 #ch 定义体素/网格/单元格大小。因此它们的值对所有以体素单位定义的
            参数都有显著的副作用。

            此参数的最小值取决于平台的浮点精度，实际最小值通常在 0.05 左右。
            """);
        DrawConfigFloat(ref AgentHeight, 0.1f, 5.0f, 0.1f, "代理：高度", """
            仍然允许地板区域被视为可行走的最小地板到"天花板"高度。[限制：>= 3 * CellHeight] [单位：世界坐标]

            此值定义代理在体素中的世界空间高度 `h`。#walkableHeight 的值应该计算为 `ceil(h / ch)`。
            请注意这基于 #ch 而不是 #cs，因为它是一个高度值。

            允许检测源几何体中的悬垂，这些悬垂使下方的几何体无法行走。
            该值通常设置为最大代理高度。
            """);
        DrawConfigFloat(ref AgentRadius, 0.0f, 5.0f, 0.1f, "代理：半径", """
            将高度场的可行走区域从障碍物侵蚀/收缩的距离。[限制：>= 0] [单位：世界坐标]

            参数 #walkableRadius 定义体素中的世界空间代理半径 `r`。
            通常，#walkableRadius 的值应该计算为 `ceil(r / cs)`。
            请注意这基于 #cs，因为代理半径总是平行于地面平面。

            如果 #walkableRadius 值大于零，导航网格的边缘将被这个距离推离所有障碍物。

            非零的 #walkableRadius 允许更简单的运行时导航网格碰撞检查。
            游戏只需要检查代理的中心点是否包含在导航网格多边形内。
            没有这种侵蚀，运行时导航检查需要将代理逻辑圆柱体的几何投影与
            导航网格多边形的边界边缘进行碰撞。

            一般来说，这是最终网格的任何部分应该接近源几何体中障碍物的最近距离。
            它通常设置为最大代理半径。

            如果你想要紧密贴合的导航网格，或者想要为具有不同半径的多个代理重用同一个导航网格，
            你可以使用 `walkableRadius` 值为零。但请注意，你需要自己处理与导航网格边缘的碰撞，
            并且网格生成中可能会出现奇怪的边缘情况问题。出于这些原因，
            指定零半径是允许的但不推荐。
            """);
        DrawConfigFloat(ref AgentMaxClimb, 0.1f, 5.0f, 0.1f, "代理：最大攀爬高度", """
            仍被认为可穿越的最大壁架高度。[限制：>= 0] [单位：世界坐标]

            #walkableClimb 值定义代理可以攀爬的壁架和台阶的最大高度。
            给定设计师定义的世界单位 `maxClimb` 距离，#walkableClimb 的值应该计算为 `ceil(maxClimb / ch)`。
            请注意这使用 #ch 而不是 #cs，因为它是基于高度的值。

            允许网格流过低矮障碍物，如路缘和上下楼梯。
            该值通常设置为代理可以上下移动的距离。
            """);
        DrawConfigFloat(ref AgentMaxSlopeDeg, 0.0f, 90.0f, 1.0f, "代理：最大坡度", """
            被认为可行走的最大坡度。[限制：0 <= 值 < 90] [单位：度]

            参数 #walkableSlopeAngle 用于过滤掉世界中地面坡度对代理来说过于陡峭而无法穿越的区域。
            该值定义为多边形表面法线可以与世界向上向量相差的最大角度（以度为单位）。
            该值必须在 `[0, 90]` 范围内。

            该参数的实际上限通常在 85 度左右。
            """);
        DrawConfigFilteringCombo(ref Filtering, "过滤", """
            选择应用于体素化几何体的过滤通道，以移除某些类别的伪像。
            """);
        DrawConfigFloat(ref RegionMinSize, 0.0f, 150.0f, 1.0f, "区域：最小大小", """
            允许形成孤立岛屿区域的最小单元格数量。[限制：>= 0] [单位：体素]

            分水岭分割对输入距离场中的噪声非常敏感。
            为了获得更好的区域，在分水岭分割后会合并区域并移除小的断开区域。
            参数 #minRegionArea 描述仍保留的最小孤立区域大小。
            如果区域中的体素数量小于 #minRegionArea 的平方，则会移除该区域。

            任何小于此区域的区域都将被标记为不可行走。
            这在移除有时会在桌面、盒子顶部等几何体上形成的无用区域时很有用。
            """);
        DrawConfigFloat(ref RegionMergeSize, 0.0f, 150.0f, 1.0f, "区域：合并大小", """
            跨度计数小于此值的任何区域，如果可能，将与较大的区域合并。[限制：>=0] [单位：体素]

            三角化过程在小的、局部化的体素区域上效果最佳。
            参数 #mergeRegionArea 控制允许与另一个区域合并的区域的最大体素区域。
            如果你看到这里那里缺少小块，你可以降低 #minRegionArea 值。
            """);
        DrawConfigPartitioningCombo(ref Partitioning, "分割算法", """
            有3种分割方法，各有优缺点。
            """);
        DrawConfigFloat(ref PolyMaxEdgeLen, 0.0f, 50.0f, 1.0f, "多边形化：最大边长", """
            沿网格边界的轮廓边的最大允许长度。[限制：>= 0] [单位：世界坐标]

            在某些情况下，长的外边缘可能会降低生成三角化的质量，
            创建非常长的细三角形。这有时可以通过限制最大边长来解决，
            使有问题的长边被分解为更小的段。

            参数 #maxEdgeLen 定义最大边长，以体素为单位定义。
            #maxEdgeLen 的好值类似于 `walkableRadius * 8`。
            调整此值的好方法是首先将其设置得很高，看看你的数据是否创建长边。
            如果是的话，减少 #maxEdgeLen，直到找到改善生成镶嵌的最大值。

            将根据需要插入额外的顶点，以保持轮廓边低于此长度。
            值为零有效地禁用此功能。
            """);
        DrawConfigFloat(ref PolyMaxSimplificationError, 0.1f, 3.0f, 0.1f, "多边形化：最大边简化误差", """
            简化轮廓的边界边应从原始轮廓偏离的最大距离。[限制：>=0] [单位：体素]

            当光栅化区域转换回矢量化表示时，
            #maxSimplificationError 描述简化的松散程度。
            简化过程使用 Ramer–Douglas-Peucker 算法，
            该值描述体素中的最大偏差。

            #maxSimplificationError 的好值在 `[1.1, 1.5]` 范围内。
            值 `1.3` 是一个好的起点，通常产生好的结果。
            如果值小于 `1.1`，生成的边缘开始出现锯齿。
            如果值大于 `1.5`，网格简化开始切掉一些不应该切掉的角。

            此参数的效果仅适用于 xz 平面。
            """);
        DrawConfigInt(ref PolyMaxVerts, 3, 12, 1, "多边形化：每个多边形的最大顶点数", """
            在轮廓到多边形转换过程中生成的多边形允许的最大顶点数。[限制：>= 3]

            如果网格数据要用于构建 Detour 导航网格，则上限
            限制为 <= #DT_VERTS_PER_POLYGON。
            """); // TODO: fix the limit to make it always suitable for detour
        DrawConfigFloat(ref DetailSampleDist, 0.0f, 16.0f, 1.0f, "详细网格：采样距离", """
            生成详细网格时使用的采样距离。[限制：0 或 >= 0.9] [单位：体素]
            """); // TODO: verify that it's actually in voxels
        DrawConfigFloat(ref DetailMaxSampleError, 0.0f, 16.0f, 1.0f, "详细网格：最大采样误差", """
            详细网格表面应从高度场数据偏离的最大距离。（仅用于高度细节。）[限制：>= 0] [单位：世界坐标]
            """); // TODO: verify that it's actually in voxels
        DrawConfigInt(ref NumTiles[0], 1, 32, 1, "L1 瓦片计数", """
            第一级细分每轴的瓦片数量。必须是2的幂。[限制：1 <= 值 <= 32]
            影响导航网格和导航体积。
            """);
        DrawConfigInt(ref NumTiles[1], 1, 32, 1, "L2 瓦片计数", """
            第二级细分每轴的瓦片数量。必须是2的幂。[限制：1 <= 值 <= 32]
            仅影响导航体积。
            """);
        DrawConfigInt(ref NumTiles[2], 1, 32, 1, "L3 体素计数", """
            每个瓦片每轴的叶体素数量。必须是2的幂。[限制：1 <= 值 <= 32]
            仅影响导航体积。
            """);
    }

    private void DrawConfigFloat(ref float value, float min, float max, float increment, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        ImGui.InputFloat(label, ref value);
        ImGuiComponents.HelpMarker(help);
    }

    private void DrawConfigInt(ref int value, int min, int max, int increment, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        ImGui.InputInt(label, ref value);
        ImGuiComponents.HelpMarker(help);
    }

    private void DrawConfigFilteringCombo(ref Filter value, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        using var combo = ImRaii.Combo(label, value.ToString());
        if (!combo)
        {
            ImGuiComponents.HelpMarker(help);
            return;
        }
        DrawConfigFilteringEnum(ref value, Filter.LowHangingObstacles, "低悬障碍物", """
            如果非可行走跨度的最大值在其下方跨度的 #walkableClimb 范围内，则将其标记为可行走。

            这会移除代理能够跨越的小障碍物和光栅化伪像，如路缘。
            它还允许代理在阶梯状结构（如楼梯）上移动。

            如果满足：obstacleSpan.smax - walkableSpan.smax < walkableClimb，则障碍物跨度被标记为可行走
            """);
        DrawConfigFilteringEnum(ref value, Filter.LedgeSpans, "壁架跨度", """
            将作为壁架的跨度标记为不可行走。

            壁架是具有一个或多个邻居的跨度，其最大值距离当前跨度的最大值超过 #walkableClimb。
            此方法移除保守体素化过度估计的影响，
            使得结果网格不会有悬挂在壁架上空的区域。

            如果满足：abs(currentSpan.smax - neighborSpan.smax) > walkableClimb，则跨度是壁架
            """);
        DrawConfigFilteringEnum(ref value, Filter.WalkableLowHeightSpans, "可行走低高度跨度", """
            如果跨度上方的净空小于指定的 #walkableHeight，则将可行走跨度标记为不可行走。

            对于此过滤器，跨度上方的净空是从跨度的最大值到同一列中
            下一个更高跨度的最小值的距离。
            如果列中没有更高的跨度，则净空计算为从跨度顶部到
            最大高度场高度的距离。
            """);
        DrawConfigFilteringEnum(ref value, Filter.Interiors, "内部", """
            将流形几何体内部（或非流形下方）的跨度标记为不可行走。
            """);
    }

    private void DrawConfigFilteringEnum(ref Filter value, Filter mask, string label, string help)
    {
        bool set = value.HasFlag(mask);
        if (ImGui.Checkbox(label, ref set))
            value ^= mask;
        ImGuiComponents.HelpMarker(help);
    }

    private void DrawConfigPartitioningCombo(ref RcPartition value, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        using var combo = ImRaii.Combo(label, value switch
        {
            RcPartition.WATERSHED => "Watershed",
            RcPartition.MONOTONE => "Monotone",
            RcPartition.LAYERS => "Layer",
            _ => "???"
        });
        if (!combo)
        {
            ImGuiComponents.HelpMarker(help);
            return;
        }

        DrawConfigPartitioningEnum(ref value, RcPartition.WATERSHED, "分水岭", """
            分水岭分割：
             - 经典的 Recast 分割方法
             - 创建最佳的镶嵌
             - 通常最慢
             - 将高度场分割为没有洞或重叠的良好区域
             - 在某些边缘情况下此方法会产生洞和重叠
                - 当小障碍物靠近大开放区域时可能出现洞（三角化可以处理这个）
                - 如果有狭窄的螺旋走廊（即楼梯），可能发生重叠，这会使三角化失败
            如果你预计算导航网格，这通常是最佳选择，如果你有大开放区域请使用此选项。
            """);
        DrawConfigPartitioningEnum(ref value, RcPartition.MONOTONE, "单调", """
            单调分割：
             - 最快
             - 将高度场分割为没有洞和重叠的区域（保证）
             - 创建长细多边形，有时会导致绕行路径
            如果你想要快速导航网格生成，请使用此选项。
            """);
        DrawConfigPartitioningEnum(ref value, RcPartition.LAYERS, "分层", """
            分层分割：
             - 相当快
             - 将高度场分割为非重叠区域
             - 依赖三角化代码来处理洞（因此比单调分割慢）
             - 产生比单调分割更好的三角形
             - 没有分水岭分割的边缘情况
             - 如果你有带小障碍物的大开放区域，可能较慢并创建有点丑陋的镶嵌
               （仍比单调好）（如果使用瓦片则不是问题）
            对于中小尺寸瓦片的平铺导航网格是很好的选择。
            """);
    }

    private void DrawConfigPartitioningEnum(ref RcPartition value, RcPartition choice, string label, string help)
    {
        if (ImGui.RadioButton(label, value.Equals(choice)))
            value = choice;
        ImGuiComponents.HelpMarker(help);
    }
}
