using System.Globalization;
using System.Text;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using DotRecast.Recast;

namespace vnavmesh.Navigation.Mesh.Build;

public class NavmeshSettings
{
    [Flags]
    public enum Filter
    {
        None                   = 0,
        LowHangingObstacles    = 1 << 0,
        LedgeSpans             = 1 << 1,
        WalkableLowHeightSpans = 1 << 2,
        Interiors              = 1 << 3
    }

    public float       CellSize                   = 0.25f;
    public float       CellHeight                 = 0.25f;
    public float       AgentHeight                = 2.0f;
    public float       AgentRadius                = 0.5f;
    public float       AgentMaxClimb              = 0.5f;
    public float       AgentMaxSlopeDeg           = 55f;
    public Filter      Filtering                  = Filter.LowHangingObstacles | Filter.LedgeSpans | Filter.WalkableLowHeightSpans;
    public float       RegionMinSize              = 8;
    public float       RegionMergeSize            = 20;
    public RcPartition Partitioning               = RcPartition.MONOTONE;
    public float       PolyMaxEdgeLen             = 8f;
    public float       PolyMaxSimplificationError = 1.1f;
    public int         PolyMaxVerts               = 6;
    public float       DetailSampleDist           = 6f;
    public float       DetailMaxSampleError       = 1f;
    public bool        FastBuild;

    public bool  GenerateEdgeClimbLinks;
    public bool  GenerateEdgeJumpLinks;
    public float GroundTolerance     = 0.3f;
    public float ClimbDownDistance   = 0.4f;
    public float ClimbDownMaxHeight  = 3.2f;
    public float ClimbDownMinHeight  = 1.5f;
    public float EdgeJumpEndDistance = 2f;
    public float EdgeJumpHeight      = 1.8f;
    public float EdgeJumpMaxDrop     = 500f;
    public float EdgeJumpMinDrop     = 1.5f;

    // we assume that bounds are constant -1024 to 1024 along each axis (since that's the quantization range of position in some packets)
    // there is some code that relies on tiling being power-of-2
    // current values mean 128x128x128 L1 tiles -> 16x16x16 L2 tiles -> 2x2x2 voxels
    public int[] NumTiles = [16, 8, 8];

    public NavmeshSettings Clone()
    {
        var clone = (NavmeshSettings)MemberwiseClone();
        clone.NumTiles = (int[])NumTiles.Clone();
        return clone;
    }

    public string BuildSignature(bool flyable)
    {
        var sb = new StringBuilder(256);
        appendFloat(nameof(CellSize),         CellSize);
        appendFloat(nameof(CellHeight),       CellHeight);
        appendFloat(nameof(AgentHeight),      AgentHeight);
        appendFloat(nameof(AgentRadius),      AgentRadius);
        appendFloat(nameof(AgentMaxClimb),    AgentMaxClimb);
        appendFloat(nameof(AgentMaxSlopeDeg), AgentMaxSlopeDeg);
        appendInt(nameof(Filtering), (int)Filtering);
        appendFloat(nameof(RegionMinSize),   RegionMinSize);
        appendFloat(nameof(RegionMergeSize), RegionMergeSize);
        appendInt(nameof(Partitioning), (int)Partitioning);
        appendFloat(nameof(PolyMaxEdgeLen),             PolyMaxEdgeLen);
        appendFloat(nameof(PolyMaxSimplificationError), PolyMaxSimplificationError);
        appendInt(nameof(PolyMaxVerts), PolyMaxVerts);
        appendFloat(nameof(DetailSampleDist),     DetailSampleDist);
        appendFloat(nameof(DetailMaxSampleError), DetailMaxSampleError);
        appendBool(nameof(FastBuild), FastBuild);
        appendBool(nameof(GenerateEdgeClimbLinks), GenerateEdgeClimbLinks);
        appendBool(nameof(GenerateEdgeJumpLinks),  GenerateEdgeJumpLinks);
        appendFloat(nameof(GroundTolerance),     GroundTolerance);
        appendFloat(nameof(ClimbDownDistance),   ClimbDownDistance);
        appendFloat(nameof(ClimbDownMaxHeight),  ClimbDownMaxHeight);
        appendFloat(nameof(ClimbDownMinHeight),  ClimbDownMinHeight);
        appendFloat(nameof(EdgeJumpEndDistance), EdgeJumpEndDistance);
        appendFloat(nameof(EdgeJumpHeight),      EdgeJumpHeight);
        appendFloat(nameof(EdgeJumpMaxDrop),     EdgeJumpMaxDrop);
        appendFloat(nameof(EdgeJumpMinDrop),     EdgeJumpMinDrop);
        appendBool("Flyable", flyable);
        appendText(nameof(NumTiles), string.Join(',', NumTiles));
        return sb.ToString();

        void appendFloat(string key, float value)
        {
            sb.Append(key).Append('=').Append(value.ToString("R", CultureInfo.InvariantCulture)).Append(';');
        }

        void appendInt(string key, int value)
        {
            sb.Append(key).Append('=').Append(value.ToString(CultureInfo.InvariantCulture)).Append(';');
        }

        void appendBool(string key, bool value)
        {
            sb.Append(key).Append('=').Append(value ? '1' : '0').Append(';');
        }

        void appendText(string key, string value)
        {
            sb.Append(key).Append('=').Append(value).Append(';');
        }
    }


    public void Draw()
    {
        DrawConfigFloat
        (
            ref CellSize,
            0.1f,
            1.0f,
            0.01f,
            "光栅化: 单元格尺寸 (#cs)",
            """
            用于场的 xz 平面单元格尺寸 [限制: > 0] [单位: 世界]

            体素化单元格尺寸 #cs 定义了地面平面 x 和 z 轴的体素尺寸 此值通常由角色半径 `r` 得出 #cs 的推荐起始值为 `r/2` 或 `r/3` #cs 值越小 光栅化分辨率和导航网格细节越高 但生成时间将呈指数增长 在室外环境中 `r/2` 通常足够 对于室内场景中的狭小空间 可能需要更高精度 因此 `r/3` 或更小的值可能效果更好

            初始直觉是将此值减小到接近零以最大化生成导航网格的细节 然而 这很快会成为收益递减的情况 超过某一点后 生成的导航网格通常没有明显差异 但生成时间会大幅增加 这会妨碍快速迭代关卡设计的能力 且收益甚微 一般建议在此使用尽可能大的 #cs 值

            #cs 和 #ch 定义了体素/网格/单元格尺寸 因此它们的值对所有以体素单位定义的参数有显著副作用

            此参数的最小值取决于平台的浮点精度 实际最小值通常在 0.05 左右
            """
        );
        DrawConfigFloat
        (
            ref CellHeight,
            0.1f,
            1.0f,
            0.01f,
            "光栅化: 单元格高度 (#ch)",
            """
            用于场的 y 轴单元格尺寸 [限制: > 0] [单位: 世界]

            体素化单元格高度 #ch 被单独定义以允许在高度测试中实现更高精度 #ch 的良好起点是 #cs 值的一半 较小的 #ch 值确保导航网格正确连接仅被小台阶或沟渠分隔的区域 如果在高度不连续处周围生成的导航网格中出现小孔 可能需要减小单元格高度值以提高 Recast 的垂直光栅化精度

            #cs 和 #ch 定义了体素/网格/单元格尺寸 因此它们的值对所有以体素单位定义的参数有显著副作用

            此参数的最小值取决于平台的浮点精度 实际最小值通常在 0.05 左右
            """
        );
        DrawConfigFloat
        (
            ref AgentHeight,
            0.1f,
            5.0f,
            0.1f,
            "代理: 高度",
            """
            最小地面到天花板高度 仍允许地面区域被视为可行走 [限制: >= 3 * CellHeight] [单位: 世界]

            此值定义了代理在世界空间中的高度 `h` 以体素为单位 #walkableHeight 的值应计算为 `ceil(h / ch)` 注意这是基于 #ch 而非 #cs 因为这是一个高度值

            允许检测源几何体中的悬垂部分 使下方几何体不可行走 该值通常设置为代理最大高度
            """
        );
        DrawConfigFloat
        (
            ref AgentRadius,
            0.0f,
            5.0f,
            0.1f,
            "代理: 半径",
            """
            侵蚀/收缩高度场可行走区域远离障碍物的距离 [限制: >= 0] [单位: 世界]

            参数 #walkableRadius 定义了世界空间中代理半径 `r` 以体素为单位 通常 #walkableRadius 的值应计算为 `ceil(r / cs)` 注意这是基于 #cs 因为代理半径始终平行于地面平面

            如果 #walkableRadius 值大于零 导航网格的边缘将被推离所有障碍物此距离

            非零的 #walkableRadius 允许更简单的运行时导航网格碰撞检测 游戏只需要检查代理中心点是否包含在导航网格多边形内 没有此侵蚀 运行时导航检查需要将代理逻辑圆柱体的几何投影与导航网格多边形的边界边进行碰撞

            一般来说 这是最终网格的任何部分与源几何体中的障碍物最接近的距离 通常设置为最大代理半径

            如果需要紧密贴合的导航网格 或希望为具有不同半径的多个代理重用相同的导航网格 可以使用 `walkableRadius` 值为零 但请注意 需要自己对导航网格边缘执行碰撞检测 并且网格生成中可能出现奇怪的边缘情况问题 由于这些原因 指定半径为零是允许的 但不推荐
            """
        );
        DrawConfigFloat
        (
            ref AgentMaxClimb,
            0.1f,
            5.0f,
            0.1f,
            "代理: 最大攀爬",
            """
            仍被视为可行走的最大台阶高度 [限制: >= 0] [单位: 世界]

            #walkableClimb 值定义了代理可以走上的台阶和阶梯的最大高度 给定设计师定义的 `maxClimb` 世界单位距离 #walkableClimb 的值应计算为 `ceil(maxClimb / ch)` 注意这是使用 #ch 而非 #cs 因为这是一个基于高度的值

            允许网格流过低矮障碍物 如台阶和上下楼梯 该值通常设置为代理可以上下的距离
            """
        );
        DrawConfigFloat
        (
            ref AgentMaxSlopeDeg,
            0.0f,
            90.0f,
            1.0f,
            "代理: 最大坡度",
            """
            被视为可行走的最大坡度 [限制: 0 <= 值 < 90] [单位: 度]

            参数 #walkableSlopeAngle 用于过滤出地面坡度过陡代理无法行走的世界区域 此值定义为多边形表面法线与世界向上向量可相差的最大角度 此值必须在 `[0, 90]` 范围内

            此参数的实际上限通常约为 85 度
            """
        );
        DrawConfigFilteringCombo
        (
            ref Filtering,
            "过滤",
            """
            选择要应用于体素化几何体的过滤通道以移除某些类别的伪影
            """
        );
        DrawConfigFloat
        (
            ref RegionMinSize,
            0.0f,
            150.0f,
            1.0f,
            "区域: 最小尺寸",
            """
            允许形成孤立岛区域的最小单元格数 [限制: >= 0] [单位: 体素]

            分水岭分割对输入距离场中的噪声非常敏感 为了获得更好的区域 在分水岭分割后合并区域并移除小的断开区域 参数 #minRegionArea 描述了仍保留的最小孤立区域大小 如果区域中的体素数小于 #minRegionArea 的平方 则移除该区域

            任何小于此面积的区域将被标记为不可行走 这有助于移除有时会形成在桌面 盒顶等几何体上的无用区域
            """
        );
        DrawConfigFloat
        (
            ref RegionMergeSize,
            0.0f,
            150.0f,
            1.0f,
            "区域: 合并尺寸",
            """
            任何跨度计数小于此值的区域 如果可能 将与较大区域合并 [限制: >=0] [单位: 体素]

            三角化过程在小型局部体素区域中效果最好 参数 #mergeRegionArea 控制允许与另一个区域合并的区域的最大体素面积 如果看到各处有小块缺失 可以降低 #minRegionArea 值
            """
        );
        DrawConfigPartitioningCombo
        (
            ref Partitioning,
            "分区算法",
            """
            有 3 种分区方法 每种都有一些优缺点
            """
        );
        DrawConfigFloat
        (
            ref PolyMaxEdgeLen,
            0.0f,
            50.0f,
            1.0f,
            "多边形化: 最大边长",
            """
            网格边界上轮廓边允许的最大长度 [限制: >= 0] [单位: 世界]

            在某些情况下 长的外边缘可能会降低结果三角化的质量 创建非常长的细长三角形 这有时可以通过限制最大边长来缓解 使有问题的长边被分割成较小的段

            参数 #maxEdgeLen 定义了最大边长 以体素为单位 #maxEdgeLen 的良好值约为 `walkableRadius * 8` 调整此值的好方法是首先将其设置得非常高 查看数据是否创建长边 如果有 减小 #maxEdgeLen 直到找到能改善结果镶嵌的最大值

            将根据需要插入额外顶点以保持轮廓边低于此长度 零值有效地禁用此功能
            """
        );
        DrawConfigFloat
        (
            ref PolyMaxSimplificationError,
            0.1f,
            3.0f,
            0.1f,
            "多边形化: 最大边简化误差",
            """
            简化轮廓边界边与原始原始轮廓的最大偏离距离 [限制: >=0] [单位: 体素]

            当光栅化区域被转换回向量表示时 #maxSimplificationError 描述了简化的宽松程度 简化过程使用 Ramer-Douglas-Peucker 算法 此值描述了体素中的最大偏差

            #maxSimplificationError 的良好值在 `[1.1, 1.5]` 范围内 `1.3` 的值是一个良好的起点 通常能产生良好的结果 如果值小于 `1.1` 在生成的边缘开始出现锯齿 如果值大于 `1.5` 网格简化开始切掉一些不应该的角落

            此参数的效果仅适用于 xz 平面
            """
        );
        DrawConfigInt
        (
            ref PolyMaxVerts,
            3,
            12,
            1,
            "多边形化: 每多边形最大顶点数",
            """
            轮廓到多边形转换过程中生成的多边形允许的最大顶点数 [限制: >= 3]

            如果网格数据用于构建 Detour 导航网格 则上限限制为 <= #DT_VERTS_PER_POLYGON
            """
        ); // TODO: fix the limit to make it always suitable for detour
        DrawConfigFloat
        (
            ref DetailSampleDist,
            0.0f,
            16.0f,
            1.0f,
            "细节网格: 采样距离",
            """
            设置生成细节网格时使用的采样距离 [限制: 0 或 >= 0.9] [单位: 体素]

            细节网格由三角形子网格组成 这些子网格被细分以匹配原始高度场 此参数定义这些三角形边的采样距离 如果值小于 0.9 则网格不会被细分 良好值约为 6

            采样距离以体素定义 零值有效地禁用此功能
            """
        );
        DrawConfigFloat
        (
            ref DetailMaxSampleError,
            0.0f,
            16.0f,
            1.0f,
            "细节网格: 最大采样误差",
            """
            细节网格表面与高度场数据的最大偏离距离 (仅用于高度细节) [限制: >= 0] [单位: 世界]
            """
        ); // TODO: verify that it's actually in voxels
        ImGui.Checkbox("快速构建（关闭细节网格）", ref FastBuild);
        DrawConfigInt
        (
            ref NumTiles[0],
            1,
            32,
            1,
            "L1 瓦片数量",
            """
            一级细分的每轴瓦片数，必须为 2 的幂次。[限制：1 <= 值 <= 32]
            同时影响导航网格与导航体积。
            """
        );
        DrawConfigInt
        (
            ref NumTiles[1],
            1,
            32,
            1,
            "L2 瓦片数量",
            """
            L2 层沿 X 轴的瓦片数量
            """
        );
        DrawConfigInt
        (
            ref NumTiles[2],
            1,
            32,
            1,
            "L3 体素数量",
            """
            L3 层每瓦片沿 X 轴的体素数量
            """
        );

        ImGui.Checkbox("生成向下攀爬链接", ref GenerateEdgeClimbLinks);
        ImGui.Checkbox("生成向下跳跃链接", ref GenerateEdgeJumpLinks);
        DrawConfigFloat(ref GroundTolerance, 0, 50, 0.1f, "地面容差", "未记录");
        DrawConfigFloat
        (
            ref ClimbDownDistance,
            0,
            100,
            0.1f,
            "向下攀爬距离",
            """
            边缘攀爬采样的水平距离
            """
        );
        DrawConfigFloat(ref ClimbDownMaxHeight,  0, 100, 0.5f, "向下攀爬最大高度", "未记录");
        DrawConfigFloat(ref ClimbDownMinHeight,  0, 100, 0.5f, "向下攀爬最小高度", "未记录");
        DrawConfigFloat(ref EdgeJumpEndDistance, 0, 100, 0.5f, "边缘跳跃结束距离", "未记录");
        DrawConfigFloat(ref EdgeJumpHeight,      0, 10,  0.1f, "边缘跳跃高度",   "未记录");
        DrawConfigFloat(ref EdgeJumpMaxDrop,     0, 100, 0.1f, "边缘跳跃最大落差", "未记录");
        DrawConfigFloat(ref EdgeJumpMinDrop,     0, 100, 0.1f, "边缘跳跃最小落差", "未记录");
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

        DrawConfigFilteringEnum
        (
            ref value,
            Filter.LowHangingObstacles,
            "低垂障碍物",
            """
            如果不可行走跨度的最大值在其下方跨度的 #walkableClimb 范围内 则将其标记为可行走

            这会移除代理能够走过的小障碍物和光栅化伪影 如路缘石 还允许代理爬上阶梯状结构如楼梯

            当 obstacleSpan.smax - walkableSpan.smax < walkableClimb 时 障碍物跨度被标记为可行走
            """
        );
        DrawConfigFilteringEnum
        (
            ref value,
            Filter.LedgeSpans,
            "边缘跨度",
            """
            将边缘跨度标记为不可行走

            边缘是指具有一个或多个邻居的跨度 其最大值与当前跨度的最大值相距超过 #walkableClimb
            此方法消除了保守体素化过度估计的影响 因此生成的网格不会在边缘上方悬挂区域

            当 abs(currentSpan.smax - neighborSpan.smax) > walkableClimb 时 跨度被视为边缘
            """
        );
        DrawConfigFilteringEnum
        (
            ref value,
            Filter.WalkableLowHeightSpans,
            "可行走低高度跨度",
            """
            如果跨度上方的间隙小于指定的 #walkableHeight 则将可行走跨度标记为不可行走

            对于此过滤器 跨度上方的间隙是从跨度的最大值到同一列中下一个较高跨度的最小值的距离
            如果列中没有更高的跨度 则间隙计算为从跨度顶部到最大高度场高度的距离
            """
        );
        DrawConfigFilteringEnum
        (
            ref value,
            Filter.Interiors,
            "内部区域",
            """
            将流形几何体内部(或非流形下方)的跨度标记为不可行走
            """
        );
    }

    private void DrawConfigFilteringEnum(ref Filter value, Filter mask, string label, string help)
    {
        var set = value.HasFlag(mask);
        if (ImGui.Checkbox(label, ref set))
            value ^= mask;
        ImGuiComponents.HelpMarker(help);
    }

    private void DrawConfigPartitioningCombo(ref RcPartition value, string label, string help)
    {
        ImGui.SetNextItemWidth(300);
        using var combo = ImRaii.Combo
        (
            label,
            value switch
            {
                RcPartition.WATERSHED => "Watershed",
                RcPartition.MONOTONE  => "Monotone",
                RcPartition.LAYERS    => "Layer",
                _                     => "???"
            }
        );

        if (!combo)
        {
            ImGuiComponents.HelpMarker(help);
            return;
        }

        DrawConfigPartitioningEnum
        (
            ref value,
            RcPartition.WATERSHED,
            "Watershed",
            """
            Watershed partitioning:
             - the classic Recast partitioning
             - creates the nicest tessellation
             - usually slowest
             - partitions the heightfield into nice regions without holes or overlaps
             - the are some corner cases where this method creates produces holes and overlaps
                - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
                - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
            Generally the best choice if you precompute the nacmesh, use this if you have large open areas.
            """
        );
        DrawConfigPartitioningEnum
        (
            ref value,
            RcPartition.MONOTONE,
            "Monotone",
            """
            Monotone partitioning:
             - fastest
             - partitions the heightfield into regions without holes and overlaps (guaranteed)
             - creates long thin polygons, which sometimes causes paths with detours
            Use this if you want fast navmesh generation.
            """
        );
        DrawConfigPartitioningEnum
        (
            ref value,
            RcPartition.LAYERS,
            "Layer",
            """
            Layer partitioning
             - quite fast
             - partitions the heighfield into non-overlapping regions
             - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
             - produces better triangles than monotone partitioning
             - does not have the corner cases of watershed partitioning
             - can be slow and create a bit ugly tessellation (still better than monotone)
               if you have large open areas with small obstacles (not a problem if you use tiles)
            Good choice to use for tiled navmesh with medium and small sized tiles.
            """
        );
    }

    private void DrawConfigPartitioningEnum(ref RcPartition value, RcPartition choice, string label, string help)
    {
        if (ImGui.RadioButton(label, value.Equals(choice)))
            value = choice;
        ImGuiComponents.HelpMarker(help);
    }
}
