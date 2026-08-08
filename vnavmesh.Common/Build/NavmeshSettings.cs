using System.Globalization;
using System.Text;
using DotRecast.Recast;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Build.Models;

namespace vnavmesh.Common.Build;

public class NavmeshSettings
{
    /// <summary>
    /// 水平方向体素尺寸，单位与场景一致。数值越小，地形细节越精确，但构建时间和内存占用越高；遇到窄缝或边缘缺失时可调小。
    /// </summary>
    public float CellSize = 0.25f;

    /// <summary>
    /// 垂直方向体素高度。数值越小，高度方向判定越精细，台阶、屋檐等结构越准确，但体素数量随之增多。
    /// </summary>
    public float CellHeight = 0.25f;

    /// <summary>
    /// 角色的可通行高度，决定需要多大净空才把区域视为可走。数值越大越保守，过小会导致角色头顶卡住。
    /// </summary>
    public float AgentHeight = 2f;

    /// <summary>
    /// 角色半径，决定地面侵蚀宽度以及路径与障碍物的距离。数值越大，狭窄通道越容易被剔除。
    /// </summary>
    public float AgentRadius = 0.5f;

    /// <summary>
    /// 角色可攀爬的最大台阶高度。高于该值的边缘会断开，寻路不会直接跨越。
    /// </summary>
    public float AgentMaxClimb = 0.5f;

    /// <summary>
    /// 可行走的最大坡度角度。超过该角度的斜面会被剔除，不参与地面寻路。
    /// </summary>
    public float AgentMaxSlopeDeg = 55f;

    /// <summary>
    /// 体素化后的过滤标记组合，控制低矮障碍、悬崖边缘、低净空与室内区域的剔除方式。按需增减 <see cref="NavmeshFilter" /> 位。
    /// </summary>
    public NavmeshFilter Filtering = NavmeshFilter.LowHangingObstacles | NavmeshFilter.WalkableLowHeightSpans;


    /// <summary>
    /// 区域最小边长，用于过滤过小且孤立的可走区域。数值越大，小平台和碎片区域越容易被丢弃。
    /// </summary>
    public float RegionMinSize = 8;

    /// <summary>
    /// 区域合并边长，控制相邻小区域合并为大区域的阈值。数值越大，网格区域越少越规整，但可能合并掉应有边界的区域。
    /// </summary>
    public float RegionMergeSize = 20;

    /// <summary>
    /// 区域划分算法。WATERSHED 质量最好但最慢，MONOTONE 与 LAYERS 更快；构建耗时可换用后两者。
    /// </summary>
    public RcPartition Partitioning = RcPartition.WATERSHED;


    /// <summary>
    /// 多边形轮廓的最大边长，用于把过长的边细分。设为 0 时按角色半径的 8 倍自动推导。
    /// </summary>
    public float PolyMaxEdgeLen = 12;

    /// <summary>
    /// 轮廓简化的最大误差，控制生成多边形对原始地形的贴合程度。数值越小越贴合，但顶点数量越多。
    /// </summary>
    public float PolyMaxSimplificationError = 1.5f;

    /// <summary>
    /// 单个多边形最大顶点数，也是导航网格多边形的顶点容量上限。数值越小多边形越简单，但总数会增多。
    /// </summary>
    public int PolyMaxVerts = 6;

    /// <summary>
    /// 细节网格采样距离，实际采样间距为数值乘以 <see cref="CellSize" />。设为 0 或小于 0.9 会关闭细节网格。
    /// </summary>
    public float DetailSampleDist = 6f;

    /// <summary>
    /// 细节网格最大采样误差，实际误差阈值为数值乘以 <see cref="CellHeight" />。数值越小，细节越贴合原始地形，但数据量越大。
    /// </summary>
    public float DetailMaxSampleError = 1f;

    /// <summary>
    /// 快速构建开关，开启后直接跳过细节网格生成。适合调试与快速预览，正式构建建议关闭。
    /// </summary>
    public bool FastBuild = false;


    /// <summary>
    /// 是否自动生成向下攀爬链接。开启后会在可跳下但不能正常走下的位置生成高到低单向连接。
    /// </summary>
    public bool GenerateEdgeClimbLinks;

    /// <summary>
    /// 是否自动生成向下跳跃链接。开启后生成高到低单向跳跃连接，用于跨越断崖或跳下平台。
    /// </summary>
    public bool GenerateEdgeJumpLinks;

    /// <summary>
    /// 边缘链接检测时的地面容差，参与轨迹碰撞判断。数值越大越容易接受贴近地面的轨迹，但可能穿过障碍。
    /// </summary>
    public float GroundTolerance = 0.3f;

    /// <summary>
    /// 向下攀爬终点相对起点的水平搜索距离。数值越大覆盖范围越广，但可能生成过远的链接。
    /// </summary>
    public float ClimbDownDistance = 0.4f;

    /// <summary>
    /// 向下攀爬允许的最小落差值，低于该值不生成链接，避免把普通台阶当成跳落。
    /// </summary>
    public float ClimbDownMinHeight = 1.5f;

    /// <summary>
    /// 向下攀爬允许的最大落差值，高于该值的落差不会生成链接。
    /// </summary>
    public float ClimbDownMaxHeight = 3.2f;

    /// <summary>
    /// 边缘跳跃终点相对起点的水平搜索距离，决定跳跃能覆盖多远。
    /// </summary>
    public float EdgeJumpEndDistance = 2f;

    /// <summary>
    /// 边缘跳跃的起跳高度参数，控制轨迹最高点。数值越大轨迹越高，越容易越过障碍，但生成成本也越高。
    /// </summary>
    public float EdgeJumpHeight = 1.8f;

    /// <summary>
    /// 边缘跳跃允许的最小落差值，低于该值不生成跳跃链接。
    /// </summary>
    public float EdgeJumpMinDrop = 1.5f;

    /// <summary>
    /// 边缘跳跃允许的最大落差值，高于该值不生成跳跃链接。默认 500 表示基本不限制。
    /// </summary>
    public float EdgeJumpMaxDrop = 500f;


    /// <summary>
    /// 地面瓦片的目标世界尺寸，用于按场景占用跨度自动推导每轴瓦片数。数值越小，瓦片越多、构建粒度越细。
    /// </summary>
    public float GroundTileSize = 64f;

    /// <summary>
    /// 地面瓦片每轴数量上限，防止超大场景生成过多瓦片。数值越大，并行度越高，但内存占用也越高。
    /// </summary>
    public int GroundTileCountMax = 32;

    /// <summary>
    /// 是否构建飞行体积。开启后额外生成多层体素体积用于飞行寻路，通常由地形定制自动设置；不需要飞行的区域保持关闭可节省构建时间。
    /// </summary>
    public bool Flyable;

    /// <summary>
    /// 飞行体积在已有地面瓦片基础上的进一步细分倍数。第一层数量跟随地面瓦片，数组每个元素再乘一次细分；数值越大越精细，但内存和时间显著增加。
    /// </summary>
    public int[] VolumeTiles = [8, 8];


    /// <summary>
    /// 构建使用的最大线程数。0 表示使用全部可用核心，负数表示从可用核心数中扣除，大于 0 时强制限制为指定值。
    /// </summary>
    public int BuildMaxCores = 0;

    /// <summary>
    /// 定制定义版本号，由定制系统在构建前写入，用于让旧缓存失效。通常不需要手工调整，修改定制定义版本时应同步增大。
    /// </summary>
    public int CustomizationVersion;

    /// <summary>
    /// 手工离网连接列表，定义桥梁、跳台等普通多边形无法表达的连接，每条记录起点、终点、半径与是否双向；由定制系统填充，也可以直接追加。
    /// </summary>
    public List<OffMeshConnection> OffMeshConnections = [];

    public NavmeshSettings Clone()
    {
        var clone = (NavmeshSettings)MemberwiseClone();
        clone.VolumeTiles        = (int[])VolumeTiles.Clone();
        clone.OffMeshConnections = [.. OffMeshConnections];
        return clone;
    }

    public string BuildSignature()
    {
        var sb = new StringBuilder(256);
        AppendFloat(nameof(CellSize),         CellSize);
        AppendFloat(nameof(CellHeight),       CellHeight);
        AppendFloat(nameof(AgentHeight),      AgentHeight);
        AppendFloat(nameof(AgentRadius),      AgentRadius);
        AppendFloat(nameof(AgentMaxClimb),    AgentMaxClimb);
        AppendFloat(nameof(AgentMaxSlopeDeg), AgentMaxSlopeDeg);
        AppendInt(nameof(Filtering), (int)Filtering);
        AppendFloat(nameof(RegionMinSize),   RegionMinSize);
        AppendFloat(nameof(RegionMergeSize), RegionMergeSize);
        AppendInt(nameof(Partitioning), (int)Partitioning);
        AppendFloat(nameof(PolyMaxEdgeLen),             PolyMaxEdgeLen);
        AppendFloat(nameof(PolyMaxSimplificationError), PolyMaxSimplificationError);
        AppendInt(nameof(PolyMaxVerts), PolyMaxVerts);
        AppendFloat(nameof(DetailSampleDist),     DetailSampleDist);
        AppendFloat(nameof(DetailMaxSampleError), DetailMaxSampleError);
        AppendBool(nameof(FastBuild),              FastBuild);
        AppendBool(nameof(GenerateEdgeClimbLinks), GenerateEdgeClimbLinks);
        AppendBool(nameof(GenerateEdgeJumpLinks),  GenerateEdgeJumpLinks);
        AppendFloat(nameof(GroundTolerance),     GroundTolerance);
        AppendFloat(nameof(ClimbDownDistance),   ClimbDownDistance);
        AppendFloat(nameof(ClimbDownMaxHeight),  ClimbDownMaxHeight);
        AppendFloat(nameof(ClimbDownMinHeight),  ClimbDownMinHeight);
        AppendFloat(nameof(EdgeJumpEndDistance), EdgeJumpEndDistance);
        AppendFloat(nameof(EdgeJumpHeight),      EdgeJumpHeight);
        AppendFloat(nameof(EdgeJumpMaxDrop),     EdgeJumpMaxDrop);
        AppendFloat(nameof(EdgeJumpMinDrop),     EdgeJumpMinDrop);
        AppendBool(nameof(Flyable), Flyable);
        AppendFloat(nameof(GroundTileSize), GroundTileSize);
        AppendInt(nameof(GroundTileCountMax), GroundTileCountMax);
        AppendText(nameof(VolumeTiles), string.Join(',', VolumeTiles));
        return sb.ToString();

        void AppendFloat
        (
            string key,
            float  value
        ) =>
            sb.Append(key).Append('=').Append(value.ToString("R", CultureInfo.InvariantCulture)).Append(';');

        void AppendInt
        (
            string key,
            int    value
        ) =>
            sb.Append(key).Append('=').Append(value.ToString(CultureInfo.InvariantCulture)).Append(';');

        void AppendBool
        (
            string key,
            bool   value
        ) =>
            sb.Append(key).Append('=').Append
            (
                value ?
                    '1' :
                    '0'
            ).Append(';');

        void AppendText
        (
            string key,
            string value
        ) =>
            sb.Append(key).Append('=').Append(value).Append(';');
    }
}
