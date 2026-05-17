using System.Globalization;
using System.Text;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Components;
using Dalamud.Interface.Utility.Raii;
using DotRecast.Recast;
using vnavmesh.Common.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Mesh.Build;

public class NavmeshSettings
{
    public float         CellSize         = 0.25f;
    public float         CellHeight       = 0.25f;
    public float         AgentHeight      = 2f;
    public float         AgentRadius      = 0.5f;
    public float         AgentMaxClimb    = 0.5f;
    public float         AgentMaxSlopeDeg = 55f;
    public NavmeshFilter Filtering        = NavmeshFilter.LowHangingObstacles | NavmeshFilter.WalkableLowHeightSpans;
    public float         RegionMinSize    = 8;
    public float         RegionMergeSize  = 20;
    public RcPartition   Partitioning     = RcPartition.WATERSHED;
    public float         PolyMaxEdgeLen   = 12;
    public float         PolyMaxSimplificationError = 1.5f;
    public int           PolyMaxVerts               = 6;
    public float         DetailSampleDist           = 6f;
    public float         DetailMaxSampleError       = 1f;
    public bool          FastBuild                  = false;

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

    public float GroundTileSize     = 64f;
    public int   GroundTileCountMax = 32;

    // first level count follows ground tiles; this array only controls further volume subdivision
    public int[] VolumeTiles = [8, 8];

    public NavmeshSettings Clone()
    {
        var clone = (NavmeshSettings)MemberwiseClone();
        clone.VolumeTiles = (int[])VolumeTiles.Clone();
        return clone;
    }

    public string BuildSignature(bool flyable)
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
        AppendBool("Flyable", flyable);
        AppendFloat(nameof(GroundTileSize), GroundTileSize);
        AppendInt(nameof(GroundTileCountMax), GroundTileCountMax);
        AppendText(nameof(VolumeTiles), string.Join(',', VolumeTiles));
        return sb.ToString();

        void AppendFloat(string key, float value)
        {
            sb.Append(key).Append('=').Append(value.ToString("R", CultureInfo.InvariantCulture)).Append(';');
        }

        void AppendInt(string key, int value)
        {
            sb.Append(key).Append('=').Append(value.ToString(CultureInfo.InvariantCulture)).Append(';');
        }

        void AppendBool(string key, bool value)
        {
            sb.Append(key).Append('=').Append(value ? '1' : '0').Append(';');
        }

        void AppendText(string key, string value)
        {
            sb.Append(key).Append('=').Append(value).Append(';');
        }
    }
}
