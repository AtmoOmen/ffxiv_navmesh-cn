using System.Globalization;
using System.Numerics;
using System.Text;
using DotRecast.Recast;
using vnavmesh.Common.Navigation.Mesh.Runtime;

namespace vnavmesh.Common.Navigation.Mesh.Build;

public sealed class NavmeshBuildSettings
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

    public float                               CellSize         = 0.25f;
    public float                               CellHeight       = 0.25f;
    public float                               AgentHeight      = 2.0f;
    public float                               AgentRadius      = 0.5f;
    public float                               AgentMaxClimb    = 0.5f;
    public float                               AgentMaxSlopeDeg = 55f;
    public Filter                              Filtering        = Filter.LowHangingObstacles | Filter.WalkableLowHeightSpans;
    public float                               RegionMinSize    = 8;
    public float                               RegionMergeSize  = 20;
    public RcPartition                         Partitioning     = RcPartition.WATERSHED;
    public float                               PolyMaxEdgeLen   = 12f;
    public float                               PolyMaxSimplificationError = 1.1f;
    public int                                 PolyMaxVerts               = 6;
    public float                               DetailSampleDist           = 6f;
    public float                               DetailMaxSampleError       = 1f;
    public bool                                FastBuild                  = false;
    public bool                                GenerateEdgeClimbLinks;
    public bool                                GenerateEdgeJumpLinks;
    public float                               GroundTolerance     = 0.3f;
    public float                               ClimbDownDistance   = 0.4f;
    public float                               ClimbDownMaxHeight  = 3.2f;
    public float                               ClimbDownMinHeight  = 1.5f;
    public float                               EdgeJumpEndDistance = 2f;
    public float                               EdgeJumpHeight      = 1.8f;
    public float                               EdgeJumpMaxDrop     = 500f;
    public float                               EdgeJumpMinDrop     = 1.5f;
    public float                               GroundTileSize      = 64f;
    public int                                 GroundTileCountMax  = 32;
    public int[]                               VolumeTiles         = [8, 8];
    public int                                 BuildMaxCores       = 1;
    public bool                                Flyable;
    public int                                 CustomizationVersion;
    public List<NavmeshBuildOffMeshConnection> OffMeshConnections = [];

    public string BuildSignature()
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
        appendBool(nameof(FastBuild),              FastBuild);
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
        appendBool(nameof(Flyable), Flyable);
        appendFloat(nameof(GroundTileSize), GroundTileSize);
        appendInt(nameof(GroundTileCountMax), GroundTileCountMax);
        appendText(nameof(VolumeTiles), string.Join(',', VolumeTiles));
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
}

public readonly record struct NavmeshBuildOffMeshConnection
(
    Vector3                      Start,
    Vector3                      End,
    float                        Radius,
    bool                         Bidirectional,
    int                          UserId,
    int                          Area,
    int                          Flags,
    int                          Kind,
    NavmeshLinkTraversalProfile? TraversalProfile = null
);
