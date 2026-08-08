namespace vnavmesh.Query.Flight;

internal static class VoxelPathfinderConstants
{
    public const float  SCORE_EPSILON                              = 0.00001f;
    public const int    MAX_NODE_COUNT                             = 3_000_000;
    public const int    MACRO_EXPANSION_BUDGET                     = 100_000;
    public const int    MID_SEGMENT_EXPANSION_BUDGET               = 100_000;
    public const int    MICRO_SEGMENT_EXPANSION_BUDGET             = 200_000;
    public const int    GLOBAL_EXPANSION_BUDGET                    = 2_000_000;
    public const int    MAX_LOS_CACHE_SIZE                         = 100_000;
    public const int    MAX_NEAREST_TRAVERSABLE_CELLS              = 4_096;
    public const int    MAX_NODE_LOOKUP_CAPACITY                   = 2048;
    public const float  DUPLICATE_WAYPOINT_DISTANCE_SQ             = 0.000001f;
    public const float  COLLINEAR_WAYPOINT_TOLERANCE               = 0.01f;
    public const ulong  VIRTUAL_GOAL_FLAG                          = 1UL << 63;
}
