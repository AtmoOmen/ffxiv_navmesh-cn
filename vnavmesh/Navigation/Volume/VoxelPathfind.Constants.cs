using System.Numerics;
using System.Runtime.CompilerServices;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Navigation.Volume.Models;
using vnavmesh.Navigation.Volume.Utils;

namespace vnavmesh.Navigation.Volume;

public partial class VoxelPathfind
{
    private readonly record struct GuidedSearchCorridor
    (
        Vector3 Start,
        Vector2 HorizontalDirection,
        float   HorizontalLength,
        float   VerticalDelta,
        float   HorizontalRadius,
        float   UpwardAllowance,
        float   DownwardAllowance,
        float   EndpointSlack
    );

    private readonly record struct LongRangeLateralBias
    (
        bool    Enabled,
        Vector3 Start,
        Vector2 Forward,
        Vector2 Right,
        float   HorizontalDistance,
        bool    PreferDescending,
        float   HeightPriority,
        float   DirectionalPenaltyScale
    );

    private readonly record struct L1TraversalState
    (
        ulong Voxel,
        byte  EntryFace
    );

    private readonly record struct L1BestEffortSearchResult
    (
        HashSet<ulong>       PathSet,
        IReadOnlyList<ulong> OrderedPath,
        bool                 ReachedGoal,
        int                  ExpandedNodes,
        float                BestDistance,
        int                  StepBudget
    );

    private const float  SCORE_EPSILON                                                   = 0.00001f;
    private const int    DEFAULT_MAX_SEARCH_STEPS                                        = 1_0000_0000;
    private const int    RAYCAST_SEARCH_STEP_BUDGET                                      = 400000;
    private const int    GUIDED_CORRIDOR_SEARCH_STEP_BUDGET                              = 2_000_000;
    private const int    GUIDED_CORRIDOR_EARLY_ABORT_MIN_VISITED                         = 100_000;
    private const int    GUIDED_CORRIDOR_EARLY_ABORT_HARD_VISITED_THRESHOLD              = 150_000;
    private const int    GUIDED_CORRIDOR_EARLY_ABORT_STALL_WINDOW                        = 80_000;
    private const int    MAX_ANCESTOR_LOOK_BACK                                          = 6;
    private const int    RAYCAST_PARALLEL_NEIGHBOUR_THRESHOLD                            = 12;
    private const float  MAX_SEARCH_RAYCAST_DISTANCE_IN_LEAF_CELLS                       = 96f;
    private const float  SHORT_RANGE_HEURISTIC_WEIGHT                                    = 1.0f;
    private const float  LONG_RANGE_HEURISTIC_WEIGHT                                     = 0.85f;
    private const float  GOAL_VISIBILITY_PROBE_DISTANCE_IN_LEAF_CELLS                    = 48f;
    private const float  BEST_NODE_RELATIVE_H_TOLERANCE_LEAF_CELLS                       = 2.50f;
    private const int    L1_A_STAR_MAX_EXPANSIONS                                        = 200_000;
    private const int    L1_DISTANCE_FIELD_BUDGET                                        = 500_000;
    private const int    LONG_RANGE_L1_BEST_EFFORT_STEP_BUDGET                           = 750_000;
    private const int    LONG_RANGE_L1_BEST_EFFORT_MAX_STEP_BUDGET                       = 2_500_000;
    private const int    LONG_RANGE_L1_GOAL_CAPTURE_BASE_STEP_BUDGET                     = 300_000;
    private const int    LONG_RANGE_L1_GOAL_CAPTURE_MAX_STEP_BUDGET                      = 1_200_000;
    private const int    LONG_RANGE_L1_GUIDED_FULL_SEARCH_BASE_CORRIDOR_RADIUS           = 6;
    private const int    LONG_RANGE_L1_GUIDED_FULL_SEARCH_MAX_CORRIDOR_RADIUS            = 12;
    private const int    LONG_RANGE_PROXY_COARSE_REENTRY_MAX_DEPTH                       = 1;
    private const int    LONG_RANGE_GLOBAL_SEARCH_STEP_BUDGET                            = 1_500_000;
    private const float  LONG_RANGE_LATERAL_DESCENT_ENABLE_MIN_DROP_LEAF_CELLS           = 2f;
    private const float  LONG_RANGE_LATERAL_DESCENT_PRIORITY_DROP_LEAF_CELLS             = 18f;
    private const float  LONG_RANGE_LATERAL_DESCENT_PRIORITY_MAX_BONUS                   = 1.40f;
    private const float  LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_ATTEMPT_STEP             = 0.18f;
    private const float  LONG_RANGE_LATERAL_DIRECTIONAL_PENALTY_MIN                      = 0.50f;
    private const float  LONG_RANGE_LATERAL_GOAL_DISTANCE_HEURISTIC_WEIGHT               = 0.32f;
    private const float  LONG_RANGE_LATERAL_FORWARD_REMAINING_HEURISTIC_WEIGHT           = 0.70f;
    private const float  LONG_RANGE_LATERAL_SIDE_OFFSET_HEURISTIC_WEIGHT                 = 0.16f;
    private const float  LONG_RANGE_LATERAL_DESCENT_ABOVE_GOAL_HEURISTIC_WEIGHT          = 1.85f;
    private const float  LONG_RANGE_LATERAL_DESCENT_BELOW_GOAL_HEURISTIC_WEIGHT          = 0.55f;
    private const float  LONG_RANGE_LATERAL_ASCENT_BELOW_GOAL_HEURISTIC_WEIGHT           = 1.35f;
    private const float  LONG_RANGE_LATERAL_ASCENT_ABOVE_GOAL_HEURISTIC_WEIGHT           = 0.65f;
    private const float  LONG_RANGE_LATERAL_REVERSE_STEP_PENALTY                         = 0.80f;
    private const float  LONG_RANGE_LATERAL_LATERAL_STALL_PENALTY                        = 0.48f;
    private const float  LONG_RANGE_LATERAL_FORWARD_PROGRESS_CREDIT                      = 0.70f;
    private const float  LONG_RANGE_LATERAL_DESCENT_PROGRESS_CREDIT                      = 0.95f;
    private const float  LONG_RANGE_LATERAL_NON_DESCENT_STALL_SCALE                      = 0.60f;
    private const float  LONG_RANGE_L1_BEST_EFFORT_MIXED_CELL_PENALTY_SCALE              = 8.00f;
    private const float  LONG_RANGE_L1_BEST_EFFORT_DISTANCE_BUDGET_PER_CELL              = 3200f;
    private const float  LONG_RANGE_L1_BEST_EFFORT_VERTICAL_DISTANCE_BUDGET_SCALE        = 1.50f;
    private const float  LONG_RANGE_L1_BEST_EFFORT_RELAXED_BUDGET_SCALE                  = 1.10f;
    private const float  LONG_RANGE_L1_GOAL_CAPTURE_DISTANCE_THRESHOLD_L1_CELLS          = 18f;
    private const float  LONG_RANGE_L1_GOAL_CAPTURE_DIRECT_DISTANCE_RATIO                = 0.18f;
    private const float  LONG_RANGE_L1_GOAL_CAPTURE_BUDGET_PER_CELL                      = 16000f;
    private const float  LONG_RANGE_L1_GUIDED_FULL_SEARCH_GAP_RADIUS_SCALE               = 1.0f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_MIN_ABOVE_GOAL_LEAF_CELLS   = 1.5f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_HORIZONTAL_HEURISTIC_SCALE  = 0.82f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_HEURISTIC_SCALE    = 0.50f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_DESTINATION_PENALTY_SCALE   = 0.72f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_REQUIRED_DESCENT_LEAF_CELLS = 1.25f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_MISSED_DESCENT_PENALTY      = 0.95f;
    private const float  LONG_RANGE_LATERAL_DOWNWARD_OPENING_VERTICAL_MISS_PENALTY       = 1.35f;
    private const float  LONG_RANGE_LATERAL_COARSE_PROMOTION_MIN_FORWARD_DOT             = -0.15f;
    private const float  LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_LEAF_CELLS             = 3.0f;
    private const float  LONG_RANGE_LATERAL_VERTICAL_ACCESS_PROBE_HEIGHT_SCALE           = 0.35f;
    private const int    MAX_NODE_COUNT                                                  = 3_000_000;
    private const int    WALL_MASK_CACHE_MAX_SIZE                                        = 500_000;
    private const int    VERTICAL_ACCESS_CACHE_MAX_SIZE                                  = 200_000;
    private const int    L1_FACE_CACHE_MAX_SIZE                                          = 200_000;
    private const int    L1_FACE_TRANSITION_CACHE_MAX_SIZE                               = 200_000;
    private const int    VISIBILITY_CACHE_MAX_SIZE                                       = 1_000_000;
    private const int    L1_DISTANCE_FIELD_PRECOMPUTE_BUDGET                             = 200_000;
    private const float  REVISIT_PENALTY_SCALE                                           = 0.5f;
    private const int    MAX_PREVIOUSLY_VISITED                                          = 500_000;
    private const float  SHORT_RANGE_EXPLORATION_MIN_HORIZONTAL_LEAF_CELLS               = 8f;
    private const float  SHORT_RANGE_EXPLORATION_MIN_HORIZONTAL_DISTANCE                 = 4f;
    private const float  SHORT_RANGE_EXPLORATION_MIN_DROP_LEAF_CELLS                     = 4f;
    private const float  SHORT_RANGE_EXPLORATION_MIN_DROP_DISTANCE                       = 2f;
    private const float  GUIDED_CORRIDOR_MIN_HORIZONTAL_DISTANCE                         = 4.00f;
    private const float  GUIDED_CORRIDOR_HORIZONTAL_RADIUS_SCALE                         = 0.18f;
    private const float  GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_DISTANCE_SCALE            = 0.35f;
    private const float  GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MIN_LEAF_CELLS                = 6.00f;
    private const float  GUIDED_CORRIDOR_HORIZONTAL_RADIUS_MAX_LEAF_CELLS                = 20.00f;
    private const float  GUIDED_CORRIDOR_UPWARD_ALLOWANCE_DISTANCE_SCALE                 = 0.40f;
    private const float  GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_DISTANCE_SCALE               = 0.40f;
    private const float  GUIDED_CORRIDOR_UPWARD_ALLOWANCE_MIN_LEAF_CELLS                 = 12.00f;
    private const float  GUIDED_CORRIDOR_DOWNWARD_ALLOWANCE_MIN_LEAF_CELLS               = 12.00f;
    private const float  GUIDED_CORRIDOR_ENDPOINT_SLACK_RADIUS_SCALE                     = 0.65f;
    private const float  GUIDED_CORRIDOR_ENDPOINT_SLACK_MIN_LEAF_CELLS                   = 3.00f;
    private const float  GUIDED_CORRIDOR_OVERFLOW_PENALTY_SCALE                          = 4.0f;
    private const float  GUIDED_CORRIDOR_EARLY_ABORT_PROGRESS_RATIO                      = 0.04f;
    private const float  GUIDED_CORRIDOR_EARLY_ABORT_PROGRESS_MIN_DISTANCE               = 8.00f;
    private const float  GUIDED_CORRIDOR_EARLY_ABORT_SUFFICIENT_PROGRESS_RATIO           = 0.55f;
    private const float  GUIDED_CORRIDOR_EARLY_ABORT_VERTICAL_PROGRESS_LEAF_CELLS        = 2.0f;
    private const float  GUIDED_CORRIDOR_EARLY_ABORT_VERTICAL_PROGRESS_MIN_DISTANCE      = 1.0f;
    private const byte   L1_FACE_NEG_Y                                                   = 0;
    private const byte   L1_FACE_POS_Y                                                   = 1;
    private const byte   L1_FACE_NEG_X                                                   = 2;
    private const byte   L1_FACE_POS_X                                                   = 3;
    private const byte   L1_FACE_NEG_Z                                                   = 4;
    private const byte   L1_FACE_POS_Z                                                   = 5;
    private const byte   L1_FACE_INSIDE                                                  = byte.MaxValue;
    private const ushort L1_ALL_FACES_MASK                                               = 0x3f;
    private const byte   SEARCH_WALL_NEG_X                                               = 1 << 0;
    private const byte   SEARCH_WALL_POS_X                                               = 1 << 1;
    private const byte   SEARCH_WALL_NEG_Y                                               = 1 << 2;
    private const byte   SEARCH_WALL_POS_Y                                               = 1 << 3;
    private const byte   SEARCH_WALL_NEG_Z                                               = 1 << 4;
    private const byte   SEARCH_WALL_POS_Z                                               = 1 << 5;
    private const float  FLIGHT_PUSH_SCAN_DISTANCE_SCALE                                 = 6.00f;
    private const float  FLIGHT_PUSH_SCAN_DISTANCE_MAX_IN_LEAF_CELLS                     = 18.00f;
    private const float  FLIGHT_PUSH_SCAN_PUSH_FRACTION                                  = 0.62f;
    private const float  FLIGHT_PUSH_MAX_CLEARANCE_FRACTION                              = 0.72f;
    private const float  FLIGHT_PUSH_MIN_DISTANCE                                        = 0.02f;
    private const float  FLIGHT_PUSH_VOXEL_INSET_RATIO                                   = 0.10f;
    private const float  FLIGHT_PUSH_VOXEL_INSET_MIN                                     = 0.01f;
    private const float  FLIGHT_PUSH_VOXEL_INSET_MAX                                     = 0.08f;
    private const float  FLIGHT_PUSH_PREFERRED_CLEARANCE_VOXEL_SCALE                     = 0.85f;
    private const float  FLIGHT_PUSH_PREFERRED_CLEARANCE_LEAF_SCALE                      = 1.60f;
    private const float  FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_VOXEL_SCALE               = 0.85f;
    private const float  FLIGHT_PUSH_PREFERRED_FLOOR_CLEARANCE_LEAF_SCALE                = 1.60f;
    private const float  FLIGHT_PUSH_RELIEF_SCALE                                        = 0.90f;
    private const int    REFINE_RELAX_ITERATION_LIMIT                                    = 6;
    private const float  FLIGHT_DESCENT_SMOOTHING_MAX_SLOPE                              = 0.90f;
    private const float  FLIGHT_DESCENT_SMOOTHING_MIN_DROP_LEAF_SCALE                    = 2.50f;
    private const float  FLIGHT_DESCENT_SMOOTHING_MIN_DROP_MIN                           = 1.00f;
    private const float  FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_LEAF_SCALE               = 2.00f;
    private const float  FLIGHT_DESCENT_SMOOTHING_NEAR_VERTICAL_MIN                      = 1.20f;
    private const int    REFINE_RELAX_EARLY_EXIT_MIN_CHANGED                                = 0;
    private const int    PATH_LOS_CACHE_MAX_SIZE                                            = 200_000;
    private const int    CLEARANCE_CACHE_MAX_SIZE                                           = 100_000;

    private struct VoxelNodeLookup
    {
        private ulong[] keys;
        private int[]   values;
        private int     mask;

        public int Count { get; private set; }

        public VoxelNodeLookup(int capacity)
        {
            var size = VoxelMathUtil.RoundUpPowerOf2(Math.Max(capacity, 16));
            keys   = new ulong[size];
            values = new int[size];
            mask   = size - 1;
            Array.Fill(keys,   VoxelMap.INVALID_VOXEL);
            Array.Fill(values, -1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetValue(ulong key, out int value)
        {
            var idx = (int)(Mix(key) & (uint)mask);

            while (true)
            {
                var k = keys[idx];

                if (k == key)
                {
                    value = values[idx];
                    return true;
                }

                if (k == VoxelMap.INVALID_VOXEL)
                {
                    value = -1;
                    return false;
                }

                idx = (idx + 1) & mask;
            }
        }

        public void Set(ulong key, int value)
        {
            if (Count >= (keys.Length * 3) >> 2)
                Grow();
            SetInternal(keys, values, mask, key, value);
            ++Count;
        }

        public void SetOrUpdate(ulong key, int value)
        {
            if (Count >= (keys.Length * 3) >> 2)
                Grow();
            var idx = (int)(Mix(key) & (uint)mask);

            while (true)
            {
                var k = keys[idx];

                if (k == key)
                {
                    values[idx] = value;
                    return;
                }

                if (k == VoxelMap.INVALID_VOXEL)
                {
                    keys[idx]   = key;
                    values[idx] = value;
                    ++Count;
                    return;
                }

                idx = (idx + 1) & mask;
            }
        }

        public void Clear()
        {
            if (Count == 0) return;
            Array.Fill(keys,   VoxelMap.INVALID_VOXEL);
            Array.Fill(values, -1);
            Count = 0;
        }

        public void TrimExcess()
        {
            if (keys.Length <= 2048) return;
            var size = VoxelMathUtil.RoundUpPowerOf2(Math.Max(Count * 2, 16));
            if (size >= keys.Length) return;
            var newKeys   = new ulong[size];
            var newValues = new int[size];
            var newMask   = size - 1;
            Array.Fill(newKeys,   VoxelMap.INVALID_VOXEL);
            Array.Fill(newValues, -1);
            for (var i = 0; i < keys.Length; ++i)
                if (keys[i] != VoxelMap.INVALID_VOXEL)
                    SetInternal(newKeys, newValues, newMask, keys[i], values[i]);
            keys   = newKeys;
            values = newValues;
            mask   = newMask;
        }

        public ClosedVoxelEnumerator GetClosedEnumerator(Span<VolumePathfindNode> nodeSpan) => new(keys, values, nodeSpan);

        private void Grow()
        {
            var newSize   = keys.Length << 1;
            var newKeys   = new ulong[newSize];
            var newValues = new int[newSize];
            var newMask   = newSize - 1;
            Array.Fill(newKeys,   VoxelMap.INVALID_VOXEL);
            Array.Fill(newValues, -1);
            for (var i = 0; i < keys.Length; ++i)
                if (keys[i] != VoxelMap.INVALID_VOXEL)
                    SetInternal(newKeys, newValues, newMask, keys[i], values[i]);
            keys   = newKeys;
            values = newValues;
            mask   = newMask;
        }

        private static void SetInternal(ulong[] keys, int[] values, int mask, ulong key, int value)
        {
            var idx = (int)(Mix(key) & (uint)mask);
            while (keys[idx] != VoxelMap.INVALID_VOXEL)
                idx = (idx + 1) & mask;
            keys[idx]   = key;
            values[idx] = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static uint Mix(ulong key)
        {
            key ^= key >> 33;
            key *= 0xff51afd7ed558ccdUL;
            key ^= key >> 33;
            return (uint)key;
        }

        public ref struct ClosedVoxelEnumerator
        (
            ulong[]                  keys,
            int[]                    values,
            Span<VolumePathfindNode> nodeSpan
        )
        {
            private readonly Span<VolumePathfindNode> nodeSpan = nodeSpan;
            private          int                      index    = -1;

            public ulong Current => keys[index];

            public bool MoveNext()
            {
                while (++index < keys.Length)
                {
                    if (keys[index] != VoxelMap.INVALID_VOXEL && nodeSpan[values[index]].Closed)
                        return true;
                }

                return false;
            }
        }
    }
}
