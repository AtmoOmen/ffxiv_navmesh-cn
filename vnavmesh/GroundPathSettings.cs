using System;

namespace Navmesh;

public readonly record struct GroundPathSettings(
    float Clearance,
    float CenterBias,
    int RelaxIterations = 3,
    int MaxLookAhead = 6,
    float MinPointSpacing = 0.35f,
    float CollinearSlack = 0.05f)
{
    public static GroundPathSettings FromConfig(Config config) => new(
        Math.Clamp(config.GroundPathClearance, 0.05f, 5f),
        Math.Clamp(config.GroundPathCenterBias, 0f, 1f));
}
