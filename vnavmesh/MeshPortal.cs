using System;
using DotRecast.Core.Numerics;

namespace Navmesh;

public readonly record struct MeshPortal(long FromRef, long ToRef, RcVec3f Left, RcVec3f Right, float EffectiveClearance = 0, bool IsNarrow = false, bool IsProtectedAnchor = false)
{
    private const float PointThreshold = 0.001f;
    private const float NarrowRatio = 0.8f;
    private const float ProtectedRatio = 0.6f;
    private const float ClearanceEpsilon = 0.01f;

    public RcVec3f Mid => RcVec3f.Lerp(Left, Right, 0.5f);
    public float Width => RcVec3f.Distance(Left, Right);
    public bool IsPoint => Width <= PointThreshold;

    public MeshPortal Trim(float clearance)
    {
        if (IsPoint)
            return this with { EffectiveClearance = 0, IsNarrow = true, IsProtectedAnchor = true };

        var width = Width;
        var maxClearance = MathF.Max(0, width * 0.5f - ClearanceEpsilon);
        var effectiveClearance = MathF.Min(clearance, maxClearance);
        var isNarrow = effectiveClearance < clearance * NarrowRatio;
        var isProtected = effectiveClearance < clearance * ProtectedRatio || width <= clearance * 1.5f;

        if (maxClearance <= PointThreshold)
        {
            var mid = Mid;
            return new(FromRef, ToRef, mid, mid, effectiveClearance, true, true);
        }

        var dir = (Right - Left) * (1 / width);
        return new(FromRef, ToRef, Left + dir * effectiveClearance, Right - dir * effectiveClearance, effectiveClearance, isNarrow, isProtected);
    }
}
