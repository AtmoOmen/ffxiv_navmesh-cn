using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using AABB = vnavmesh.Common.Models.AABB;
using GameAABB = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB;
using GameMatrix4x3 = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.Matrix4x3;
using Matrix4x3 = vnavmesh.Common.Models.Matrix4x3;

namespace vnavmesh.Build.Scene;

public static class SceneMathConversion
{
    public static AABB ToCommon
    (
        this GameAABB bounds
    ) =>
        new(bounds.Min, bounds.Max);

    public static GameAABB ToGame
    (
        this AABB bounds
    ) =>
        new() { Min = bounds.Min, Max = bounds.Max };

    public static Matrix4x3 ToCommon
    (
        this GameMatrix4x3 matrix
    ) =>
        new(matrix.Row0, matrix.Row1, matrix.Row2, matrix.Row3);

    public static GameMatrix4x3 ToGame
    (
        this Matrix4x3 matrix
    ) =>
        new() { Row0 = matrix.Row0, Row1 = matrix.Row1, Row2 = matrix.Row2, Row3 = matrix.Row3 };
}
