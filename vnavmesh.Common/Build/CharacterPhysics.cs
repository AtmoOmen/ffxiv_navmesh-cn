namespace vnavmesh.Common.Build;

/// <summary>
///     角色运动物理常量
/// </summary>
public static class CharacterPhysics
{
    /// <summary>
    ///     基础跑速，单位为每秒。
    /// </summary>
    public const float MOVE_SPEED = 6f;

    /// <summary>
    ///     疾跑相对基础跑速的倍率。游戏内加速模式提升 30%。
    /// </summary>
    public const float SPRINT_SPEED_MULTIPLIER = 1.3f;

    /// <summary>
    ///     重力加速度，单位为每秒平方。
    /// </summary>
    public const float GRAVITY = 30f;

    /// <summary>
    ///     跳跃的垂直初速度，单位为每秒。对应约 1.8 米的跳跃高度。
    /// </summary>
    public const float JUMP_SPEED = 10.4f;
}
