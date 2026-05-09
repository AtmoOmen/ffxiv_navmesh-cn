using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Game.Config;
using Dalamud.Hooking;
using Dalamud.Utility.Signatures;
using FFXIVClientStructs.FFXIV.Client.Game.Control;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Models;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Interop;

[StructLayout(LayoutKind.Explicit, Size = 0x18)]
public struct PlayerMoveControllerFlyInput
{
    [FieldOffset(0x0)]
    public float Forward;

    [FieldOffset(0x4)]
    public float Left;

    [FieldOffset(0x8)]
    public float Up;

    [FieldOffset(0xC)]
    public float Turn;

    [FieldOffset(0x10)]
    public float u10;

    [FieldOffset(0x14)]
    public byte DirMode;

    [FieldOffset(0x15)]
    public byte HaveBackwardOrStrafe;
}

public unsafe class MovementInputController : IDisposable
{
    public bool Enabled
    {
        get => rmiWalkHook.IsEnabled;
        set
        {
            if (value)
            {
                rmiWalkHook.Enable();
                rmiFlyHook.Enable();
            }
            else
            {
                UserInput = false;
                rmiWalkHook.Disable();
                rmiFlyHook.Disable();
            }
        }
    }

    public bool    IgnoreUserInput      { get; set; }
    public bool    AllowVerticalControl { get; set; } = true;
    public bool    EnableFacingAlign    { get; set; }
    public Vector3 DesiredPosition      { get; set; }
    public Angle   DesiredFacing        { get; set; }
    public float   Precision            { get; set; } = 0.01f;
    public float   FacingPrecisionRad   { get; set; } = 1.Degrees().Rad;
    public bool    UserInput            { get; private set; }

    private bool legacyMode;

    private const float FULL_TURN_INPUT_ANGLE = MathF.PI / 4;

    private delegate bool RMIWalkIsInputEnabled(void* self);

    [Signature("E8 ?? ?? ?? ?? 84 C0 75 10 38 43 3C")]
    private readonly RMIWalkIsInputEnabled rmiWalkIsInputEnabled1 = null!;

    [Signature("E8 ?? ?? ?? ?? 84 C0 75 03 88 47 3F")]
    private readonly RMIWalkIsInputEnabled rmiWalkIsInputEnabled2 = null!;

    private delegate void RMIWalkDelegate
        (void* self, float* sumLeft, float* sumForward, float* sumTurnLeft, byte* haveBackwardOrStrafe, byte* a6, byte bAdditiveUnk);

    [Signature("E8 ?? ?? ?? ?? 80 7B 3E 00 48 8D 3D", DetourName = nameof(RMIWalkDetour))]
    private Hook<RMIWalkDelegate> rmiWalkHook = null!;

    private delegate void RMIFlyDelegate(void* self, PlayerMoveControllerFlyInput* result);

    [Signature("E8 ?? ?? ?? ?? 0F B6 0D ?? ?? ?? ?? B8", DetourName = nameof(RMIFlyDetour))]
    private Hook<RMIFlyDelegate> rmiFlyHook = null!;

    public MovementInputController()
    {
        Service.Hook.InitializeFromAttributes(this);
        Service.GameConfig.UiControlChanged += OnConfigChanged;
        UpdateLegacyMode();
    }

    public void Dispose()
    {
        Service.GameConfig.UiControlChanged -= OnConfigChanged;
        rmiWalkHook.Dispose();
        rmiFlyHook.Dispose();
    }

    private void RMIWalkDetour(void* self, float* sumLeft, float* sumForward, float* sumTurnLeft, byte* haveBackwardOrStrafe, byte* a6, byte bAdditiveUnk)
    {
        rmiWalkHook.Original(self, sumLeft, sumForward, sumTurnLeft, haveBackwardOrStrafe, a6, bAdditiveUnk);
        var movementAllowed = bAdditiveUnk == 0 && rmiWalkIsInputEnabled1(self) && rmiWalkIsInputEnabled2(self);
        UserInput = *sumLeft != 0 || *sumForward != 0;

        if (movementAllowed && (IgnoreUserInput || *sumLeft == 0 && *sumForward == 0) && DirectionToDestination(false) is { } relDir)
        {
            var dir = relDir.h.ToDirection();
            *sumLeft    = dir.X;
            *sumForward = dir.Y;
        }

        if (movementAllowed && (IgnoreUserInput || *sumTurnLeft == 0) && ResolveFacingTurnInput() is { } turnInput)
            *sumTurnLeft = turnInput;
    }

    private void RMIFlyDetour(void* self, PlayerMoveControllerFlyInput* result)
    {
        rmiFlyHook.Original(self, result);
        UserInput = result->Forward != 0 || result->Left != 0 || result->Up != 0;

        if ((IgnoreUserInput || result->Forward == 0 && result->Left == 0 && result->Up == 0) && DirectionToDestination(AllowVerticalControl) is { } relDir)
        {
            var dir = relDir.h.ToDirection();
            result->Forward = dir.Y;
            result->Left    = dir.X;
            result->Up      = AllowVerticalControl ? relDir.v.Rad : 0;
        }

        if ((IgnoreUserInput || result->Turn == 0) && ResolveFacingTurnInput() is { } turnInput)
            result->Turn = turnInput;
    }

    private (Angle h, Angle v)? DirectionToDestination(bool allowVertical)
    {
        var player = Service.ObjectTable.LocalPlayer;
        if (player == null)
            return null;

        var dist = DesiredPosition - player.Position;
        if (dist.LengthSquared() <= Precision * Precision)
            return null;

        var dirH = Angle.FromDirectionXZ(dist);
        var dirV = allowVertical ? Angle.FromDirection(new(dist.Y, new Vector2(dist.X, dist.Z).Length())) : default;
        var refDir = legacyMode
                         ? ((CameraEx*)CameraManager.Instance()->GetActiveCamera())->DirH.Radians() + 180.Degrees()
                         : player.Rotation.Radians();
        return (dirH - refDir, dirV);
    }

    private float? ResolveFacingTurnInput()
    {
        if (!EnableFacingAlign)
            return null;

        var player = Service.ObjectTable.LocalPlayer;
        if (player == null)
            return null;

        var delta = (DesiredFacing - player.Rotation.Radians()).Normalized().Rad;
        if (MathF.Abs(delta) <= FacingPrecisionRad)
            return 0;

        return Math.Clamp(delta / FULL_TURN_INPUT_ANGLE, -1f, 1f);
    }

    private void OnConfigChanged(object? sender, ConfigChangeEvent evt) => UpdateLegacyMode();

    private void UpdateLegacyMode() => legacyMode = Service.GameConfig.UiControl.TryGetUInt("MoveMode", out var mode) && mode == 1;
}
