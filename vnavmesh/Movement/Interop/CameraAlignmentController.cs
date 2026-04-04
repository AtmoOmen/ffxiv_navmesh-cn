using System.Runtime.InteropServices;
using Dalamud.Hooking;
using Dalamud.Utility.Signatures;
using FFXIVClientStructs.FFXIV.Client.System.Framework;
using vnavmesh.Bootstrap;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Interop;

[StructLayout(LayoutKind.Explicit, Size = 0x2B0)]
public struct CameraEx
{
    [FieldOffset(0x140)]
    public float DirH;

    [FieldOffset(0x144)]
    public float DirV;

    [FieldOffset(0x148)]
    public float InputDeltaHAdjusted;

    [FieldOffset(0x14C)]
    public float InputDeltaVAdjusted;

    [FieldOffset(0x150)]
    public float InputDeltaH;

    [FieldOffset(0x154)]
    public float InputDeltaV;

    [FieldOffset(0x158)]
    public float DirVMin;

    [FieldOffset(0x15C)]
    public float DirVMax;
}

public unsafe class CameraAlignmentController : IDisposable
{
    public bool Enabled
    {
        get => rmiCameraHook.IsEnabled;
        set
        {
            if (value == rmiCameraHook.IsEnabled)
                return;

            ResetSmoothing();
            if (value)
                rmiCameraHook.Enable();
            else
                rmiCameraHook.Disable();
        }
    }

    public bool  IgnoreUserInput { get; set; }
    public Angle DesiredAzimuth  { get; set; }
    public Angle DesiredAltitude { get; set; }
    public Angle SpeedH          { get; set; } = 360.Degrees();
    public Angle SpeedV          { get; set; } = 360.Degrees();
    public bool  EnableSmoothing { get; set; } = true;
    public float SmoothTimeH     { get; set; } = 0.25f;
    public float SmoothTimeV     { get; set; } = 0.25f;

    private delegate void RMICameraDelegate(CameraEx* self, int inputMode, float speedH, float speedV);

    [Signature("48 8B C4 53 48 81 EC ?? ?? ?? ?? 44 0F 29 50 ??", DetourName = nameof(RMICameraDetour))]
    private Hook<RMICameraDelegate> rmiCameraHook = null!;

    private float velocityH;
    private float velocityV;

    public CameraAlignmentController()
    {
        Service.Hook.InitializeFromAttributes(this);
        Service.Log.Information($"RMICamera address: 0x{rmiCameraHook.Address:X}");
    }

    public void Dispose() =>
        rmiCameraHook.Dispose();

    private void ResetSmoothing()
    {
        velocityH = 0;
        velocityV = 0;
    }

    private void RMICameraDetour(CameraEx* self, int inputMode, float speedH, float speedV)
    {
        rmiCameraHook.Original(self, inputMode, speedH, speedV);

        if (IgnoreUserInput || inputMode == 0)
        {
            var dt = Framework.Instance()->FrameDeltaTime;
            if (dt <= 0)
                return;

            var deltaH = (DesiredAzimuth  - self->DirH.Radians()).Normalized();
            var deltaV = (DesiredAltitude - self->DirV.Radians()).Normalized();
            var maxH   = SpeedH.Rad * dt;
            var maxV   = SpeedV.Rad * dt;

            if (!EnableSmoothing || SmoothTimeH <= 0 && SmoothTimeV <= 0)
            {
                self->InputDeltaH = Math.Clamp(deltaH.Rad, -maxH, maxH);
                self->InputDeltaV = Math.Clamp(deltaV.Rad, -maxV, maxV);
                return;
            }

            var desiredH = self->DirH + deltaH.Rad;
            var desiredV = Math.Clamp(self->DirV + deltaV.Rad, self->DirVMin, self->DirVMax);
            var newH     = SmoothDamp(self->DirH, desiredH, ref velocityH, SmoothTimeH, SpeedH.Rad, dt);
            var newV     = SmoothDamp(self->DirV, desiredV, ref velocityV, SmoothTimeV, SpeedV.Rad, dt);

            self->InputDeltaH = Math.Clamp(newH - self->DirH, -maxH, maxH);
            self->InputDeltaV = Math.Clamp(newV - self->DirV, -maxV, maxV);
        }
    }

    private static float SmoothDamp(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed, float deltaTime)
    {
        if (smoothTime <= 0)
            return target;

        smoothTime = Math.Max(0.0001f, smoothTime);
        var omega      = 2f    / smoothTime;
        var x          = omega * deltaTime;
        var exp        = 1f    / (1f + x + 0.48f * x * x + 0.235f * x * x * x);
        var change     = current - target;
        var originalTo = target;
        var maxChange  = maxSpeed * smoothTime;
        change = Math.Clamp(change, -maxChange, maxChange);
        target = current - change;

        var temp = (currentVelocity + omega * change) * deltaTime;
        currentVelocity = (currentVelocity - omega           * temp) * exp;
        var output = target                + (change + temp) * exp;

        var origMinusCurrent = originalTo - current;
        var outMinusOrig     = output     - originalTo;

        if (origMinusCurrent > 0 == outMinusOrig > 0)
        {
            output          = originalTo;
            currentVelocity = (output - originalTo) / deltaTime;
        }

        return output;
    }
}
