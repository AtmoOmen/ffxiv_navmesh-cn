using Dalamud.Hooking;
using Dalamud.Utility.Signatures;
using FFXIVClientStructs.FFXIV.Client.System.Framework;
using System;
using System.Runtime.InteropServices;

namespace Navmesh.Movement;

[StructLayout(LayoutKind.Explicit, Size = 0x2B0)]
public unsafe struct CameraEx
{
    [FieldOffset(0x140)] public float DirH; // 0 is north, increases CW
    [FieldOffset(0x144)] public float DirV; // 0 is horizontal, positive is looking up, negative looking down
    [FieldOffset(0x148)] public float InputDeltaHAdjusted;
    [FieldOffset(0x14C)] public float InputDeltaVAdjusted;
    [FieldOffset(0x150)] public float InputDeltaH;
    [FieldOffset(0x154)] public float InputDeltaV;
    [FieldOffset(0x158)] public float DirVMin; // -85deg by default
    [FieldOffset(0x15C)] public float DirVMax; // +45deg by default
}

public unsafe class OverrideCamera : IDisposable
{
    public bool Enabled
    {
        get => _rmiCameraHook.IsEnabled;
        set
        {
            if (value == _rmiCameraHook.IsEnabled)
                return;

            ResetSmoothing();
            if (value)
                _rmiCameraHook.Enable();
            else
                _rmiCameraHook.Disable();
        }
    }

    public bool IgnoreUserInput; // if true - override even if user tries to change camera orientation, otherwise override only if user does nothing
    public Angle DesiredAzimuth;
    public Angle DesiredAltitude;
    public Angle SpeedH = 360.Degrees(); // per second
    public Angle SpeedV = 360.Degrees(); // per second
    public bool EnableSmoothing = true;
    public float SmoothTimeH = 0.25f;
    public float SmoothTimeV = 0.25f;

    private delegate void RMICameraDelegate(CameraEx* self, int inputMode, float speedH, float speedV);
    [Signature("48 8B C4 53 48 81 EC ?? ?? ?? ?? 44 0F 29 50 ??")]
    private Hook<RMICameraDelegate> _rmiCameraHook = null!;

    private float velocityH;
    private float velocityV;

    public OverrideCamera()
    {
        Service.Hook.InitializeFromAttributes(this);
        Service.Log.Information($"RMICamera address: 0x{_rmiCameraHook.Address:X}");
    }

    public void Dispose()
    {
        _rmiCameraHook.Dispose();
    }

    private void ResetSmoothing()
    {
        velocityH = 0;
        velocityV = 0;
    }

    private void RMICameraDetour(CameraEx* self, int inputMode, float speedH, float speedV)
    {
        _rmiCameraHook.Original(self, inputMode, speedH, speedV);
        if (IgnoreUserInput || inputMode == 0) // let user override...
        {
            var dt = Framework.Instance()->FrameDeltaTime;
            if (dt <= 0)
                return;

            var deltaH = (DesiredAzimuth - self->DirH.Radians()).Normalized();
            var deltaV = (DesiredAltitude - self->DirV.Radians()).Normalized();

            var maxH = SpeedH.Rad * dt;
            var maxV = SpeedV.Rad * dt;

            if (!EnableSmoothing || (SmoothTimeH <= 0 && SmoothTimeV <= 0))
            {
                self->InputDeltaH = Math.Clamp(deltaH.Rad, -maxH, maxH);
                self->InputDeltaV = Math.Clamp(deltaV.Rad, -maxV, maxV);
                return;
            }

            var desiredH = self->DirH + deltaH.Rad;
            var desiredV = Math.Clamp(self->DirV + deltaV.Rad, self->DirVMin, self->DirVMax);

            var newH = SmoothDamp(self->DirH, desiredH, ref velocityH, SmoothTimeH, SpeedH.Rad, dt);
            var newV = SmoothDamp(self->DirV, desiredV, ref velocityV, SmoothTimeV, SpeedV.Rad, dt);

            self->InputDeltaH = Math.Clamp(newH - self->DirH, -maxH, maxH);
            self->InputDeltaV = Math.Clamp(newV - self->DirV, -maxV, maxV);
        }
    }

    private static float SmoothDamp(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed, float deltaTime)
    {
        if (smoothTime <= 0)
            return target;

        smoothTime = Math.Max(0.0001f, smoothTime);
        var omega = 2f / smoothTime;

        var x = omega * deltaTime;
        var exp = 1f / (1f + x + 0.48f * x * x + 0.235f * x * x * x);

        var change = current - target;
        var originalTo = target;

        var maxChange = maxSpeed * smoothTime;
        change = Math.Clamp(change, -maxChange, maxChange);
        target = current - change;

        var temp = (currentVelocity + omega * change) * deltaTime;
        currentVelocity = (currentVelocity - omega * temp) * exp;

        var output = target + (change + temp) * exp;

        var origMinusCurrent = originalTo - current;
        var outMinusOrig = output - originalTo;
        if (origMinusCurrent > 0 == outMinusOrig > 0)
        {
            output = originalTo;
            currentVelocity = (output - originalTo) / deltaTime;
        }

        return output;
    }
}
