using System.Numerics;
using vnavmesh.Common.Models;

namespace vnavmesh.Movement.Execution;

internal readonly record struct MovementFrameCommand
(
    Vector3 DesiredPosition,
    bool    MovementEnabled,
    bool    AllowVerticalControl,
    bool    EnableCameraAlign,
    Angle   DesiredAzimuth,
    Angle   DesiredAltitude,
    bool    RequestJump,
    bool    EnableFacingAlign,
    Angle   DesiredFacing
);
