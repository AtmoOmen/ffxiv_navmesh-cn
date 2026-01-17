using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using FFXIVClientStructs.FFXIV.Client.Game;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Navmesh.Movement;

public class FollowPath : IDisposable
{
    public bool MovementAllowed = true;
    public bool IgnoreDeltaY = false;
    public float Tolerance = 0.25f;
    public float DestinationTolerance = 0;
    public float ImplicitDestinationTolerance = 0.25f;
    public float LookAheadDistance = 1f;
    public List<Vector3> Waypoints = new();

    private IDalamudPluginInterface _dalamud;
    private NavmeshManager _manager;
    private OverrideCamera _camera = new();
    private OverrideMovement _movement = new();
    private DateTime _nextJump;

    private Vector3? posPreviousFrame;

    private int _millisecondsWithNoSignificantMovement = 0;

    public event Action<Vector3, bool, float>? OnStuck;

    // entries in dalamud shared data cache must be reference types, so we use an array
    private readonly bool[] _sharedPathIsRunning;
    private bool sharedPathIsRunningValue;

    private const string _sharedPathTag = "vnav.PathIsRunning";

    public FollowPath(IDalamudPluginInterface dalamud, NavmeshManager manager)
    {
        _dalamud = dalamud;
        _sharedPathIsRunning = _dalamud.GetOrCreateData<bool[]>(_sharedPathTag, () => [false]);
        sharedPathIsRunningValue = _sharedPathIsRunning[0];
        _manager = manager;
        _manager.OnNavmeshChanged += OnNavmeshChanged;
        OnNavmeshChanged(_manager.Navmesh, _manager.Query);
    }

    public void Dispose()
    {
        UpdateSharedState(false);
        _dalamud.RelinquishData(_sharedPathTag);
        _manager.OnNavmeshChanged -= OnNavmeshChanged;
        _camera.Dispose();
        _movement.Dispose();
    }

    private void UpdateSharedState(bool isRunning)
    {
        if (sharedPathIsRunningValue == isRunning)
            return;

        sharedPathIsRunningValue = isRunning;
        _sharedPathIsRunning[0] = isRunning;
    }

    public void Update(IFramework fwk)
    {
        var player = Service.ObjectTable.LocalPlayer;
        if (player == null)
            return;

        var playerPosition = player.Position;

        var destinationTolerance = GetDestinationTolerance();
        if (Waypoints.Count > 0 && destinationTolerance > 0 && DistanceSquared(playerPosition, Waypoints[^1], IgnoreDeltaY) <= destinationTolerance * destinationTolerance)
            Waypoints.Clear();

        AdvanceWaypoints(playerPosition);

        UpdateSharedState(Waypoints.Count > 0);

        if (Waypoints.Count == 0)
        {
            posPreviousFrame = playerPosition;
            _movement.Enabled = _camera.Enabled = false;
            _camera.SpeedH = _camera.SpeedV = default;
            _movement.DesiredPosition = playerPosition;
        }
        else
        {
            if (Service.Config.CancelMoveOnUserInput && _movement.UserInput)
            {
                Stop();
                return;
            }

            var desiredPosition = SelectDesiredPosition(playerPosition);

            var isTryingToTakeOff = !IgnoreDeltaY
                && desiredPosition.Y > playerPosition.Y
                && !Service.Condition[Dalamud.Game.ClientState.Conditions.ConditionFlag.InFlight]
                && !Service.Condition[Dalamud.Game.ClientState.Conditions.ConditionFlag.Diving];

            OverrideAFK.ResetTimers();
            _movement.Enabled = MovementAllowed;
            _movement.DesiredPosition = desiredPosition;

            if (isTryingToTakeOff) //Only do this bit if on a flying path
            {
                // walk->fly transition (TODO: reconsider?)
                if (Service.Condition[Dalamud.Game.ClientState.Conditions.ConditionFlag.Mounted])
                    ExecuteJump(); // Spam jump to take off
                else
                {
                    _movement.Enabled = false; // Don't move, since it'll just run on the spot
                    posPreviousFrame = playerPosition;
                    _millisecondsWithNoSignificantMovement = 0;
                    return;
                }
            }

            _camera.Enabled = Service.Config.AlignCameraToMovement;
            _camera.SpeedH = _camera.SpeedV = 360.Degrees();
            _camera.DesiredAzimuth = Angle.FromDirectionXZ(_movement.DesiredPosition - playerPosition) + 180.Degrees();
            _camera.DesiredAltitude = Service.Config.AlignCameraHeight.Degrees();

            if (!MovementAllowed)
                _millisecondsWithNoSignificantMovement = 0;

            if (Service.Config.StopOnStuck && MovementAllowed && posPreviousFrame.HasValue)
            {
                var seconds = fwk.UpdateDelta.TotalSeconds;
                if (seconds > 0)
                {
                    var dispSquared = DistanceSquared(playerPosition, posPreviousFrame.Value, IgnoreDeltaY);
                    var speedSquared = dispSquared / (float)(seconds * seconds);
                    var toleranceSquared = Service.Config.StuckTolerance * Service.Config.StuckTolerance;

                    if (speedSquared <= toleranceSquared)
                        _millisecondsWithNoSignificantMovement += (int)fwk.UpdateDelta.TotalMilliseconds;
                    else
                        _millisecondsWithNoSignificantMovement = 0;

                    if (_millisecondsWithNoSignificantMovement >= Service.Config.StuckTimeoutMs)
                    {
                        var destination = Waypoints[^1];
                        Stop();
                        OnStuck?.Invoke(destination, !IgnoreDeltaY, DestinationTolerance);
                        return;
                    }
                }
            }

            posPreviousFrame = playerPosition;
        }
    }

    private void AdvanceWaypoints(Vector3 playerPosition)
    {
        if (Waypoints.Count == 0)
            return;

        var destinationTolerance = GetDestinationTolerance();
        if (Waypoints.Count == 1 && destinationTolerance > 0 && DistanceSquared(playerPosition, Waypoints[0], IgnoreDeltaY) <= destinationTolerance * destinationTolerance)
        {
            Waypoints.Clear();
            return;
        }

        var waypointPassTolerance = GetWaypointPassTolerance();
        var waypointPassToleranceSquared = waypointPassTolerance * waypointPassTolerance;

        var toleranceSquared = Tolerance * Tolerance;
        var prev = posPreviousFrame ?? playerPosition;

        while (Waypoints.Count > 1)
        {
            var waypoint = Waypoints[0];

            if (DistanceSquared(playerPosition, waypoint, IgnoreDeltaY) <= toleranceSquared)
            {
                Waypoints.RemoveAt(0);
                continue;
            }

            var distanceToNextSegmentSquared = IgnoreDeltaY
                ? DistanceToLineSegmentSquaredXZ(playerPosition, Waypoints[0], Waypoints[1])
                : DistanceToLineSegmentSquared(playerPosition, Waypoints[0], Waypoints[1]);
            if (distanceToNextSegmentSquared <= waypointPassToleranceSquared && IsPastWaypointOnSegment(playerPosition, Waypoints[0], Waypoints[1], IgnoreDeltaY))
            {
                Waypoints.RemoveAt(0);
                continue;
            }

            var distToSegmentSquared = IgnoreDeltaY
                ? DistanceToLineSegmentSquaredXZ(waypoint, playerPosition, prev)
                : DistanceToLineSegmentSquared(waypoint, playerPosition, prev);

            if (distToSegmentSquared > toleranceSquared)
                break;

            Waypoints.RemoveAt(0);
        }

        SkipWaypointsBehind(playerPosition, waypointPassToleranceSquared);
    }

    public void Stop()
    {
        UpdateSharedState(false);
        _millisecondsWithNoSignificantMovement = 0;
        Waypoints.Clear();
    }

    private unsafe void ExecuteJump()
    {
        // Unable to jump while diving, prevents spamming error messages.
        if (Service.Condition[Dalamud.Game.ClientState.Conditions.ConditionFlag.Diving])
            return;

        if (DateTime.Now >= _nextJump)
        {
            ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
            _nextJump = DateTime.Now.AddMilliseconds(100);
        }
    }

    public void Move(List<Vector3> waypoints, bool ignoreDeltaY, float destTolerance = 0)
    {
        if (waypoints.Count == 0)
        {
            Stop();
            return;
        }

        _millisecondsWithNoSignificantMovement = 0;
        posPreviousFrame = null;
        UpdateSharedState(true);
        waypoints[0] = Service.ObjectTable.LocalPlayer?.Position ?? default;
        Waypoints = waypoints;
        IgnoreDeltaY = ignoreDeltaY;
        DestinationTolerance = destTolerance;
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query)
    {
        UpdateSharedState(false);
        Waypoints.Clear();
    }

    private float GetDestinationTolerance()
    {
        if (DestinationTolerance > 0)
            return DestinationTolerance;

        if (ImplicitDestinationTolerance > 0)
            return ImplicitDestinationTolerance;

        return Tolerance;
    }

    private float GetWaypointPassTolerance() => MathF.Max(Tolerance, GetDestinationTolerance());

    private Vector3 SelectDesiredPosition(Vector3 playerPosition)
    {
        if (Waypoints.Count == 0)
            return playerPosition;

        if (Waypoints.Count == 1)
            return Waypoints[0];

        var lookAhead = LookAheadDistance;
        if (lookAhead <= 0)
            return Waypoints[0];

        var remaining = lookAhead;
        var from = playerPosition;
        var startIndex = 0;

        var waypointPassTolerance = GetWaypointPassTolerance();
        var waypointPassToleranceSquared = waypointPassTolerance * waypointPassTolerance;
        var startSegmentDistanceSquared = IgnoreDeltaY
            ? DistanceToLineSegmentSquaredXZ(playerPosition, Waypoints[0], Waypoints[1])
            : DistanceToLineSegmentSquared(playerPosition, Waypoints[0], Waypoints[1]);
        if (startSegmentDistanceSquared <= waypointPassToleranceSquared)
        {
            from = IgnoreDeltaY
                ? ClosestPointOnSegmentXZ(playerPosition, Waypoints[0], Waypoints[1])
                : ClosestPointOnSegment(playerPosition, Waypoints[0], Waypoints[1]);
            startIndex = 1;
        }

        for (int i = startIndex; i < Waypoints.Count; i++)
        {
            var to = Waypoints[i];
            var segmentLength = Distance(from, to, IgnoreDeltaY);
            if (segmentLength <= float.Epsilon)
            {
                from = to;
                continue;
            }

            if (remaining <= segmentLength)
                return LerpByDistance(from, to, remaining, IgnoreDeltaY);

            remaining -= segmentLength;
            from = to;
        }

        return Waypoints[^1];
    }

    private void SkipWaypointsBehind(Vector3 playerPosition, float waypointPassToleranceSquared)
    {
        while (Waypoints.Count > 1)
        {
            var a = Waypoints[0];
            var b = Waypoints[1];
            var ab = b - a;
            var ap = playerPosition - a;

            if (IgnoreDeltaY)
            {
                ab.Y = 0;
                ap.Y = 0;
            }

            var abLenSq = ab.LengthSquared();
            if (abLenSq <= float.Epsilon)
            {
                Waypoints.RemoveAt(0);
                continue;
            }

            var t = Vector3.Dot(ap, ab) / abLenSq;
            if (t < 0.1f)
                break;

            if (DistanceSquared(playerPosition, b, IgnoreDeltaY) > DistanceSquared(playerPosition, a, IgnoreDeltaY))
                break;

            var distanceToSegmentSquared = IgnoreDeltaY
                ? DistanceToLineSegmentSquaredXZ(playerPosition, a, b)
                : DistanceToLineSegmentSquared(playerPosition, a, b);
            if (distanceToSegmentSquared > waypointPassToleranceSquared)
                break;

            Waypoints.RemoveAt(0);
        }
    }

    private static bool IsPastWaypointOnSegment(Vector3 p, Vector3 a, Vector3 b, bool ignoreDeltaY)
    {
        var ab = b - a;
        var ap = p - a;

        if (ignoreDeltaY)
        {
            ab.Y = 0;
            ap.Y = 0;
        }

        var abLenSq = ab.LengthSquared();
        if (abLenSq <= float.Epsilon)
            return true;

        var t = Vector3.Dot(ap, ab) / abLenSq;
        return t >= 0;
    }

    private static Vector3 ClosestPointOnSegment(Vector3 p, Vector3 a, Vector3 b)
    {
        var ab = b - a;
        var ap = p - a;
        var abLenSq = ab.LengthSquared();
        if (abLenSq <= float.Epsilon)
            return a;

        var t = Math.Clamp(Vector3.Dot(ap, ab) / abLenSq, 0, 1);
        return a + ab * t;
    }

    private static Vector3 ClosestPointOnSegmentXZ(Vector3 p, Vector3 a, Vector3 b)
    {
        var p2 = new Vector2(p.X, p.Z);
        var a2 = new Vector2(a.X, a.Z);
        var b2 = new Vector2(b.X, b.Z);

        var ab = b2 - a2;
        var ap = p2 - a2;
        var abLenSq = ab.LengthSquared();
        if (abLenSq <= float.Epsilon)
            return new Vector3(a2.X, p.Y, a2.Y);

        var t = Math.Clamp(Vector2.Dot(ap, ab) / abLenSq, 0, 1);
        var closest = a2 + ab * t;
        return new Vector3(closest.X, p.Y, closest.Y);
    }

    private static float Distance(Vector3 a, Vector3 b, bool ignoreDeltaY)
    {
        if (ignoreDeltaY)
        {
            var d = new Vector2(a.X - b.X, a.Z - b.Z);
            return d.Length();
        }

        return (a - b).Length();
    }

    private static Vector3 LerpByDistance(Vector3 from, Vector3 to, float distance, bool ignoreDeltaY)
    {
        float segmentLength;
        if (ignoreDeltaY)
        {
            var d = new Vector2(to.X - from.X, to.Z - from.Z);
            segmentLength = d.Length();
        }
        else
        {
            segmentLength = (to - from).Length();
        }

        if (segmentLength <= float.Epsilon)
            return to;

        var t = Math.Clamp(distance / segmentLength, 0, 1);
        return Vector3.Lerp(from, to, t);
    }

    private static float DistanceSquared(Vector3 a, Vector3 b, bool ignoreDeltaY)
    {
        var d = a - b;
        if (ignoreDeltaY)
            d.Y = 0;
        return d.LengthSquared();
    }

    private static float DistanceToLineSegmentSquared(Vector3 p, Vector3 a, Vector3 b)
    {
        var ab = b - a;
        var ap = p - a;
        var abLenSq = ab.LengthSquared();
        if (abLenSq <= float.Epsilon)
            return ap.LengthSquared();

        var t = Vector3.Dot(ap, ab) / abLenSq;
        if (t <= 0)
            return ap.LengthSquared();
        if (t >= 1)
            return (p - b).LengthSquared();

        var closest = a + ab * t;
        return (p - closest).LengthSquared();
    }

    private static float DistanceToLineSegmentSquaredXZ(Vector3 p, Vector3 a, Vector3 b)
    {
        var p2 = new Vector2(p.X, p.Z);
        var a2 = new Vector2(a.X, a.Z);
        var b2 = new Vector2(b.X, b.Z);

        var ab = b2 - a2;
        var ap = p2 - a2;
        var abLenSq = ab.LengthSquared();
        if (abLenSq <= float.Epsilon)
            return ap.LengthSquared();

        var t = Vector2.Dot(ap, ab) / abLenSq;
        if (t <= 0)
            return ap.LengthSquared();
        if (t >= 1)
            return (p2 - b2).LengthSquared();

        var closest = a2 + ab * t;
        return (p2 - closest).LengthSquared();
    }
}
