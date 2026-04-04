using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Game.ClientState.Objects.SubKinds;
using Dalamud.Plugin;
using Dalamud.Plugin.Services;
using FFXIVClientStructs.FFXIV.Client.Game;
using vnavmesh.Models;
using vnavmesh.Movement.Drivers;
using vnavmesh.Movement.Interop;
using vnavmesh.Movement.Planning;
using vnavmesh.Navmesh;

namespace vnavmesh.Movement.Execution;

public sealed class MovementPlanExecutor : IDisposable
{
    public bool                                    MovementAllowed = true;
    public float                                   Tolerance => _nextToleranceOverride ?? (IsRunning ? _activePathTolerance : Service.Config.PathTolerance);
    public bool                                    IsRunning => _activePlan != null;
    public List<Vector3>                           Waypoints => CollectWaypoints();
    internal event Action<MovementFailureContext>? OnMovementFailure;

    private readonly IDalamudPluginInterface   _dalamud;
    private readonly NavmeshManager            _manager;
    private readonly CameraAlignmentController _camera        = new();
    private readonly MovementInputController   _movement      = new();
    private readonly GroundTraverseDriver      _groundDriver  = new();
    private readonly FlightTraverseDriver      _flightDriver  = new();
    private readonly TakeoffDriver             _takeoffDriver = new();
    private readonly bool[]                    _sharedPathIsRunning;

    private const string SharedPathTag = "vnav.PathIsRunning";

    private MovementPlan?           _activePlan;
    private int                     _activeSegmentIndex;
    private IMovementSegmentDriver? _activeDriver;
    private DateTime                _nextJump;
    private Vector3?                _previousPosition;
    private float                   _activePathTolerance = 0.05f;
    private float?                  _nextToleranceOverride;
    private int                     _millisecondsWithNoSignificantMovement;

    public MovementPlanExecutor(IDalamudPluginInterface dalamud, NavmeshManager manager)
    {
        _dalamud                  =  dalamud;
        _manager                  =  manager;
        _sharedPathIsRunning      =  _dalamud.GetOrCreateData<bool[]>(SharedPathTag, () => [false]);
        _activePathTolerance      =  Service.Config.PathTolerance;
        _manager.OnNavmeshChanged += OnNavmeshChanged;
        OnNavmeshChanged(_manager.Navmesh, _manager.Query);
    }

    public void Dispose()
    {
        ResetControllers();
        UpdateSharedState(false);
        _dalamud.RelinquishData(SharedPathTag);
        _manager.OnNavmeshChanged -= OnNavmeshChanged;
        _camera.Dispose();
        _movement.Dispose();
    }

    public void Update(IFramework framework)
    {
        var player = Service.ObjectTable.LocalPlayer;
        if(player == null)
            return;

        if(_activePlan == null)
        {
            _previousPosition = player.Position;
            ResetControllers();
            return;
        }

        AdvanceCompletedSegments(framework, player);

        if(_activePlan == null)
        {
            _previousPosition = player.Position;
            ResetControllers();
            return;
        }

        if(Service.Config.StopOnStuck && _previousPosition.HasValue)
        {
            var delta    = framework.UpdateDelta.Milliseconds / 1000f;
            var distance = delta > 0 ? Vector3.Distance(player.Position, _previousPosition.Value) / delta : 0;
            if(distance <= Service.Config.StuckTolerance)
                _millisecondsWithNoSignificantMovement += framework.UpdateDelta.Milliseconds;
            else
                _millisecondsWithNoSignificantMovement = 0;

            if(_millisecondsWithNoSignificantMovement >= Service.Config.StuckTimeoutMs)
            {
                Fail(MovementFailureReason.Stuck);
                _previousPosition = player.Position;
                return;
            }
        }

        _previousPosition = player.Position;

        if(Service.Config.CancelMoveOnUserInput && _movement.UserInput)
        {
            Stop();
            return;
        }

        var context = BuildContext(framework, player);
        GameplayActivityBridge.ResetAFKTime();
        var update = _activeDriver!.Update(context);

        if(update.Failure is { } failure)
        {
            Stop();
            OnMovementFailure?.Invoke(failure);
            return;
        }

        ApplyFrameCommand(update.Command, player.Position);
    }

    internal void Execute(MovementPlan plan)
    {
        Stop();
        if(plan.Segments.Count == 0)
            return;

        _activePlan                            = plan;
        _activeSegmentIndex                    = 0;
        _activePathTolerance                   = ConsumeNextTolerance();
        _millisecondsWithNoSignificantMovement = 0;
        UpdateSharedState(true);
        EnterCurrentSegment();
    }

    public void Move(List<Vector3> waypoints, bool ignoreDeltaY, float destTolerance = 0, Vector3? goalPosition = null, float? tolerance = null)
    {
        if(waypoints.Count == 0)
        {
            Stop();
            return;
        }

        var requestedMode     = ignoreDeltaY ? MovementMode.Ground : MovementMode.Flight;
        var resolvedGoal      = goalPosition ?? waypoints[^1];
        var resolvedTolerance = tolerance    ?? ConsumeNextTolerance();
        var segments          = new List<MovementSegment>();
        if(requestedMode == MovementMode.Flight && !IsAirborne)
            segments.Add
            (
                new TakeoffSegment
                {
                    CompletionTolerance = 0
                }
            );

        segments.Add
        (
            requestedMode == MovementMode.Flight
                ? new FlightTraverseSegment
                {
                    CompletionTolerance = resolvedTolerance,
                    Waypoints           = [.. waypoints]
                }
                : new GroundTraverseSegment
                {
                    CompletionTolerance = resolvedTolerance,
                    Waypoints           = [.. waypoints]
                }
        );

        Execute
        (
            new()
            {
                RequestedMode        = requestedMode,
                RequestedDestination = resolvedGoal,
                FinalDestination     = waypoints[^1],
                DestinationTolerance = destTolerance,
                Segments             = segments
            }
        );
    }

    public void Stop()
    {
        ExitCurrentSegment();
        _activePlan                            = null;
        _activeSegmentIndex                    = 0;
        _activePathTolerance                   = Service.Config.PathTolerance;
        _millisecondsWithNoSignificantMovement = 0;
        UpdateSharedState(false);
        ResetControllers();
    }

    public float ConsumeNextTolerance()
    {
        var tolerance = _nextToleranceOverride ?? Service.Config.PathTolerance;
        _nextToleranceOverride = null;
        return tolerance;
    }

    public void SetNextTolerance(float tolerance)
    {
        _nextToleranceOverride = MathF.Max(0, tolerance);
    }

    private void AdvanceCompletedSegments(IFramework framework, IPlayerCharacter player)
    {
        while (_activePlan != null)
        {
            var context = BuildContext(framework, player);
            if(!_activeDriver!.ShouldAdvance(context))
                return;

            ExitCurrentSegment();
            _activeSegmentIndex++;

            if(_activeSegmentIndex >= _activePlan.Segments.Count)
            {
                Stop();
                return;
            }

            EnterCurrentSegment();
        }
    }

    private void EnterCurrentSegment()
    {
        if(_activePlan == null)
            return;

        _activeDriver        = ResolveDriver(_activePlan.Segments[_activeSegmentIndex].Kind);
        _activePathTolerance = _activePlan.Segments[_activeSegmentIndex].CompletionTolerance;
        _activeDriver.Enter(BuildContextForCurrentSegment());
    }

    private void ExitCurrentSegment()
    {
        if(_activePlan == null || _activeDriver == null || _activeSegmentIndex >= _activePlan.Segments.Count)
            return;

        _activeDriver.Exit(BuildContextForCurrentSegment());
        _activeDriver = null;
    }

    private MovementExecutionContext BuildContext(IFramework framework, IPlayerCharacter player) =>
        new()
        {
            Framework        = framework,
            Player           = player,
            Plan             = _activePlan!,
            SegmentIndex     = _activeSegmentIndex,
            Segment          = _activePlan!.Segments[_activeSegmentIndex],
            MovementAllowed  = MovementAllowed,
            PathTolerance    = _activePathTolerance,
            PreviousPosition = _previousPosition
        };

    private MovementExecutionContext BuildContextForCurrentSegment()
    {
        var player = Service.ObjectTable.LocalPlayer ?? throw new InvalidOperationException("本地玩家不存在，无法构建移动上下文");
        return new()
        {
            Framework        = Service.Framework,
            Player           = player,
            Plan             = _activePlan!,
            SegmentIndex     = _activeSegmentIndex,
            Segment          = _activePlan!.Segments[_activeSegmentIndex],
            MovementAllowed  = MovementAllowed,
            PathTolerance    = _activePathTolerance,
            PreviousPosition = _previousPosition
        };
    }

    private IMovementSegmentDriver ResolveDriver(MovementSegmentKind kind) => kind switch
    {
        MovementSegmentKind.GroundTraverse => _groundDriver,
        MovementSegmentKind.Takeoff        => _takeoffDriver,
        MovementSegmentKind.FlightTraverse => _flightDriver,
        _                                  => throw new ArgumentOutOfRangeException(nameof(kind), kind, "未知移动阶段")
    };

    private void ApplyFrameCommand(MovementFrameCommand command, Vector3 currentPosition)
    {
        _movement.Enabled              = command.MovementEnabled;
        _movement.AllowVerticalControl = command.AllowVerticalControl;
        _movement.DesiredPosition      = command.DesiredPosition;

        _camera.Enabled         = command.EnableCameraAlign;
        _camera.SpeedH          = _camera.SpeedV = 360.Degrees();
        _camera.DesiredAzimuth  = command.DesiredAzimuth;
        _camera.DesiredAltitude = command.DesiredAltitude;

        if(command.RequestJump)
            ExecuteJump();

        if(!command.MovementEnabled)
            _movement.DesiredPosition = currentPosition;
    }

    private void ResetControllers()
    {
        _movement.Enabled = false;
        _camera.Enabled   = false;
        _camera.SpeedH    = _camera.SpeedV = default;
        if(Service.ObjectTable.LocalPlayer is { } player)
            _movement.DesiredPosition = player.Position;
    }

    private void Fail(MovementFailureReason reason)
    {
        if(_activePlan == null)
            return;

        var activeWaypoint = ResolveActiveWaypoint();
        var context = new MovementFailureContext
        (
            reason,
            _activePlan.RequestedMode,
            _activePlan.Segments[_activeSegmentIndex].Kind,
            _activePlan.RequestedDestination,
            _activePlan.DestinationTolerance,
            activeWaypoint
        );
        Stop();
        OnMovementFailure?.Invoke(context);
    }

    private Vector3 ResolveActiveWaypoint()
    {
        if(_activePlan == null)
            return default;

        for (var i = _activeSegmentIndex; i < _activePlan.Segments.Count; i++)
            if(_activePlan.Segments[i].Waypoints.Count > 0)
                return _activePlan.Segments[i].Waypoints[0];

        return _activePlan.FinalDestination;
    }

    private List<Vector3> CollectWaypoints()
    {
        if(_activePlan == null)
            return [];

        return [.. _activePlan.Segments.Skip(_activeSegmentIndex).SelectMany(segment => segment.Waypoints)];
    }

    private unsafe void ExecuteJump()
    {
        if(Service.Condition[ConditionFlag.Diving])
            return;

        if(DateTime.Now >= _nextJump)
        {
            ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
            _nextJump = DateTime.Now.AddMilliseconds(100);
        }
    }

    private void OnNavmeshChanged(Navmesh.Navmesh? navmesh, NavmeshQuery? query)
    {
        Stop();
    }

    private        void UpdateSharedState(bool isRunning) => _sharedPathIsRunning[0] = isRunning;
    private static bool IsAirborne                        => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];
}
