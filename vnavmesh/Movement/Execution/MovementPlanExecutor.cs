using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Game.ClientState.Objects.SubKinds;
using Dalamud.Plugin.Services;
using FFXIVClientStructs.FFXIV.Client.Game;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Movement.Drivers;
using vnavmesh.Movement.Interop;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Planning;
using vnavmesh.Shared.Models;

namespace vnavmesh.Movement.Execution;

public sealed class MovementPlanExecutor : IDisposable
{
    public bool                                    MovementAllowed = true;
    public float                                   Tolerance => _nextToleranceOverride ?? (IsRunning ? _activeDestinationTolerance : _config.PathTolerance);
    public bool                                    IsRunning => _activePlan != null;
    public List<Vector3>                           Waypoints => CollectWaypoints();
    internal event Action<MovementFailureContext>? OnMovementFailure;

    private readonly Config                    _config;
    private readonly NavmeshManager            _manager;
    private readonly CameraAlignmentController _camera        = new();
    private readonly MovementInputController   _movement      = new();
    private readonly MovementUnstuckController _unstuck;
    private readonly GroundTraverseDriver      _groundDriver  = new();
    private readonly FlightTraverseDriver      _flightDriver  = new();
    private readonly TakeoffDriver             _takeoffDriver = new();
    private readonly bool[]                    _sharedPathIsRunning;

    private const string SharedPathTag             = "vnav.PathIsRunning";
    private const float  TakeoffResumeDeltaY       = 0.1f;

    private MovementPlan?           _activePlan;
    private int                     _activeSegmentIndex;
    private int[]                   _segmentWaypointIndices = [];
    private IMovementSegmentDriver? _activeDriver;
    private DateTime                _nextJump;
    private Vector3?                _previousPosition;
    private float                   _activeDestinationTolerance = 0.05f;
    private float?                  _nextToleranceOverride;

    public MovementPlanExecutor(Config config, NavmeshManager manager)
    {
        _config                   =  config;
        _manager                  =  manager;
        _unstuck                  =  new(config, manager);
        _sharedPathIsRunning      =  Service.PluginInterface.GetOrCreateData<bool[]>(SharedPathTag, () => [false]);
        _activeDestinationTolerance = _config.PathTolerance;
        _manager.OnNavmeshChanged += OnNavmeshChanged;
        OnNavmeshChanged(_manager.Navmesh, _manager.Query);
    }

    public void Dispose()
    {
        ResetControllers();
        UpdateSharedState(false);
        Service.PluginInterface.RelinquishData(SharedPathTag);
        _manager.OnNavmeshChanged -= OnNavmeshChanged;
        _camera.Dispose();
        _movement.Dispose();
    }

    public void Update(IFramework framework)
    {
        var player = Service.ObjectTable.LocalPlayer;
        if (player == null)
            return;

        var frameCurrentPosition  = player.Position;
        var framePreviousPosition = _previousPosition ?? frameCurrentPosition;

        if (_activePlan == null)
        {
            _previousPosition = frameCurrentPosition;
            _unstuck.Reset();
            ResetControllers();
            return;
        }

        var pathSuspended = _unstuck.SuspendsPathExecution;
        SyncActiveDriver(player, framePreviousPosition);
        if (!pathSuspended)
            AdvanceCompletedSegments(player, framePreviousPosition);

        if (_activePlan == null)
        {
            _previousPosition = frameCurrentPosition;
            _unstuck.Reset();
            ResetControllers();
            return;
        }

        SyncActiveDriver(player, framePreviousPosition);

        if (_config.CancelMoveOnUserInput && _movement.UserInput)
        {
            Stop();
            _previousPosition = frameCurrentPosition;
            return;
        }

        var context       = BuildContext(player, framePreviousPosition);
        var unstuckUpdate = _unstuck.Update(context, SuspendsUnstuck());
        if (unstuckUpdate.RequestRepath)
        {
            Fail(MovementFailureReason.RepathRequiredAfterUnstuck);
            _previousPosition = frameCurrentPosition;
            return;
        }

        if (unstuckUpdate.SuspendPathExecution || _unstuck.SuspendsPathExecution)
        {
            GameplayActivityBridge.ResetAFKTime();
            if (unstuckUpdate.OverrideCommand is { } overrideCommand)
                ApplyFrameCommand(overrideCommand, frameCurrentPosition);
            _previousPosition = frameCurrentPosition;
            return;
        }

        GameplayActivityBridge.ResetAFKTime();
        var update = _activeDriver!.Update(context);

        if (update.ActiveWaypointIndex is { } nextWaypointIndex)
            _segmentWaypointIndices[_activeSegmentIndex] = Math.Clamp(nextWaypointIndex, 0, _activePlan!.Segments[_activeSegmentIndex].Waypoints.Count);

        if (update.Failure is { } failure)
        {
            Stop();
            OnMovementFailure?.Invoke(failure);
            return;
        }

        ApplyFrameCommand(unstuckUpdate.OverrideCommand ?? update.Command, frameCurrentPosition);
        _previousPosition = frameCurrentPosition;
    }

    internal void Execute(MovementPlan plan)
    {
        Stop();
        if (plan.Segments.Count == 0)
            return;

        _activePlan                            = plan;
        _activeSegmentIndex                    = 0;
        _segmentWaypointIndices                = new int[plan.Segments.Count];
        _activeDestinationTolerance            = plan.DestinationTolerance;
        _previousPosition                      = Service.ObjectTable.LocalPlayer?.Position;
        _unstuck.Reset();
        UpdateSharedState(true);
        EnterCurrentSegment(_previousPosition);
    }

    public void Move(List<Vector3> waypoints, bool ignoreDeltaY, float destTolerance = 0, Vector3? goalPosition = null, float? tolerance = null)
    {
        if (waypoints.Count == 0)
        {
            Stop();
            return;
        }

        var requestedMode     = ignoreDeltaY ? MovementMode.Ground : MovementMode.Flight;
        var resolvedGoal                 = goalPosition ?? waypoints[^1];
        var resolvedDestinationTolerance = destTolerance > 0 ? destTolerance : tolerance ?? ConsumeNextTolerance();
        var segments                     = new List<MovementSegment>();
        var normalizedWaypoints = requestedMode == MovementMode.Flight && !IsAirborne
            ? FlightWaypointNormalizer.NormalizeForTakeoff(waypoints, Service.ObjectTable.LocalPlayer?.Position ?? waypoints[0])
            : waypoints.ToList();

        Service.Log.Debug("收到执行器原始路径输入：该入口会绕过算路层与后处理层");

        if (requestedMode == MovementMode.Flight && !IsAirborne)
        {
            segments.Add
            (
                new TakeoffSegment
                {
                    CompletionTolerance = 0
                }
            );
        }

        segments.Add
        (
            requestedMode == MovementMode.Flight
                ? new FlightTraverseSegment
                {
                    CompletionTolerance = 0,
                    StartPosition       = Service.ObjectTable.LocalPlayer?.Position ?? normalizedWaypoints[0],
                    GeometryOwnership   = PathGeometryOwnership.ExternalInput,
                    ReachabilitySource  = PathReachabilitySource.ExternalInput,
                    Waypoints           = normalizedWaypoints
                }
                : new GroundTraverseSegment
                {
                    CompletionTolerance = 0,
                    StartPosition       = Service.ObjectTable.LocalPlayer?.Position ?? normalizedWaypoints[0],
                    GeometryOwnership   = PathGeometryOwnership.ExternalInput,
                    ReachabilitySource  = PathReachabilitySource.ExternalInput,
                    Waypoints           = normalizedWaypoints
                }
        );

        Execute
        (
            new()
            {
                RequestedMode        = requestedMode,
                RequestedDestination = resolvedGoal,
                FinalDestination     = waypoints[^1],
                DestinationTolerance = resolvedDestinationTolerance,
                Segments             = segments
            }
        );
    }

    public void Stop()
    {
        ExitCurrentSegment(_previousPosition);
        _activePlan                            = null;
        _activeSegmentIndex                    = 0;
        _segmentWaypointIndices                = [];
        _activeDestinationTolerance            = _config.PathTolerance;
        _unstuck.Reset();
        UpdateSharedState(false);
        ResetControllers();
    }

    public float ConsumeNextTolerance()
    {
        var tolerance = _nextToleranceOverride ?? _config.PathTolerance;
        _nextToleranceOverride = null;
        return tolerance;
    }

    public void SetNextTolerance(float tolerance) =>
        _nextToleranceOverride = MathF.Max(0, tolerance);

    private void AdvanceCompletedSegments(IPlayerCharacter player, Vector3? previousPosition)
    {
        while (_activePlan != null)
        {
            var context = BuildContext(player, previousPosition);
            if (!_activeDriver!.ShouldAdvance(context))
                return;

            ExitCurrentSegment(previousPosition);
            _activeSegmentIndex++;

            if (_activeSegmentIndex >= _activePlan.Segments.Count)
            {
                Stop();
                return;
            }

            EnterCurrentSegment(previousPosition);
        }
    }

    private void EnterCurrentSegment(Vector3? previousPosition)
    {
        if (_activePlan == null)
            return;

        var context  = BuildContextForCurrentSegment(previousPosition);
        _activeDriver = ResolveDriver(context);
        _unstuck.Reset();
        _activeDriver.Enter(context);
        ConsumeInitialWaypoints(previousPosition);
    }

    private void ExitCurrentSegment(Vector3? previousPosition)
    {
        if (_activePlan == null || _activeDriver == null || _activeSegmentIndex >= _activePlan.Segments.Count)
            return;

        _activeDriver.Exit(BuildContextForCurrentSegment(previousPosition));
        _activeDriver = null;
    }

    private void SyncActiveDriver(IPlayerCharacter player, Vector3? previousPosition)
    {
        if (_activePlan == null || _activeSegmentIndex >= _activePlan.Segments.Count)
            return;

        var desiredDriver = ResolveDriver(BuildContext(player, previousPosition));
        if (ReferenceEquals(_activeDriver, desiredDriver))
            return;

        SwitchDriver(desiredDriver, previousPosition);
    }

    private void SwitchDriver(IMovementSegmentDriver driver, Vector3? previousPosition)
    {
        if (_activePlan == null)
            return;

        if (_activeDriver != null)
            _activeDriver.Exit(BuildContextForCurrentSegment(previousPosition));

        _activeDriver = driver;
        _unstuck.Reset();
        _activeDriver.Enter(BuildContextForCurrentSegment(previousPosition));
    }

    private MovementExecutionContext BuildContext(IPlayerCharacter player, Vector3? previousPosition) =>
        new()
        {
            Config                 = _config,
            Player                 = player,
            Query                  = _manager.Query,
            Plan                   = _activePlan!,
            SegmentIndex           = _activeSegmentIndex,
            Segment                = _activePlan!.Segments[_activeSegmentIndex],
            ActiveWaypointIndex    = _segmentWaypointIndices[_activeSegmentIndex],
            SegmentWaypointIndices = _segmentWaypointIndices,
            MovementAllowed        = MovementAllowed,
            PreviousPosition       = previousPosition
        };

    private MovementExecutionContext BuildContextForCurrentSegment(Vector3? previousPosition)
    {
        var player = Service.ObjectTable.LocalPlayer ?? throw new InvalidOperationException("本地玩家不存在，无法构建移动上下文");
        return new()
        {
            Config                 = _config,
            Player                 = player,
            Query                  = _manager.Query,
            Plan                   = _activePlan!,
            SegmentIndex           = _activeSegmentIndex,
            Segment                = _activePlan!.Segments[_activeSegmentIndex],
            ActiveWaypointIndex    = _segmentWaypointIndices[_activeSegmentIndex],
            SegmentWaypointIndices = _segmentWaypointIndices,
            MovementAllowed        = MovementAllowed,
            PreviousPosition       = previousPosition
        };
    }

    private void ConsumeInitialWaypoints(Vector3? previousPosition)
    {
        if (_activePlan == null || _activeSegmentIndex >= _activePlan.Segments.Count)
            return;

        var context = BuildContextForCurrentSegment(previousPosition);
        var nextWaypointIndex = context.Segment.Kind switch
        {
            MovementSegmentKind.GroundTraverse => DriverMath.ConsumeGroundWaypoints(context),
            MovementSegmentKind.FlightTraverse => DriverMath.ConsumeFlightWaypoints(context),
            _                                  => -1
        };

        if (nextWaypointIndex >= 0)
            _segmentWaypointIndices[_activeSegmentIndex] = Math.Clamp(nextWaypointIndex, 0, context.WaypointCount);
    }

    private IMovementSegmentDriver ResolveDriver(MovementExecutionContext context)
    {
        if (context.Segment.Kind == MovementSegmentKind.FlightTraverse
            && !IsAirborne
            && context.TryGetFirstElevatedRemainingWaypoint(TakeoffResumeDeltaY, out _))
            return _takeoffDriver;

        return ResolveDriver(context.Segment.Kind);
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
        _movement.Enabled              = command.MovementEnabled || command.EnableFacingAlign;
        _movement.AllowVerticalControl = command.AllowVerticalControl;
        _movement.DesiredPosition      = command.DesiredPosition;
        _movement.EnableFacingAlign    = command.EnableFacingAlign;
        _movement.DesiredFacing        = command.DesiredFacing;

        _camera.Enabled         = command.EnableCameraAlign;
        _camera.SpeedH          = _camera.SpeedV = 360.Degrees();
        _camera.DesiredAzimuth  = command.DesiredAzimuth;
        _camera.DesiredAltitude = command.DesiredAltitude;

        if (command.RequestJump)
            ExecuteJump();

        if (!command.MovementEnabled)
            _movement.DesiredPosition = currentPosition;
    }

    private void ResetControllers()
    {
        _movement.Enabled           = false;
        _movement.EnableFacingAlign = false;
        _movement.DesiredFacing     = default;
        _camera.Enabled             = false;
        _camera.SpeedH              = _camera.SpeedV = default;
        if (Service.ObjectTable.LocalPlayer is { } player)
            _movement.DesiredPosition = player.Position;
    }

    private void Fail(MovementFailureReason reason)
    {
        if (_activePlan == null)
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
        if (_activePlan == null)
            return default;

        for (var i = _activeSegmentIndex; i < _activePlan.Segments.Count; i++)
        {
            var segment       = _activePlan.Segments[i];
            var waypointIndex = _segmentWaypointIndices[i];
            if (waypointIndex < segment.Waypoints.Count)
                return segment.Waypoints[waypointIndex];
        }

        return _activePlan.FinalDestination;
    }

    private List<Vector3> CollectWaypoints()
    {
        if (_activePlan == null)
            return [];

        List<Vector3> result = [];

        for (var i = _activeSegmentIndex; i < _activePlan.Segments.Count; i++)
        {
            var segment = _activePlan.Segments[i];
            var start   = _segmentWaypointIndices[i];
            for (var waypointIndex = start; waypointIndex < segment.Waypoints.Count; waypointIndex++)
                result.Add(segment.Waypoints[waypointIndex]);
        }

        return result;
    }

    private unsafe void ExecuteJump()
    {
        if (Service.Condition[ConditionFlag.Diving])
            return;

        if (DateTime.Now >= _nextJump)
        {
            ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
            _nextJump = DateTime.Now.AddMilliseconds(100);
        }
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query) =>
        Stop();

    private void UpdateSharedState(bool isRunning) => _sharedPathIsRunning[0] = isRunning;

    private bool SuspendsUnstuck() =>
        ReferenceEquals(_activeDriver, _takeoffDriver) || _activePlan == null || _activeSegmentIndex >= _activePlan.Segments.Count;

    private static bool IsAirborne => Service.Condition[ConditionFlag.InFlight] || Service.Condition[ConditionFlag.Diving];
}
