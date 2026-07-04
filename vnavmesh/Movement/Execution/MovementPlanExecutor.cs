using System.Collections.Frozen;
using System.Numerics;
using Dalamud.Game.ClientState.Conditions;
using Dalamud.Game.ClientState.Objects.SubKinds;
using Dalamud.Plugin.Services;
using FFXIVClientStructs.FFXIV.Client.Game;
using vnavmesh.Common.Models;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Internal;
using vnavmesh.Movement.Drivers;
using vnavmesh.Movement.Interop;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation;
using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement.Execution;

public sealed class MovementPlanExecutor : IDisposable
{
    public bool MovementAllowed = true;

    public float Tolerance => nextToleranceOverride ??
                              (IsRunning ?
                                   activeDestinationTolerance :
                                   config.PathTolerance);

    public bool          IsRunning => activePlan != null;
    public List<Vector3> Waypoints => CollectWaypoints();

    internal event Action<MovementFailureContext>? OnMovementFailure;

    private const string SHARED_PATH_TAG        = "vnav.PathIsRunning";
    private const float  TAKEOFF_RESUME_DELTA_Y = 0.1f;

    private readonly PluginConfig              config;
    private readonly NavmeshManager            manager;
    private readonly CameraAlignmentController camera   = new();
    private readonly MovementInputController   movement = new();
    private readonly MovementUnstuckController unstuck;
    private readonly TraverseDriver            traverseDriver = new();
    private readonly TakeoffDriver             takeoffDriver  = new();
    private readonly bool[]                    sharedPathIsRunning;

    private MovementPlan?           activePlan;
    private int                     activeSegmentIndex;
    private int[]                   segmentWaypointIndices = [];
    private IMovementSegmentDriver? activeDriver;
    private DateTime                nextJump;
    private Vector3?                previousPosition;
    private float                   activeDestinationTolerance = 0.05f;
    private float?                  nextToleranceOverride;

    public MovementPlanExecutor(PluginConfig config, NavmeshManager manager)
    {
        this.config                       =  config;
        this.manager                      =  manager;
        unstuck                           =  new(config, manager);
        sharedPathIsRunning               =  Service.PluginInterface.GetOrCreateData<bool[]>(SHARED_PATH_TAG, () => [false]);
        activeDestinationTolerance        =  this.config.PathTolerance;
        this.manager.OnNavmeshChanged     += OnNavmeshChanged;
        Service.Condition.ConditionChange += OnConditionChange;
        OnNavmeshChanged(this.manager.Navmesh, this.manager.Query);
    }

    public void Dispose()
    {
        ResetControllers();
        UpdateSharedState(false);
        Service.PluginInterface.RelinquishData(SHARED_PATH_TAG);
        manager.OnNavmeshChanged          -= OnNavmeshChanged;
        Service.Condition.ConditionChange -= OnConditionChange;
        camera.Dispose();
        movement.Dispose();
    }

    public void Update(IFramework framework)
    {
        var player = Service.ObjectTable.LocalPlayer;
        if (player == null)
            return;

        var frameCurrentPosition  = player.Position;
        var framePreviousPosition = previousPosition ?? frameCurrentPosition;

        if (activePlan == null)
        {
            previousPosition = frameCurrentPosition;
            unstuck.Reset();
            ResetControllers();
            return;
        }

        var pathSuspended = unstuck.SuspendsPathExecution;
        SyncActiveDriver(player, framePreviousPosition);
        if (!pathSuspended)
            AdvanceCompletedSegments(player, framePreviousPosition);

        if (activePlan == null)
        {
            previousPosition = frameCurrentPosition;
            unstuck.Reset();
            ResetControllers();
            return;
        }

        SyncActiveDriver(player, framePreviousPosition);

        if (config.CancelMoveOnUserInput && movement.UserInput)
        {
            Stop();
            previousPosition = frameCurrentPosition;
            return;
        }

        var context       = BuildContext(player, framePreviousPosition);
        var unstuckUpdate = unstuck.Update(context, SuspendsUnstuck());

        if (unstuckUpdate.RequestRepath)
        {
            Fail(MovementFailureReason.RepathRequiredAfterUnstuck);
            previousPosition = frameCurrentPosition;
            return;
        }

        if (unstuckUpdate.SuspendPathExecution || unstuck.SuspendsPathExecution)
        {
            GameplayActivityBridge.ResetAFKTime();
            if (unstuckUpdate.OverrideCommand is { } overrideCommand)
                ApplyFrameCommand(overrideCommand, frameCurrentPosition);
            previousPosition = frameCurrentPosition;
            return;
        }

        GameplayActivityBridge.ResetAFKTime();
        var update = activeDriver!.Update(context);

        if (update.ActiveWaypointIndex is { } nextWaypointIndex)
            segmentWaypointIndices[activeSegmentIndex] = Math.Clamp(nextWaypointIndex, 0, activePlan!.Segments[activeSegmentIndex].Waypoints.Count);

        if (update.Failure is { } failure)
        {
            Stop();
            OnMovementFailure?.Invoke(failure);
            return;
        }

        ApplyFrameCommand(unstuckUpdate.OverrideCommand ?? update.Command, frameCurrentPosition);
        previousPosition = frameCurrentPosition;
    }

    internal void Execute(MovementPlan plan)
    {
        Stop();
        if (plan.Segments.Count == 0)
            return;

        activePlan                 = plan;
        activeSegmentIndex         = 0;
        segmentWaypointIndices     = new int[plan.Segments.Count];
        activeDestinationTolerance = plan.DestinationTolerance;
        previousPosition           = Service.ObjectTable.LocalPlayer?.Position;
        unstuck.Reset();
        UpdateSharedState(true);
        EnterCurrentSegment(previousPosition);
    }

    public void Move
    (
        List<Vector3> waypoints,
        bool          ignoreDeltaY,
        float         destTolerance = 0,
        Vector3?      goalPosition  = null,
        float?        tolerance     = null
    )
    {
        if (waypoints.Count == 0)
        {
            Stop();
            return;
        }

        var requestedMode = ignoreDeltaY ?
                                MovementMode.Ground :
                                MovementMode.Flight;
        var resolvedGoal = goalPosition ?? waypoints[^1];
        var resolvedDestinationTolerance = destTolerance > 0 ?
                                               destTolerance :
                                               tolerance ?? ConsumeNextTolerance();
        var segments = new List<MovementSegment>();
        var normalizedWaypoints = requestedMode == MovementMode.Flight && !IsAirborne ?
                                      FlightWaypointNormalizer.NormalizeForTakeoff(waypoints, Service.ObjectTable.LocalPlayer?.Position ?? waypoints[0]) :
                                      waypoints.ToList();

        if (requestedMode == MovementMode.Flight && !IsAirborne)
        {
            segments.Add
            (
                new()
                {
                    Kind                = MovementSegmentKind.Takeoff,
                    MovementMode        = MovementMode.Flight,
                    CompletionTolerance = 0
                }
            );
        }

        segments.Add
        (
            requestedMode == MovementMode.Flight ?
                new MovementSegment
                {
                    Kind                = MovementSegmentKind.FlightTraverse,
                    MovementMode        = MovementMode.Flight,
                    CompletionTolerance = 0,
                    StartPosition       = Service.ObjectTable.LocalPlayer?.Position ?? normalizedWaypoints[0],
                    Waypoints           = normalizedWaypoints
                } :
                new MovementSegment
                {
                    Kind                = MovementSegmentKind.GroundTraverse,
                    MovementMode        = MovementMode.Ground,
                    CompletionTolerance = 0,
                    StartPosition       = Service.ObjectTable.LocalPlayer?.Position ?? normalizedWaypoints[0],
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
        ExitCurrentSegment(previousPosition);
        activePlan                 = null;
        activeSegmentIndex         = 0;
        segmentWaypointIndices     = [];
        activeDestinationTolerance = config.PathTolerance;
        unstuck.Reset();
        UpdateSharedState(false);
        ResetControllers();
    }

    public float ConsumeNextTolerance()
    {
        var tolerance = nextToleranceOverride ?? config.PathTolerance;
        nextToleranceOverride = null;
        return tolerance;
    }

    public void SetNextTolerance(float tolerance) =>
        nextToleranceOverride = MathF.Max(0, tolerance);

    private void AdvanceCompletedSegments(IPlayerCharacter player, Vector3? previousPosition)
    {
        while (activePlan != null)
        {
            var context = BuildContext(player, previousPosition);
            if (!activeDriver!.ShouldAdvance(context))
                return;

            ExitCurrentSegment(previousPosition);
            activeSegmentIndex++;

            if (activeSegmentIndex >= activePlan.Segments.Count)
            {
                Stop();
                return;
            }

            EnterCurrentSegment(previousPosition);
        }
    }

    private void EnterCurrentSegment(Vector3? previousPosition)
    {
        if (activePlan == null)
            return;

        var context = BuildContextForCurrentSegment(previousPosition);
        activeDriver = ResolveDriver(context);
        unstuck.Reset();
        activeDriver.Enter(context);
        ConsumeInitialWaypoints(previousPosition);
    }

    private void ExitCurrentSegment(Vector3? previousPosition)
    {
        if (activePlan == null || activeDriver == null || activeSegmentIndex >= activePlan.Segments.Count)
            return;

        activeDriver.Exit(BuildContextForCurrentSegment(previousPosition));
        activeDriver = null;
    }

    private void SyncActiveDriver(IPlayerCharacter player, Vector3? previousPosition)
    {
        if (activePlan == null || activeSegmentIndex >= activePlan.Segments.Count)
            return;

        var desiredDriver = ResolveDriver(BuildContext(player, previousPosition));
        if (ReferenceEquals(activeDriver, desiredDriver))
            return;

        SwitchDriver(desiredDriver, previousPosition);
    }

    private void SwitchDriver(IMovementSegmentDriver driver, Vector3? previousPosition)
    {
        if (activePlan == null)
            return;

        if (activeDriver != null)
            activeDriver.Exit(BuildContextForCurrentSegment(previousPosition));

        activeDriver = driver;
        unstuck.Reset();
        activeDriver.Enter(BuildContextForCurrentSegment(previousPosition));
    }

    private MovementExecutionContext BuildContext(IPlayerCharacter player, Vector3? previousPosition) =>
        new()
        {
            Config                 = config,
            Player                 = player,
            Query                  = manager.Query,
            Plan                   = activePlan!,
            SegmentIndex           = activeSegmentIndex,
            Segment                = activePlan!.Segments[activeSegmentIndex],
            ActiveWaypointIndex    = segmentWaypointIndices[activeSegmentIndex],
            SegmentWaypointIndices = segmentWaypointIndices,
            MovementAllowed        = MovementAllowed,
            PreviousPosition       = previousPosition
        };

    private MovementExecutionContext BuildContextForCurrentSegment(Vector3? previousPosition)
    {
        var player = Service.ObjectTable.LocalPlayer ?? throw new InvalidOperationException("本地玩家不存在，无法构建移动上下文");
        return new()
        {
            Config                 = config,
            Player                 = player,
            Query                  = manager.Query,
            Plan                   = activePlan!,
            SegmentIndex           = activeSegmentIndex,
            Segment                = activePlan!.Segments[activeSegmentIndex],
            ActiveWaypointIndex    = segmentWaypointIndices[activeSegmentIndex],
            SegmentWaypointIndices = segmentWaypointIndices,
            MovementAllowed        = MovementAllowed,
            PreviousPosition       = previousPosition
        };
    }

    private void ConsumeInitialWaypoints(Vector3? previousPosition)
    {
        if (activePlan == null || activeSegmentIndex >= activePlan.Segments.Count)
            return;

        var context = BuildContextForCurrentSegment(previousPosition);
        var nextWaypointIndex = context.Segment.Kind switch
        {
            MovementSegmentKind.GroundTraverse => WaypointProgression.ConsumeGroundWaypoints(context),
            MovementSegmentKind.FlightTraverse => WaypointProgression.ConsumeFlightWaypoints(context),
            _                                  => -1
        };

        if (nextWaypointIndex >= 0)
            segmentWaypointIndices[activeSegmentIndex] = Math.Clamp(nextWaypointIndex, 0, context.WaypointCount);
    }

    private IMovementSegmentDriver ResolveDriver(MovementExecutionContext context)
    {
        if (context.Segment.Kind == MovementSegmentKind.FlightTraverse && !IsAirborne && context.TryGetFirstElevatedRemainingWaypoint(TAKEOFF_RESUME_DELTA_Y, out _))
            return takeoffDriver;

        return context.Segment.Kind == MovementSegmentKind.Takeoff ?
                   takeoffDriver :
                   traverseDriver;
    }

    private void ApplyFrameCommand(MovementFrameCommand command, Vector3 currentPosition)
    {
        movement.Enabled              = command.MovementEnabled || command.EnableFacingAlign;
        movement.AllowVerticalControl = command.AllowVerticalControl;
        movement.DesiredPosition      = command.DesiredPosition;
        movement.EnableFacingAlign    = command.EnableFacingAlign;
        movement.DesiredFacing        = command.DesiredFacing;

        camera.Enabled         = command.EnableCameraAlign;
        camera.SpeedH          = camera.SpeedV = 360.Degrees();
        camera.DesiredAzimuth  = command.DesiredAzimuth;
        camera.DesiredAltitude = command.DesiredAltitude;

        if (command.RequestJump)
            ExecuteJump();

        if (!command.MovementEnabled)
            movement.DesiredPosition = currentPosition;
    }

    private void ResetControllers()
    {
        movement.Enabled           = false;
        movement.EnableFacingAlign = false;
        movement.DesiredFacing     = default;
        camera.Enabled             = false;
        camera.SpeedH              = camera.SpeedV = default;
        if (Service.ObjectTable.LocalPlayer is { } player)
            movement.DesiredPosition = player.Position;
    }

    private void Fail(MovementFailureReason reason)
    {
        if (activePlan == null)
            return;

        var activeWaypoint = ResolveActiveWaypoint();
        var context = new MovementFailureContext
        (
            reason,
            activePlan.RequestedMode,
            activePlan.Segments[activeSegmentIndex].Kind,
            activePlan.RequestedDestination,
            activePlan.DestinationTolerance,
            activeWaypoint
        );

        Stop();
        OnMovementFailure?.Invoke(context);
    }

    private Vector3 ResolveActiveWaypoint()
    {
        if (activePlan == null)
            return default;

        for (var i = activeSegmentIndex; i < activePlan.Segments.Count; i++)
        {
            var segment       = activePlan.Segments[i];
            var waypointIndex = segmentWaypointIndices[i];
            if (waypointIndex < segment.Waypoints.Count)
                return segment.Waypoints[waypointIndex];
        }

        return activePlan.FinalDestination;
    }

    private List<Vector3> CollectWaypoints()
    {
        if (activePlan == null)
            return [];

        List<Vector3> result = [];

        for (var i = activeSegmentIndex; i < activePlan.Segments.Count; i++)
        {
            var segment = activePlan.Segments[i];
            var start   = segmentWaypointIndices[i];
            for (var waypointIndex = start; waypointIndex < segment.Waypoints.Count; waypointIndex++)
                result.Add(segment.Waypoints[waypointIndex]);
        }

        return result;
    }

    private unsafe void ExecuteJump()
    {
        if (Service.Condition[ConditionFlag.Diving])
            return;

        if (DateTime.Now >= nextJump)
        {
            ActionManager.Instance()->UseAction(ActionType.GeneralAction, 2);
            nextJump = DateTime.Now.AddMilliseconds(100);
        }
    }

    private void OnNavmeshChanged(Navmesh? navmesh, NavmeshQuery? query) =>
        Stop();

    private void OnConditionChange(ConditionFlag flag, bool value)
    {
        if (value || !RepathConditions.Contains(flag) || activePlan == null) return;

        Service.Log.Debug("检测到 Condition 变化，重新算路");
        Fail(MovementFailureReason.RepathRequired);
    }

    private void UpdateSharedState(bool isRunning) =>
        sharedPathIsRunning[0] = isRunning;

    private bool SuspendsUnstuck() =>
        ReferenceEquals(activeDriver, takeoffDriver) || activePlan == null || activeSegmentIndex >= activePlan.Segments.Count;

    private static bool IsAirborne =>
        Service.Condition.Any(ConditionFlag.InFlight, ConditionFlag.Diving);

    private static readonly FrozenSet<ConditionFlag> RepathConditions =
    [
        ConditionFlag.WatchingCutscene,
        ConditionFlag.Unknown101
    ];
}
