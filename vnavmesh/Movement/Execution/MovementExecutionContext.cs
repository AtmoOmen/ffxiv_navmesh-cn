using System.Numerics;
using Dalamud.Game.ClientState.Objects.SubKinds;
using Dalamud.Plugin.Services;
using vnavmesh.Movement.Planning;

namespace vnavmesh.Movement.Execution;

internal sealed class MovementExecutionContext
{
    public required IFramework       Framework        { get; init; }
    public required IPlayerCharacter Player           { get; init; }
    public required MovementPlan     Plan             { get; init; }
    public required int              SegmentIndex     { get; init; }
    public required MovementSegment  Segment          { get; init; }
    public required bool             MovementAllowed  { get; init; }
    public required float            PathTolerance    { get; init; }
    public          Vector3?         PreviousPosition { get; init; }
}