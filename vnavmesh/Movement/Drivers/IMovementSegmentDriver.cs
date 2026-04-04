using vnavmesh.Movement.Execution;

namespace vnavmesh.Movement.Drivers;

internal interface IMovementSegmentDriver
{
    void                Enter(MovementExecutionContext         context);
    SegmentDriverUpdate Update(MovementExecutionContext        context);
    bool                ShouldAdvance(MovementExecutionContext context);
    void                Exit(MovementExecutionContext          context);
}