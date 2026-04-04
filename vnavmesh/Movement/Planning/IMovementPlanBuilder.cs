using vnavmesh.Navigation.Planning;

namespace vnavmesh.Movement.Planning;

internal interface IMovementPlanBuilder
{
    MovementPlan Build(PostprocessedPath path);
}
