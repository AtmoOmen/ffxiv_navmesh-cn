using vnavmesh.PathPostprocess;

namespace vnavmesh.Movement.Planning;

internal interface IMovementPlanBuilder
{
    MovementPlan Build(PostprocessedPath path);
}
