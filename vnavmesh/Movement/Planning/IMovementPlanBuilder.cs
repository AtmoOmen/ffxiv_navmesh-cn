using vnavmesh.NavPathfind;

namespace vnavmesh.Movement.Planning;

internal interface IMovementPlanBuilder
{
    MovementPlan Build(PathfindResult result, float destinationTolerance, float pathTolerance);
}
