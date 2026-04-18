namespace vnavmesh.Navigation.Volume.Pathfinding;

public enum VolumeSearchTermination : byte
{
    ReachedGoal,
    SearchExhausted,
    StepBudgetReached
}
