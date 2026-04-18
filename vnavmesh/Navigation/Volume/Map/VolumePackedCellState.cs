namespace vnavmesh.Navigation.Volume;

internal enum VolumePackedCellState : byte
{
    Empty     = 0,
    SolidLeaf = 1,
    Subtree   = 2
}
