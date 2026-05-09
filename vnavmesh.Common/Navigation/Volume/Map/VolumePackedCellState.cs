namespace vnavmesh.Common.Navigation.Volume.Map;

internal enum VolumePackedCellState : byte
{
    Empty     = 0,
    SolidLeaf = 1,
    Subtree   = 2
}
