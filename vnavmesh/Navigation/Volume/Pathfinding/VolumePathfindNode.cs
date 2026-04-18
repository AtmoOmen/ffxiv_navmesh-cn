using System.Numerics;

namespace vnavmesh.Navigation.Volume;

public struct VolumePathfindNode
{
    public float   GScore;
    public float   HScore;
    public ulong   Voxel;
    public int     ParentIndex;
    public int     OpenHeapIndex;
    public int     Revision;
    public bool    Closed;
    public Vector3 Position;
}
