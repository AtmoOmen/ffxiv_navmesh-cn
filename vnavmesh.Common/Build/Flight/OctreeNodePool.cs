namespace vnavmesh.Common.Build.Flight;

public sealed class OctreeNodePool
{
    private const int InitialNodeCapacity   = 16;
    private const int InitialChildCapacity  = 128;

    public byte[] States;
    public int[]  FirstChild;
    public int[]  Children;
    public int    NodeCount;
    public int    ChildSlotsUsed;

    public OctreeNodePool()
    {
        States     = new byte[InitialNodeCapacity];
        FirstChild = new int[InitialNodeCapacity];
        Children   = new int[InitialChildCapacity];
        NodeCount  = 0;
        ChildSlotsUsed = 0;
    }

    public void EnsureRoot()
    {
        EnsureNodeCapacity(1);
        States[0]     = SparseVoxelOctree.NODE_EMPTY;
        FirstChild[0] = -1;
        NodeCount     = 1;
    }

    public int AllocSolidLeaf()
    {
        EnsureNodeCapacity(NodeCount + 1);
        var index = NodeCount++;
        States[index]     = SparseVoxelOctree.NODE_SOLID_LEAF;
        FirstChild[index] = -1;
        return index;
    }

    public int AllocMixedNode()
    {
        EnsureNodeCapacity(NodeCount + 1);
        EnsureChildCapacity(ChildSlotsUsed + 8);
        var index = NodeCount++;
        States[index]     = SparseVoxelOctree.NODE_MIXED;
        FirstChild[index] = ChildSlotsUsed;
        ChildSlotsUsed += 8;
        return index;
    }

    public int AllocateChildrenForNode
    (
        int nodeIndex
    )
    {
        EnsureChildCapacity(ChildSlotsUsed + 8);
        FirstChild[nodeIndex] = ChildSlotsUsed;
        ChildSlotsUsed += 8;
        return FirstChild[nodeIndex];
    }

    public int GetChild
    (
        int nodeIndex,
        int childIndex
    ) => Children[FirstChild[nodeIndex] + childIndex];

    public void SetChild
    (
        int nodeIndex,
        int childIndex,
        int value
    ) => Children[FirstChild[nodeIndex] + childIndex] = value;

    public void Trim()
    {
        if (States.Length != NodeCount)
            Array.Resize(ref States, NodeCount);
        if (FirstChild.Length != NodeCount)
            Array.Resize(ref FirstChild, NodeCount);
        if (Children.Length != ChildSlotsUsed)
            Array.Resize(ref Children, ChildSlotsUsed);
    }

    private void EnsureNodeCapacity
    (
        int required
    )
    {
        if (required <= States.Length)
            return;

        var newSize = Math.Max(States.Length * 2, required);
        Array.Resize(ref States, newSize);
        Array.Resize(ref FirstChild, newSize);
    }

    private void EnsureChildCapacity
    (
        int required
    )
    {
        if (required <= Children.Length)
            return;

        var newSize = Math.Max(Children.Length * 2, required);
        Array.Resize(ref Children, newSize);
    }
}
