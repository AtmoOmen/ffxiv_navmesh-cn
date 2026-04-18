using System.Numerics;

namespace vnavmesh.Navigation.Volume;

public class PathfindLoopException
(
    ulong   from,
    ulong   to,
    Vector3 fromP,
    Vector3 toP
) : Exception
{
    public readonly ulong   FromVoxel = from;
    public readonly ulong   ToVoxel   = to;
    public readonly Vector3 FromPos   = fromP;
    public readonly Vector3 ToPos     = toP;

    public override string Message => 
        $"体素寻路射线步进陷入死循环。（起点体素 = {FromVoxel:X} / {FromPos}，终点体素 = {ToVoxel:X} / {ToPos}）";
}
