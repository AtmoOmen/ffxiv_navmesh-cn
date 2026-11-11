using vnavmesh.Common.Build.Flight;

namespace vnavmesh.Query.Flight;

internal sealed class VoxelReachability
{
    private readonly Dictionary<ulong, int> componentByVoxel = [];

    internal VoxelReachability(VoxelMap volume, CancellationToken cancel)
    {
        var traversal = new VoxelPathfind(volume);
        var frontier = new Queue<ulong>();
        var component = 0;

        foreach (var seed in volume.RootTile.EnumerateLeafVoxels(volume.RootTile.BoundsMin, volume.RootTile.BoundsMax))
        {
            cancel.ThrowIfCancellationRequested();
            if (!seed.empty
                || !componentByVoxel.TryAdd(seed.index, component))
                continue;

            frontier.Enqueue(seed.index);
            while (frontier.TryDequeue(out var current))
            {
                cancel.ThrowIfCancellationRequested();
                foreach (var neighbour in traversal.CollectNeighbours(current))
                {
                    var bounds = volume.VoxelBounds(neighbour, 0);
                    var leaf = volume.FindLeafVoxel((bounds.min + bounds.max) * 0.5f);
                    if (componentByVoxel.TryAdd(leaf.voxel, component))
                        frontier.Enqueue(leaf.voxel);
                }
            }

            component++;
        }
    }

    internal bool IsConnected(ulong source, ulong destination) =>
        componentByVoxel.TryGetValue(source, out var sourceComponent)
        && componentByVoxel.TryGetValue(destination, out var destinationComponent)
        && sourceComponent == destinationComponent;
}
