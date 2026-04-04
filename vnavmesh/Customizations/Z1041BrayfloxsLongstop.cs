using DotRecast.Recast;
using vnavmesh.Navmesh;

namespace vnavmesh.Customizations;

[CustomizationTerritory(1041)]
internal class Z1041BrayfloxsLongstop : NavmeshCustomization
{
    public override int Version => 1;

    public Z1041BrayfloxsLongstop() =>
        Settings.Partitioning = RcPartition.MONOTONE;
}
