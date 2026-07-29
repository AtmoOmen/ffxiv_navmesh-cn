using DotRecast.Recast;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(1041)]
internal class Z1041休养胜地布雷福洛克斯野营地 : NavmeshCustomization
{
    public override int Version => 1;

    public Z1041休养胜地布雷福洛克斯野营地() =>
        Settings.Partitioning = RcPartition.MONOTONE;
}
