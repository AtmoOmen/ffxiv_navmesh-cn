using DotRecast.Recast;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(1041)]
internal class Z1041休养胜地布雷福洛克斯野营地 : NavmeshCustomization
{
    public override int Version => 1;

    public Z1041休养胜地布雷福洛克斯野营地() =>
        Settings.Partitioning = RcPartition.MONOTONE;
}
