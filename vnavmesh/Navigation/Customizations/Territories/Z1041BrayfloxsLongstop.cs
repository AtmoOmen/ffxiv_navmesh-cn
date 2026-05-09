using DotRecast.Recast;
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(1041)]
internal class Z1041BrayfloxsLongstop : NavmeshCustomization
{
    public override int Version => 1;

    public Z1041BrayfloxsLongstop() =>
        Settings.Partitioning = RcPartition.MONOTONE;
}
