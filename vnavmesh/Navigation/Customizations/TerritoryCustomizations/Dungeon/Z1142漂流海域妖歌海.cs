using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Dungeon;

[CustomizationTerritory(1142)]
internal class Z1142漂流海域妖歌海 : NavmeshCustomization
{
    public override int Version => 1;

    public Z1142漂流海域妖歌海() =>
        Settings.Filtering -= NavmeshFilter.LedgeSpans; // this allows mesh to go down the bowsprit to the land from the boat
}
