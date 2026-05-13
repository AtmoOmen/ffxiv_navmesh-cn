using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Dungeon;

[CustomizationTerritory(1044)]
internal class Z1044最终决战天幕魔导城 : NavmeshCustomization
{
    public override int Version => 3;

    public Z1044最终决战天幕魔导城()
    {
        // allow connection between higher and lower section of broken ramp right after the magitek armor
        Settings.Filtering -= NavmeshFilter.LedgeSpans;
    }
}
