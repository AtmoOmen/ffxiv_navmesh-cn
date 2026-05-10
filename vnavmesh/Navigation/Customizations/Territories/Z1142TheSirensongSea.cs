using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Mesh.Build;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(1142)]
internal class Z1142TheSirensongSea : NavmeshCustomization
{
    public override int Version => 1;

    public Z1142TheSirensongSea() =>
        Settings.Filtering -= NavmeshFilter.LedgeSpans; // this allows mesh to go down the bowsprit to the land from the boat
}
