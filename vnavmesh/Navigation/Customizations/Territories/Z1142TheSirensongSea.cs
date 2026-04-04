using vnavmesh.Navigation.Mesh.Build;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(1142)]
internal class Z1142TheSirensongSea : NavmeshCustomization
{
    public override int Version => 1;

    public Z1142TheSirensongSea() =>
        Settings.Filtering -= NavmeshSettings.Filter.LedgeSpans; // this allows mesh to go down the bowsprit to the land from the boat
}
