using vnavmesh.Navigation.Mesh.Build;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(1044)]
internal class Z1044ThePraetorium : NavmeshCustomization
{
    public override int Version => 3;

    public Z1044ThePraetorium()
    {
        // allow connection between higher and lower section of broken ramp right after the magitek armor
        Settings.Filtering -= NavmeshSettings.Filter.LedgeSpans;
    }
}
