namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(128)]
internal class Z0128LimsaLominsaUpperDecks : NavmeshCustomization
{
    public override int Version => 1;

    public Z0128LimsaLominsaUpperDecks() =>
        Settings.AgentRadius = 0.75f;
}
