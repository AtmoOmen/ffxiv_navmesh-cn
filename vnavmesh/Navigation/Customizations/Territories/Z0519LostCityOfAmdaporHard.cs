namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(519)]
internal class Z0519LostCityOfAmdaporHard : NavmeshCustomization
{
    public override int Version => 1;

    public Z0519LostCityOfAmdaporHard() =>
        Settings.AgentMaxClimb = 0.75f; // web bridges - TODO: think about a better systemic solution
}
