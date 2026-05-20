using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.Dungeon;

[CustomizationTerritory(519)]
internal class Z0519神圣遗迹无限城市街古迹 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0519神圣遗迹无限城市街古迹() =>
        Settings.AgentMaxClimb = 0.75f; // web bridges - TODO: think about a better systemic solution
}
