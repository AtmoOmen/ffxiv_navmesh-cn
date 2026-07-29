using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;

namespace vnavmesh.Build.Custom.Implementations.Territory.Dungeon;

[CustomizationTerritory(519)]
internal class Z0519神圣遗迹无限城市街古迹 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0519神圣遗迹无限城市街古迹() =>
        Settings.AgentMaxClimb = 0.75f; // web bridges - TODO: think about a better systemic solution
}
