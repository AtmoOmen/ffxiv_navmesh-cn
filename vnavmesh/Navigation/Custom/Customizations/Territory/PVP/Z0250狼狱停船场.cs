using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.PVP;

[CustomizationTerritory(250)]
internal class Z0250狼狱停船场 : NavmeshCustomization
{
    public override int Version => 1;

    public override bool IsFlyingSupported(SceneDefinition definition) => false; // this is unflyable, despite intended use being 1
}
