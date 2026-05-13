using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations;

[CustomizationTerritory(250)]
internal class Z0250狼狱停船场 : NavmeshCustomization
{
    public override int Version => 1;

    public override bool IsFlyingSupported(SceneDefinition definition) => false; // this is unflyable, despite intended use being 1
}
