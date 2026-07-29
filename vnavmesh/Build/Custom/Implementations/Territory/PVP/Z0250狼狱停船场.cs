using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Territory.PVP;

[CustomizationTerritory(250)]
internal class Z0250狼狱停船场 : NavmeshCustomization
{
    public override int Version => 1;

    public override bool IsFlyingSupported
    (
        SceneDefinition definition
    ) => false; // this is unflyable, despite intended use being 1
}
