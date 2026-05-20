using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Abstractions;

public abstract class SceneNavmeshCustomization : NavmeshCustomization
{
    public abstract bool Matches(SceneDefinition definition);
}
