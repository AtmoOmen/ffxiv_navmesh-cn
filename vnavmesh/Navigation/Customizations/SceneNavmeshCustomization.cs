using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations;

// base class for scene-matched navmesh customizations that can apply to multiple territories
public abstract class SceneNavmeshCustomization : NavmeshCustomization
{
    public abstract bool Matches(SceneDefinition definition);
}
