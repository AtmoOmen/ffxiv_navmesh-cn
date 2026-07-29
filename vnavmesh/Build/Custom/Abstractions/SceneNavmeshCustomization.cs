using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Abstractions;

public abstract class SceneNavmeshCustomization : NavmeshCustomization
{
    public abstract bool Matches
    (
        SceneDefinition definition
    );
}
