using System;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;

namespace vnavmesh.Navigation.Customizations;

[NavmeshCustomizationIgnore]
internal sealed class CompositeNavmeshCustomization
(
    IReadOnlyList<NavmeshCustomization> customizations
) : NavmeshCustomization
{
    public override int Version
    {
        get
        {
            var hash = 0x811C9DC5u;

            foreach (var customization in customizations)
            {
                hash = Fnv1A(hash, customization.GetType().FullName ?? customization.GetType().Name);
                hash = Fnv1A(hash, customization.Version);
            }

            return unchecked((int)hash);
        }
    }

    public override NavmeshSettings GetBuildSettings(SceneDefinition definition)
    {
        var settings = new NavmeshSettings();

        foreach (var customization in customizations)
            customization.ApplyBuildSettings(definition, settings);

        return settings;
    }

    public override bool IsFlyingSupported(SceneDefinition definition)
    {
        var result = false;

        foreach (var customization in customizations)
            result = customization.IsFlyingSupported(definition);

        return result;
    }

    public override void CustomizeScene(SceneExtractor scene)
    {
        foreach (var customization in customizations)
            customization.CustomizeScene(scene);
    }

    public override void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile)
    {
        foreach (var customization in customizations)
            customization.CustomizeBuildProfile(definition, profile);
    }

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings)
    {
        foreach (var customization in customizations)
            customization.CustomizeBuildSettings(definition, settings);
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        foreach (var customization in customizations)
            customization.CustomizeSettings(config);
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        foreach (var customization in customizations)
            customization.CustomizeMesh(mesh, festivalLayers);
    }

    private static uint Fnv1A(uint hash, string text)
    {
        foreach (var ch in text)
        {
            hash ^= ch;
            hash *= 0x01000193u;
        }

        return hash;
    }

    private static uint Fnv1A(uint hash, int value)
    {
        unchecked
        {
            hash ^= (uint)value;
            hash *= 0x01000193u;
            return hash;
        }
    }
}
