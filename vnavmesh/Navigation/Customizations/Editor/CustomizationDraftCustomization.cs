using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Editor;

[NavmeshCustomizationIgnore]
public sealed class CustomizationDraftCustomization
(
    NavmeshCustomization baseCustomization,
    CustomizationDraft   draft
) : NavmeshCustomization
{
    public NavmeshCustomization BaseCustomization { get; } = baseCustomization;
    public CustomizationDraft    Draft             { get; } = draft;

    public override int Version
    {
        get
        {
            var hash = 0x811C9DC5u;
            hash = Fnv1A(hash, BaseCustomization.GetType().FullName ?? BaseCustomization.GetType().Name);
            hash = Fnv1A(hash, BaseCustomization.Version);
            hash = Fnv1A(hash, Draft.ComputeVersion());
            return unchecked((int)hash);
        }
    }

    public override bool IsFlyingSupported(SceneDefinition definition) =>
        Draft.FlyingSupportedOverride ?? BaseCustomization.IsFlyingSupported(definition);

    public override NavmeshSettings GetBuildSettings(SceneDefinition definition)
    {
        var settings = new NavmeshSettings();
        ApplyBuildSettings(definition, settings);
        return settings;
    }

    protected internal override void ApplyBuildSettings(SceneDefinition definition, NavmeshSettings settings)
    {
        BaseCustomization.ApplyBuildSettings(definition, settings);
        var profile  = new NavmeshBuildProfile();
        CustomizationDraftApplier.ApplyBuildProfile(profile, Draft);
        profile.ApplyTo(settings);
        CustomizationDraftApplier.ApplyBuildSettings(settings, Draft);
    }

    public override void CustomizeScene(SceneExtractor scene)
    {
        BaseCustomization.CustomizeScene(scene);
        CustomizationDraftApplier.ApplyScene(scene, Draft);
    }

    public override void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile)
    {
        BaseCustomization.CustomizeBuildProfile(definition, profile);
        CustomizationDraftApplier.ApplyBuildProfile(profile, Draft);
    }

    public override void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings)
    {
        BaseCustomization.CustomizeBuildSettings(definition, settings);
        CustomizationDraftApplier.ApplyBuildSettings(settings, Draft);
    }

    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        BaseCustomization.CustomizeSettings(config);
        CustomizationDraftApplier.ApplySettings(config, Draft);
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        BaseCustomization.CustomizeMesh(mesh, festivalLayers);

        foreach (var link in Draft.MeshLinks)
        {
            if (!link.Enabled)
                continue;

            switch (link.Kind)
            {
                case DraftMeshLinkKind.Points:
                    LinkPoints(mesh, link.Start, link.End);
                    break;
                case DraftMeshLinkKind.Drop:
                    LinkDrop(mesh, link.Start, link.End);
                    break;
                case DraftMeshLinkKind.ClientPath:
                    LinkClientPath(mesh, link.Start, link.End);
                    break;
            }
        }
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
