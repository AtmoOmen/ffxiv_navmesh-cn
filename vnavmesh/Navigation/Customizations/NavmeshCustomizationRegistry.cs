using System.Reflection;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations;

// registry containing all customizations
public static class NavmeshCustomizationRegistry
{
    public static NavmeshCustomization                   Default      = new();
    public static Dictionary<uint, NavmeshCustomization> PerTerritory = new();
    public static List<SceneNavmeshCustomization>        PerScene     = [];

    static NavmeshCustomizationRegistry()
    {
        var baseType = typeof(NavmeshCustomization);

        foreach (var t in Assembly.GetExecutingAssembly().DefinedTypes.Where(t => t.IsSubclassOf(baseType) && !t.IsAbstract))
        {
            var instance = Activator.CreateInstance(t) as NavmeshCustomization;

            if (instance == null)
            {
                Service.Log.Error($"Failed to create instance of customization class {t}");
                continue;
            }

            if (instance is SceneNavmeshCustomization sceneCustomization)
                PerScene.Add(sceneCustomization);

            foreach (var attr in t.GetCustomAttributes<CustomizationTerritoryAttribute>())
                PerTerritory.Add(attr.TerritoryID, instance);
        }

        PerScene.Sort(static (left, right) => StringComparer.Ordinal.Compare(left.GetType().FullName, right.GetType().FullName));
    }

    public static NavmeshCustomization GetForTerritory(uint id) => PerTerritory.GetValueOrDefault(id, Default);

    public static NavmeshCustomization GetForScene(SceneDefinition definition)
    {
        List<NavmeshCustomization> matches = [Default];

        foreach (var customization in PerScene)
        {
            if (customization.Matches(definition))
                matches.Add(customization);
        }

        if (PerTerritory.TryGetValue(definition.TerritoryID, out var territoryCustomization))
            matches.Add(territoryCustomization);

        return matches.Count == 1 ? matches[0] : new CompositeNavmeshCustomization(matches);
    }
}
