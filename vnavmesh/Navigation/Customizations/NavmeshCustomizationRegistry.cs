using System.Reflection;
using vnavmesh.Bootstrap;

namespace vnavmesh.Navigation.Customizations;

// registry containing all customizations
public static class NavmeshCustomizationRegistry
{
    public static NavmeshCustomization                   Default      = new();
    public static Dictionary<uint, NavmeshCustomization> PerTerritory = new();

    static NavmeshCustomizationRegistry()
    {
        var baseType = typeof(NavmeshCustomization);

        foreach (var t in Assembly.GetExecutingAssembly().DefinedTypes.Where(t => t.IsSubclassOf(baseType)))
        {
            var instance = Activator.CreateInstance(t) as NavmeshCustomization;

            if (instance == null)
            {
                Service.Log.Error($"Failed to create instance of customization class {t}");
                continue;
            }

            foreach (var attr in t.GetCustomAttributes<CustomizationTerritoryAttribute>()) PerTerritory.Add(attr.TerritoryID, instance);
        }
    }

    public static NavmeshCustomization GetForTerritory(uint id) => PerTerritory.GetValueOrDefault(id, Default);
}
