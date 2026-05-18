using System.Reflection;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
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
        var sceneTypes = new List<Type>();
        var sceneInstances = new Dictionary<Type, SceneNavmeshCustomization>();

        foreach (var t in Assembly
                     .GetExecutingAssembly()
                     .DefinedTypes
                     .Where(t => t.IsSubclassOf(baseType) && !t.IsAbstract && !t.IsDefined(typeof(NavmeshCustomizationIgnoreAttribute), false))
                     .OrderBy(t => t.Namespace?.Contains(".Generated", StringComparison.Ordinal) == true ? 1 : 0)
                     .ThenBy(t => t.FullName ?? t.Name, StringComparer.Ordinal))
        {
            var instance = Activator.CreateInstance(t) as NavmeshCustomization;

            if (instance == null)
            {
                Service.Log.Error($"Failed to create instance of customization class {t}");
                continue;
            }

            if (instance is SceneNavmeshCustomization sceneCustomization)
            {
                sceneTypes.Add(t.AsType());
                sceneInstances.Add(t.AsType(), sceneCustomization);
            }

            foreach (var attr in t.GetCustomAttributes<CustomizationTerritoryAttribute>())
            {
                if (!PerTerritory.TryAdd(attr.TerritoryID, instance))
                    Service.Log.Warning($"忽略重复的 Territory 自定义: {t.FullName} -> {attr.TerritoryID}");
            }
        }

        PerScene = OrderSceneCustomizations(sceneTypes, sceneInstances);
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

    private static List<SceneNavmeshCustomization> OrderSceneCustomizations(List<Type> sceneTypes, IReadOnlyDictionary<Type, SceneNavmeshCustomization> sceneInstances)
    {
        var sceneTypeSet = sceneTypes.ToHashSet();
        var outgoing = sceneTypes.ToDictionary(static t => t, static _ => new HashSet<Type>());
        var incomingCounts = sceneTypes.ToDictionary(static t => t, static _ => 0);

        foreach (var sceneType in sceneTypes)
        {
            foreach (var attr in sceneType.GetCustomAttributes<CustomizationScenePriorityAboveAttribute>())
            {
                if (!typeof(SceneNavmeshCustomization).IsAssignableFrom(attr.LowerPriorityType))
                    throw new InvalidOperationException($"{sceneType.FullName} 指定了无效的 Scene 自定义优先级目标: {attr.LowerPriorityType.FullName}");

                if (!sceneTypeSet.Contains(attr.LowerPriorityType))
                    continue;

                if (attr.LowerPriorityType == sceneType)
                    throw new InvalidOperationException($"{sceneType.FullName} 不能将自己声明为更低优先级");

                if (outgoing[attr.LowerPriorityType].Add(sceneType))
                    incomingCounts[sceneType]++;
            }
        }

        var ready = new PriorityQueue<Type, string>();
        foreach (var sceneType in sceneTypes.Where(t => incomingCounts[t] == 0))
            ready.Enqueue(sceneType, sceneType.FullName ?? sceneType.Name);

        List<SceneNavmeshCustomization> ordered = [];

        while (ready.Count > 0)
        {
            var current = ready.Dequeue();
            ordered.Add(sceneInstances[current]);

            foreach (var next in outgoing[current])
            {
                incomingCounts[next]--;
                if (incomingCounts[next] == 0)
                    ready.Enqueue(next, next.FullName ?? next.Name);
            }
        }

        if (ordered.Count != sceneTypes.Count)
        {
            var blocked = sceneTypes.Where(t => incomingCounts[t] > 0).Select(t => t.FullName ?? t.Name).OrderBy(static n => n, StringComparer.Ordinal).ToArray();
            throw new InvalidOperationException($"Scene 自定义优先级存在循环依赖: {string.Join(" -> ", blocked)}");
        }

        return ordered;
    }
}
