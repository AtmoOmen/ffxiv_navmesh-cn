using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories.Alexandar;

[CustomizationTerritory(444)]
internal class Z0444启动之章3 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0444启动之章3()
    {
        Settings.AgentRadius = 0.3f;
        Settings.AgentHeight = 1f;
        Settings.FastBuild   = false;
        Settings.CellSize    = 0.125f;
        Settings.CellHeight  = 0.125f;
    }
    
    public override void CustomizeScene(SceneExtractor scene)
    {
        // Force all instances of Mesh as unwalkable
        if (scene.Meshes.TryGetValue("bg/ex1/02_dra_d2/alx/d2a3/collision/tr1615.pcb", out var mesh))
        {
            foreach (var instance in mesh.Instances)
                instance.ForceSetPrimFlags |= SceneExtractor.PrimitiveFlags.ForceWalkable;
        }
    }
    
    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 传送带
        LinkPoints(mesh, new(58.3f, -0.0f, -20.4f), new(58.0f, -9.0f, -41.8f));
        
        // 进门前
        LinkPoints(mesh, new(41.5f, 0.0f, -0.0f),  new(56.1f, 0.0f, 0.1f));
        LinkPoints(mesh, new(29.9f, 0.0f, -11.1f), new(35.2f, 0.0f, 0.1f));
        LinkPoints(mesh, new(35.3f, 0.0f, -6.0f),  new(40.2f, 0.0f, 0.5f));
        LinkPoints(mesh, new(41.5f, 0.0f, 0.0f),   new(47.3f, 0.0f, 0.2f));
        
        // 两个下落点
        LinkDrop(mesh, new(21.6f, 36.0f, -13.3f), new(25.0f, 23.6f, -20.7f));
        LinkDrop(mesh, new(17.9f, 18.0f, -17.5f), new(20.0f, 8.8f, -23.6f));
    }
}
