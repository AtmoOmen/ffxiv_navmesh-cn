using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Territories.Town;

[CustomizationTerritory(131)]
internal class Z0131乌尔达哈来生回廊 : NavmeshCustomization
{
    public override int Version => 2;

    public override void CustomizeScene(SceneExtractor scene)
    {
        // Force all instances of Mesh as unwalkable
        if (scene.Meshes.TryGetValue("bg/ffxiv/wil_w1/twn/common/collision/w1t0_f0_kadn1.pcb", out var mesh))
        {
            foreach (var instance in mesh.Instances)
                instance.ForceSetPrimFlags |= SceneExtractor.PrimitiveFlags.ForceUnwalkable;
        }
    }
    
    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 很诡异但是得要有
        LinkPoints(mesh, new(-87.3f, 42.0f, 48.3f), new(-83.9f, 42.0f, 35.3f));
        
        LinkPoints(mesh, new(-22.7f, 30.0f, -6.6f), new(-68.6f, 37.9f, -18.6f));
    }
}
