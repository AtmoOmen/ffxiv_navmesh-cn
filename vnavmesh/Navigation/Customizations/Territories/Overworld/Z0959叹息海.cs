using DotRecast.Detour;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;

namespace vnavmesh.Navigation.Customizations.Territories.Overworld;

[CustomizationTerritory(959)]
public class Z0959叹息海 : NavmeshCustomization
{
    public override int Version => 1;

    public Z0959叹息海()
    {
        Settings.AgentRadius = 0.5f;
        Settings.AgentHeight = 1.5f;
        Settings.CellSize    = 0.25f;
        Settings.CellHeight  = 0.125f;
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        // 最佳威兔洞
        LinkPoints(mesh, new(10.6f, -105.5f, -379.6f), new(53.5f, -77.8f, -379.7f));

        LinkPoints(mesh, new(53.2f, -77.8f, -356.6f), new(10.2f, -49.9f, -356.2f));
        
        LinkPoints(mesh, new(-0.6f, -49.9f, -354.6f), new(1.2f, -29.7f, -222.4f));
    }
}
