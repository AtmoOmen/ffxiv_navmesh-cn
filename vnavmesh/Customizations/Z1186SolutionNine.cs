using System.Collections.Generic;
using System.Numerics;
using Navmesh;

namespace vnavmesh.Customizations;

[CustomizationTerritory(1186)]
internal class Z1186SolutionNine : NavmeshCustomization
{
    public override int Version => 1;
    public override void CustomizeMesh(Navmesh.Navmesh mesh, List<uint> festivalLayers)
    {
        Vector3 AetherytePlazaEastDepart = new(143.61594f, 0.55f, 4.898482f);
        Vector3 AetherytePlazaEastArrive = new(219.98926f, 60.7f, 4.989685f);
        Vector3 TrueVueWestDepart = new(221.16905f, 60.7f, -4.981979f);
        Vector3 TrueVueWestArrive = new(146.59338f, 0.55f, -5.0202637f);

        LinkPoints(mesh, AetherytePlazaEastDepart, AetherytePlazaEastArrive);
        LinkPoints(mesh, TrueVueWestDepart, TrueVueWestArrive);

        Vector3 AetherytePlazaNorthDepart = new(4.97122f, 0.5f, -114.9926f);
        Vector3 AetherytePlazaNorthArrive = new(4.989685f, 36.7f, -170.0008f);
        Vector3 ResolutionSouthDepart = new(-5.040139f, 36.69999f, -172.79323f);
        Vector3 ResolutionSouthArrive = new(-5.0202637f, 0.5f, -117.021484f);

        LinkPoints(mesh, AetherytePlazaNorthDepart, AetherytePlazaNorthArrive);
        LinkPoints(mesh, ResolutionSouthDepart, ResolutionSouthArrive);

        Vector3 ArcadeLiftGroundDepart = new(-218.04851f, 1.1641076f, -66.99873f);
        Vector3 ArcadeLiftGroundArrive = new(-224.56708f, 36.1f, -73.56378f);
        Vector3 ArcadeLiftTopDepart = new(-228.85681f, 36.1f, -70.94159f);
        Vector3 ArcadeLiftTopArrive = new(-222.30872f, 1.0844975f, -64.19476f);

        LinkPoints(mesh, ArcadeLiftGroundDepart, ArcadeLiftGroundArrive);
        LinkPoints(mesh, ArcadeLiftTopDepart, ArcadeLiftTopArrive);
    }
}

