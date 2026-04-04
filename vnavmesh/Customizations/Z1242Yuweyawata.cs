using System.Numerics;
using vnavmesh.Navmesh;

namespace vnavmesh.Customizations;

[CustomizationTerritory(1242)]
internal class Z1242Yuweyawata : NavmeshCustomization
{
    public override int Version => 1;

    public override void CustomizeScene(SceneExtractor scene)
    {
        // avoid the hole in last boss arena
        scene.InsertCylinderCollider(new Vector3(11, 1, 11), new(34, -87.9f, -710), SceneExtractor.PrimitiveFlags.ForceUnwalkable);
    }
}
