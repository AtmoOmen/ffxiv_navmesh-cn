using System.Numerics;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Implementations.Territory.Overworld;

[CustomizationTerritory(155)]
internal class Z0155库尔札斯中央高地 : NavmeshCustomization
{
    public override int Version => 9;

    public Z0155库尔札斯中央高地() =>
        ApplyAgentRadiusOneSettings();

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        // add a fake stair in front of the disconnected second staircase on the second floor of the building in Whitebrim Front
        scene.InsertAABoxCollider(new Vector3(1.5f, 0.15f, 0.2f), new(-417.5f, 221.5f, -288.8f));

        // set the doorstep of Monument Tower as walkable, though you can't land on it
        if (scene.Meshes.TryGetValue("bg/ffxiv/roc_r1/fld/r1f1/collision/r1f1_b7_astr1.pcb", out var tower))
        {
            foreach (var inst in tower.Instances)
                inst.ForceSetPrimFlags |= PrimitiveFlags.ForceWalkable;
        }
    }

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {
        // 白云崖前哨门洞
        List<Vector3> doorwayWhitebrim =
        [
            new(-454.47195f, 211.4575f, -291.54233f),
            new(-457.46005f, 211.4575f, -291.67108f)
        ];

        for (var i = 0; i < doorwayWhitebrim.Count - 1; i++)
            LinkShortcut(mesh, doorwayWhitebrim[i], doorwayWhitebrim[i + 1], true);

        // 阿德内尔占星台与巨龙首之间的门洞
        List<Vector3> doorwayMartiallais =
        [
            new(198.35889f, 257f, 76.33306f),
            new(205.2478f, 256.14615f, 83.13481f)
        ];

        for (var i = 0; i < doorwayMartiallais.Count - 1; i++)
            LinkShortcut(mesh, doorwayMartiallais[i], doorwayMartiallais[i + 1], true);

        // 巨龙首营地
        List<Vector3> walkway =
        [
            new(279.8393f, 319f, -232.66096f),
            new(279.70917f, 324.0579f, -225.59511f),
            new(279.66046f, 324.0003f, -221.4777f),
            new(279.6848f, 324.0004f, -200.20921f),
            new(279.67496f, 324.0007f, -195.25887f),
            new(279.564f, 324.0007f, -186.17705f),
            new(279.57233f, 324.0007f, -182.04602f),
            new(279.4925f, 324f, -159.71964f),
            new(266.03177f, 324f, -156.23119f),
            new(261.66354f, 323.969f, -156.22154f)
        ];

        for (var i = 0; i < walkway.Count - 1; i++)
            LinkShortcut(mesh, walkway[i], walkway[i + 1], true);
    }
}
