using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Custom.Extensions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.CosmicExploration;

[CustomizationTerritory(1237)]
internal class Z1237憧憬湾 : NavmeshCustomization
{
    public override int Version => 7;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        // annoying rock
        scene.InsertCylinderCollider(new Vector3(2, 10, 2), new Vector3(-206.5f, 29, 301.5f));

        // add box collider blocking entry path for green (depart) cosmoliner to discourage vnav from going through green cosmoliners on short paths
        // later we add off-mesh connections between liners, so those will be used for long paths
        string[] doubleLiners = ["bg/ffxiv/cos_c1/hou/common/collision/c1w0_03_t300a.pcb", "bg/ffxiv/cos_c1/hou/common/collision/c1w0_03_t200a.pcb"];

        foreach (var liner in doubleLiners)
        {
            if (scene.Meshes.TryGetValue(liner, out var cl))
            {
                var box = SceneExtractor.BuildBoxMesh()[0];

                foreach (ref var vert in CollectionsMarshal.AsSpan(box.Vertices))
                {
                    vert *= new Vector3(1.5f, 3.75f, 1.5f);
                    vert += new Vector3(4.5f, 6.25f, -1);
                }

                cl.Parts.Add(box);
            }
        }

        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/common/collision/c1w0_03_t100a.pcb", out var mesh))
        {
            var box = SceneExtractor.BuildBoxMesh()[0];

            foreach (ref var vert in CollectionsMarshal.AsSpan(box.Vertices))
            {
                vert *= new Vector3(1.5f, 3.75f, 1.5f);
                vert += new Vector3(0,    6.25f, -1);
            }

            mesh.Parts.Add(box);
        }

        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/common/collision/c1w0_00_bx00d.pcb", out var mesh2))
        {
            foreach (var inst in mesh2.Instances)
                inst.WorldTransform.Row1.Y *= 2;
        }
    }

    public override void CustomizeMesh
    (
        Navmesh    mesh,
        List<uint> festivalLayers
    )
    {

        #region base liners

        // base <-> N
        LinkPoints(mesh, new(4.500f, 3.500f, -62.200f),    new(4.500f, 40.700f, -377.300f));
        LinkPoints(mesh, new(-4.500f, 40.500f, -378.800f), new(-4.500f, 3.700f, -63.700f));
        // base <-> E
        LinkPoints(mesh, new(62.200f, 3.500f, 4.500f),     new(377.300f, 42.700f, 4.500f));
        LinkPoints(mesh, new(378.800f, 42.500f, -4.500f),  new(63.700f, 3.700f, -4.500f));
        // base <-> S
        LinkPoints(mesh, new(-4.500f, 3.500f, 62.200f),    new(-4.500f, 37.700f, 377.300f));
        LinkPoints(mesh, new(4.500f, 37.500f, 378.800f),   new(4.500f, 3.700f, 63.700f));
        // base <-> W
        LinkPoints(mesh, new(-62.200f, 3.500f, -4.500f),   new(-377.300f, 38.700f, -4.500f));
        LinkPoints(mesh, new(-378.800f, 38.500f, 4.500f),  new(-63.700f, 3.700f, 4.500f));

        #endregion

        #region inner ring liners

        // N <-> NE
        LinkPoints(mesh, new(21.200f, 40.500f, -395.500f),   new(269.388f, 43.200f, -316.112f));
        LinkPoints(mesh, new(277.247f, 43.000f, -320.747f),  new(22.700f, 40.700f, -404.500f));
        // NE <-> E
        LinkPoints(mesh, new(306.403f, 43.000f, -272.003f),  new(395.500f, 42.700f, -22.700f));
        LinkPoints(mesh, new(404.500f, 42.500f, -21.200f),   new(314.262f, 43.200f, -276.638f));
        // E <-> SE
        LinkPoints(mesh, new(395.500f, 42.500f, 21.200f),    new(292.869f, 27.700f, 260.768f));
        LinkPoints(mesh, new(298.175f, 27.500f, 268.190f),   new(404.500f, 42.700f, 22.700f));
        // SE <-> S
        LinkPoints(mesh, new(261.825f, 27.500f, 291.810f),   new(22.700f, 37.700f, 395.500f));
        LinkPoints(mesh, new(21.200f, 37.500f, 404.500f),    new(267.131f, 27.700f, 299.232f));
        // S <-> SW
        LinkPoints(mesh, new(-21.200f, 37.500f, 395.500f),   new(-260.765f, 31.700f, 292.871f));
        LinkPoints(mesh, new(-268.191f, 31.500f, 298.171f),  new(-22.700f, 37.700f, 404.500f));
        // SW <-> W
        LinkPoints(mesh, new(-291.809f, 31.500f, 261.829f),  new(-395.500f, 38.700f, 22.700f));
        LinkPoints(mesh, new(-404.500f, 38.500f, 21.200f),   new(-299.235f, 31.700f, 267.129f));
        // W <-> NW
        LinkPoints(mesh, new(-395.500f, 38.500f, -21.200f),  new(-292.869f, 36.700f, -260.767f));
        LinkPoints(mesh, new(-298.175f, 36.500f, -268.189f), new(-404.500f, 38.700f, -22.700f));
        // NW <-> N
        LinkPoints(mesh, new(-261.825f, 36.500f, -291.809f), new(-22.700f, 40.700f, -395.500f));
        LinkPoints(mesh, new(-21.200f, 40.500f, -404.500f),  new(-267.131f, 36.700f, -299.231f));

        #endregion

        #region NE caves

        // NE -> downstairs
        LinkPoints(mesh, new(322.35117f, 43, -306.3f),     new(404.5141f, -56.8f, -375.2349f));
        // downstairs -> NE
        LinkPoints(mesh, new(390.42264f, -57, -394.7571f), new(308.2982f, 43.2f, -325.8215f));

        // downstairs <-> NNEE
        LinkPoints(mesh, new(434.172f, -57.000f, -409.888f), new(627.882f, -71.800f, -553.477f));
        LinkPoints(mesh, new(623.526f, -72.000f, -561.494f), new(429.538f, -56.800f, -417.748f));

        // NNEE <-> NNEEEE
        LinkPoints(mesh, new(652.506f, -72.000f, -551.525f), new(866.105f, -55.300f, -369.717f));
        LinkPoints(mesh, new(873.037f, -55.500f, -375.649f), new(660.523f, -71.800f, -555.882f));

        // NNEE -> loop
        LinkPoints(mesh, new(625.157f, -71.970f, -583.71f),   new(366.911f, -117.3f, -834.9056f));
        LinkPoints(mesh, new(388.186f, -117.470f, -848.963f), new(622.1744f, -108.3f, -944.7515f));
        LinkPoints(mesh, new(646.472f, -108.480f, -924.356f), new(646.622f, -71.8f, -592.223f));

        #endregion

        #region SE

        // E <-> EE
        LinkPoints(mesh, new(421.200f, 42.500f, 4.500f),   new(721.300f, 61.700f, 0.500f));
        LinkPoints(mesh, new(722.800f, 61.500f, -8.500f),  new(422.700f, 42.700f, -4.500f));
        // EE <-> SSEE
        LinkPoints(mesh, new(739.500f, 61.500f, 17.200f),  new(439.967f, 47.700f, 491.452f));
        LinkPoints(mesh, new(445.273f, 47.500f, 498.874f), new(748.500f, 61.700f, 18.700f));
        // SE <-> SSEE
        LinkPoints(mesh, new(291.809f, 27.500f, 298.171f), new(407.863f, 47.700f, 497.813f));
        LinkPoints(mesh, new(415.289f, 47.500f, 492.513f), new(299.235f, 27.700f, 292.871f));
        // SSEE <-> SS
        LinkPoints(mesh, new(408.923f, 47.500f, 522.493f), new(-77.300f, 53.200f, 745.500f));
        LinkPoints(mesh, new(-78.800f, 53.000f, 754.500f), new(414.229f, 47.700f, 529.915f));
        // S <-> SS
        LinkPoints(mesh, new(-4.500f, 37.500f, 421.200f),  new(-104.500f, 53.200f, 727.300f));
        LinkPoints(mesh, new(-95.500f, 53.000f, 728.800f), new(4.500f, 37.700f, 422.700f));

        #endregion

        #region SW crater

        // SS -> tunnel
        LinkPoints(mesh, new(-122.029f, 55, 740.012f), new(-316.2774f, 55.2f, 740));
        // tunnel -> SS
        LinkPoints(mesh, new(-317.979f, 55, 759.97f),  new(-123.75f, 55.2f, 760));

        // crater SE <-> N
        LinkPoints(mesh, new(-335.500f, 53.000f, 728.800f), new(-597.300f, 52.700f, 385.500f));
        LinkPoints(mesh, new(-598.800f, 52.500f, 394.500f), new(-344.500f, 53.200f, 727.300f));
        // crater N <-> SW
        LinkPoints(mesh, new(-641.200f, 52.500f, 385.500f), new(-738.500f, 93.700f, 741.300f));
        LinkPoints(mesh, new(-729.500f, 93.500f, 742.800f), new(-642.700f, 52.700f, 394.500f));
        // crater SW <-> SE
        LinkPoints(mesh, new(-712.800f, 93.500f, 768.500f), new(-362.700f, 53.200f, 754.500f));
        LinkPoints(mesh, new(-361.200f, 53.000f, 745.500f), new(-711.300f, 93.700f, 759.500f));

        #endregion

        #region NW

        // W <-> WW
        LinkPoints(mesh, new(-421.200f, 38.500f, -4.500f),   new(-676.300f, 62.700f, 5.500f));
        LinkPoints(mesh, new(-677.800f, 62.500f, 14.500f),   new(-422.700f, 38.700f, 4.500f));
        // NW <-> NNWW
        LinkPoints(mesh, new(-291.809f, 36.500f, -298.170f), new(-520.765f, 61.700f, -527.129f));
        LinkPoints(mesh, new(-528.191f, 61.500f, -521.829f), new(-299.235f, 36.700f, -292.870f));
        // WW <-> NNWW
        LinkPoints(mesh, new(-694.500f, 62.500f, -11.200f),  new(-552.869f, 61.700f, -520.768f));
        LinkPoints(mesh, new(-558.175f, 61.500f, -528.190f), new(-703.500f, 62.700f, -12.700f));

        #endregion

    }
}
