using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Custom.Attributes;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Territory.CosmicExploration;

[CustomizationTerritory(1291)]
internal class Z1291法恩娜行星 : NavmeshCustomization
{
    public override int Version => 8;

    public override void CustomizeScene
    (
        SceneExtractor scene
    )
    {
        string[] doubleLiners = ["bg/ffxiv/cos_c1/hou/c1w2/collision/c1w2_03_t200a.pcb"];

        foreach (var liner in doubleLiners)
        {
            if (scene.Meshes.TryGetValue(liner, out var cl))
            {
                // prevent agent from trying to climb the side of the ramp of a green cosmoliner - can cause issues if idiots set a very high path tolerance
                var departVerts = CollectionsMarshal.AsSpan(cl.Parts[29].Vertices);
                departVerts[129].Y += 1;
                departVerts[130].Y += 1;
                departVerts[132].Y += 1;
                departVerts[133].Y += 1;

                var box = SceneExtractor.BuildBoxMesh()[0];

                foreach (ref var vert in CollectionsMarshal.AsSpan(box.Vertices))
                {
                    vert *= new Vector3(1.5f, 3.75f, 1.5f);
                    vert += new Vector3(4.5f, 6.25f, -1);
                }

                cl.Parts.Add(box);
            }
        }

        // shitty rocks 1
        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/c1w2/collision/c1w2_t0_roc31.pcb", out var rock))
        {
            var p = SceneExtractor.BuildBoxMesh()[0];

            foreach (ref var vert in CollectionsMarshal.AsSpan(p.Vertices))
            {
                vert *= new Vector3(0.5f, 2, 0.5f);
                vert += new Vector3(-1,   0, -0.5f);
            }

            rock.Parts.Add(p);
        }

        // shitty rocks 2
        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/c1w2/collision/c1w2_t0_roc32.pcb", out var rock2))
        {
            foreach (var inst in rock2.Instances)
                inst.WorldTransform.M22 *= 2;
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
        LinkPoints(mesh, new(344.500f, 55.000f, -485.200f),  new(304.500f, 137.700f, -757.800f));
        LinkPoints(mesh, new(295.500f, 137.500f, -756.800f), new(335.500f, 55.200f, -484.200f));
        // base <-> E
        LinkPoints(mesh, new(405.200f, 55.000f, -415.500f),  new(757.800f, 54.700f, -425.500f));
        LinkPoints(mesh, new(756.800f, 54.500f, -434.500f),  new(404.200f, 55.200f, -424.500f));
        // base <-> S
        LinkPoints(mesh, new(335.500f, 55.000f, -354.800f),  new(325.500f, 55.200f, -150.200f));
        LinkPoints(mesh, new(334.500f, 55.000f, -151.200f),  new(344.500f, 55.200f, -355.800f));
        // base <-> W
        LinkPoints(mesh, new(274.800f, 55.000f, -424.500f),  new(-53.800f, 28.200f, -406.500f));
        LinkPoints(mesh, new(-52.800f, 28.000f, -397.500f),  new(275.800f, 55.200f, -415.500f));

        #endregion

        #region inner ring liners

        // N <-> NE
        LinkPoints(mesh, new(323.200f, 137.500f, -775.500f), new(686.452f, 47.700f, -725.524f));
        LinkPoints(mesh, new(690.087f, 47.500f, -733.818f),  new(322.200f, 137.700f, -784.500f));
        // NE <-> E
        LinkPoints(mesh, new(708.354f, 47.500f, -698.892f),  new(775.500f, 54.700f, -452.200f));
        LinkPoints(mesh, new(784.500f, 54.500f, -453.200f),  new(716.469f, 47.700f, -702.909f));
        // E <-> SE
        LinkPoints(mesh, new(775.500f, 54.500f, -406.800f),  new(725.500f, 38.700f, -59.200f));
        LinkPoints(mesh, new(734.500f, 38.500f, -60.200f),   new(784.500f, 54.700f, -407.800f));
        // SE <-> S
        LinkPoints(mesh, new(706.800f, 38.500f, -41.500f),   new(352.200f, 55.200f, -132.500f));
        LinkPoints(mesh, new(353.200f, 55.000f, -123.500f),  new(707.800f, 38.700f, -32.500f));
        // S <-> SW
        LinkPoints(mesh, new(306.800f, 55.000f, -132.500f),  new(22.515f, -7.300f, -145.878f));
        LinkPoints(mesh, new(29.589f, -7.500f, -140.224f),   new(307.800f, 55.200f, -123.500f));
        // SW <-> W
        LinkPoints(mesh, new(-3.222f, -7.500f, -146.586f),   new(-71.500f, 28.200f, -379.800f));
        LinkPoints(mesh, new(-80.500f, 28.000f, -378.800f),  new(-8.882f, -7.300f, -139.517f));
        // W <-> NW
        LinkPoints(mesh, new(-71.500f, 28.000f, -425.200f),  new(-126.581f, 65.200f, -733.275f));
        LinkPoints(mesh, new(-135.460f, 65.000f, -731.497f), new(-80.500f, 28.200f, -424.200f));
        // NW <-> N
        LinkPoints(mesh, new(-109.498f, 65.000f, -752.540f), new(277.800f, 137.700f, -775.500f));
        LinkPoints(mesh, new(276.800f, 137.500f, -784.500f), new(-111.274f, 65.200f, -761.419f));
        // S <-> soda-lime float
        LinkPoints(mesh, new(325.500f, 55.000f, -104.800f),  new(250.500f, -6.800f, 109.800f));
        LinkPoints(mesh, new(259.500f, -7.000f, 108.800f),   new(334.500f, 55.200f, -105.800f));
        // soda-lime float <-> SW
        LinkPoints(mesh, new(231.800f, -7.000f, 127.500f),   new(28.882f, -7.300f, -114.483f));
        LinkPoints(mesh, new(23.222f, -7.500f, -107.414f),   new(232.800f, -6.800f, 136.500f));

        #endregion

        #region peninsula

        // soda-lime float <-> peninsula E
        LinkPoints(mesh, new(250.500f, -7.000f, 155.200f),  new(180.500f, -2.800f, 407.800f));
        LinkPoints(mesh, new(189.500f, -3.000f, 406.800f),  new(259.500f, -6.800f, 154.200f));
        // peninsula E <-> peninsula SW
        LinkPoints(mesh, new(180.500f, -3.000f, 453.200f),  new(-65.800f, 36.700f, 655.500f));
        LinkPoints(mesh, new(-64.800f, 36.500f, 664.500f),  new(189.500f, -2.800f, 452.200f));
        // peninsula E <-> peninsula NW
        LinkPoints(mesh, new(161.800f, -3.000f, 425.500f),  new(-137.800f, 31.200f, 300.500f));
        LinkPoints(mesh, new(-136.800f, 31.000f, 309.500f), new(162.800f, -2.800f, 434.500f));
        // peninsula SW <-> peninsula NW
        LinkPoints(mesh, new(-83.500f, 36.500f, 636.800f),  new(-155.500f, 31.200f, 327.200f));
        LinkPoints(mesh, new(-164.500f, 31.000f, 328.200f), new(-92.500f, 36.700f, 637.800f));

        #endregion

        #region scoresheen sands

        // N sands <-> NW
        LinkPoints(mesh, new(-620.411f, 0.500f, -653.224f),  new(-154.726f, 65.200f, -748.581f));
        LinkPoints(mesh, new(-156.502f, 65.000f, -757.460f), new(-627.484f, 0.700f, -658.878f));
        // N sands <-> E1 sands
        LinkPoints(mesh, new(-626.778f, 0.500f, -620.414f),  new(-424.996f, 0.700f, -426.975f));
        LinkPoints(mesh, new(-417.704f, 0.500f, -432.344f),  new(-621.118f, 0.700f, -627.483f));
        // N sands <-> W sands
        LinkPoints(mesh, new(-659.589f, 0.500f, -626.776f),  new(-772.500f, 16.200f, -292.200f));
        LinkPoints(mesh, new(-763.500f, 16.000f, -293.200f), new(-652.516f, 0.700f, -621.122f));
        // E1 sands <-> W
        LinkPoints(mesh, new(-387.657f, 0.500f, -417.703f),  new(-98.200f, 28.200f, -397.500f));
        LinkPoints(mesh, new(-99.200f, 28.000f, -406.500f),  new(-393.024f, 0.700f, -424.996f));
        // E1 sands <-> W sands
        LinkPoints(mesh, new(-432.343f, 0.500f, -402.297f),  new(-745.800f, 16.200f, -274.500f));
        LinkPoints(mesh, new(-744.800f, 16.000f, -265.500f), new(-426.976f, 0.700f, -395.004f));
        // E1 sands <-> E2 sands
        LinkPoints(mesh, new(-402.296f, 0.500f, -387.656f),  new(-328.882f, -2.300f, -147.517f));
        LinkPoints(mesh, new(-323.222f, -2.500f, -154.586f), new(-395.004f, 0.700f, -393.025f));
        // E2 sands <-> SW
        LinkPoints(mesh, new(-290.411f, -2.500f, -148.224f), new(-2.515f, -7.300f, -108.122f));
        LinkPoints(mesh, new(-9.589f, -7.500f, -113.776f),   new(-297.485f, -2.300f, -153.878f));
        // E2 sands <-> peninsula NW
        LinkPoints(mesh, new(-296.778f, -2.500f, -115.414f), new(-164.500f, 31.200f, 282.800f));
        LinkPoints(mesh, new(-155.500f, 31.000f, 281.800f),  new(-291.118f, -2.300f, -122.483f));
        // E2 sands <-> S sands
        LinkPoints(mesh, new(-329.589f, -2.500f, -121.776f), new(-557.800f, 27.200f, 45.500f));
        LinkPoints(mesh, new(-556.800f, 27.000f, 54.500f),   new(-322.515f, -2.300f, -116.122f));
        // W sands <-> S sands
        LinkPoints(mesh, new(-772.500f, 16.000f, -246.800f), new(-602.200f, 27.200f, 54.500f));
        LinkPoints(mesh, new(-603.200f, 27.000f, 45.500f),   new(-763.500f, 16.200f, -247.800f));

        #endregion

        #region pools

        // soda-lime float <-> pools E
        LinkPoints(mesh, new(278.200f, -7.000f, 136.500f),   new(826.387f, -165.300f, 418.230f));
        LinkPoints(mesh, new(834.502f, -165.500f, 414.213f), new(277.200f, -6.800f, 127.500f));
        // pools E <-> pools S
        LinkPoints(mesh, new(826.045f, -165.500f, 452.709f), new(547.141f, -217.300f, 744.355f));
        LinkPoints(mesh, new(549.691f, -217.500f, 753.044f), new(834.844f, -165.300f, 454.847f));
        // pools S <-> chasm
        LinkPoints(mesh, new(513.247f, -217.500f, 738.030f), new(410.577f, -227.300f, 252.646f));
        LinkPoints(mesh, new(401.540f, -227.500f, 252.064f), new(509.612f, -217.300f, 746.324f));
        // chasm <-> pools middle
        LinkPoints(mesh, new(432.065f, -227.500f, 238.460f), new(658.089f, -239.300f, 424.454f));
        LinkPoints(mesh, new(663.749f, -239.500f, 417.385f), new(432.645f, -227.300f, 229.424f));

        #endregion

        #region southwestern penis

        // southwestern penis 1
        LinkPoints(mesh, new(-584.500f, 27.000f, 73.200f),  new(-366.469f, 13.700f, 379.114f));
        LinkPoints(mesh, new(-359.177f, 13.500f, 373.745f), new(-575.500f, 27.200f, 72.200f));
        // southwestern penis 2
        LinkPoints(mesh, new(-360.819f, 13.500f, 413.125f), new(-581.800f, 30.700f, 710.500f));
        LinkPoints(mesh, new(-580.800f, 30.500f, 719.500f), new(-351.782f, 13.700f, 413.707f));

        #endregion

    }
}
