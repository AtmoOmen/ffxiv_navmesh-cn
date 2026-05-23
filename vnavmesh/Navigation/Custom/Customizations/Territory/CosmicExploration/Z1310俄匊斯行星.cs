using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Custom.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Territory.CosmicExploration;

[CustomizationTerritory(1310)]
internal class Z1310俄匊斯行星 : NavmeshCustomization
{
    public override int Version => 4;

    public override void CustomizeScene(SceneExtractor scene)
    {
        string[] doubleLiners = ["bg/ffxiv/cos_c1/hou/c1w3/collision/c1w3_03_t200a.pcb"];

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

        if (scene.Meshes.TryGetValue("bg/ffxiv/cos_c1/hou/c1w3/collision/c1w3_03_t600a.pcb", out var mesh))
        {
            var box = SceneExtractor.BuildBoxMesh()[0];

            foreach (ref var vert in CollectionsMarshal.AsSpan(box.Vertices))
            {
                vert *= new Vector3(1.5f, 3.75f, 1.5f);
                vert += new Vector3(0,    6.25f, -1);
            }

            mesh.Parts.Add(box);
        }
    }

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        var festivalVersion = festivalLayers.LastOrDefault() >> 16;

        // add jump down point for a raised rock that a drone spawns on
        LinkPoints(mesh, new(148.5f, -92, -540), new(150.25f, -92.725f, -536f));

        #region base

        // base -> N
        LinkClientPath(mesh, new(-175.5f, 3f, 52.8f),       new(-145.5f, -20.3f, -214.8f));
        LinkClientPath(mesh, new(-154.5f, -20.5f, -213.8f), new(-184.5f, 3.2f, 53.8f));

        // base -> W
        LinkClientPath(mesh, new(-298.2f, 3f, 133.5f),    new(-519.8f, 25.7f, 115.5f));
        LinkClientPath(mesh, new(-518.8f, 25.5f, 124.5f), new(-297.2f, 3.2f, 142.5f));

        // base -> E
        LinkClientPath(mesh, new(-61.8f, 3f, 142.5f), new(157.8f, 2.2f, -3.5f));
        LinkClientPath(mesh, new(156.8f, 2f, -12.5f), new(-62.8f, 3.2f, 133.5f));

        #endregion

        #region E

        // North
        LinkClientPath(mesh, new(184.5f, 2f, -31.2f),    new(196.5f, -51.8f, -377.8f));
        LinkClientPath(mesh, new(187.5f, -52f, -376.8f), new(175.5f, 2.2f, -30.2f));

        // East
        LinkClientPath(mesh, new(203.2f, 2f, -3.5f),     new(500.5f, -49.8f, -290.8f));
        LinkClientPath(mesh, new(491.5f, -50f, -289.8f), new(202.2f, 2.2f, -12.5f));

        #endregion

        #region NE

        // E -> Far East (against map)
        LinkClientPath(mesh, new(215.2f, -52f, -395.5f), new(473.8f, -49.8f, -308.5f));
        LinkClientPath(mesh, new(472.8f, -50f, -317.5f), new(214.2f, -51.8f, -404.5f));

        // N -> Far N (against map, beside pin)
        LinkClientPath(mesh, new(196.5f, -52f, -423.2f),    new(314.5f, -151.3f, -603.8f));
        LinkClientPath(mesh, new(305.5f, -151.5f, -602.8f), new(187.5f, -51.8f, -422.2f));

        #endregion

        #region NE far

        // NE above location -> E below location name
        LinkClientPath(mesh, new(333.2f, -151.5f, -621.5f), new(491.5f, -49.8f, -335.2f));
        LinkClientPath(mesh, new(500.5f, -50f, -336.2f),    new(332.2f, -151.3f, -630.5f));

        if (festivalVersion >= 0x1A)
        {
            // NE cave
            LinkClientPath(mesh, new(286.8f, -151.5f, -630.5f),    new(92.692f, -190.3f, -780.797f));
            LinkClientPath(mesh, new(89.056f, -190.5f, -772.504f), new(287.8f, -151.3f, -621.5f));

            // entrance 1
            LinkClientPath(mesh, new(1.4f, -189.05f, -799.4f), new(-89.8f, -188.85f, -799.4f));
            // entrance 2
            LinkClientPath(mesh, new(-126.6f, -189.05f, -841.2f), new(-126.6f, -188.85f, -893.8f));
            // entrance 3
            LinkClientPath(mesh, new(-183.2f, -189.05f, -949.4f), new(-290.8f, -188.85f, -949.4f));
            // entrance 4
            LinkClientPath(mesh, new(-325.4f, -189.05f, -913.8f), new(-325.4f, -188.85f, -835.2f));

            // exit 4
            LinkClientPath(mesh, new(-306.6f, -189.05f, -836.2f), new(-306.6f, -188.85f, -914.8f));
            // exit 3
            LinkClientPath(mesh, new(-289.8f, -189.05f, -930.6f), new(-182.2f, -188.85f, -930.6f));
            // exit 2
            LinkClientPath(mesh, new(-145.4f, -189.05f, -892.8f), new(-145.4f, -188.85f, -840.2f));
            // exit 1
            LinkClientPath(mesh, new(-88.8f, -189.05f, -780.6f), new(2.4f, -188.85f, -780.6f));
        }

        #endregion

        #region EE

        if (festivalVersion >= 0x0B)
        {
            // shadefleet N <-> NNE
            LinkClientPath(mesh, new(728.5f, 220.75f, -124.2f), new(518.2f, -49.8f, -317.5f));
            LinkClientPath(mesh, new(519.2f, -50f, -308.5f),    new(719.5f, 220.95f, -123.2f));

            // shadefleet S <-> shadefleet N
            LinkClientPath(mesh, new(674.5f, 136f, 266.8f),    new(728.5f, 220.95f, -78.8f));
            LinkClientPath(mesh, new(719.5f, 220.75f, -77.8f), new(665.5f, 136.2f, 267.8f));

            // shadefleet S <-> mid east
            LinkClientPath(mesh, new(646.8f, 136f, 285.5f),       new(356.976f, 102.7f, 380.004f));
            LinkClientPath(mesh, new(362.343f, 102.5f, 387.297f), new(647.8f, 136.2f, 294.5f));
        }

        #endregion

        #region SE

        // North -> Mid East
        LinkClientPath(mesh, new(96.5f, 100f, 329.8f), new(184.5f, 2.2f, 14.2f));
        LinkClientPath(mesh, new(175.5f, 2f, 15.2f),   new(87.5f, 100.2f, 330.8f));

        // E -> Single on E
        LinkClientPath(mesh, new(115.2f, 100f, 357.5f),       new(323.024f, 102.7f, 409.996f));
        LinkClientPath(mesh, new(317.657f, 102.5f, 402.703f), new(114.2f, 100.2f, 348.5f));

        #endregion

        #region SW far

        // North Side -> SW Cosmo
        LinkClientPath(mesh, new(-440.5f, 104f, 746.8f), new(-383.5f, 47.2f, 423.2f));
        LinkClientPath(mesh, new(-392.5f, 47f, 424.2f),  new(-449.5f, 104.2f, 747.8f));

        // West Side -> West Cosmo
        LinkClientPath(mesh, new(-468.2f, 104f, 765.5f),  new(-656.5f, 30.7f, 453.2f));
        LinkClientPath(mesh, new(-665.5f, 30.5f, 454.2f), new(-467.2f, 104.2f, 774.5f));

        #endregion

        #region SW wall

        // N -> West of base
        LinkClientPath(mesh, new(-656.5f, 30.5f, 407.8f), new(-564.2f, 25.7f, 124.5f));
        LinkClientPath(mesh, new(-565.2f, 25.5f, 115.5f), new(-665.5f, 30.7f, 408.8f));

        // E -> SE Cosmoliner
        LinkClientPath(mesh, new(-637.8f, 30.5f, 435.5f), new(-410.2f, 47.2f, 405.5f));
        LinkClientPath(mesh, new(-411.2f, 47f, 396.5f),   new(-638.8f, 30.7f, 426.5f));

        #endregion

        #region SW

        // N -> West of base
        LinkClientPath(mesh, new(-383.5f, 47f, 377.8f),   new(-537.5f, 25.7f, 142.2f));
        LinkClientPath(mesh, new(-546.5f, 25.5f, 143.2f), new(-392.5f, 47.2f, 378.8f));

        // E -> SE of base
        LinkClientPath(mesh, new(-364.8f, 47f, 405.5f), new(69.8f, 100.2f, 357.5f));
        LinkClientPath(mesh, new(68.8f, 100f, 348.5f),  new(-365.8f, 47.2f, 396.5f));

        #endregion

        #region W

        // North -> NW (Below Erg Eris)
        LinkClientPath(mesh, new(-537.5f, 25.5f, 96.8f),  new(-525.5f, -24.8f, -217.8f));
        LinkClientPath(mesh, new(-534.5f, -25f, -216.8f), new(-546.5f, 25.7f, 97.8f));

        #endregion

        #region NW

        // N -> NW (Above Erg Eris, below flower looking hole)
        LinkClientPath(mesh, new(-525.5f, -25f, -263.2f),   new(-697.5f, -85.3f, -471.8f));
        LinkClientPath(mesh, new(-706.5f, -85.5f, -470.8f), new(-534.5f, -24.8f, -262.2f));

        // E -> Cosmoliner N of base
        LinkClientPath(mesh, new(-506.8f, -25f, -235.5f),   new(-172.2f, -20.3f, -232.5f));
        LinkClientPath(mesh, new(-173.2f, -20.5f, -241.5f), new(-507.8f, -24.8f, -244.5f));

        #endregion

        #region N

        // N -> Far North
        LinkClientPath(mesh, new(-145.5f, -20.5f, -260.2f), new(-127.5f, -72.3f, -559.8f));
        LinkClientPath(mesh, new(-136.5f, -72.5f, -558.8f), new(-154.5f, -20.3f, -259.2f));

        // E -> NE
        LinkClientPath(mesh, new(-126.8f, -20.5f, -232.5f), new(169.8f, -51.8f, -395.5f));
        LinkClientPath(mesh, new(168.8f, -52f, -404.5f),    new(-127.8f, -20.3f, -241.5f));

        #endregion

        #region NN

        // W -> Far NW (East Side
        LinkClientPath(mesh, new(-155.2f, -73f, -586.5f),   new(-679.8f, -85.3f, -498.5f));
        LinkClientPath(mesh, new(-678.8f, -85.5f, -489.5f), new(-154.2f, -72.8f, -577.5f));

        // N -> FARR NW/Against N Wall
        LinkClientPath(mesh, new(-127.5f, -73f, -605.2f),    new(-457.8f, -102.3f, -764.5f));
        LinkClientPath(mesh, new(-456.8f, -102.5f, -755.5f), new(-136.5f, -72.8f, -604.2f));

        #endregion

        #region NNW

        // North -> NW Cosmoliner
        LinkClientPath(mesh, new(-697.5f, -85.5f, -517.2f),  new(-502.2f, -102.3f, -755.5f));
        LinkClientPath(mesh, new(-503.2f, -102.5f, -764.5f), new(-706.5f, -85.3f, -516.2f));

        #endregion

    }
}
