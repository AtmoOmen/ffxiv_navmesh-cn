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
    public override int Version => 3;

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

    private const float pi  = MathF.PI;
    private const float hpi = pi / 2;

    public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
    {
        var festivalVersion = festivalLayers.LastOrDefault() >> 16;

        // add jump down point for a raised rock that a drone spawns on
        LinkPoints(mesh, new(148.5f, -92, -540), new(150.25f, -92.725f, -536f));

        #region base

        // base -> N
        AddCosmoliner(new(-180, 0.5f, 52), default, new(-150, -23, -213), new(pi, 0, pi));

        // base -> W
        AddCosmoliner(new(-299, 0.5f, 138), new Vector3(0, hpi, 0), new(-518, 23, 120), new(0, -hpi, 0));

        // base -> E
        AddCosmoliner(new(-61, 0.5f, 138), new Vector3(0, -hpi, 0), new(156, -0.5f, -8), new(0, hpi, 0));

        #endregion

        #region E

        // North
        AddCosmoliner(new(180, -0.5f, -32), default, new(192, -54.5f, -376), new(pi, 0, pi));

        // East
        AddCosmoliner(new(204, -0.5f, -8), new(0, -hpi, 0), new(496, -52.5f, -289), new(-pi, 0, pi));

        #endregion

        #region NE

        // E -> Far East (against map)
        AddCosmoliner(new(216, -54.5f, -400), new(0, -hpi, 0), new(472, -52.5f, -313), new(0, hpi, 0));

        // N -> Far N (against map, beside pin)
        AddCosmoliner(new(192, -54.5f, -424), default, new(310, -154, -602), new(-pi, 0, -pi));

        #endregion

        #region NE far

        // NE above location -> E below location name
        AddCosmoliner(new(334, -154, -626), new(0, -hpi, 0), new(496, -52.5f, -337), default);

        if (festivalVersion >= 0x1A)
        {
            // NE cave
            AddCosmoliner(new(286, -154, -626), new(0, hpi, 0), new(92, -193, -776), new(-pi, 1.047f, pi));

            // entrance 1
            AddSoloLiner(new(0.6f, -191.55f, -799.4f), new(0, hpi, 0), new(-88, -191.55f, -799.4f), new(0, -hpi, 0));
            // entrance 2
            AddSoloLiner(new(-126.6f, -191.55f, -842), default, new(-126.6f, -191.55f, -892), new(-pi, 0, -pi));
            // entrance 3
            AddSoloLiner(new(-184, -191.55f, -949.4f), new(0, hpi, 0), new(-289, -191.55f, -949.4f), new(0, -hpi, 0));
            // entrance 4
            AddSoloLiner(new(-325.4f, -191.55f, -913), new(pi, 0, pi), new(-325.4f, -191.55f, -837), default);

            // exit 4
            AddSoloLiner(new(-306.6f, -191.55f, -837), default, new(-306.6f, -191.55f, -913), new(pi, 0, pi));
            // exit 3
            AddSoloLiner(new(-289, -191.55f, -930.6f), new(0, -hpi, 0), new(-184, -191.55f, -930.6f), new(0, hpi, 0));
            // exit 2
            AddSoloLiner(new(-145.4f, -191.55f, -892), new(-pi, 0, -pi), new(-145.4f, -191.55f, -842), default);
            // exit 1
            AddSoloLiner(new(-88, -191.55f, -780.6f), new(0, -hpi, 0), new(0.6f, -191.55f, -780.6f), new(0, hpi, 0));
        }

        #endregion

        #region EE

        if (festivalVersion >= 0x0B)
        {
            // shadefleet N <-> NNE
            AddCosmoliner(new(724, 218.25f, -125), default, new(520, -52.5f, -313), new(0, -hpi, 0));

            // shadefleet S <-> shadefleet N
            AddCosmoliner(new(670, 133.5f, 266), default, new(724, 218.25f, -77), new(pi, 0, pi));

            // shadefleet S <-> mid east
            AddCosmoliner(new(646, 133.5f, 290), new(0, hpi, 0), new(360.785f, 100, 383), new(0, -1.047f, 0));
        }

        #endregion

        #region SE

        // North -> Mid East
        AddCosmoliner(new(92, 97.5f, 329), default, new(180, -0.5f, 16), new(pi, 0, pi));

        // E -> Single on E
        AddCosmoliner(new(116, 97.5f, 353), new(0, -hpi, 0), new(319.215f, 100, 407), new(pi, -1.047f, pi));

        #endregion

        #region SW far

        // North Side -> SW Cosmo
        AddCosmoliner(new(-445, 101.5f, 746), default, new(-388, 44.5f, 425), new(pi, 0, pi));

        // West Side -> West Cosmo
        AddCosmoliner(new(-469, 101.5f, 770), new(0, hpi, 0), new(-661, 28, 455), new(-pi, 0, -pi));

        #endregion

        #region SW wall

        // N -> West of base
        AddCosmoliner(new(-661, 28, 407), default, new(-566, 23, 120), new(0, hpi, 0));

        // E -> SE Cosmoliner
        AddCosmoliner(new(-637, 28, 431), new(0, -hpi, 0), new(-412, 44.5f, 401), new(0, hpi, 0));

        #endregion

        #region SW

        // N -> West of base
        AddCosmoliner(new(-388, 44.5f, 377), default, new(-542, 23, 144), new(-pi, 0, -pi));

        // E -> SE of base
        AddCosmoliner(new(-364, 44.5f, 401), new(0, -hpi, 0), new(68, 97.5f, 353), new(0, hpi, 0));

        #endregion

        #region W

        // North -> NW (Below Erg Eris)
        AddCosmoliner(new(-542, 23, 96), default, new(-530, -27.5f, -216), new(pi, 0, -pi));

        #endregion

        #region NW

        // N -> NW (Above Erg Eris, below flower looking hole)
        AddCosmoliner(new(-530, -27.5f, -264), default, new(-702, -88, -470), new(pi, 0, -pi));

        // E -> Cosmoliner N of base
        AddCosmoliner(new(-506, -27.5f, -240), new(0, -hpi, 0), new(-174, -23, -237), new(0, hpi, 0));

        #endregion

        #region N

        // N -> Far North
        AddCosmoliner(new(-150, -23, -261), default, new(-132, -75, -558), new(pi, 0, -pi));

        // E -> NE
        AddCosmoliner(new(-126, -23, -237), new(0, -hpi, 0), new(168, -54.5f, -400), new(0, hpi, 0));

        #endregion

        #region NN

        // W -> Far NW (East Side
        AddCosmoliner(new(-156, -75.5f, -582), new(0, hpi, 0), new(-678, -88, -494), new(0, -hpi, 0));

        // N -> FARR NW/Against N Wall
        AddCosmoliner(new(-132, -75.5f, -606), default, new(-456, -105f, -760), new(0, -hpi, 0));

        #endregion

        #region NNW

        // North -> NW Cosmoliner
        AddCosmoliner(new(-702, -88, -518), default, new(-504, -105, -760), new(0, hpi, 0));

        #endregion

        return;

        void AddCosmoliner(Vector3 pointAPos, Vector3 pointARotation, Vector3 pointBPos, Vector3 pointBRotation)
        {
            var (depA, arrA) = GetPoints(pointAPos, pointARotation);
            var (depB, arrB) = GetPoints(pointBPos, pointBRotation);

            LinkClientPath(mesh, depA, arrB);
            LinkClientPath(mesh, depB, arrA);
        }

        void AddSoloLiner(Vector3 pointAPos, Vector3 pointARotation, Vector3 pointBPos, Vector3 pointBRotation)
        {
            var adjA = Vector3.Transform(new(0, 2.5f, 0.8f), Quaternion.CreateFromYawPitchRoll(pointARotation.Y, pointARotation.X, pointARotation.Z));
            var adjB = Vector3.Transform(new(0, 2.7f, 1.8f), Quaternion.CreateFromYawPitchRoll(pointBRotation.Y, pointBRotation.X, pointBRotation.Z));
            var ptA  = pointAPos + adjA;
            var ptB  = pointBPos + adjB;

            LinkClientPath(mesh, ptA, ptB);
        }

        (Vector3 DepartPoint, Vector3 ArrivePoint) GetPoints(Vector3 worldPos, Vector3 rotation)
        {
            var q    = Quaternion.CreateFromYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
            var adjD = Vector3.Transform(new(4.5f, 2.5f, 0.8f),  q);
            var adjA = Vector3.Transform(new(-4.5f, 2.7f, 1.8f), q);
            return (adjD + worldPos, adjA + worldPos);
        }
    }
}
