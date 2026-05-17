using DotRecast.Detour;
using System;
using System.Collections.Generic;
using System.Text;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Customizations.Extensions;

namespace vnavmesh.Navigation.Customizations.TerritoryCustomizations.Overworld;

[CustomizationTerritory(959)]
public class Z0959叹息海 : NavmeshCustomization
{
    public override int Version => 2;

    public Z0959叹息海() { }


    public override void CustomizeSettings(DtNavMeshCreateParams config)
    {
        config.AddOffMeshConnection
        (
            new(-51.783928f, 42.883057f, 466.92993f),
            new(-51.87583f, 43.93661f, 473.01425f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh,
            null
        );
        config.AddOffMeshConnection
        (
            new(131.59196f, 54.555054f, 468.59067f),
            new(127.68939f, 52.27452f, 465.185f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh,
            null
        );
        config.AddOffMeshConnection
        (
            new(113.688866f, 45.559593f, 460.71576f),
            new(109.07486f, 43.212875f, 457.20346f),
            0.5f,
            true,
            0,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh,
            null
        );
    }
}
