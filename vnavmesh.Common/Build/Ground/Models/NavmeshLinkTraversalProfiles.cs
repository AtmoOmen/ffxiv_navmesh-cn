using System.Numerics;
using vnavmesh.Common.Build.Ground.Enums;

namespace vnavmesh.Common.Build.Ground.Models;

public static class NavmeshLinkTraversalProfiles
{
    public static readonly NavmeshLinkTraversalProfile Ground             = new(1f, 0f);
    public static readonly NavmeshLinkTraversalProfile GeneratedClimbDown = new(1f, 0f);
    public static readonly NavmeshLinkTraversalProfile GeneratedEdgeJump  = new(1f, 12f);
    public static readonly NavmeshLinkTraversalProfile ManualOffMesh      = new(1f, 0f);
    public static readonly NavmeshLinkTraversalProfile Shortcut           = new(1f, 0f);
    public static readonly NavmeshLinkTraversalProfile Teleport           = new(0f, 0f);
    public static readonly NavmeshLinkTraversalProfile ClientPath         = new(0f, 0f);

    public static NavmeshLinkTraversalProfile Resolve
    (
        NavmeshOffMeshKind           kind,
        NavmeshLinkTraversalProfile? overrideProfile = null
    ) =>
        overrideProfile ??
        kind switch
        {
            NavmeshOffMeshKind.GeneratedClimbDown => GeneratedClimbDown,
            NavmeshOffMeshKind.GeneratedEdgeJump  => GeneratedEdgeJump,
            NavmeshOffMeshKind.ManualOffMesh      => ManualOffMesh,
            NavmeshOffMeshKind.Shortcut           => Shortcut,
            NavmeshOffMeshKind.Teleport           => Teleport,
            NavmeshOffMeshKind.ClientPath         => ClientPath,
            _                                     => Ground
        };

    public static NavmeshOffMeshKind? ResolveKind
    (
        NavmeshArea area
    ) =>
        area switch
        {
            NavmeshArea.GeneratedClimbDown => NavmeshOffMeshKind.GeneratedClimbDown,
            NavmeshArea.GeneratedEdgeJump  => NavmeshOffMeshKind.GeneratedEdgeJump,
            NavmeshArea.ManualOffMesh      => NavmeshOffMeshKind.ManualOffMesh,
            NavmeshArea.Shortcut           => NavmeshOffMeshKind.Shortcut,
            NavmeshArea.Teleport           => NavmeshOffMeshKind.Teleport,
            NavmeshArea.ClientPath         => NavmeshOffMeshKind.ClientPath,
            _                              => null
        };

    public static float EstimateCost
    (
        Vector3                      start,
        Vector3                      end,
        NavmeshOffMeshKind           kind,
        NavmeshLinkTraversalProfile? overrideProfile = null
    )
    {
        var profile  = Resolve(kind, overrideProfile);
        var distance = Vector3.Distance(start, end);
        return (distance * profile.DistanceScale) + profile.FixedPenalty;
    }
}
