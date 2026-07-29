using System.Numerics;
using vnavmesh.Common.Build.Ground;

namespace vnavmesh.Build.Custom.Editor;

public sealed class DraftOffMeshConnectionPatch
{
    public bool                         Enabled = true;
    public string                       Note    = "";
    public Vector3                      Start;
    public Vector3                      End;
    public float                        Radius = 0.5f;
    public bool                         Bidirectional;
    public int                          UserId;
    public NavmeshArea                  Area  = NavmeshArea.ManualOffMesh;
    public NavmeshPolyFlags             Flags = NavmeshPolyFlags.ManualOffMesh;
    public NavmeshOffMeshKind           Kind  = NavmeshOffMeshKind.ManualOffMesh;
    public NavmeshLinkTraversalProfile? TraversalProfile;
}
