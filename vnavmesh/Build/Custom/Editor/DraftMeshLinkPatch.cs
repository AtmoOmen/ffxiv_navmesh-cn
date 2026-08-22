using System.Numerics;
using vnavmesh.Common.Build.Ground;
using vnavmesh.Common.Build.Ground.Models;

namespace vnavmesh.Build.Custom.Editor;

public sealed class DraftMeshLinkPatch
{
    public bool                         Enabled = true;
    public DraftMeshLinkKind            Kind;
    public string                       Note = "";
    public Vector3                      Start;
    public Vector3                      End;
    public bool                         Bidirectional;
    public NavmeshLinkTraversalProfile? TraversalProfile;
}
