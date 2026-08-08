using System.Runtime.InteropServices;
using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Models;

namespace vnavmesh.Common.Build.Models;

public sealed class Mesh
{
    public List<MeshPart>     Parts     = [];
    public List<MeshInstance> Instances = [];
    public MeshType           MeshType;
    public AABB               LocalBounds;

    public Span<MeshPart> PartSpan => CollectionsMarshal.AsSpan(Parts);
}
