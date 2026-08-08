using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Common.Models;

namespace vnavmesh.Common.Build.Models;

public sealed class MeshPart
{
    public List<Vector3>   Vertices   = [];
    public List<Primitive> Primitives = [];
    public AABB            LocalBounds;

    public Span<Vector3>   VertexSpan    => CollectionsMarshal.AsSpan(Vertices);
    public Span<Primitive> PrimitiveSpan => CollectionsMarshal.AsSpan(Primitives);
}
