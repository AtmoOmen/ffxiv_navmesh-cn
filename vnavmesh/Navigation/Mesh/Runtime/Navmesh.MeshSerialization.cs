using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Detour.Io;

namespace vnavmesh.Navigation.Mesh.Runtime;

public partial record class Navmesh
{
    private static DtNavMesh DeserializeMesh(BinaryReader reader) =>
        new DtMeshSetReader().Read(reader);

    private static void SerializeMesh(BinaryWriter writer, DtNavMesh mesh) =>
        new DtMeshSetWriter().Write(writer, mesh, RcByteOrder.LITTLE_ENDIAN, false);
}
