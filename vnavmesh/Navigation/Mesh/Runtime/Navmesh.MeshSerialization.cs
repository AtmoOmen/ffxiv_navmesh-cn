using DotRecast.Core;
using DotRecast.Detour;
using DotRecast.Detour.Io;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

public partial record class Navmesh
{
    private static DtNavMesh DeserializeMesh(BinaryReader reader) =>
        new DtMeshSetReader().Read(reader);

    private static (byte[] Payload, CacheSegmentTelemetry Telemetry) DecodeDeferredMeshSegment(CacheSegmentDescriptor descriptor, Stream source)
    {
        var timer   = StopWatchTimer.Create();
        var payload = ReadSegmentPayload(source, descriptor);
        return (payload, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static DtNavMesh DeserializeMeshPayload(byte[] payload, CacheCodec codec, long expectedBytes)
    {
        var       decodedPayload = DecodePayload(payload, codec, expectedBytes);
        using var stream         = new MemoryStream(decodedPayload, 0, decodedPayload.Length, false, true);
        using var reader         = new BinaryReader(stream);
        return DeserializeMesh(reader);
    }

    private static void SerializeMesh(BinaryWriter writer, DtNavMesh mesh) =>
        new DtMeshSetWriter().Write(writer, mesh, RcByteOrder.LITTLE_ENDIAN, false);
}
