using System.Buffers;
using System.IO.Compression;
using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

public partial record class Navmesh
{
    private static EncodedSegment EncodeSegment(CacheSegmentKind kind, CacheCodec codec, Action<BinaryWriter> serialize)
    {
        var       timer          = StopWatchTimer.Create();
        using var payloadStream  = new MemoryStream();
        var       countingStream = CreateSegmentWriteStream(payloadStream, codec, out var disposableStream);

        using (disposableStream)
        {
            using var segmentWriter = new BinaryWriter(countingStream);
            serialize(segmentWriter);
            segmentWriter.Flush();
            countingStream.Flush();
        }

        var payload = payloadStream.ToArray();
        return new(payload, countingStream.BytesProcessed, new(kind, payload.LongLength, countingStream.BytesProcessed, timer.Value()));
    }

    private static byte[] ReadSegmentPayload(Stream source, CacheSegmentDescriptor descriptor)
    {
        if (descriptor.Offset < 0 || descriptor.CompressedBytes < 0 || descriptor.Offset + descriptor.CompressedBytes > source.Length)
            throw new Exception($"缓存段越界: {descriptor.Kind} @ {descriptor.Offset} + {descriptor.CompressedBytes}");

        var payload = GC.AllocateUninitializedArray<byte>(checked((int)descriptor.CompressedBytes));
        source.Position = descriptor.Offset;
        source.ReadExactly(payload);
        return payload;
    }

    private static (T Value, CacheSegmentTelemetry Telemetry) DecodeSegment<T>(CacheSegmentDescriptor descriptor, byte[] payload, Func<BinaryReader, T> deserialize)
    {
        var       timer          = StopWatchTimer.Create();
        using var segmentStream  = new MemoryStream(payload, false);
        var       countingStream = CreateSegmentReadStream(segmentStream, descriptor.Codec, out var disposableStream);
        using var _              = disposableStream;
        using var segmentReader  = new BinaryReader(countingStream);
        var       value          = deserialize(segmentReader);
        DrainToEnd(countingStream);
        return (value, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static (T Value, CacheSegmentTelemetry Telemetry) DecodeSegment<T>(Stream source, CacheSegmentDescriptor descriptor, Func<BinaryReader, T> deserialize)
    {
        var       timer          = StopWatchTimer.Create();
        var       segmentStream  = new SegmentReadStream(source, descriptor.Offset, descriptor.CompressedBytes);
        var       countingStream = CreateSegmentReadStream(segmentStream, descriptor.Codec, out var disposableStream);
        using var _              = disposableStream;
        using var segmentReader  = new BinaryReader(countingStream);
        var       value          = deserialize(segmentReader);
        DrainToEnd(countingStream);
        return (value, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static void DrainToEnd(Stream stream)
    {
        Span<byte> buffer = stackalloc byte[4096];

        while (stream.Read(buffer) > 0)
        {
        }
    }

    private static CountingStream CreateSegmentWriteStream(Stream destination, CacheCodec codec, out IDisposable disposableStream)
    {
        var stream = codec switch
        {
            CacheCodec.None          => destination,
            CacheCodec.BrotliFastest => new BrotliStream(destination, CompressionLevel.Fastest, true),
            _                        => throw new Exception($"Unsupported cache codec: {codec}")
        };

        var counting = new CountingStream(stream, codec == CacheCodec.None);
        disposableStream = counting;
        return counting;
    }

    private static CountingStream CreateSegmentReadStream(Stream source, CacheCodec codec, out IDisposable disposableStream)
    {
        var stream = codec switch
        {
            CacheCodec.None          => source,
            CacheCodec.BrotliFastest => new BrotliStream(source, CompressionMode.Decompress, true),
            _                        => throw new Exception($"Unsupported cache codec: {codec}")
        };

        var counting = new CountingStream(stream, codec == CacheCodec.None);
        disposableStream = counting;
        return counting;
    }

    private static CacheSegmentDescriptor ReadSegmentDescriptor(BinaryReader reader) => new
    (
        (CacheSegmentKind)reader.ReadInt32(),
        (CacheCodec)reader.ReadInt32(),
        reader.ReadInt64(),
        reader.ReadInt64(),
        reader.ReadInt64()
    );

    private static void WriteSegmentDescriptor(BinaryWriter writer, CacheSegmentDescriptor descriptor)
    {
        writer.Write((int)descriptor.Kind);
        writer.Write((int)descriptor.Codec);
        writer.Write(descriptor.Offset);
        writer.Write(descriptor.CompressedBytes);
        writer.Write(descriptor.UncompressedBytes);
    }

    private static float[] ReadSingleArray(BinaryReader reader, int count)
    {
        if (count == 0)
            return [];

        var result = GC.AllocateUninitializedArray<float>(count);
        reader.BaseStream.ReadExactly(MemoryMarshal.AsBytes(result.AsSpan()));
        return result;
    }

    private static void WriteSingleArray(BinaryWriter writer, ReadOnlySpan<float> values)
    {
        if (!values.IsEmpty)
            writer.BaseStream.Write(MemoryMarshal.AsBytes(values));
    }

    private static int[] ReadByteBackedIntArray(BinaryReader reader, int count)
    {
        if (count == 0)
            return [];

        var bytes = GC.AllocateUninitializedArray<byte>(count);
        reader.BaseStream.ReadExactly(bytes);
        var result = GC.AllocateUninitializedArray<int>(count);
        for (var i = 0; i < count; ++i)
            result[i] = bytes[i];
        return result;
    }

    private static void WriteByteBackedIntArray(BinaryWriter writer, ReadOnlySpan<int> values)
    {
        if (values.IsEmpty)
            return;

        var rented = ArrayPool<byte>.Shared.Rent(values.Length);

        try
        {
            var bytes = rented.AsSpan(0, values.Length);
            for (var i = 0; i < values.Length; ++i)
                bytes[i] = (byte)values[i];
            writer.BaseStream.Write(bytes);
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(rented);
        }
    }

    private static (Vector3 min, Vector3 max) DeserializeBounds(BinaryReader reader) => (DeserializeVector3(reader), DeserializeVector3(reader));

    private static void SerializeBounds(BinaryWriter writer, Vector3 min, Vector3 max)
    {
        SerializeVector3(writer, min);
        SerializeVector3(writer, max);
    }

    private static Vector3 DeserializeVector3(BinaryReader reader) => new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

    private static void SerializeVector3(BinaryWriter writer, Vector3 v)
    {
        writer.Write(v.X);
        writer.Write(v.Y);
        writer.Write(v.Z);
    }
}
