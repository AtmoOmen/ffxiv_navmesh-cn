using System.Buffers;
using System.Numerics;
using System.Runtime.InteropServices;
using DotRecast.Core.Compression;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

public partial record class Navmesh
{
    private static EncodedSegment EncodeSegment(CacheSegmentKind kind, CacheCodec codec, Action<BinaryWriter> serialize)
    {
        var       timer         = StopWatchTimer.Create();
        using var payloadStream = new MemoryStream();

        using (var segmentWriter = new BinaryWriter(payloadStream))
        {
            serialize(segmentWriter);
            segmentWriter.Flush();
        }

        var rawPayload = payloadStream.ToArray();
        var payload    = EncodePayload(rawPayload, codec);
        return new(codec, payload, rawPayload.LongLength, new(kind, payload.LongLength, rawPayload.LongLength, timer.Value()));
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
        var       decodedPayload = DecodePayload(payload, descriptor.Codec, descriptor.UncompressedBytes);
        using var segmentStream  = new MemoryStream(decodedPayload, 0, decodedPayload.Length, false, true);
        using var segmentReader  = new BinaryReader(segmentStream);
        var       value          = deserialize(segmentReader);
        return (value, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static (T Value, CacheSegmentTelemetry Telemetry) DecodeSegment<T>(CacheSegmentDescriptor descriptor, Stream source, Func<BinaryReader, T> deserialize)
    {
        var       timer         = StopWatchTimer.Create();
        using var segmentStream = OpenDecodedSegmentStream(source, descriptor);
        using var segmentReader = new BinaryReader(segmentStream);
        var       value         = deserialize(segmentReader);
        return (value, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static Stream OpenDecodedSegmentStream(Stream source, CacheSegmentDescriptor descriptor) => descriptor.Codec switch
    {
        CacheCodec.None => new SegmentReadStream(source, descriptor.Offset, descriptor.CompressedBytes),
        CacheCodec.FastLz => new MemoryStream
        (
            DecodePayload(ReadSegmentPayload(source, descriptor), descriptor.Codec, descriptor.UncompressedBytes),
            0,
            checked((int)descriptor.UncompressedBytes),
            false,
            true
        ),
        _ => throw new Exception($"不支持的缓存编码: {descriptor.Codec}")
    };

    private static byte[] EncodePayload(byte[] rawPayload, CacheCodec codec) => codec switch
    {
        CacheCodec.None   => rawPayload,
        CacheCodec.FastLz => CompressFastLz(rawPayload),
        _                 => throw new Exception($"不支持的缓存编码: {codec}")
    };

    private static byte[] DecodePayload(byte[] payload, CacheCodec codec, long expectedBytes) => codec switch
    {
        CacheCodec.None   => payload,
        CacheCodec.FastLz => DecompressFastLz(payload, expectedBytes),
        _                 => throw new Exception($"不支持的缓存编码: {codec}")
    };

    private static byte[] CompressFastLz(byte[] rawPayload)
    {
        if (rawPayload.Length == 0)
            return [];

        var compressedBuffer = GC.AllocateUninitializedArray<byte>(checked((int)FastLZ.EstimateCompressedSize(rawPayload.Length)));
        var compressedBytes  = checked((int)FastLZ.CompressLevel(2, rawPayload, 0, rawPayload.Length, compressedBuffer));
        var compressed       = GC.AllocateUninitializedArray<byte>(compressedBytes);
        Buffer.BlockCopy(compressedBuffer, 0, compressed, 0, compressedBytes);
        return compressed;
    }

    private static byte[] DecompressFastLz(byte[] payload, long expectedBytes)
    {
        var outputLength = checked((int)expectedBytes);
        if (outputLength == 0)
            return [];

        var decompressed = GC.AllocateUninitializedArray<byte>(outputLength);
        var actualBytes  = FastLZ.Decompress(payload, 0, payload.Length, decompressed, 0, outputLength);
        if (actualBytes != outputLength)
            throw new Exception($"FastLZ 解压失败: 期望 {outputLength} 字节, 实际 {actualBytes} 字节");
        return decompressed;
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
