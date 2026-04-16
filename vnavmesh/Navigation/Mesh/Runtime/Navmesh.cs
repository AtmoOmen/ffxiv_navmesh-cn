using DotRecast.Detour;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Runtime;

// full set of data needed for navigation in the zone
public partial record class Navmesh
(
    int       CustomizationVersion,
    string    BuildSignature,
    bool      CustomizationApplied,
    DtNavMesh Mesh,
    VoxelMap? Volume
)
{
    public static readonly uint Magic   = 0x444D564E; // 'NVMD'
    public static readonly uint Version = 35;         // 更新后触发一次全量重构建

    public int GeneratedClimbDownLinkCount { get; set; }
    public int GeneratedEdgeJumpLinkCount  { get; set; }

    public readonly List<NavmeshLink>
        Links = []; // not serialized! actual links are added directly to the DtNavMesh, this field exists for visualization purposes

    // throws an exception on failure
    public static DeserializeResult Deserialize(BinaryReader reader, int expectedCustomizationVersion, string expectedBuildSignature)
    {
        EnsureLittleEndian();

        var magic   = reader.ReadUInt32();
        var version = reader.ReadUInt32();
        if (magic != Magic || version != Version)
            throw new Exception("缓存头无效");

        var customizationVersion = reader.ReadInt32();
        if (customizationVersion != expectedCustomizationVersion)
            throw new Exception("缓存定制版本已过期");

        var buildSignature = reader.ReadString();
        if (buildSignature != expectedBuildSignature)
            throw new Exception("缓存构建签名已过期");

        var customizationApplied = reader.ReadBoolean();
        var segmentCount         = reader.ReadInt32();
        if (segmentCount <= 0)
            throw new Exception("缓存段表无效");

        CacheSegmentDescriptor? meshDescriptor   = null;
        CacheSegmentDescriptor? volumeDescriptor = null;

        for (var i = 0; i < segmentCount; ++i)
        {
            var descriptor = ReadSegmentDescriptor(reader);

            switch (descriptor.Kind)
            {
                case CacheSegmentKind.Mesh:
                    meshDescriptor = descriptor;
                    break;
                case CacheSegmentKind.Volume:
                    volumeDescriptor = descriptor;
                    break;
            }
        }

        var meshSegment   = meshDescriptor   ?? throw new Exception("缓存缺少 Mesh 段");
        var volumeSegment = volumeDescriptor ?? throw new Exception("缓存缺少 Volume 段");
        var meshPayload   = ReadSegmentPayload(reader.BaseStream, meshSegment);

        DtNavMesh?            mesh            = null;
        VoxelMap?             volume          = null;
        CacheSegmentTelemetry meshTelemetry   = default;
        CacheSegmentTelemetry volumeTelemetry = default;
        Parallel.Invoke
        (
            () => (mesh, meshTelemetry)     = DecodeSegment(meshSegment,   meshPayload,   DeserializeMesh),
            () => (volume, volumeTelemetry) = DecodeSegment(reader.BaseStream, volumeSegment, DeserializeVolume)
        );
        return new(new(customizationVersion, buildSignature, customizationApplied, mesh!, volume), new(meshTelemetry, volumeTelemetry));
    }

    public CacheTelemetry Serialize(BinaryWriter writer)
    {
        EnsureLittleEndian();

        EncodedSegment meshSegment   = default;
        EncodedSegment volumeSegment = default;
        Parallel.Invoke
        (
            () => meshSegment   = EncodeSegment(CacheSegmentKind.Mesh,   CacheCodec.None, meshWriter => SerializeMesh(meshWriter, Mesh)),
            () => volumeSegment = EncodeSegment(CacheSegmentKind.Volume, CacheCodec.None, volumeWriter => SerializeVolume(volumeWriter, Volume))
        );

        writer.Write(Magic);
        writer.Write(Version);
        writer.Write(CustomizationVersion);
        writer.Write(BuildSignature);
        writer.Write(CustomizationApplied);

        writer.Write(2);
        var payloadOffset = writer.BaseStream.Position + 2L * CacheSegmentDescriptorSize;
        var meshDescriptor = new CacheSegmentDescriptor
        (
            CacheSegmentKind.Mesh,
            meshSegment.Codec,
            payloadOffset,
            meshSegment.Payload.LongLength,
            meshSegment.UncompressedBytes
        );
        var volumeDescriptor = new CacheSegmentDescriptor
        (
            CacheSegmentKind.Volume,
            volumeSegment.Codec,
            payloadOffset + meshSegment.Payload.LongLength,
            volumeSegment.Payload.LongLength,
            volumeSegment.UncompressedBytes
        );
        WriteSegmentDescriptor(writer, meshDescriptor);
        WriteSegmentDescriptor(writer, volumeDescriptor);
        writer.BaseStream.Write(meshSegment.Payload);
        writer.BaseStream.Write(volumeSegment.Payload);
        return new(meshSegment.Telemetry, volumeSegment.Telemetry);
    }


    public enum CacheSegmentKind
    {
        Mesh   = 1,
        Volume = 2
    }

    private enum CacheCodec
    {
        None = 0
    }

    private enum VolumeTileEncoding : byte
    {
        Empty     = 0,
        SolidLeaf = 1,
        Mixed     = 2
    }

    private enum VolumeCellState : byte
    {
        Empty     = 0,
        SolidLeaf = 1,
        Subtree   = 2
    }

    public readonly record struct CacheSegmentTelemetry
    (
        CacheSegmentKind Kind,
        long             CompressedBytes,
        long             UncompressedBytes,
        TimeSpan         Duration
    );

    public readonly record struct CacheTelemetry
    (
        CacheSegmentTelemetry Mesh,
        CacheSegmentTelemetry Volume
    )
    {
        public long TotalCompressedBytes   => Mesh.CompressedBytes   + Volume.CompressedBytes;
        public long TotalUncompressedBytes => Mesh.UncompressedBytes + Volume.UncompressedBytes;
    }

    public readonly record struct DeserializeResult
    (
        Navmesh        Navmesh,
        CacheTelemetry Telemetry
    );

    private const int CacheSegmentDescriptorSize = sizeof(int) * 2 + sizeof(long) * 3;

    private readonly record struct EncodedSegment
    (
        CacheCodec            Codec,
        byte[]                Payload,
        long                  UncompressedBytes,
        CacheSegmentTelemetry Telemetry
    );

    private readonly record struct CacheSegmentDescriptor
    (
        CacheSegmentKind Kind,
        CacheCodec       Codec,
        long             Offset,
        long             CompressedBytes,
        long             UncompressedBytes
    );

    private static void EnsureLittleEndian()
    {
        if (!BitConverter.IsLittleEndian)
            throw new PlatformNotSupportedException("缓存格式仅支持小端架构");
    }

    private sealed class CountingStream
    (
        Stream inner,
        bool   leaveOpen
    ) : Stream
    {
        public long BytesProcessed { get; private set; }

        public override bool CanRead  => inner.CanRead;
        public override bool CanSeek  => inner.CanSeek;
        public override bool CanWrite => inner.CanWrite;
        public override long Length   => inner.Length;

        public override long Position
        {
            get => inner.Position;
            set => inner.Position = value;
        }

        public override void Flush() => inner.Flush();

        public override int Read(byte[] buffer, int offset, int count)
        {
            var read = inner.Read(buffer, offset, count);
            BytesProcessed += read;
            return read;
        }

        public override int Read(Span<byte> buffer)
        {
            var read = inner.Read(buffer);
            BytesProcessed += read;
            return read;
        }

        public override long Seek(long offset, SeekOrigin origin) => inner.Seek(offset, origin);

        public override void SetLength(long value) => inner.SetLength(value);

        public override void Write(byte[] buffer, int offset, int count)
        {
            inner.Write(buffer, offset, count);
            BytesProcessed += count;
        }

        public override void Write(ReadOnlySpan<byte> buffer)
        {
            inner.Write(buffer);
            BytesProcessed += buffer.Length;
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing && !leaveOpen)
                inner.Dispose();
            base.Dispose(disposing);
        }
    }

    private sealed class SegmentReadStream
    (
        Stream inner,
        long   offset,
        long   length
    ) : Stream
    {
        private long _position;

        public override bool CanRead  => true;
        public override bool CanSeek  => true;
        public override bool CanWrite => false;
        public override long Length   => length;

        public override long Position
        {
            get => _position;
            set => Seek(value, SeekOrigin.Begin);
        }

        public override void Flush()
        {
        }

        public override int Read(byte[] buffer, int offsetBytes, int count)
        {
            if (_position >= length)
                return 0;

            var toRead = (int)Math.Min(count, length - _position);

            lock (inner)
            {
                inner.Position = offset + _position;
                var read = inner.Read(buffer, offsetBytes, toRead);
                _position += read;
                return read;
            }
        }

        public override int Read(Span<byte> buffer)
        {
            if (_position >= length)
                return 0;

            var toRead = (int)Math.Min(buffer.Length, length - _position);

            lock (inner)
            {
                inner.Position = offset + _position;
                var read = inner.Read(buffer[..toRead]);
                _position += read;
                return read;
            }
        }

        public override long Seek(long seekOffset, SeekOrigin origin)
        {
            var target = origin switch
            {
                SeekOrigin.Begin   => seekOffset,
                SeekOrigin.Current => _position + seekOffset,
                SeekOrigin.End     => length    + seekOffset,
                _                  => throw new ArgumentOutOfRangeException(nameof(origin))
            };

            if (target < 0 || target > length)
                throw new IOException($"Invalid segment seek: {target} / {length}");

            _position = target;
            return _position;
        }

        public override void SetLength(long value) => throw new NotSupportedException();

        public override void Write(byte[] buffer, int offsetBytes, int count) => throw new NotSupportedException();

        public override void Write(ReadOnlySpan<byte> buffer) => throw new NotSupportedException();
    }
}
