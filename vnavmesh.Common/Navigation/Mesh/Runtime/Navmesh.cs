using System.Buffers;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using DotRecast.Core;
using DotRecast.Core.Compression;
using DotRecast.Detour;
using DotRecast.Detour.Io;
using vnavmesh.Common.Navigation.Mesh.Build;
using vnavmesh.Common.Navigation.Volume.Map;
using vnavmesh.Common.Utilities;

namespace vnavmesh.Common.Navigation.Mesh.Runtime;

// full set of data needed for navigation in the zone
public record class Navmesh
{
    public static readonly uint Magic   = 0x444D564E; // 'NVMD'
    public static readonly uint Version = 36;         // 更新后触发一次全量重构建

    public int       CustomizationVersion { get; init; }
    public string    BuildSignature       { get; init; }
    public bool      CustomizationApplied { get; init; }
    public VoxelMap? Volume               { get; init; }

    private          DtNavMesh?         _mesh;
    private          byte[]?            _deferredMeshPayload;
    private          long               _deferredMeshExpectedBytes;
    private          CacheCodec         _deferredMeshCodec;
    private          Action<DtNavMesh>? _deferredMeshMutator;
    private readonly SemaphoreSlim      _meshMaterializationGate = new(1, 1);

    public DtNavMesh Mesh
    {
        get
        {
            EnsureMeshMaterialized();
            return _mesh ?? throw new InvalidOperationException("地面导航网格尚未就绪");
        }
        init => _mesh = value;
    }

    public Navmesh(int customizationVersion, string buildSignature, bool customizationApplied, DtNavMesh mesh, VoxelMap? volume)
    {
        CustomizationVersion = customizationVersion;
        BuildSignature       = buildSignature;
        CustomizationApplied = customizationApplied;
        _mesh                = mesh;
        Volume               = volume;
    }

    public int GeneratedClimbDownLinkCount { get; set; }
    public int GeneratedEdgeJumpLinkCount  { get; set; }
    public bool HasHeuristicSensitiveOffMeshLinks
    {
        get
        {
            foreach (var link in _offMeshLinksByPolyRef.Values)
            {
                var profile = NavmeshLinkTraversalProfiles.Resolve(link.Kind, link.TraversalProfile);
                if (profile.DistanceScale == 0f)
                    return true;
            }

            EnsureMeshMaterialized();
            if (_mesh == null)
                return false;

            for (var tileIndex = 0; tileIndex < _mesh.GetMaxTiles(); ++tileIndex)
            {
                var tile = _mesh.GetTile(tileIndex);
                if (tile?.data?.header == null)
                    continue;

                for (var polyIndex = tile.data.header.offMeshBase; polyIndex < tile.data.header.polyCount; ++polyIndex)
                {
                    var area = (NavmeshArea)tile.data.polys[polyIndex].GetArea();
                    if (area is NavmeshArea.Teleport or NavmeshArea.ClientPath)
                        return true;
                }
            }

            return false;
        }
    }

    public readonly List<NavmeshLink>
        Links = []; // not serialized! actual links are added directly to the DtNavMesh, this field exists for visualization purposes

    private readonly Dictionary<long, NavmeshLink> _offMeshLinksByPolyRef = [];

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

        var meshSegment     = meshDescriptor   ?? throw new Exception("缓存缺少 Mesh 段");
        var volumeSegment   = volumeDescriptor ?? throw new Exception("缓存缺少 Volume 段");
        var requiresRewrite = volumeSegment.Codec != CacheCodec.FastLz;
        var source          = reader.BaseStream;
        var (meshPayload, meshTelemetry) = DecodeDeferredMeshSegment(meshSegment, source);
        var (volume, volumeTelemetry) = volumeSegment.Codec == CacheCodec.FastLz
                                            ? DecodeDeferredVolumeSegment(volumeSegment, source)
                                            : DecodeSegment(volumeSegment, source, DeserializeVolume);
        var navmesh = new Navmesh(customizationVersion, buildSignature, customizationApplied, null!, volume);
        navmesh.SetDeferredMeshPayload(meshPayload, meshSegment.Codec, meshSegment.UncompressedBytes);
        return new(navmesh, new(meshTelemetry, volumeTelemetry), requiresRewrite);
    }

    public CacheTelemetry Serialize(BinaryWriter writer)
    {
        EnsureLittleEndian();

        EncodedSegment meshSegment   = default;
        EncodedSegment volumeSegment = default;
        Parallel.Invoke
        (
            () => meshSegment   = EncodeSegment(CacheSegmentKind.Mesh,   CacheCodec.None,   meshWriter => SerializeMesh(meshWriter, Mesh)),
            () => volumeSegment = EncodeSegment(CacheSegmentKind.Volume, CacheCodec.FastLz, volumeWriter => SerializeVolume(volumeWriter, Volume))
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

    private static VoxelMap? DeserializeVolume(BinaryReader reader)
    {
        var volume = DeserializeVolumeHeader(reader);
        if (volume == null)
            return null;

        if (TryCaptureDeferredVolumeTree(reader.BaseStream, out var payload, out var offset, out var length))
        {
            volume.SetDeferredTreePayload(payload, offset, length);
            reader.BaseStream.Position = reader.BaseStream.Length;
        }
        else DeserializeVolumeTile(reader, volume.RootTile);

        return volume;
    }

    private static VoxelMap? DeserializeVolumeHeader(BinaryReader reader)
    {
        if (!reader.ReadBoolean())
            return null;

        var numLevels = reader.ReadInt32();
        if (numLevels <= 0)
            throw new Exception("体积缓存层级无效");

        var tilesPerLevel = new int[numLevels];
        foreach (ref var l in tilesPerLevel.AsSpan())
            l = reader.ReadInt32();

        var (min, max) = DeserializeBounds(reader);
        return new(min, max, tilesPerLevel);
    }

    private static VoxelMap? DeserializeCompressedDeferredVolume(byte[] payload, long expectedBytes)
    {
        var       decodedPayload = DecompressFastLz(payload, expectedBytes);
        using var stream         = new MemoryStream(decodedPayload, 0, decodedPayload.Length, false, true);
        using var reader         = new BinaryReader(stream);
        var       volume         = DeserializeVolumeHeader(reader);
        if (volume == null)
            return null;

        var treeOffset = checked((int)stream.Position);
        volume.SetDeferredTreeMaterializer(v => MaterializeDeferredCompressedVolumeTree(v, payload, expectedBytes, treeOffset));
        return volume;
    }

    private static void SerializeVolume(BinaryWriter writer, VoxelMap? volume)
    {
        writer.Write(volume != null);
        if (volume == null)
            return;

        writer.Write(volume.Levels.Length);
        foreach (ref var l in volume.Levels.AsSpan())
            writer.Write(l.NumCellsX);

        volume.EnsureMaterialized();
        volume.CompactRetainedState();
        SerializeBounds(writer, volume.RootTile.BoundsMin, volume.RootTile.BoundsMax);
        SerializeVolumeTile(writer, volume.RootTile);
    }

    internal static void MaterializeDeferredVolumeTree(VoxelMap volume, byte[] payload, int offset, int length)
    {
        using var stream = new MemoryStream(payload, offset, length, false);
        using var reader = new BinaryReader(stream);
        DeserializeVolumeTile(reader, volume.RootTile);
    }

    internal static void MaterializeDeferredCompressedVolumeTree(VoxelMap volume, byte[] payload, long expectedBytes, int offset)
    {
        var       decodedPayload = DecompressFastLz(payload, expectedBytes);
        using var stream         = new MemoryStream(decodedPayload, offset, decodedPayload.Length - offset, false);
        using var reader         = new BinaryReader(stream);
        DeserializeVolumeTile(reader, volume.RootTile);
    }

    private static (VoxelMap? Value, CacheSegmentTelemetry Telemetry) DecodeDeferredVolumeSegment(CacheSegmentDescriptor descriptor, Stream source)
    {
        var timer   = StopWatchTimer.Create();
        var payload = ReadSegmentPayload(source, descriptor);
        var volume  = DeserializeCompressedDeferredVolume(payload, descriptor.UncompressedBytes);
        return (volume, new(descriptor.Kind, descriptor.CompressedBytes, descriptor.UncompressedBytes, timer.Value()));
    }

    private static void DeserializeVolumeTile(BinaryReader reader, VolumeTile tile)
    {
        var encoding = (VolumeTileEncoding)reader.ReadByte();

        switch (encoding)
        {
            case VolumeTileEncoding.Empty:
                tile.SetUniformEmpty();
                return;
            case VolumeTileEncoding.SolidLeaf:
                tile.SetUniformSolidLeaf();
                return;
            case VolumeTileEncoding.Mixed:
                break;
            default:
                throw new Exception($"未知的体积编码类型: {encoding}");
        }

        tile.ClearSubdivision();
        var packedBytes  = PackedStateBytes(tile.CellCount);
        var packedStates = GC.AllocateUninitializedArray<byte>(packedBytes);
        reader.BaseStream.ReadExactly(packedStates);

        var subtreeCount = 0;

        for (var i = 0; i < packedStates.Length; ++i)
        {
            var packedState = packedStates[i];
            if (s_invalidPackedState[packedState])
                throw new Exception($"未知的体积单元状态字节: 0x{packedState:X2}");

            subtreeCount += s_subtreeCountByPackedState[packedState];
        }

        tile.SetPackedStates(packedStates);
        tile.EnsureSubdivisionCapacity(subtreeCount);

        if (subtreeCount == 0)
            return;

        var baseIndex = 0;

        for (var i = 0; i < packedStates.Length; ++i, baseIndex += 4)
        {
            var subtreeMask = s_subtreeMaskByPackedState[packedStates[i]];

            while (subtreeMask != 0)
            {
                var localOffset = BitOperations.TrailingZeroCount((uint)subtreeMask);
                subtreeMask = (byte)(subtreeMask & subtreeMask - 1);
                DeserializeVolumeSubtile(reader, tile, baseIndex + localOffset);
            }
        }
    }

    private static void DeserializeVolumeSubtile(BinaryReader reader, VolumeTile parent, int flatIndex)
    {
        var localId = parent.SubdivisionCount;
        if (localId >= VoxelMap.VOXEL_ID_MASK)
            throw new Exception("体积子树数量超出上限");

        var subBounds = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel((ushort)flatIndex));
        var child     = new VolumeTile(parent.Owner, subBounds.min, subBounds.max, parent.Level + 1, false);
        parent.AddSubdivision(child);
        DeserializeVolumeTile(reader, child);
    }

    private static void SerializeVolumeTile(BinaryWriter writer, VolumeTile tile)
    {
        tile.CompactRetainedState();

        switch (tile.StorageKind)
        {
            case VolumeTileStorageKind.AllEmpty:
                writer.Write((byte)VolumeTileEncoding.Empty);
                return;
            case VolumeTileStorageKind.SolidLeaf:
                writer.Write((byte)VolumeTileEncoding.SolidLeaf);
                return;
            case VolumeTileStorageKind.PackedMixed:
                writer.Write((byte)VolumeTileEncoding.Mixed);
                writer.BaseStream.Write(tile.PackedStates);
                break;
            case VolumeTileStorageKind.Dense:
                throw new InvalidOperationException("体积瓦片序列化前未完成压缩");
            default:
                throw new InvalidOperationException($"未知的体积瓦片存储类型: {tile.StorageKind}");
        }

        for (var i = 0; i < tile.CellCount; ++i)
        {
            if (!tile.IsSubdividedCell(i))
                continue;

            var localId = tile.GetSubdivisionIndex(i);
            if (localId >= tile.SubdivisionCount)
                throw new Exception($"体积子树索引越界: {localId} / {tile.SubdivisionCount}");
            SerializeVolumeTile(writer, tile.GetSubdivision(localId));
        }
    }


    public enum CacheSegmentKind
    {
        Mesh   = 1,
        Volume = 2
    }

    private enum CacheCodec
    {
        None   = 0,
        FastLz = 1
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
        CacheTelemetry Telemetry,
        bool           RequiresRewrite
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

    private void SetDeferredMeshPayload(byte[] payload, CacheCodec codec, long expectedBytes)
    {
        _mesh                      = null;
        _deferredMeshPayload       = payload;
        _deferredMeshCodec         = codec;
        _deferredMeshExpectedBytes = expectedBytes;
    }

    public void DeferMeshMutation(Action<DtNavMesh> mutator) =>
        _deferredMeshMutator += mutator;

    public void RegisterOffMeshLink(long polyRef, NavmeshLink link, bool includeInVisualization = false)
    {
        _offMeshLinksByPolyRef[polyRef] = link;
        if (includeInVisualization)
            Links.Add(link);
    }

    public bool TryGetOffMeshLink(long polyRef, out NavmeshLink link) =>
        _offMeshLinksByPolyRef.TryGetValue(polyRef, out link);

    public void RegisterBuildTimeOffMeshConnections(IEnumerable<NavmeshBuildOffMeshConnection> connections)
    {
        EnsureMeshMaterialized();

        foreach (var connection in connections)
        {
            if (!TryFindMatchingOffMeshPolyRef(connection, out var polyRef))
                continue;

            RegisterOffMeshLink
            (
                polyRef,
                new
                (
                    connection.Start,
                    connection.End,
                    (NavmeshOffMeshKind)connection.Kind,
                    connection.Bidirectional,
                    connection.UserId,
                    connection.TraversalProfile
                )
            );
        }
    }

    internal void EnsureMeshMaterialized()
    {
        if (_mesh != null && _deferredMeshMutator == null)
            return;

        _meshMaterializationGate.Wait();

        try
        {
            if (_mesh == null)
            {
                if (_deferredMeshPayload == null)
                    throw new InvalidOperationException("缺少地面导航网格载荷");

                _mesh                      = DeserializeMeshPayload(_deferredMeshPayload, _deferredMeshCodec, _deferredMeshExpectedBytes);
                _deferredMeshPayload       = null;
                _deferredMeshExpectedBytes = 0;
            }

            if (_deferredMeshMutator is not { } mutator)
                return;

            mutator(_mesh);
            _deferredMeshMutator = null;
        }
        finally
        {
            _meshMaterializationGate.Release();
        }
    }

    public void ReleaseRetainedState()
    {
        _deferredMeshPayload       = null;
        _deferredMeshExpectedBytes = 0;
        _deferredMeshMutator       = null;
        Volume?.ReleaseRetainedState();
        _offMeshLinksByPolyRef.Clear();
        Links.Clear();
        Links.TrimExcess();
        _mesh?.Release();
        _mesh = null;
    }

    private bool TryFindMatchingOffMeshPolyRef(NavmeshBuildOffMeshConnection connection, out long polyRef)
    {
        if (_mesh == null)
            throw new InvalidOperationException("地面导航网格尚未就绪");

        for (var tileIndex = 0; tileIndex < _mesh.GetMaxTiles(); ++tileIndex)
        {
            var tile = _mesh.GetTile(tileIndex);
            if (tile?.data?.header == null || tile.data.offMeshCons == null)
                continue;

            for (var connectionIndex = 0; connectionIndex < tile.data.header.offMeshConCount; ++connectionIndex)
            {
                var offMeshConnection = tile.data.offMeshCons[connectionIndex];
                var poly             = tile.data.polys[offMeshConnection.poly];
                if (poly == null)
                    continue;

                if ((NavmeshArea)poly.GetArea() != (NavmeshArea)connection.Area ||
                    poly.flags                   != connection.Flags             ||
                    offMeshConnection.userId     != connection.UserId            ||
                    (offMeshConnection.flags != 0) != connection.Bidirectional)
                    continue;

                if (!ApproximatelyEquals(offMeshConnection.pos[0], connection.Start) || !ApproximatelyEquals(offMeshConnection.pos[1], connection.End))
                    continue;

                polyRef = DtDetour.EncodePolyId(tile.salt, tile.index, offMeshConnection.poly);
                return true;
            }
        }

        polyRef = 0;
        return false;
    }

    private static bool ApproximatelyEquals(DotRecast.Core.Numerics.RcVec3f actual, Vector3 expected)
    {
        var dx = MathF.Abs(actual.X - expected.X);
        var dy = MathF.Abs(actual.Y - expected.Y);
        var dz = MathF.Abs(actual.Z - expected.Z);
        return dx <= 0.05f && dy <= 0.05f && dz <= 0.05f;
    }

    private static readonly byte[] s_subtreeMaskByPackedState  = BuildSubtreeMaskByPackedState();
    private static readonly byte[] s_subtreeCountByPackedState = BuildSubtreeCountByPackedState();
    private static readonly bool[] s_invalidPackedState        = BuildInvalidPackedState();

    private static int PackedStateBytes(int numCells) => numCells + 3 >> 2;

    private static bool TryCaptureDeferredVolumeTree(Stream stream, out byte[] payload, out int offset, out int length)
    {
        if (stream is MemoryStream memoryStream && memoryStream.TryGetBuffer(out var buffer) && buffer.Array != null)
        {
            payload = buffer.Array;
            offset  = buffer.Offset + checked((int)memoryStream.Position);
            length  = checked((int)(memoryStream.Length - memoryStream.Position));
            return true;
        }

        payload = [];
        offset  = 0;
        length  = 0;
        return false;
    }

    private static byte[] BuildSubtreeMaskByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>(byte.MaxValue + 1);

        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        {
            byte mask = 0;
            for (var offset = 0; offset < 4; ++offset)
                if ((VolumeCellState)(packedState >> offset * 2 & 0x3) == VolumeCellState.Subtree)
                    mask |= (byte)(1              << offset);
            table[packedState] = mask;
        }

        return table;
    }

    private static byte[] BuildSubtreeCountByPackedState()
    {
        var table = GC.AllocateUninitializedArray<byte>(byte.MaxValue + 1);
        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
            table[packedState] = (byte)BitOperations.PopCount(s_subtreeMaskByPackedState[packedState]);
        return table;
    }

    private static bool[] BuildInvalidPackedState()
    {
        var table = GC.AllocateUninitializedArray<bool>(byte.MaxValue + 1);

        for (var packedState = 0; packedState <= byte.MaxValue; ++packedState)
        for (var offset = 0; offset < 4; ++offset)
            if ((VolumeCellState)(packedState >> offset * 2 & 0x3) == (VolumeCellState)3)
            {
                table[packedState] = true;
                break;
            }

        return table;
    }
}
