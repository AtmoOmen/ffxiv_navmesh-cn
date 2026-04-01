using DotRecast.Detour;
using Navmesh.NavVolume;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Numerics;

namespace Navmesh;

// full set of data needed for navigation in the zone
public record class Navmesh(int CustomizationVersion, string BuildSignature, bool CustomizationApplied, DtNavMesh Mesh, VoxelMap? Volume)
{
	public static readonly uint Magic = 0x444D564E; // 'NVMD'
	public static readonly uint Version = 28;
	public const int FLAG_UNREACHABLE = 0x10;
	public const int AREAID_TELEPORT = 5;
	public readonly List<(Vector3 Start, Vector3 End)> Links = []; // not serialized! actual links are added directly to the DtNavMesh, this field exists for visualization purposes

	// throws an exception on failure
	public static DeserializeResult Deserialize(BinaryReader reader, int expectedCustomizationVersion, string expectedBuildSignature)
	{
		var magic = reader.ReadUInt32();
		var version = reader.ReadUInt32();
		if (magic != Magic || version != Version)
			throw new Exception("Incorrect header");

		var customizationVersion = reader.ReadInt32();
		if (customizationVersion != expectedCustomizationVersion)
			throw new Exception("Outdated customization version");

		var buildSignature = reader.ReadString();
		if (buildSignature != expectedBuildSignature)
			throw new Exception("缓存构建签名已过期");

		var customizationApplied = reader.ReadBoolean();
		var segmentCount = reader.ReadInt32();
		if (segmentCount <= 0)
			throw new Exception("缓存段表无效");

		CacheSegmentDescriptor? meshDescriptor = null;
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

		var meshSegment = meshDescriptor ?? throw new Exception("缓存缺少 Mesh 段");
		var volumeSegment = volumeDescriptor ?? throw new Exception("缓存缺少 Volume 段");

		var (mesh, meshTelemetry) = ReadSegment(reader.BaseStream, meshSegment, DeserializeMesh);
		var (volume, volumeTelemetry) = ReadSegment(reader.BaseStream, volumeSegment, DeserializeVolume);
		return new(new(customizationVersion, buildSignature, customizationApplied, mesh, volume), new(meshTelemetry, volumeTelemetry));
	}

	public CacheTelemetry Serialize(BinaryWriter writer)
	{
		writer.Write(Magic);
		writer.Write(Version);
		writer.Write(CustomizationVersion);
		writer.Write(BuildSignature);
		writer.Write(CustomizationApplied);

		var descriptors = new[]
		{
			new CacheSegmentDescriptor(CacheSegmentKind.Mesh, CacheCodec.BrotliFastest, 0, 0, 0),
			new CacheSegmentDescriptor(CacheSegmentKind.Volume, CacheCodec.BrotliFastest, 0, 0, 0)
		};

		writer.Write(descriptors.Length);
		var descriptorOffsets = new long[descriptors.Length];
		for (var i = 0; i < descriptors.Length; ++i)
		{
			descriptorOffsets[i] = writer.BaseStream.Position;
			WriteSegmentDescriptor(writer, descriptors[i]);
		}

		var meshTelemetry = WriteSegment(writer, descriptorOffsets[0], descriptors[0], meshWriter => SerializeMesh(meshWriter, Mesh));
		var volumeWorkspace = new VolumeCodecWorkspace();
		var volumeTelemetry = WriteSegment(writer, descriptorOffsets[1], descriptors[1], volumeWriter => SerializeVolume(volumeWriter, Volume, volumeWorkspace));
		return new(meshTelemetry, volumeTelemetry);
	}

	private static CacheSegmentTelemetry WriteSegment(BinaryWriter writer, long descriptorOffset, CacheSegmentDescriptor descriptor, Action<BinaryWriter> serialize)
	{
		var timer = Timer.Create();
		var offset = writer.BaseStream.Position;
		var countingStream = CreateSegmentWriteStream(writer.BaseStream, descriptor.Codec, out var disposableStream);
		long uncompressedBytes;
		try
		{
			using var _ = disposableStream;
			using var segmentWriter = new BinaryWriter(countingStream);
			serialize(segmentWriter);
			segmentWriter.Flush();
			countingStream.Flush();
			uncompressedBytes = countingStream.BytesProcessed;
		}
		finally
		{
			writer.Flush();
		}

		var endOffset = writer.BaseStream.Position;
		var compressedBytes = endOffset - offset;
		var updatedDescriptor = descriptor with { Offset = offset, CompressedBytes = compressedBytes, UncompressedBytes = uncompressedBytes };
		var resumeOffset = endOffset;
		writer.BaseStream.Position = descriptorOffset;
		WriteSegmentDescriptor(writer, updatedDescriptor);
		writer.BaseStream.Position = resumeOffset;
		return new(descriptor.Kind, compressedBytes, uncompressedBytes, timer.Value());
	}

	private static (T Value, CacheSegmentTelemetry Telemetry) ReadSegment<T>(Stream source, CacheSegmentDescriptor descriptor, Func<BinaryReader, T> deserialize)
	{
		if (descriptor.Offset < 0 || descriptor.CompressedBytes < 0 || descriptor.Offset + descriptor.CompressedBytes > source.Length)
			throw new Exception($"缓存段越界: {descriptor.Kind} @ {descriptor.Offset} + {descriptor.CompressedBytes}");

		var timer = Timer.Create();
		using var segmentStream = new SegmentReadStream(source, descriptor.Offset, descriptor.CompressedBytes);
		var countingStream = CreateSegmentReadStream(segmentStream, descriptor.Codec, out var disposableStream);
		using var _ = disposableStream;
		using var segmentReader = new BinaryReader(countingStream);
		var value = deserialize(segmentReader);
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
		Stream stream = codec switch
		{
			CacheCodec.None => destination,
			CacheCodec.BrotliFastest => new BrotliStream(destination, CompressionLevel.Fastest, true),
			_ => throw new Exception($"Unsupported cache codec: {codec}")
		};

		var counting = new CountingStream(stream, codec == CacheCodec.None);
		disposableStream = counting;
		return counting;
	}

	private static CountingStream CreateSegmentReadStream(Stream source, CacheCodec codec, out IDisposable disposableStream)
	{
		Stream stream = codec switch
		{
			CacheCodec.None => source,
			CacheCodec.BrotliFastest => new BrotliStream(source, CompressionMode.Decompress, true),
			_ => throw new Exception($"Unsupported cache codec: {codec}")
		};

		var counting = new CountingStream(stream, codec == CacheCodec.None);
		disposableStream = counting;
		return counting;
	}

	private static CacheSegmentDescriptor ReadSegmentDescriptor(BinaryReader reader) => new(
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

	private static DtNavMesh DeserializeMesh(BinaryReader reader)
	{
		var numTiles = reader.ReadInt32();
		var opts = DeserializeMeshParams(reader);
		var result = new DtNavMesh(opts, reader.ReadInt32());
		for (int i = 0; i < numTiles; ++i)
		{
			var tileRef = reader.ReadInt64();
			var tile = DeserializeMeshTile(reader);
			result.AddTile(tile, i, tileRef);
		}
		return result;
	}

	private static void SerializeMesh(BinaryWriter writer, DtNavMesh mesh)
	{
		writer.Write(mesh.GetTileCount());
		SerializeMeshParams(writer, mesh.GetParams());
		writer.Write(mesh.GetMaxVertsPerPoly());

		for (int i = 0; i < mesh.GetMaxTiles(); ++i)
		{
			DtMeshTile tile = mesh.GetTile(i);
			if (tile?.data?.header == null)
				continue;
			writer.Write(mesh.GetTileRef(tile));
			SerializeMeshTile(writer, tile.data);
		}
	}

	private static DtNavMeshParams DeserializeMeshParams(BinaryReader reader) => new()
	{
		orig = DeserializeVector3(reader).SystemToRecast(),
		tileWidth = reader.ReadSingle(),
		tileHeight = reader.ReadSingle(),
		maxTiles = reader.ReadInt32(),
		maxPolys = reader.ReadInt32()
	};

	private static void SerializeMeshParams(BinaryWriter writer, DtNavMeshParams opt)
	{
		SerializeVector3(writer, opt.orig.RecastToSystem());
		writer.Write(opt.tileWidth);
		writer.Write(opt.tileHeight);
		writer.Write(opt.maxTiles);
		writer.Write(opt.maxPolys);
	}

	private static DtMeshData DeserializeMeshTile(BinaryReader reader)
	{
		var tile = new DtMeshData();
		tile.header = new();
		tile.header.magic = DtNavMesh.DT_NAVMESH_MAGIC;
		tile.header.version = DtNavMesh.DT_NAVMESH_VERSION;
		tile.header.x = reader.ReadInt32();
		tile.header.y = reader.ReadInt32();
		tile.header.layer = reader.ReadInt32();
		tile.header.userId = reader.ReadInt32();
		tile.header.walkableHeight = reader.ReadSingle();
		tile.header.walkableRadius = reader.ReadSingle();
		tile.header.walkableClimb = reader.ReadSingle();
		var bounds = DeserializeBounds(reader);
		tile.header.bmin = bounds.min.SystemToRecast();
		tile.header.bmax = bounds.max.SystemToRecast();

		tile.header.vertCount = reader.ReadInt32();
		tile.verts = new float[tile.header.vertCount * 3];
		for (int i = 0; i < tile.verts.Length; ++i)
			tile.verts[i] = reader.ReadSingle();

		tile.header.polyCount = reader.ReadInt32();
		tile.polys = new DtPoly[tile.header.polyCount];
		for (int i = 0; i < tile.header.polyCount; ++i)
		{
			var nv = reader.ReadByte();
			var poly = tile.polys[i] = new DtPoly(i, nv);
			poly.vertCount = nv;
			poly.areaAndtype = reader.ReadByte();
			poly.flags = reader.ReadUInt16();
			for (int j = 0; j < nv; ++j)
				poly.verts[j] = reader.ReadUInt16();
			for (int j = 0; j < nv; ++j)
				poly.neis[j] = reader.ReadUInt16();
		}

		tile.header.detailMeshCount = reader.ReadInt32();
		tile.detailMeshes = new DtPolyDetail[tile.header.detailMeshCount];
		for (int i = 0; i < tile.header.detailMeshCount; ++i)
			tile.detailMeshes[i] = new(reader.ReadInt32(), reader.ReadInt32(), reader.ReadByte(), reader.ReadByte());

		tile.header.detailVertCount = reader.ReadInt32();
		tile.detailVerts = new float[tile.header.detailVertCount * 3];
		for (int i = 0; i < tile.detailVerts.Length; ++i)
			tile.detailVerts[i] = reader.ReadSingle();

		tile.header.detailTriCount = reader.ReadInt32();
		tile.detailTris = new int[tile.header.detailTriCount * 4];
		for (int i = 0; i < tile.detailTris.Length; ++i)
			tile.detailTris[i] = reader.ReadByte();

		tile.header.bvQuantFactor = reader.ReadSingle();
		tile.header.bvNodeCount = reader.ReadInt32();
		tile.bvTree = new DtBVNode[tile.header.bvNodeCount];
		for (int i = 0; i < tile.header.bvNodeCount; ++i)
		{
			var node = tile.bvTree[i] = new();
			node.bmin[0] = reader.ReadInt32();
			node.bmin[1] = reader.ReadInt32();
			node.bmin[2] = reader.ReadInt32();
			node.bmax[0] = reader.ReadInt32();
			node.bmax[1] = reader.ReadInt32();
			node.bmax[2] = reader.ReadInt32();
			node.i = reader.ReadInt32();
		}

		tile.header.offMeshBase = reader.ReadInt32();
		tile.header.offMeshConCount = reader.ReadInt32();
		tile.offMeshCons = new DtOffMeshConnection[tile.header.offMeshConCount];
		for (int i = 0; i < tile.header.offMeshConCount; i++)
		{
			var conn = tile.offMeshCons[i] = new();
			conn.pos[0] = DeserializeVector3(reader).SystemToRecast();
			conn.pos[1] = DeserializeVector3(reader).SystemToRecast();
			conn.rad = reader.ReadSingle();
			conn.poly = reader.ReadUInt16();
			conn.flags = reader.ReadByte();
			conn.side = reader.ReadByte();
			conn.userId = reader.ReadInt32();
		}

		return tile;
	}

	private static void SerializeMeshTile(BinaryWriter writer, DtMeshData tile)
	{
		writer.Write(tile.header.x);
		writer.Write(tile.header.y);
		writer.Write(tile.header.layer);
		writer.Write(tile.header.userId);
		writer.Write(tile.header.walkableHeight);
		writer.Write(tile.header.walkableRadius);
		writer.Write(tile.header.walkableClimb);
		SerializeBounds(writer, tile.header.bmin.RecastToSystem(), tile.header.bmax.RecastToSystem());

		writer.Write(tile.header.vertCount);
		for (int i = 0; i < tile.header.vertCount * 3; ++i)
			writer.Write(tile.verts[i]);

		writer.Write(tile.header.polyCount);
		for (int i = 0; i < tile.header.polyCount; ++i)
		{
			var poly = tile.polys[i];
			writer.Write((byte)poly.vertCount);
			writer.Write((byte)poly.areaAndtype);
			writer.Write((ushort)poly.flags);
			for (int j = 0; j < poly.vertCount; ++j)
				writer.Write((ushort)poly.verts[j]);
			for (int j = 0; j < poly.vertCount; ++j)
				writer.Write((ushort)poly.neis[j]);
		}

		writer.Write(tile.header.detailMeshCount);
		for (int i = 0; i < tile.header.detailMeshCount; ++i)
		{
			ref var mesh = ref tile.detailMeshes[i];
			writer.Write(mesh.vertBase);
			writer.Write(mesh.triBase);
			writer.Write((byte)mesh.vertCount);
			writer.Write((byte)mesh.triCount);
		}

		writer.Write(tile.header.detailVertCount);
		for (int i = 0; i < tile.header.detailVertCount * 3; ++i)
			writer.Write(tile.detailVerts[i]);

		writer.Write(tile.header.detailTriCount);
		for (int i = 0; i < tile.header.detailTriCount * 4; ++i)
			writer.Write((byte)tile.detailTris[i]);

		writer.Write(tile.header.bvQuantFactor);
		writer.Write(tile.header.bvNodeCount);
		for (int i = 0; i < tile.header.bvNodeCount; ++i)
		{
			var node = tile.bvTree[i];
			writer.Write(node.bmin[0]);
			writer.Write(node.bmin[1]);
			writer.Write(node.bmin[2]);
			writer.Write(node.bmax[0]);
			writer.Write(node.bmax[1]);
			writer.Write(node.bmax[2]);
			writer.Write(node.i);
		}

		writer.Write(tile.header.offMeshBase);
		writer.Write(tile.header.offMeshConCount);
		for (int i = 0; i < tile.header.offMeshConCount; i++)
		{
			var conn = tile.offMeshCons[i];
			SerializeVector3(writer, conn.pos[0].RecastToSystem());
			SerializeVector3(writer, conn.pos[1].RecastToSystem());
			writer.Write(conn.rad);
			writer.Write((ushort)conn.poly);
			writer.Write((byte)conn.flags);
			writer.Write((byte)conn.side);
			writer.Write(conn.userId);
		}
	}

	private static VoxelMap? DeserializeVolume(BinaryReader reader)
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
		var volume = new VoxelMap(min, max, tilesPerLevel);
		var workspace = new VolumeCodecWorkspace();
		DeserializeVolumeTile(reader, volume.RootTile, workspace);
		return volume;
	}

	private static void SerializeVolume(BinaryWriter writer, VoxelMap? volume, VolumeCodecWorkspace workspace)
	{
		writer.Write(volume != null);
		if (volume == null)
			return;

		writer.Write(volume.Levels.Length);
		foreach (ref var l in volume.Levels.AsSpan())
			writer.Write(l.NumCellsX); // note: current assumption is that all dimensions are identical

		SerializeBounds(writer, volume.RootTile.BoundsMin, volume.RootTile.BoundsMax);
		SerializeVolumeTile(writer, volume.RootTile, workspace);
	}

	private static void DeserializeVolumeTile(BinaryReader reader, VoxelMap.Tile tile, VolumeCodecWorkspace workspace)
	{
		var encoding = (VolumeTileEncoding)reader.ReadByte();
		switch (encoding)
		{
			case VolumeTileEncoding.Empty:
				Array.Clear(tile.Contents);
				tile.Subdivision.Clear();
				return;

			case VolumeTileEncoding.SolidLeaf:
				Array.Fill(tile.Contents, ushort.MaxValue);
				tile.Subdivision.Clear();
				return;

			case VolumeTileEncoding.Mixed:
				break;

			default:
				throw new Exception($"未知的体积编码类型: {encoding}");
		}

		tile.Subdivision.Clear();
		var packedBytes = PackedStateBytes(tile.Contents.Length);
		var packedStates = workspace.PackedStates(packedBytes);
		reader.ReadExactly(packedStates);

		for (var i = 0; i < tile.Contents.Length; ++i)
		{
			var state = (VolumeCellState)((packedStates[i >> 2] >> ((i & 3) * 2)) & 0x3);
			tile.Contents[i] = state switch
			{
				VolumeCellState.Empty => 0,
				VolumeCellState.SolidLeaf => ushort.MaxValue,
				VolumeCellState.Subtree => DeserializeVolumeSubtile(reader, tile, i, workspace),
				_ => throw new Exception($"未知的体积单元状态: {state}")
			};
		}
	}

	private static ushort DeserializeVolumeSubtile(BinaryReader reader, VoxelMap.Tile parent, int flatIndex, VolumeCodecWorkspace workspace)
	{
		var localId = parent.Subdivision.Count;
		if (localId >= VoxelMap.VoxelIdMask)
			throw new Exception("体积子树数量超出上限");

		var subBounds = parent.CalculateSubdivisionBounds(parent.LevelDesc.IndexToVoxel((ushort)flatIndex));
		var child = new VoxelMap.Tile(parent.Owner, subBounds.min, subBounds.max, parent.Level + 1);
		parent.Subdivision.Add(child);
		DeserializeVolumeTile(reader, child, workspace);
		return (ushort)(VoxelMap.VoxelOccupiedBit | localId);
	}

	private static void SerializeVolumeTile(BinaryWriter writer, VoxelMap.Tile tile, VolumeCodecWorkspace workspace)
	{
		var encoding = ClassifyTile(tile);
		writer.Write((byte)encoding);
		if (encoding != VolumeTileEncoding.Mixed)
			return;

		var packedBytes = PackedStateBytes(tile.Contents.Length);
		var packedStates = workspace.PackedStates(packedBytes);
		packedStates.Clear();
		for (var i = 0; i < tile.Contents.Length; ++i)
		{
			var state = ClassifyCell(tile.Contents[i]);
			packedStates[i >> 2] |= (byte)((byte)state << ((i & 3) * 2));
		}

		writer.Write(packedStates);
		for (var i = 0; i < tile.Contents.Length; ++i)
		{
			if (ClassifyCell(tile.Contents[i]) != VolumeCellState.Subtree)
				continue;

			var localId = tile.Contents[i] & VoxelMap.VoxelIdMask;
			if (localId >= tile.Subdivision.Count)
				throw new Exception($"体积子树索引越界: {localId} / {tile.Subdivision.Count}");
			SerializeVolumeTile(writer, tile.Subdivision[localId], workspace);
		}
	}

	private static VolumeTileEncoding ClassifyTile(VoxelMap.Tile tile)
	{
		var allEmpty = true;
		var allSolidLeaf = tile.Subdivision.Count == 0;
		foreach (var cell in tile.Contents)
		{
			allEmpty &= cell == 0;
			allSolidLeaf &= cell == ushort.MaxValue;
			if (!allEmpty && !allSolidLeaf)
				return VolumeTileEncoding.Mixed;
		}

		return allEmpty ? VolumeTileEncoding.Empty : VolumeTileEncoding.SolidLeaf;
	}

	private static VolumeCellState ClassifyCell(ushort value) => value switch
	{
		0 => VolumeCellState.Empty,
		ushort.MaxValue => VolumeCellState.SolidLeaf,
		_ => VolumeCellState.Subtree
	};

	private static int PackedStateBytes(int numCells) => (numCells + 3) >> 2;

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
	
	public enum CacheSegmentKind : int
	{
		Mesh = 1,
		Volume = 2
	}

	private enum CacheCodec : int
	{
		None = 0,
		BrotliFastest = 1
	}

	private enum VolumeTileEncoding : byte
	{
		Empty = 0,
		SolidLeaf = 1,
		Mixed = 2
	}

	private enum VolumeCellState : byte
	{
		Empty = 0,
		SolidLeaf = 1,
		Subtree = 2
	}

	public readonly record struct CacheSegmentTelemetry(CacheSegmentKind Kind, long CompressedBytes, long UncompressedBytes, TimeSpan Duration);
	public readonly record struct CacheTelemetry(CacheSegmentTelemetry Mesh, CacheSegmentTelemetry Volume)
	{
		public long TotalCompressedBytes => Mesh.CompressedBytes + Volume.CompressedBytes;
		public long TotalUncompressedBytes => Mesh.UncompressedBytes + Volume.UncompressedBytes;
	}

	public readonly record struct DeserializeResult(Navmesh Navmesh, CacheTelemetry Telemetry);

	private readonly record struct CacheSegmentDescriptor(CacheSegmentKind Kind, CacheCodec Codec, long Offset, long CompressedBytes, long UncompressedBytes);

	private sealed class VolumeCodecWorkspace
	{
		private byte[] _packedStates = [];

		public Span<byte> PackedStates(int size)
		{
			if (_packedStates.Length < size)
				_packedStates = GC.AllocateUninitializedArray<byte>(size);
			return _packedStates.AsSpan(0, size);
		}
	}

	private sealed class CountingStream(Stream inner, bool leaveOpen) : Stream
	{
		public long BytesProcessed { get; private set; }

		public override bool CanRead => inner.CanRead;
		public override bool CanSeek => inner.CanSeek;
		public override bool CanWrite => inner.CanWrite;
		public override long Length => inner.Length;
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

	private sealed class SegmentReadStream(Stream inner, long offset, long length) : Stream
	{
		private long _position;

		public override bool CanRead => true;
		public override bool CanSeek => true;
		public override bool CanWrite => false;
		public override long Length => length;
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
				SeekOrigin.Begin => seekOffset,
				SeekOrigin.Current => _position + seekOffset,
				SeekOrigin.End => length + seekOffset,
				_ => throw new ArgumentOutOfRangeException(nameof(origin))
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
