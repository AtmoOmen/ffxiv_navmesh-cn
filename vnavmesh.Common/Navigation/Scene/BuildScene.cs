using System.Numerics;
using System.Runtime.InteropServices;
using vnavmesh.Common.Numerics;

namespace vnavmesh.Common.Navigation.Scene;

public sealed class BuildScene
{
    public Dictionary<string, Mesh> Meshes { get; } = [];

    public enum MeshType
    {
        None          = 0,
        Terrain       = 1 << 0,
        FileMesh      = 1 << 1,
        CylinderMesh  = 1 << 2,
        AnalyticShape = 1 << 3,
        AnalyticPlane = 1 << 4,
        All           = (1 << 5) - 1
    }

    [Flags]
    public enum PrimitiveFlags
    {
        None            = 0,
        ForceUnwalkable = 1 << 0,
        FlyThrough      = 1 << 1,
        Unlandable      = 1 << 2,
        ForceWalkable   = 1 << 3,
        Fishable        = 1 << 4
    }

    public sealed class Mesh
    {
        public List<MeshPart>     Parts     = [];
        public List<MeshInstance> Instances = [];
        public MeshType           MeshType;
        public Aabb               LocalBounds;

        public Span<MeshPart> PartSpan => CollectionsMarshal.AsSpan(Parts);
    }

    public sealed class MeshPart
    {
        public List<Vector3>   Vertices   = [];
        public List<Primitive> Primitives = [];
        public Aabb            LocalBounds;

        public Span<Vector3>   VertexSpan    => CollectionsMarshal.AsSpan(Vertices);
        public Span<Primitive> PrimitiveSpan => CollectionsMarshal.AsSpan(Primitives);
    }

    public sealed class MeshInstance
    (
        ulong          id,
        Matrix4x3      worldTransform,
        Aabb           worldBounds,
        ulong          material,
        PrimitiveFlags forceSetPrimFlags,
        PrimitiveFlags forceClearPrimFlags
    )
    {
        public ulong          Id                  = id;
        public ulong          Material            = material;
        public Matrix4x3      WorldTransform      = worldTransform;
        public Aabb           WorldBounds         = worldBounds;
        public PrimitiveFlags ForceSetPrimFlags   = forceSetPrimFlags;
        public PrimitiveFlags ForceClearPrimFlags = forceClearPrimFlags;
    }

    public readonly record struct Primitive
    (
        int            V1,
        int            V2,
        int            V3,
        PrimitiveFlags Flags,
        ulong          Material = 0
    );

    public static BuildScene Read(BinaryReader reader)
    {
        var scene     = new BuildScene();
        var meshCount = reader.ReadInt32();

        for (var i = 0; i < meshCount; ++i)
        {
            var key = reader.ReadString();
            scene.Meshes[key] = ReadMesh(reader);
        }

        return scene;
    }

    public void Write(BinaryWriter writer)
    {
        writer.Write(Meshes.Count);

        foreach (var (key, mesh) in Meshes)
        {
            writer.Write(key);
            WriteMesh(writer, mesh);
        }
    }

    private static Mesh ReadMesh(BinaryReader reader)
    {
        var mesh = new Mesh
        {
            MeshType    = (MeshType)reader.ReadInt32(),
            LocalBounds = Aabb.Read(reader)
        };

        var partCount = reader.ReadInt32();
        for (var i = 0; i < partCount; ++i)
            mesh.Parts.Add(ReadPart(reader));

        var instanceCount = reader.ReadInt32();
        for (var i = 0; i < instanceCount; ++i)
            mesh.Instances.Add(ReadInstance(reader));

        return mesh;
    }

    private static void WriteMesh(BinaryWriter writer, Mesh mesh)
    {
        writer.Write((int)mesh.MeshType);
        mesh.LocalBounds.Write(writer);
        writer.Write(mesh.Parts.Count);
        foreach (var part in mesh.Parts)
            WritePart(writer, part);
        writer.Write(mesh.Instances.Count);
        foreach (var instance in mesh.Instances)
            WriteInstance(writer, instance);
    }

    private static MeshPart ReadPart(BinaryReader reader)
    {
        var part = new MeshPart
        {
            LocalBounds = Aabb.Read(reader)
        };

        var vertexCount = reader.ReadInt32();
        for (var i = 0; i < vertexCount; ++i)
            part.Vertices.Add(new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()));

        var primitiveCount = reader.ReadInt32();
        for (var i = 0; i < primitiveCount; ++i)
            part.Primitives.Add(new(reader.ReadInt32(), reader.ReadInt32(), reader.ReadInt32(), (PrimitiveFlags)reader.ReadInt32(), reader.ReadUInt64()));

        return part;
    }

    private static void WritePart(BinaryWriter writer, MeshPart part)
    {
        part.LocalBounds.Write(writer);
        writer.Write(part.Vertices.Count);

        foreach (var v in part.Vertices)
        {
            writer.Write(v.X);
            writer.Write(v.Y);
            writer.Write(v.Z);
        }

        writer.Write(part.Primitives.Count);

        foreach (var p in part.Primitives)
        {
            writer.Write(p.V1);
            writer.Write(p.V2);
            writer.Write(p.V3);
            writer.Write((int)p.Flags);
            writer.Write(p.Material);
        }
    }

    private static MeshInstance ReadInstance(BinaryReader reader) =>
        new
        (
            reader.ReadUInt64(),
            Matrix4x3.Read(reader),
            Aabb.Read(reader),
            reader.ReadUInt64(),
            (PrimitiveFlags)reader.ReadInt32(),
            (PrimitiveFlags)reader.ReadInt32()
        );

    private static void WriteInstance(BinaryWriter writer, MeshInstance instance)
    {
        writer.Write(instance.Id);
        instance.WorldTransform.Write(writer);
        instance.WorldBounds.Write(writer);
        writer.Write(instance.Material);
        writer.Write((int)instance.ForceSetPrimFlags);
        writer.Write((int)instance.ForceClearPrimFlags);
    }
}
