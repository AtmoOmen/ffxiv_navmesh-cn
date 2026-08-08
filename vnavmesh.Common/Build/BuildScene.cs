using vnavmesh.Common.Build.Enums;
using vnavmesh.Common.Build.Models;
using vnavmesh.Common.Models;

namespace vnavmesh.Common.Build;

public class BuildScene
{
    public Dictionary<string, Mesh> Meshes { get; } = [];

    public static BuildScene Read
    (
        BinaryReader reader
    )
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

    public void Write
    (
        BinaryWriter writer
    )
    {
        writer.Write(Meshes.Count);

        foreach (var (key, mesh) in Meshes)
        {
            writer.Write(key);
            WriteMesh(writer, mesh);
        }
    }

    private static Mesh ReadMesh
    (
        BinaryReader reader
    )
    {
        var mesh = new Mesh
        {
            MeshType    = (MeshType)reader.ReadInt32(),
            LocalBounds = AABB.Read(reader)
        };

        var partCount = reader.ReadInt32();
        for (var i = 0; i < partCount; ++i)
            mesh.Parts.Add(ReadPart(reader));

        var instanceCount = reader.ReadInt32();
        for (var i = 0; i < instanceCount; ++i)
            mesh.Instances.Add(ReadInstance(reader));

        return mesh;
    }

    private static void WriteMesh
    (
        BinaryWriter writer,
        Mesh         mesh
    )
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

    private static MeshPart ReadPart
    (
        BinaryReader reader
    )
    {
        var part = new MeshPart
        {
            LocalBounds = AABB.Read(reader)
        };

        var vertexCount = reader.ReadInt32();
        for (var i = 0; i < vertexCount; ++i)
            part.Vertices.Add(new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()));

        var primitiveCount = reader.ReadInt32();
        for (var i = 0; i < primitiveCount; ++i)
            part.Primitives.Add(new(reader.ReadInt32(), reader.ReadInt32(), reader.ReadInt32(), (PrimitiveFlags)reader.ReadInt32(), reader.ReadUInt64()));

        return part;
    }

    private static void WritePart
    (
        BinaryWriter writer,
        MeshPart     part
    )
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

    private static MeshInstance ReadInstance
    (
        BinaryReader reader
    ) =>
        new
        (
            reader.ReadUInt64(),
            Matrix4x3.Read(reader),
            AABB.Read(reader),
            reader.ReadUInt64(),
            (PrimitiveFlags)reader.ReadInt32(),
            (PrimitiveFlags)reader.ReadInt32()
        );

    private static void WriteInstance
    (
        BinaryWriter writer,
        MeshInstance instance
    )
    {
        writer.Write(instance.ID);
        instance.WorldTransform.Write(writer);
        instance.WorldBounds.Write(writer);
        writer.Write(instance.Material);
        writer.Write((int)instance.ForceSetPrimFlags);
        writer.Write((int)instance.ForceClearPrimFlags);
    }
}
