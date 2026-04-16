using DotRecast.Detour;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

using static DotRecast.Detour.DtDetour;

public partial record class Navmesh
{
    private static DtNavMesh DeserializeMesh(BinaryReader reader)
    {
        var numTiles = reader.ReadInt32();
        var opts     = DeserializeMeshParams(reader);
        var result   = new DtNavMesh();
        result.Init(opts, reader.ReadInt32());

        for (var i = 0; i < numTiles; ++i)
        {
            var tileRef = reader.ReadInt64();
            var tile    = DeserializeMeshTile(reader);
            result.AddTile(tile, i, tileRef, out _);
        }

        return result;
    }

    private static void SerializeMesh(BinaryWriter writer, DtNavMesh mesh)
    {
        var numTiles = 0;
        for (var i = 0; i < mesh.GetMaxTiles(); ++i)
        {
            var tile = mesh.GetTile(i);
            if (tile?.data?.header != null)
                ++numTiles;
        }

        writer.Write(numTiles);
        SerializeMeshParams(writer, mesh.GetParams());
        writer.Write(mesh.GetMaxVertsPerPoly());

        for (var i = 0; i < mesh.GetMaxTiles(); ++i)
        {
            var tile = mesh.GetTile(i);
            if (tile?.data?.header == null)
                continue;
            writer.Write(mesh.GetTileRef(tile));
            SerializeMeshTile(writer, tile.data);
        }
    }

    private static DtNavMeshParams DeserializeMeshParams(BinaryReader reader) => new()
    {
        orig       = DeserializeVector3(reader).SystemToRecast(),
        tileWidth  = reader.ReadSingle(),
        tileHeight = reader.ReadSingle(),
        maxTiles   = reader.ReadInt32(),
        maxPolys   = reader.ReadInt32()
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
        tile.header                = new();
        tile.header.magic          = DT_NAVMESH_MAGIC;
        tile.header.version        = DT_NAVMESH_VERSION;
        tile.header.x              = reader.ReadInt32();
        tile.header.y              = reader.ReadInt32();
        tile.header.layer          = reader.ReadInt32();
        tile.header.userId         = reader.ReadInt32();
        tile.header.walkableHeight = reader.ReadSingle();
        tile.header.walkableRadius = reader.ReadSingle();
        tile.header.walkableClimb  = reader.ReadSingle();
        var bounds = DeserializeBounds(reader);
        tile.header.bmin = bounds.min.SystemToRecast();
        tile.header.bmax = bounds.max.SystemToRecast();

        tile.header.vertCount = reader.ReadInt32();
        tile.verts            = ReadSingleArray(reader, tile.header.vertCount * 3);

        tile.header.polyCount = reader.ReadInt32();
        tile.polys            = new DtPoly[tile.header.polyCount];

        for (var i = 0; i < tile.header.polyCount; ++i)
        {
            var nv   = reader.ReadByte();
            var poly = tile.polys[i] = new DtPoly(i, nv);
            poly.vertCount   = nv;
            poly.areaAndtype = reader.ReadByte();
            poly.flags       = reader.ReadUInt16();
            for (var j = 0; j < nv; ++j)
                poly.verts[j] = reader.ReadUInt16();
            for (var j = 0; j < nv; ++j)
                poly.neis[j] = reader.ReadUInt16();
        }

        tile.header.detailMeshCount = reader.ReadInt32();
        tile.detailMeshes           = new DtPolyDetail[tile.header.detailMeshCount];
        for (var i = 0; i < tile.header.detailMeshCount; ++i)
            tile.detailMeshes[i] = new(reader.ReadInt32(), reader.ReadInt32(), reader.ReadByte(), reader.ReadByte());

        tile.header.detailVertCount = reader.ReadInt32();
        tile.detailVerts            = ReadSingleArray(reader, tile.header.detailVertCount * 3);

        tile.header.detailTriCount = reader.ReadInt32();
        tile.detailTris            = ReadByteBackedIntArray(reader, tile.header.detailTriCount * 4);

        tile.header.bvQuantFactor = reader.ReadSingle();
        tile.header.bvNodeCount   = reader.ReadInt32();
        tile.bvTree               = new DtBVNode[tile.header.bvNodeCount];

        for (var i = 0; i < tile.header.bvNodeCount; ++i)
        {
            var node = tile.bvTree[i] = new();
            node.bmin.X = reader.ReadInt32();
            node.bmin.Y = reader.ReadInt32();
            node.bmin.Z = reader.ReadInt32();
            node.bmax.X = reader.ReadInt32();
            node.bmax.Y = reader.ReadInt32();
            node.bmax.Z = reader.ReadInt32();
            node.i       = reader.ReadInt32();
        }

        tile.header.offMeshBase     = reader.ReadInt32();
        tile.header.offMeshConCount = reader.ReadInt32();
        tile.offMeshCons            = new DtOffMeshConnection[tile.header.offMeshConCount];

        for (var i = 0; i < tile.header.offMeshConCount; i++)
        {
            var conn = tile.offMeshCons[i] = new();
            conn.pos[0] = DeserializeVector3(reader).SystemToRecast();
            conn.pos[1] = DeserializeVector3(reader).SystemToRecast();
            conn.rad    = reader.ReadSingle();
            conn.poly   = reader.ReadUInt16();
            conn.flags  = reader.ReadByte();
            conn.side   = reader.ReadByte();
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
        WriteSingleArray(writer, tile.verts);
        writer.Write(tile.header.polyCount);

        for (var i = 0; i < tile.header.polyCount; ++i)
        {
            var poly = tile.polys[i];
            writer.Write((byte)poly.vertCount);
            writer.Write((byte)poly.areaAndtype);
            writer.Write((ushort)poly.flags);
            for (var j = 0; j < poly.vertCount; ++j)
                writer.Write((ushort)poly.verts[j]);
            for (var j = 0; j < poly.vertCount; ++j)
                writer.Write((ushort)poly.neis[j]);
        }

        writer.Write(tile.header.detailMeshCount);

        for (var i = 0; i < tile.header.detailMeshCount; ++i)
        {
            ref var mesh = ref tile.detailMeshes[i];
            writer.Write(mesh.vertBase);
            writer.Write(mesh.triBase);
            writer.Write((byte)mesh.vertCount);
            writer.Write((byte)mesh.triCount);
        }

        writer.Write(tile.header.detailVertCount);
        WriteSingleArray(writer, tile.detailVerts);
        writer.Write(tile.header.detailTriCount);
        WriteByteBackedIntArray(writer, tile.detailTris);

        writer.Write(tile.header.bvQuantFactor);
        writer.Write(tile.header.bvNodeCount);

        for (var i = 0; i < tile.header.bvNodeCount; ++i)
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

        for (var i = 0; i < tile.header.offMeshConCount; i++)
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
}
