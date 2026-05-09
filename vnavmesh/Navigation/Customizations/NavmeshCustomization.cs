using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Customizations;

using static DtDetour;

// base class for per-territory navmesh customizations
public class NavmeshCustomization
{
    // every time defaults change, we need to bump global navmesh version - this should be kept at zero
    // every time customization changes, we can bump the local version field, to avoid invalidating whole cache
    // each derived class should set it to non-zero value
    public virtual int Version => 0;

    public NavmeshSettings Settings = new();

    public NavmeshSettings GetBuildSettings(SceneDefinition definition)
    {
        var settings = Settings.Clone();
        var profile  = new NavmeshBuildProfile();
        CustomizeBuildProfile(definition, profile);
        profile.ApplyTo(settings);
        return settings;
    }

    public virtual bool IsFlyingSupported(SceneDefinition definition) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID)?.TerritoryIntendedUse.RowId is 1 or 49 or 47; // 野外、无人岛、云冠群岛

    // this is a customization point to add or remove colliders in the scene
    public virtual void CustomizeScene(SceneExtractor scene) { }

    public virtual void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile) { }

    public virtual void CustomizeSettings(DtNavMeshCreateParams config) { }

    public virtual void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers) { }

    protected static void LinkPoints(Navmesh meshData, Vector3 startPos, Vector3 endPos) =>
        LinkPoints(meshData, startPos, endPos, NavmeshArea.Teleport, NavmeshPolyFlags.Teleport, NavmeshOffMeshKind.Teleport);

    protected static void LinkClientPath(Navmesh meshData, Vector3 startPos, Vector3 endPos) =>
        LinkPoints(meshData, startPos, endPos, NavmeshArea.ClientPath, NavmeshPolyFlags.ClientPath, NavmeshOffMeshKind.ClientPath);

    private static void LinkPoints
    (
        Navmesh            meshData,
        Vector3            startPos,
        Vector3            endPos,
        NavmeshArea        area,
        NavmeshPolyFlags   flags,
        NavmeshOffMeshKind kind
    )
    {
        meshData.Links.Add(new(startPos, endPos, kind, false, 0));
        var mesh = meshData.Mesh;
        var (startRef, startTile, startPoly, projectedStart) = ResolvePointPoly(mesh, startPos);
        var (endRef, _, _, projectedEnd)                     = ResolvePointPoly(mesh, endPos);
        var polyIndex   = startTile.data.header.polyCount;
        var vertexIndex = startTile.data.header.vertCount;
        var offMeshPoly = new DtPoly(polyIndex, mesh.GetMaxVertsPerPoly())
        {
            firstLink = DT_NULL_LINK,
            vertCount = 2,
            flags     = (int)flags
        };
        offMeshPoly.SetArea((int)area);
        offMeshPoly.SetPolyType(DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION);
        offMeshPoly.verts[0] = vertexIndex;
        offMeshPoly.verts[1] = vertexIndex + 1;

        startTile.data.header.polyCount       += 1;
        startTile.data.header.vertCount       += 2;
        startTile.data.header.offMeshConCount += 1;
        Array.Resize(ref startTile.data.polys,       startTile.data.header.polyCount);
        Array.Resize(ref startTile.data.verts,       startTile.data.header.vertCount * 3);
        Array.Resize(ref startTile.data.offMeshCons, startTile.data.header.offMeshConCount);

        startTile.data.polys[^1]                        = offMeshPoly;
        startTile.data.verts[vertexIndex * 3]           = projectedStart.X;
        startTile.data.verts[vertexIndex * 3 + 1]       = projectedStart.Y;
        startTile.data.verts[vertexIndex * 3 + 2]       = projectedStart.Z;
        startTile.data.verts[(vertexIndex + 1) * 3]     = projectedEnd.X;
        startTile.data.verts[(vertexIndex + 1) * 3 + 1] = projectedEnd.Y;
        startTile.data.verts[(vertexIndex + 1) * 3 + 2] = projectedEnd.Z;

        var offMeshConnection = new DtOffMeshConnection
        {
            poly   = polyIndex,
            rad    = 0.1f,
            side   = DtNavMeshBuilder.ClassifyOffMeshPoint(projectedEnd, startTile.data.header.bmin, startTile.data.header.bmax),
            userId = 0,
            pos    =
            {
                [0] = projectedStart,
                [1] = projectedEnd
            }
        };
        startTile.data.offMeshCons[^1] = offMeshConnection;

        var offMeshRef = EncodePolyId(DecodePolyIdSalt(startRef), startTile.index, polyIndex);
        var idx        = AllocLink(startTile);
        var link       = startTile.links[idx];
        link.refs             = startRef;
        link.edge             = 0;
        link.side             = 0xff;
        link.bmin             = link.bmax = 0;
        link.next             = offMeshPoly.firstLink;
        offMeshPoly.firstLink = idx;

        idx                 = AllocLink(startTile);
        link                = startTile.links[idx];
        link.refs           = offMeshRef;
        link.edge           = 0xff;
        link.side           = 0xff;
        link.bmin           = link.bmax = 0;
        link.next           = startPoly.firstLink;
        startPoly.firstLink = idx;

        idx                   = AllocLink(startTile);
        link                  = startTile.links[idx];
        link.refs             = endRef;
        link.edge             = 1;
        link.side             = (byte)offMeshConnection.side;
        link.bmin             = link.bmax = 0;
        link.next             = offMeshPoly.firstLink;
        offMeshPoly.firstLink = idx;
    }

    private static (long PolyRef, DtMeshTile Tile, DtPoly Poly, RcVec3f ProjectedPoint) ResolvePointPoly(DtNavMesh mesh, Vector3 pos)
    {
        var query = new DtNavMeshQuery(mesh);

        var status = query.FindNearestPoly(pos.SystemToRecast(), new(5, 5, 5), new DtQueryDefaultFilter(), out var polyRef, out var projectedPoint, out _);
        if (status.Failed() || polyRef == 0)
            throw new ArgumentException($"无法为传送端点定位地面多边形: {pos:f3}");

        mesh.GetTileAndPolyByRefUnsafe(polyRef, out var tile, out var poly);
        return (polyRef, tile, poly, projectedPoint);
    }

    private static int AllocLink(DtMeshTile tile)
    {
        if (tile.linksFreeList == DT_NULL_LINK)
            throw new InvalidOperationException("导航网格链接池已耗尽");

        var linkIndex = tile.linksFreeList;
        tile.linksFreeList = tile.links[linkIndex].next;
        return linkIndex;
    }
}
