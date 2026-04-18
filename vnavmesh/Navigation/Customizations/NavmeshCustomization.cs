using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using System.Reflection;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Customizations;

using static DtDetour;

public sealed class NavmeshBuildProfile
{
    public RcPartition? PartitioningOverride;
    public float?       CellSizeOverride;
    public float?       CellHeightOverride;
    public float?       RegionMinSizeOverride;
    public float?       RegionMergeSizeOverride;
    public float?       PolyMaxEdgeLenOverride;
    public float?       PolyMaxSimplificationErrorOverride;
    public float?       AgentRadiusOverride;
    public int[]?       VolumeTilesOverride;
    public float?       DetailSampleDistOverride;
    public bool?        GenerateEdgeClimbLinksOverride;
    public bool?        GenerateEdgeJumpLinksOverride;

    public void ApplyTo(NavmeshSettings settings)
    {
        if (PartitioningOverride is { } partitioning)
            settings.Partitioning = partitioning;
        if (CellSizeOverride is { } cellSize)
            settings.CellSize = cellSize;
        if (CellHeightOverride is { } cellHeight)
            settings.CellHeight = cellHeight;
        if (RegionMinSizeOverride is { } regionMinSize)
            settings.RegionMinSize = regionMinSize;
        if (RegionMergeSizeOverride is { } regionMergeSize)
            settings.RegionMergeSize = regionMergeSize;
        if (PolyMaxEdgeLenOverride is { } polyMaxEdgeLen)
            settings.PolyMaxEdgeLen = polyMaxEdgeLen;
        if (PolyMaxSimplificationErrorOverride is { } polyMaxSimplificationError)
            settings.PolyMaxSimplificationError = polyMaxSimplificationError;
        if (AgentRadiusOverride is { } agentRadius)
            settings.AgentRadius = agentRadius;
        if (VolumeTilesOverride is { } volumeTiles)
            settings.VolumeTiles = (int[])volumeTiles.Clone();
        if (DetailSampleDistOverride is { } detailSampleDist)
            settings.DetailSampleDist = detailSampleDist;
        if (GenerateEdgeClimbLinksOverride is { } generateEdgeClimbLinks)
            settings.GenerateEdgeClimbLinks = generateEdgeClimbLinks;
        if (GenerateEdgeJumpLinksOverride is { } generateEdgeJumpLinks)
            settings.GenerateEdgeJumpLinks = generateEdgeJumpLinks;
    }
}

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

    public virtual bool IsFlyingSupported
        (SceneDefinition definition) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID)?.TerritoryIntendedUse.RowId is 1 or 49 or 47; // 1 is normal outdoor, 49 is island, 47 is Diadem

    // this is a customization point to add or remove colliders in the scene
    public virtual void CustomizeScene(SceneExtractor scene) { }

    public virtual void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile) { }

    public virtual void CustomizeSettings(DtNavMeshCreateParams config) { }

    public virtual void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers) { }

    protected static void LinkPoints(Navmesh nmesh, Vector3 startPos, Vector3 endPos)
    {
        nmesh.Links.Add(new(startPos, endPos, NavmeshOffMeshKind.Teleport, false, 0));
        var mesh                                              = nmesh.Mesh;
        var (startRef, startTile, startPoly, projectedStart) = ResolvePointPoly(mesh, startPos);
        var (endRef, _, _, projectedEnd)                      = ResolvePointPoly(mesh, endPos);
        var polyIndex                                         = startTile.data.header.polyCount;
        var vertexIndex                                       = startTile.data.header.vertCount;
        var offMeshPoly                                       = new DtPoly(polyIndex, mesh.GetMaxVertsPerPoly())
        {
            firstLink = DT_NULL_LINK,
            vertCount = 2,
            flags     = (int)NavmeshPolyFlags.Teleport
        };
        offMeshPoly.SetArea((int)NavmeshArea.Teleport);
        offMeshPoly.SetPolyType(DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION);
        offMeshPoly.verts[0] = vertexIndex;
        offMeshPoly.verts[1] = vertexIndex + 1;

        startTile.data.header.polyCount += 1;
        startTile.data.header.vertCount += 2;
        startTile.data.header.offMeshConCount += 1;
        Array.Resize(ref startTile.data.polys, startTile.data.header.polyCount);
        Array.Resize(ref startTile.data.verts, startTile.data.header.vertCount * 3);
        Array.Resize(ref startTile.data.offMeshCons, startTile.data.header.offMeshConCount);

        startTile.data.polys[^1] = offMeshPoly;
        startTile.data.verts[vertexIndex * 3] = projectedStart.X;
        startTile.data.verts[vertexIndex * 3 + 1] = projectedStart.Y;
        startTile.data.verts[vertexIndex * 3 + 2] = projectedStart.Z;
        startTile.data.verts[(vertexIndex + 1) * 3] = projectedEnd.X;
        startTile.data.verts[(vertexIndex + 1) * 3 + 1] = projectedEnd.Y;
        startTile.data.verts[(vertexIndex + 1) * 3 + 2] = projectedEnd.Z;

        var offMeshConnection = new DtOffMeshConnection
        {
            poly   = polyIndex,
            rad    = 0.1f,
            side   = DtNavMeshBuilder.ClassifyOffMeshPoint(projectedEnd, startTile.data.header.bmin, startTile.data.header.bmax),
            userId = 0
        };
        offMeshConnection.pos[0] = projectedStart;
        offMeshConnection.pos[1] = projectedEnd;
        startTile.data.offMeshCons[^1] = offMeshConnection;

        var offMeshRef = EncodePolyId(DecodePolyIdSalt(startRef), startTile.index, polyIndex);
        var idx  = AllocLink(startTile);
        var link = startTile.links[idx];
        link.refs           = startRef;
        link.edge           = 0;
        link.side           = 0xff;
        link.bmin           = link.bmax = 0;
        link.next           = offMeshPoly.firstLink;
        offMeshPoly.firstLink = idx;

        idx                 = AllocLink(startTile);
        link                = startTile.links[idx];
        link.refs           = offMeshRef;
        link.edge           = 0xff;
        link.side           = 0xff;
        link.bmin           = link.bmax = 0;
        link.next           = startPoly.firstLink;
        startPoly.firstLink = idx;

        idx                 = AllocLink(startTile);
        link                = startTile.links[idx];
        link.refs           = endRef;
        link.edge           = 1;
        link.side           = (byte)offMeshConnection.side;
        link.bmin           = link.bmax = 0;
        link.next           = offMeshPoly.firstLink;
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

// attribute that defines which territories particular customization applies to
[AttributeUsage(AttributeTargets.Class, AllowMultiple = true, Inherited = false)]
public class CustomizationTerritoryAttribute : Attribute
{
    public uint TerritoryID;

    public CustomizationTerritoryAttribute(uint territoryID) => TerritoryID = territoryID;
}

// registry containing all customizations
public static class NavmeshCustomizationRegistry
{
    public static NavmeshCustomization                   Default      = new();
    public static Dictionary<uint, NavmeshCustomization> PerTerritory = new();

    static NavmeshCustomizationRegistry()
    {
        var baseType = typeof(NavmeshCustomization);

        foreach (var t in Assembly.GetExecutingAssembly().DefinedTypes.Where(t => t.IsSubclassOf(baseType)))
        {
            var instance = Activator.CreateInstance(t) as NavmeshCustomization;

            if (instance == null)
            {
                Service.Log.Error($"Failed to create instance of customization class {t}");
                continue;
            }

            foreach (var attr in t.GetCustomAttributes<CustomizationTerritoryAttribute>()) PerTerritory.Add(attr.TerritoryID, instance);
        }
    }

    public static NavmeshCustomization ForTerritory(uint id) => PerTerritory.GetValueOrDefault(id, Default);
}

public static class SceneExtensions
{
    private static void InsertAxisAlignedCollider
    (
        this SceneExtractor           scene,
        string                        meshKey,
        Vector3                       scale,
        Vector3                       worldTransform,
        SceneExtractor.PrimitiveFlags forceSetFlags   = default,
        SceneExtractor.PrimitiveFlags forceClearFlags = default
    )
    {
        var transform = Matrix4x3.Identity;
        transform.M11  = scale.X;
        transform.M22  = scale.Y;
        transform.M33  = scale.Z;
        transform.Row3 = worldTransform;
        var aabb         = new AABB { Min = transform.Row3 - scale, Max = transform.Row3 + scale };
        var existingMesh = scene.Meshes[meshKey];
        var id           = 0xbaadf00d00000001ul + (uint)existingMesh.Instances.Count;
        existingMesh.Instances.Insert(0, new(id, transform, aabb, 0, forceSetFlags, forceClearFlags));
    }

    public static void InsertAABoxCollider
    (
        this SceneExtractor           scene,
        Vector3                       scale,
        Vector3                       worldTransform,
        SceneExtractor.PrimitiveFlags forceSetFlags   = default,
        SceneExtractor.PrimitiveFlags forceClearFlags = default
    ) =>
        scene.InsertAxisAlignedCollider("<box>", scale, worldTransform, forceSetFlags, forceClearFlags);

    public static void InsertAABoxCollider
        (this SceneExtractor scene, AABB bounds, SceneExtractor.PrimitiveFlags forceSetFlags = default, SceneExtractor.PrimitiveFlags forceClearFlags = default)
    {
        var scale     = (bounds.Max - bounds.Min) * 0.5f;
        var transform = (bounds.Min + bounds.Max) * 0.5f;
        scene.InsertAABoxCollider(scale, transform, forceSetFlags, forceClearFlags);
    }

    public static void InsertCylinderCollider
    (
        this SceneExtractor           scene,
        Vector3                       scale,
        Vector3                       worldTransform,
        SceneExtractor.PrimitiveFlags forceSetFlags   = default,
        SceneExtractor.PrimitiveFlags forceClearFlags = default
    ) =>
        scene.InsertAxisAlignedCollider("<cylinder>", scale, worldTransform, forceSetFlags, forceClearFlags);

    public static void InsertCylinderCollider
        (this SceneExtractor scene, AABB bounds, SceneExtractor.PrimitiveFlags forceSetFlags = default, SceneExtractor.PrimitiveFlags forceClearFlags = default)
    {
        var scale     = (bounds.Max - bounds.Min) * 0.5f;
        var transform = (bounds.Min + bounds.Max) * 0.5f;
        scene.InsertCylinderCollider(scale, transform, forceSetFlags, forceClearFlags);
    }
}

public static class CreateParamsExtensions
{
    public static void AddOffMeshConnection
    (
        this DtNavMeshCreateParams config,
        Vector3                    ptA,
        Vector3                    ptB,
        float                      radius        = 0.5f,
        bool                       bidirectional = false,
        int                        userID        = 0
    ) =>
        config.AddOffMeshConnection
            (ptA, ptB, radius, bidirectional, userID, NavmeshArea.ManualOffMesh, NavmeshPolyFlags.ManualOffMesh, NavmeshOffMeshKind.ManualOffMesh);

    public static void AddOffMeshConnection
    (
        this DtNavMeshCreateParams config,
        Vector3                    ptA,
        Vector3                    ptB,
        float                      radius,
        bool                       bidirectional,
        int                        userID,
        NavmeshArea                area,
        NavmeshPolyFlags           flags,
        NavmeshOffMeshKind         kind
    )
    {
        bool insideTile(Vector3 p)
        {
            return p.X >= config.bmin.X && p.Y >= config.bmin.Y && p.Z >= config.bmin.Z && p.X <= config.bmax.X && p.Y <= config.bmax.Y && p.Z <= config.bmax.Z;
        }

        var aInside = insideTile(ptA);
        if (!aInside)
            return;

        Extend(ref config.offMeshConVerts, 6);
        config.offMeshConVerts[^6] = ptA.X;
        config.offMeshConVerts[^5] = ptA.Y;
        config.offMeshConVerts[^4] = ptA.Z;
        config.offMeshConVerts[^3] = ptB.X;
        config.offMeshConVerts[^2] = ptB.Y;
        config.offMeshConVerts[^1] = ptB.Z;

        Extend(ref config.offMeshConDir, 1);
        config.offMeshConDir[^1] = bidirectional ? DT_OFFMESH_CON_BIDIR : 0;

        Extend(ref config.offMeshConFlags, 1);
        config.offMeshConFlags[^1] = (int)flags;

        config.offMeshConCount++;

        Extend(ref config.offMeshConRad, 1);
        config.offMeshConRad[^1] = radius;

        Extend(ref config.offMeshConAreas, 1);
        config.offMeshConAreas[^1] = (int)area;

        Extend(ref config.offMeshConUserID, 1);
        config.offMeshConUserID[^1] = userID;

        Service.Log.Debug($"[NavmeshBuilder] 已加入离网连接: 类型 = {kind}, 区域 = {area}, 标记 = {flags}, 起点 = {ptA:f3}, 终点 = {ptB:f3}, 双向 = {bidirectional}");
    }

    private static void Extend<T>([NotNull] ref T[]? arr, int add)
    {
        arr ??= [];
        Array.Resize(ref arr, arr.Length + add);
    }
}
