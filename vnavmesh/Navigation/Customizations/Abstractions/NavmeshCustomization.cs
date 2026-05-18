using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using Lumina.Excel.Sheets;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Common.Utilities;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.Abstractions;

using static DtDetour;

public class NavmeshCustomization
{
    private static readonly NavmeshSettings DefaultSettings = new();

    #region 覆写

    public virtual int Version => 0;

    public virtual NavmeshSettings GetBuildSettings(SceneDefinition definition)
    {
        var settings = new NavmeshSettings();
        ApplyBuildSettings(definition, settings);
        return settings;
    }

    // 野外、无人岛、云冠群岛
    public virtual bool IsFlyingSupported(SceneDefinition definition) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID)?.TerritoryIntendedUse.RowId is 1 or 49 or 47;

    public virtual void CustomizeScene(SceneExtractor scene) { }

    public virtual void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile) { }

    public virtual void CustomizeBuildSettings(SceneDefinition definition, NavmeshSettings settings) { }

    public virtual void CustomizeSettings(DtNavMeshCreateParams config) { }

    public virtual void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers) { }

    #endregion

    #region 保护

    protected NavmeshSettings Settings = new();

    protected static void LinkPoints
    (
        Navmesh                      meshData,
        Vector3                      startPos,
        Vector3                      endPos,
        bool                         bidirectional = false,
        NavmeshLinkTraversalProfile? traversalProfile = null
    ) =>
        LinkPoints(meshData, startPos, endPos, NavmeshArea.Teleport, NavmeshPolyFlags.Teleport, NavmeshOffMeshKind.Teleport, bidirectional, traversalProfile);

    protected static void LinkClientPath
    (
        Navmesh                      meshData,
        Vector3                      startPos,
        Vector3                      endPos,
        bool                         bidirectional = false,
        NavmeshLinkTraversalProfile? traversalProfile = null
    ) =>
        LinkPoints(meshData, startPos, endPos, NavmeshArea.ClientPath, NavmeshPolyFlags.ClientPath, NavmeshOffMeshKind.ClientPath, bidirectional, traversalProfile);

    protected static void LinkDrop(Navmesh meshData, Vector3 edgePos, Vector3 landingHint, NavmeshLinkTraversalProfile? traversalProfile = null)
    {
        var mesh = meshData.Mesh;
        var (startRef, startTile, startPoly, projectedStart) = ResolvePointPoly(mesh, edgePos);
        var (endRef, endTile, endPoly, projectedEnd)         = ResolveDropLandingPoly(mesh, edgePos, landingHint);
        LinkResolvedPoints
        (
            meshData,
            startRef,
            startTile,
            startPoly,
            endRef,
            endTile,
            endPoly,
            projectedStart,
            projectedEnd,
            NavmeshArea.ManualOffMesh,
            NavmeshPolyFlags.ManualOffMesh,
            NavmeshOffMeshKind.ManualOffMesh,
            false,
            traversalProfile
        );
    }

    protected internal virtual void ApplyBuildSettings(SceneDefinition definition, NavmeshSettings settings)
    {
        ApplyLegacySettingsOverrides(settings);

        var profile = new NavmeshBuildProfile();
        CustomizeBuildProfile(definition, profile);
        profile.ApplyTo(settings);
        CustomizeBuildSettings(definition, settings);
    }
    
    /// <summary>
    ///     <see cref="NavmeshSettings.AgentRadius" /> = 1f <br />
    /// </summary>
    protected void ApplyAgentRadiusOneSettings() =>
        Settings.AgentRadius = 1f;

    #endregion

    private void ApplyLegacySettingsOverrides(NavmeshSettings settings)
    {
        if (Settings.CellSize != DefaultSettings.CellSize)
            settings.CellSize = Settings.CellSize;
        if (Settings.CellHeight != DefaultSettings.CellHeight)
            settings.CellHeight = Settings.CellHeight;
        if (Settings.AgentHeight != DefaultSettings.AgentHeight)
            settings.AgentHeight = Settings.AgentHeight;
        if (Settings.AgentRadius != DefaultSettings.AgentRadius)
            settings.AgentRadius = Settings.AgentRadius;
        if (Settings.AgentMaxClimb != DefaultSettings.AgentMaxClimb)
            settings.AgentMaxClimb = Settings.AgentMaxClimb;
        if (Settings.AgentMaxSlopeDeg != DefaultSettings.AgentMaxSlopeDeg)
            settings.AgentMaxSlopeDeg = Settings.AgentMaxSlopeDeg;
        if (Settings.Filtering != DefaultSettings.Filtering)
            settings.Filtering = Settings.Filtering;
        if (Settings.RegionMinSize != DefaultSettings.RegionMinSize)
            settings.RegionMinSize = Settings.RegionMinSize;
        if (Settings.RegionMergeSize != DefaultSettings.RegionMergeSize)
            settings.RegionMergeSize = Settings.RegionMergeSize;
        if (Settings.Partitioning != DefaultSettings.Partitioning)
            settings.Partitioning = Settings.Partitioning;
        if (Settings.PolyMaxEdgeLen != DefaultSettings.PolyMaxEdgeLen)
            settings.PolyMaxEdgeLen = Settings.PolyMaxEdgeLen;
        if (Settings.PolyMaxSimplificationError != DefaultSettings.PolyMaxSimplificationError)
            settings.PolyMaxSimplificationError = Settings.PolyMaxSimplificationError;
        if (Settings.PolyMaxVerts != DefaultSettings.PolyMaxVerts)
            settings.PolyMaxVerts = Settings.PolyMaxVerts;
        if (Settings.DetailSampleDist != DefaultSettings.DetailSampleDist)
            settings.DetailSampleDist = Settings.DetailSampleDist;
        if (Settings.DetailMaxSampleError != DefaultSettings.DetailMaxSampleError)
            settings.DetailMaxSampleError = Settings.DetailMaxSampleError;
        if (Settings.FastBuild != DefaultSettings.FastBuild)
            settings.FastBuild = Settings.FastBuild;
        if (Settings.GenerateEdgeClimbLinks != DefaultSettings.GenerateEdgeClimbLinks)
            settings.GenerateEdgeClimbLinks = Settings.GenerateEdgeClimbLinks;
        if (Settings.GenerateEdgeJumpLinks != DefaultSettings.GenerateEdgeJumpLinks)
            settings.GenerateEdgeJumpLinks = Settings.GenerateEdgeJumpLinks;
        if (Settings.GroundTolerance != DefaultSettings.GroundTolerance)
            settings.GroundTolerance = Settings.GroundTolerance;
        if (Settings.ClimbDownDistance != DefaultSettings.ClimbDownDistance)
            settings.ClimbDownDistance = Settings.ClimbDownDistance;
        if (Settings.ClimbDownMaxHeight != DefaultSettings.ClimbDownMaxHeight)
            settings.ClimbDownMaxHeight = Settings.ClimbDownMaxHeight;
        if (Settings.ClimbDownMinHeight != DefaultSettings.ClimbDownMinHeight)
            settings.ClimbDownMinHeight = Settings.ClimbDownMinHeight;
        if (Settings.EdgeJumpEndDistance != DefaultSettings.EdgeJumpEndDistance)
            settings.EdgeJumpEndDistance = Settings.EdgeJumpEndDistance;
        if (Settings.EdgeJumpHeight != DefaultSettings.EdgeJumpHeight)
            settings.EdgeJumpHeight = Settings.EdgeJumpHeight;
        if (Settings.EdgeJumpMaxDrop != DefaultSettings.EdgeJumpMaxDrop)
            settings.EdgeJumpMaxDrop = Settings.EdgeJumpMaxDrop;
        if (Settings.EdgeJumpMinDrop != DefaultSettings.EdgeJumpMinDrop)
            settings.EdgeJumpMinDrop = Settings.EdgeJumpMinDrop;
        if (Settings.GroundTileSize != DefaultSettings.GroundTileSize)
            settings.GroundTileSize = Settings.GroundTileSize;
        if (Settings.GroundTileCountMax != DefaultSettings.GroundTileCountMax)
            settings.GroundTileCountMax = Settings.GroundTileCountMax;
        if (!Settings.VolumeTiles.SequenceEqual(DefaultSettings.VolumeTiles))
            settings.VolumeTiles = (int[])Settings.VolumeTiles.Clone();
    }

    private static void LinkPoints
    (
        Navmesh            meshData,
        Vector3            startPos,
        Vector3            endPos,
        NavmeshArea        area,
        NavmeshPolyFlags   flags,
        NavmeshOffMeshKind kind,
        bool               bidirectional = false,
        NavmeshLinkTraversalProfile? traversalProfile = null
    )
    {
        var mesh = meshData.Mesh;
        var (startRef, startTile, startPoly, projectedStart) = ResolvePointPoly(mesh, startPos);
        var (endRef, endTile, endPoly, projectedEnd)         = ResolvePointPoly(mesh, endPos);
        LinkResolvedPoints(meshData, startRef, startTile, startPoly, endRef, endTile, endPoly, projectedStart, projectedEnd, area, flags, kind, bidirectional, traversalProfile);
    }

    private static void LinkResolvedPoints
    (
        Navmesh            meshData,
        long               startRef,
        DtMeshTile         startTile,
        DtPoly             startPoly,
        long               endRef,
        DtMeshTile         endTile,
        DtPoly             endPoly,
        RcVec3f            projectedStart,
        RcVec3f            projectedEnd,
        NavmeshArea        area,
        NavmeshPolyFlags   flags,
        NavmeshOffMeshKind kind,
        bool               bidirectional = false,
        NavmeshLinkTraversalProfile? traversalProfile = null
    )
    {
        var mesh        = meshData.Mesh;
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
            flags  = bidirectional ? DT_OFFMESH_CON_BIDIR : 0,
            side   = DtNavMeshBuilder.ClassifyOffMeshPoint(projectedEnd, startTile.data.header.bmin, startTile.data.header.bmax),
            userId = 0,
            pos =
            {
                [0] = projectedStart,
                [1] = projectedEnd
            }
        };
        startTile.data.offMeshCons[^1] = offMeshConnection;

        var offMeshRef = EncodePolyId(DecodePolyIdSalt(startRef), startTile.index, polyIndex);
        meshData.RegisterOffMeshLink
        (
            offMeshRef,
            new(projectedStart.RecastToSystem(), projectedEnd.RecastToSystem(), kind, bidirectional, 0, traversalProfile),
            includeInVisualization: true
        );
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

        if (bidirectional)
        {
            var reverseSide = offMeshConnection.side == 0xff ? (byte)0xff : (byte)DtUtils.OppositeTile(offMeshConnection.side);
            idx               = AllocLink(endTile);
            link              = endTile.links[idx];
            link.refs         = offMeshRef;
            link.edge         = 0xff;
            link.side         = reverseSide;
            link.bmin         = link.bmax = 0;
            link.next         = endPoly.firstLink;
            endPoly.firstLink = idx;
        }
    }

    private static (long PolyRef, DtMeshTile Tile, DtPoly Poly, RcVec3f ProjectedPoint) ResolveDropLandingPoly
    (
        DtNavMesh mesh,
        Vector3   edgePos,
        Vector3   landingHint
    )
    {
        var query = new DtNavMeshQuery(mesh);
        var extents = new RcVec3f
        (
            5f,
            MathF.Max(6f, MathF.Abs(edgePos.Y - landingHint.Y) + 6f),
            5f
        );

        var         bestRef       = 0L;
        DtMeshTile? bestTile      = null;
        DtPoly?     bestPoly      = null;
        var         bestProjected = default(RcVec3f);
        var         bestDistance  = float.MaxValue;
        var         bestDrop      = float.MinValue;
        var         baseY         = MathF.Max(edgePos.Y - 0.5f, landingHint.Y + 0.5f);

        for (var i = 0; i < 12; i++)
        {
            var probeY = baseY - i * 2f;
            var probe  = new Vector3(landingHint.X, probeY, landingHint.Z);
            var status = query.FindNearestPoly(probe.SystemToRecast(), extents, new DtQueryDefaultFilter(), out var polyRef, out var projectedPoint, out _);
            if (status.Failed() || polyRef == 0)
                continue;

            var drop = edgePos.Y - projectedPoint.Y;
            if (drop <= 0.25f)
                continue;

            var dx       = projectedPoint.X - landingHint.X;
            var dz       = projectedPoint.Z - landingHint.Z;
            var distance = dx * dx          + dz * dz;
            if (distance > bestDistance || distance == bestDistance && drop <= bestDrop)
                continue;

            mesh.GetTileAndPolyByRefUnsafe(polyRef, out var tile, out var poly);
            bestRef       = polyRef;
            bestTile      = tile;
            bestPoly      = poly;
            bestProjected = projectedPoint;
            bestDistance  = distance;
            bestDrop      = drop;
        }

        if (bestRef == 0)
            throw new ArgumentException($"无法为下落连接定位落点多边形: 边缘 = {edgePos:f3}, 参考落点 = {landingHint:f3}");

        return (bestRef, bestTile!, bestPoly!, bestProjected);
    }

    private static (long PolyRef, DtMeshTile Tile, DtPoly Poly, RcVec3f ProjectedPoint) ResolvePointPoly
    (
        DtNavMesh mesh,
        Vector3   pos
    )
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
