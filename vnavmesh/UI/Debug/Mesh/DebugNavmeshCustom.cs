using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility.Raii;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Recast;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.Navigation.Customizations;
using vnavmesh.Navigation.Mesh.Build;
using vnavmesh.Navigation.Mesh.Query;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;
using vnavmesh.Navigation.Volume.Map;
using vnavmesh.Navigation.Volume.Pathfinding;
using vnavmesh.Shared.Models;
using vnavmesh.Shared.Utilities;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Debug.Recast;

namespace vnavmesh.UI.Debug.Mesh;

internal class DebugNavmeshCustom : IDisposable
{
    private record struct HeightfieldComparison
    (
        float DurationOld,
        float DurationNew,
        bool  Identical
    );

    public class Customization : NavmeshCustomization
    {
        public bool Flyable;
        public bool LoadExisting = true;

        public override int Version => 1;

        public override bool IsFlyingSupported(SceneDefinition definition) => Flyable;

        private static NavmeshCustomization? Existing =>
            NavmeshCustomizationRegistry.ForTerritory(Service.ClientState.TerritoryType) is { } t && t.Version > 0 ? t : null;

        public override void CustomizeScene(SceneExtractor scene)
        {
            if (LoadExisting)
                Existing?.CustomizeScene(scene);
        }

        public override void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile)
        {
            if (LoadExisting)
                Existing?.CustomizeBuildProfile(definition, profile);
        }

        public override void CustomizeSettings(DtNavMeshCreateParams config)
        {
            if (LoadExisting)
                Existing?.CustomizeSettings(config);
        }

        public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
        {
            if (LoadExisting)
                Existing?.CustomizeMesh(mesh, festivalLayers);
        }
    }

    // async navmesh builder
    public class AsyncBuilder
    (
        NavmeshManager manager,
        Config         config
    ) : IDisposable
    {
        public enum State
        {
            NotBuilt,
            InProgress,
            Failed,
            Ready
        }

        public class IntermediateData
        {
            public int                 NumTilesX;
            public int                 NumTilesZ;
            public RcBuilderResult?[,] Tiles;

            public IntermediateData(int numTilesX, int numTilesZ)
            {
                NumTilesX = numTilesX;
                NumTilesZ = numTilesZ;
                Tiles     = new RcBuilderResult?[numTilesX, numTilesZ];
            }
        }

        // results - should not be accessed while task is running
        private SceneDefinition?  _scene;
        private NavmeshBuilder?   _builder;
        private NavmeshQuery?     _query;
        private IntermediateData? _intermediates;
        private Task?             _task;
        private NavmeshManager    _manager = manager;
        private Config            _config  = config;

        public State CurrentState => _task == null ? State.NotBuilt : !_task.IsCompleted ? State.InProgress : _task.IsFaulted ? State.Failed : State.Ready;
        public SceneDefinition? Scene => _task != null && _task.IsCompletedSuccessfully ? _scene : null;
        public SceneExtractor? Extractor => _task != null && _task.IsCompletedSuccessfully ? _builder?.Scene : null;
        public IntermediateData? Intermediates => _task != null && _task.IsCompletedSuccessfully ? _intermediates : null;
        public NavmeshQuery? Query => _task != null && _task.IsCompletedSuccessfully ? _query : null;
        public DtNavMesh? Navmesh => _task != null && _task.IsCompletedSuccessfully ? _builder?.Navmesh.Mesh : null;
        public DtNavMeshQuery? MeshQuery => _task != null && _task.IsCompletedSuccessfully ? _query?.MeshQuery : null;
        public VoxelMap? Volume => _task != null && _task.IsCompletedSuccessfully ? _builder?.Navmesh.Volume : null;
        public VoxelPathfind? VolumeQuery => _task != null && _task.IsCompletedSuccessfully ? _query?.VolumeQuery : null;
        public NavmeshBuilder.BuildTelemetrySummary? BuildTelemetry => _task != null && _task.IsCompletedSuccessfully ? _builder?.LastBuildTelemetry : null;

        public void Dispose() =>
            Clear();

        public void Rebuild(Customization settings, bool includeTiles)
        {
            Clear();
            Service.Log.Debug("[navmesh] extract from scene");
            _scene = new();
            _scene.FillFromActiveLayout();
            Service.Log.Debug("[navmesh] schedule async build");
            _task = Task.Run(() => BuildNavmesh(_scene, settings, includeTiles));
        }

        public void Clear()
        {
            if (_task != null)
            {
                if (!_task.IsCompleted)
                    _task.Wait();
                _task.Dispose();
                _task = null;
            }

            _scene         = null;
            _builder       = null;
            _query         = null;
            _intermediates = null;
            //GC.Collect();
        }

        private void BuildNavmesh(SceneDefinition scene, NavmeshCustomization customization, bool includeTiles)
        {
            try
            {
                var timer = StopWatchTimer.Create();
                _builder = new(scene, customization, _config);

                List<((int X, int Z), Task<NavmeshBuilder.Intermediates> Task)> _tiles = [];

                // create tile data and add to navmesh
                _intermediates = new(_builder.NumTilesX, _builder.NumTilesZ);

                if (includeTiles)
                {
                    foreach (var result in _builder.BuildTiles())
                        _intermediates.Tiles[result.TileX, result.TileZ] = result;

                    //int x = 9, z = 15;
                    //_intermediates.Tiles[x, z] = _builder.BuildTile(x, z);
                    Service.Log.Debug("running customization code");
                    customization.CustomizeMesh(_builder.Navmesh, [.. scene.FestivalLayers]);
                }

                _query = new(_builder.Navmesh, _config);
                Service.Log.Debug($"navmesh build time: {timer.Value().TotalMilliseconds}ms");
                _manager.ReplaceMesh(_builder.Navmesh);
            }
            catch (Exception ex)
            {
                Service.Log.Error($"Error building navmesh: {ex}");
                throw;
            }
        }
    }

    // TODO: should each debug drawer handle tiled geometry itself?
    private class PerTile : IDisposable
    {
        public DebugSolidHeightfield?   DrawSolidHeightfield;
        public DebugCompactHeightfield? DrawCompactHeightfield;
        public DebugContourSet?         DrawContourSet;
        public DebugPolyMesh?           DrawPolyMesh;
        public DebugPolyMeshDetail?     DrawPolyMeshDetail;
        public HeightfieldComparison?   HFC;

        public void Dispose()
        {
            DrawSolidHeightfield?.Dispose();
            DrawCompactHeightfield?.Dispose();
            DrawContourSet?.Dispose();
            DrawPolyMesh?.Dispose();
            DrawPolyMeshDetail?.Dispose();
        }
    }

    private Customization            _settings = new();
    private AsyncBuilder             _navmesh;
    private UITree                   _tree = new();
    private DebugDrawer              _dd;
    private DebugGameCollision       _coll;
    private DebugExtractedCollision? _drawExtracted;
    private HeightfieldComparison?   _globalHFC;
    private PerTile[,]?              _debugTiles;

    private Vector3 _dest;
    private Vector3 _seamInspectPoint;

    private string _configDirectory;

    public DebugNavmeshCustom(Config config, DebugDrawer dd, DebugGameCollision coll, NavmeshManager manager, string configDir)
    {
        _dd              = dd;
        _coll            = coll;
        _navmesh         = new(manager, config);
        _configDirectory = configDir;
    }

    public void Dispose()
    {
        _drawExtracted?.Dispose();

        if (_debugTiles != null)
        {
            foreach (var t in _debugTiles)
                t?.Dispose();
        }
    }

    public void Draw()
    {
        using (var nsettings = _tree.Node("Navmesh properties"))
        {
            if (nsettings.Opened)
            {
                ImGui.Checkbox("Support flying",                        ref _settings.Flyable);
                ImGui.Checkbox("Load existing territory customization", ref _settings.LoadExisting);
                _settings.Settings.Draw();
            }
        }

        using (var d = ImRaii.Disabled(_navmesh.CurrentState == AsyncBuilder.State.InProgress))
        {
            if (ImGui.Button("Rebuild navmesh"))
            {
                Clear();
                _navmesh.Rebuild(_settings, true);
            }

            ImGui.SameLine();

            if (ImGui.Button("Rebuild scene extract only"))
            {
                Clear();
                _navmesh.Rebuild(_settings, false);
            }

            ImGui.SameLine();
            ImGui.TextUnformatted($"State: {_navmesh.CurrentState}");
        }

        if (_navmesh.CurrentState != AsyncBuilder.State.Ready)
            return;

        using (var nt = _tree.Node("构建诊断"))
        {
            if (nt.Opened)
                DrawBuildTelemetry();
        }

        ImGui.InputFloat("X", ref _dest.X);
        ImGui.InputFloat("Y", ref _dest.Y);
        ImGui.InputFloat("Z", ref _dest.Z);

        if (ImGui.Button("Pathfind"))
        {
            var player           = Service.ObjectTable.LocalPlayer;
            var currentPlayerPos = player?.Position ?? default;
            _navmesh.Query!.PathfindMesh(currentPlayerPos, _dest, true, true, 0, new());
        }

        var navmesh   = _navmesh.Navmesh!;
        var playerPos = Service.ObjectTable.LocalPlayer?.Position ?? default;
        navmesh.CalcTileLoc(playerPos.SystemToRecast(), out var playerTileX, out var playerTileZ);
        _tree.LeafNode($"玩家所在区块：{playerTileX}x{playerTileZ}");

        using (var ns = _tree.Node("接缝检查"))
        {
            if (ns.Opened)
                DrawSeamInspection(playerPos);
        }

        _drawExtracted ??= new(_navmesh.Scene!, _navmesh.Extractor!, _tree, _dd, _coll, _configDirectory);
        _drawExtracted.Draw();
        var intermediates = _navmesh.Intermediates;

        if (intermediates != null)
        {
            using var n = _tree.Node("Intermediates");

            if (n.Opened)
            {
                _debugTiles ??= new PerTile[intermediates.NumTilesX, intermediates.NumTilesZ];

                using (var ng = _tree.Node("Global"))
                {
                    if (ng.Opened)
                    {
                        _globalHFC ??= CompareAllHeightfields(_navmesh.Extractor!);
                        _tree.LeafNode($"Old: {_globalHFC.Value.DurationOld:f3}");
                        _tree.LeafNode($"New: {_globalHFC.Value.DurationNew:f3}");
                        _tree.LeafNode($"Match: {_globalHFC.Value.Identical}");
                    }
                }

                for (var z = 0; z < intermediates.NumTilesZ; ++z)
                for (var x = 0; x < intermediates.NumTilesX; ++x)
                {
                    var inter = intermediates.Tiles[x, z];
                    if (inter == null)
                        continue;
                    DrawIntermediateTile(x, z, inter, $"区块 {x}x{z}");
                }
            }
        }

        using var dt = _tree.Node("Detour navmesh");
        if (dt.Opened)
            _tree.LeafNode("Loaded mesh replaced with custom build, check Navmesh Manager tab");
    }

    private void DrawSeamInspection(Vector3 playerPos)
    {
        ImGui.InputFloat("检查点 X", ref _seamInspectPoint.X);
        ImGui.InputFloat("检查点 Y", ref _seamInspectPoint.Y);
        ImGui.InputFloat("检查点 Z", ref _seamInspectPoint.Z);
        if (ImGui.Button("设为玩家位置"))
            _seamInspectPoint = playerPos;
        ImGui.SameLine();
        if (ImGui.Button("设为目标点"))
            _seamInspectPoint = _dest;

        if (_navmesh.Query == null || _navmesh.Intermediates == null)
        {
            _tree.LeafNode("当前没有可用的导航网格查询或中间结果。");
            return;
        }

        var query         = _navmesh.Query;
        var intermediates = _navmesh.Intermediates;
        _debugTiles            ??= new PerTile[intermediates.NumTilesX, intermediates.NumTilesZ];
        var (tileX, tileZ)     =   query.FindMeshTile(_seamInspectPoint);
        var (tileMin, tileMax) =   query.GetMeshTileBounds(tileX, tileZ);
        var distMinX        = MathF.Abs(_seamInspectPoint.X - tileMin.X);
        var distMaxX        = MathF.Abs(tileMax.X           - _seamInspectPoint.X);
        var distMinZ        = MathF.Abs(_seamInspectPoint.Z - tileMin.Z);
        var distMaxZ        = MathF.Abs(tileMax.Z           - _seamInspectPoint.Z);
        var nearestBoundary = MathF.Min(MathF.Min(distMinX, distMaxX), MathF.Min(distMinZ, distMaxZ));

        _tree.LeafNode($"检查点：{_seamInspectPoint:f3}");
        _tree.LeafNode($"所属区块：{tileX}x{tileZ}，边界 = {tileMin:f3} - {tileMax:f3}");
        _tree.LeafNode($"距区块边界：X- = {distMinX:f3}，X+ = {distMaxX:f3}，Z- = {distMinZ:f3}，Z+ = {distMaxZ:f3}，最近 = {nearestBoundary:f3}");
        _tree.LeafNode("重点查看中心区块与相邻区块边界带里的紧凑高度场、轮廓集和多边形网格。");

        using var nn = _tree.Node("邻接区块中间结果");
        if (!nn.Opened)
            return;

        for (var z = Math.Max(0, tileZ - 1); z <= Math.Min(intermediates.NumTilesZ - 1, tileZ + 1); ++z)
        for (var x = Math.Max(0, tileX - 1); x <= Math.Min(intermediates.NumTilesX - 1, tileX + 1); ++x)
        {
            var inter = intermediates.Tiles[x, z];
            var role  = x == tileX && z == tileZ ? "中心" : "相邻";

            if (inter == null)
            {
                _tree.LeafNode($"区块 {x}x{z}（{role}）：没有中间结果，请使用“Rebuild navmesh”重新构建。");
                continue;
            }

            DrawIntermediateTile(x, z, inter, $"区块 {x}x{z}（{role}）");
        }
    }

    private void DrawIntermediateTile(int x, int z, RcBuilderResult inter, string label)
    {
        using var nt = _tree.Node(label);
        if (!nt.Opened)
            return;

        _debugTiles ??= new PerTile[_navmesh.Intermediates!.NumTilesX, _navmesh.Intermediates.NumTilesZ];
        var debug = _debugTiles[x, z] ??= new();
        debug.DrawSolidHeightfield ??= new(inter.SolidHeightfiled, _tree, _dd);
        debug.DrawSolidHeightfield.Draw();
        debug.DrawCompactHeightfield ??= new(inter.CompactHeightfield, _tree, _dd);
        debug.DrawCompactHeightfield.Draw();
        debug.DrawContourSet ??= new(inter.ContourSet, _tree, _dd);
        debug.DrawContourSet.Draw();
        debug.DrawPolyMesh ??= new(inter.Mesh, _tree, _dd);
        debug.DrawPolyMesh.Draw();

        if (inter.MeshDetail is { } det)
        {
            debug.DrawPolyMeshDetail ??= new(det, _tree, _dd);
            debug.DrawPolyMeshDetail.Draw();
        }

        using var nhfc = _tree.Node("高度场对比");
        if (!nhfc.Opened)
            return;

        debug.HFC ??= CompareHeightfields(x, z, _navmesh.Extractor!);
        _tree.LeafNode($"旧版耗时：{debug.HFC.Value.DurationOld:f3}");
        _tree.LeafNode($"新版耗时：{debug.HFC.Value.DurationNew:f3}");
        _tree.LeafNode($"结果一致：{debug.HFC.Value.Identical}");
    }

    private void Clear()
    {
        _drawExtracted?.Dispose();
        _drawExtracted = null;
        _globalHFC     = null;

        if (_debugTiles != null)
        {
            foreach (var t in _debugTiles)
                t?.Dispose();
        }

        _debugTiles = null;
        _navmesh.Clear();
    }

    private void DrawBuildTelemetry()
    {
        var telemetry = _navmesh.BuildTelemetry;
        if (telemetry == null)
        {
            _tree.LeafNode("当前没有构建统计");
            return;
        }

        _tree.LeafNode($"配置核心数：{telemetry.ConfiguredBuildMaxCores}");
        _tree.LeafNode($"可用核心数：{telemetry.MaxAvailableCores}");
        _tree.LeafNode($"实际线程数：{telemetry.ThreadCount}");
        _tree.LeafNode($"并行构建耗时：{telemetry.ParallelTicks / (double)TimeSpan.TicksPerMillisecond:f1} ms");

        foreach (var phase in telemetry.Phases)
            _tree.LeafNode($"{phase.Name}：{phase.TotalTicks / (double)TimeSpan.TicksPerMillisecond:f1} ms，占比 {phase.ShareOfPhaseTicks:P1}，最慢 {phase.SlowestTileX}x{phase.SlowestTileZ}");

        foreach (var tile in telemetry.SlowTiles)
            _tree.LeafNode($"慢瓦片 {tile.TileX}x{tile.TileZ}：{tile.TotalTicks / (double)TimeSpan.TicksPerMillisecond:f1} ms，几何 job {tile.GeometryJobCount}，地形 job {tile.TerrainJobCount}，唯一 job {tile.UniqueJobCount}，Primitive {tile.PrimitiveCount}，span 权重 {tile.EstimatedSpanWeight}，紧凑前 span {tile.PreCompactSpanCount}，Poly {tile.PolyCount}，Vert {tile.VertCount}，DetailTri {tile.DetailTriCount}");
    }

    private HeightfieldComparison CompareHeightfields(int tx, int tz, SceneExtractor scene)
    {
        var telemetry               = new RcContext();
        var boundsMin               = new Vector3(-1024);
        var boundsMax               = new RcVec3f(1024);
        var numTilesXZ              = Math.Max(1, _navmesh.Intermediates?.NumTilesX ?? _settings.Settings.GroundTileCountMax);
        var tileWidth               = (boundsMax.X - boundsMin.X) / numTilesXZ;
        var tileHeight              = (boundsMax.Z - boundsMin.Z) / numTilesXZ;
        var walkableClimbVoxels     = (int)MathF.Floor(_settings.Settings.AgentMaxClimb / _settings.Settings.CellHeight);
        var walkableRadiusVoxels    = (int)MathF.Ceiling(_settings.Settings.AgentRadius / _settings.Settings.CellSize);
        var walkableNormalThreshold = _settings.Settings.AgentMaxSlopeDeg.Degrees().Cos();
        var borderSizeVoxels        = 3 + walkableRadiusVoxels;
        var borderSizeWorld         = borderSizeVoxels * _settings.Settings.CellSize;
        var tileSizeXVoxels         = (int)MathF.Ceiling(tileWidth  / _settings.Settings.CellSize) + 2 * borderSizeVoxels;
        var tileSizeZVoxels         = (int)MathF.Ceiling(tileHeight / _settings.Settings.CellSize) + 2 * borderSizeVoxels;
        var tileBoundsMin           = new Vector3(boundsMin.X     + tx * tileWidth, boundsMin.Y, boundsMin.Z     + tz * tileHeight);
        var tileBoundsMax           = new Vector3(tileBoundsMin.X + tileWidth,      boundsMax.Y, tileBoundsMin.Z + tileHeight);
        tileBoundsMin.X -= borderSizeWorld;
        tileBoundsMin.Z -= borderSizeWorld;
        tileBoundsMax.X += borderSizeWorld;
        tileBoundsMax.Z += borderSizeWorld;

        var timerOld = StopWatchTimer.Create();
        var shfOld = new RcHeightfield
        (
            tileSizeXVoxels,
            tileSizeZVoxels,
            tileBoundsMin.SystemToRecast(),
            tileBoundsMax.SystemToRecast(),
            _settings.Settings.CellSize,
            _settings.Settings.CellHeight,
            borderSizeVoxels
        );
        var rasterizerOld = new NavmeshRasterizer(shfOld, walkableNormalThreshold, walkableClimbVoxels, 0, false, null, telemetry);
        rasterizerOld.RasterizeOld(scene, SceneExtractor.MeshType.All);
        var dur1 = (float)timerOld.Value().TotalSeconds;

        var timerNew = StopWatchTimer.Create();
        var shfNew = new RcHeightfield
        (
            tileSizeXVoxels,
            tileSizeZVoxels,
            tileBoundsMin.SystemToRecast(),
            tileBoundsMax.SystemToRecast(),
            _settings.Settings.CellSize,
            _settings.Settings.CellHeight,
            borderSizeVoxels
        );
        var rasterizerNew = new NavmeshRasterizer(shfNew, walkableNormalThreshold, walkableClimbVoxels, 0, false, null, telemetry);
        rasterizerNew.Rasterize(scene, SceneExtractor.MeshType.All, false, false);
        var dur2 = (float)timerNew.Value().TotalSeconds;

        var identical = true;
        var ispan     = 0;

        for (var z = 0; z < tileSizeZVoxels; ++z)
        for (var x = 0; x < tileSizeXVoxels; ++x)
        {
            var so = shfOld.spans[ispan];
            var sn = shfNew.spans[ispan];

            while (so != null && sn != null)
            {
                identical &= so.smin == sn.smin && so.smax == sn.smax && so.area == sn.area;
                so        =  so.next;
                sn        =  sn.next;
            }

            identical &= so == null && sn == null;
            ispan++;
        }

        return new(dur1, dur2, identical);
    }

    private HeightfieldComparison CompareAllHeightfields(SceneExtractor scene)
    {
        float dur1       = 0, dur2 = 0;
        var   identical  = true;
        var   numTilesXZ = Math.Max(1, _navmesh.Intermediates?.NumTilesX ?? _settings.Settings.GroundTileCountMax);

        for (var tz = 0; tz < numTilesXZ; ++tz)
        for (var tx = 0; tx < numTilesXZ; ++tx)
        {
            var hfc = CompareHeightfields(tx, tz, scene);
            dur1      += hfc.DurationOld;
            dur2      += hfc.DurationNew;
            identical &= hfc.Identical;
        }

        return new(dur1, dur2, identical);
    }
}
