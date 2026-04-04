using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
    private (TileBuildInput[] Inputs, int[] TileBuildOrder, (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] GeometryInstances, (
        SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] TerrainInstances) BucketTileInputs()
    {
        var tileCount      = NumTilesX * NumTilesZ;
        var geometryCounts = new int[tileCount];
        var terrainCounts  = new int[tileCount];

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                GetTileRange(instance.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
                var isGeometry =
                    (mesh.MeshType & (SceneExtractor.MeshType.FileMesh | SceneExtractor.MeshType.CylinderMesh | SceneExtractor.MeshType.AnalyticShape)) != 0;
                var isTerrain = (mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0;

                for (var z = minZ; z <= maxZ; ++z)
                {
                    var rowBase = z * NumTilesX;

                    for (var x = minX; x <= maxX; ++x)
                    {
                        var index = rowBase + x;
                        if (isGeometry)
                            ++geometryCounts[index];
                        else if (isTerrain)
                            ++terrainCounts[index];
                    }
                }
            }
        }

        var result        = new TileBuildInput[tileCount];
        var geometryTotal = 0;
        var terrainTotal  = 0;

        for (var i = 0; i < tileCount; ++i)
        {
            result[i] = new()
            {
                GeometryStart = geometryTotal,
                GeometryCount = geometryCounts[i],
                TerrainStart  = terrainTotal,
                TerrainCount  = terrainCounts[i]
            };
            geometryTotal += geometryCounts[i];
            terrainTotal  += terrainCounts[i];
        }

        var tileBuildOrder = new int[tileCount];
        for (var i = 0; i < tileCount; ++i)
            tileBuildOrder[i] = i;
        Array.Sort
        (
            tileBuildOrder,
            (lhs, rhs) =>
            {
                var leftWeight  = terrainCounts[lhs] * 32 + geometryCounts[lhs] * 4;
                var rightWeight = terrainCounts[rhs] * 32 + geometryCounts[rhs] * 4;
                var compare     = rightWeight.CompareTo(leftWeight);
                return compare != 0 ? compare : lhs.CompareTo(rhs);
            }
        );

        var geometryInstances = new (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[geometryTotal];
        var terrainInstances  = new (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[terrainTotal];
        var geometryOffsets   = new int[tileCount];
        var terrainOffsets    = new int[tileCount];

        for (var i = 0; i < tileCount; ++i)
        {
            geometryOffsets[i] = result[i].GeometryStart;
            terrainOffsets[i]  = result[i].TerrainStart;
        }

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                GetTileRange(instance.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
                var isGeometry =
                    (mesh.MeshType & (SceneExtractor.MeshType.FileMesh | SceneExtractor.MeshType.CylinderMesh | SceneExtractor.MeshType.AnalyticShape)) != 0;
                var isTerrain = (mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0;

                for (var z = minZ; z <= maxZ; ++z)
                {
                    var rowBase = z * NumTilesX;

                    for (var x = minX; x <= maxX; ++x)
                    {
                        var index = rowBase + x;
                        if (isGeometry)
                            geometryInstances[geometryOffsets[index]++] = (mesh, instance);
                        else if (isTerrain)
                            terrainInstances[terrainOffsets[index]++] = (mesh, instance);
                    }
                }
            }
        }

        return (result, tileBuildOrder, geometryInstances, terrainInstances);
    }

    private void GetTileRange(AABB bounds, out int minX, out int maxX, out int minZ, out int maxZ)
    {
        minX = (int)MathF.Floor((bounds.Min.X                    - _borderSizeWorld - BoundsMin.X) * _invTileWidthWorld);
        maxX = (int)MathF.Floor((bounds.Max.X + _borderSizeWorld - BoundsMin.X)                    * _invTileWidthWorld);
        minZ = (int)MathF.Floor((bounds.Min.Z                    - _borderSizeWorld - BoundsMin.Z) * _invTileHeightWorld);
        maxZ = (int)MathF.Floor((bounds.Max.Z + _borderSizeWorld - BoundsMin.Z)                    * _invTileHeightWorld);

        minX = Math.Clamp(minX, 0, NumTilesX - 1);
        maxX = Math.Clamp(maxX, 0, NumTilesX - 1);
        minZ = Math.Clamp(minZ, 0, NumTilesZ - 1);
        maxZ = Math.Clamp(maxZ, 0, NumTilesZ - 1);
    }

    private void EnsureVolumeScratch(BuildThreadScratch scratch)
    {
        if (Navmesh.Volume == null)
            return;

        scratch.VolumeRoot ??= new Voxelizer(_voxelizerNumX, _voxelizerNumY, _voxelizerNumZ);
    }

    private int ResolveThreadCount()
    {
        var maxThreads    = Environment.ProcessorCount;
        var wantedThreads = _config.BuildMaxCores;
        var threadCount   = wantedThreads <= 0 ? maxThreads + wantedThreads : wantedThreads;
        return Math.Clamp(threadCount, 1, maxThreads);
    }
}
