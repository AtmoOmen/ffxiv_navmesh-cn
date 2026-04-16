using System.Numerics;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
    private (TileBuildInput[] Inputs, int[] TileBuildOrder, (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[] GeometryInstances, NavmeshRasterizer.PartInstance[] TerrainParts)
        BucketTileInputs()
    {
        var tileCount              = NumTilesX * NumTilesZ;
        var geometryCounts         = new int[tileCount];
        var terrainInstanceCounts  = new int[tileCount];
        var terrainPartCounts      = new int[tileCount];

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
                            ++terrainInstanceCounts[index];
                    }
                }

                if (!isTerrain)
                    continue;

                foreach (var part in mesh.Parts)
                {
                    GetTileRange(instance.WorldTransform, part.LocalBounds, out minX, out maxX, out minZ, out maxZ);

                    for (var z = minZ; z <= maxZ; ++z)
                    {
                        var rowBase = z * NumTilesX;

                        for (var x = minX; x <= maxX; ++x)
                            ++terrainPartCounts[rowBase + x];
                    }
                }
            }
        }

        var result        = new TileBuildInput[tileCount];
        var geometryTotal = 0;
        var terrainPartTotal = 0;

        for (var i = 0; i < tileCount; ++i)
        {
            result[i] = new()
            {
                GeometryStart         = geometryTotal,
                GeometryCount         = geometryCounts[i],
                TerrainPartStart      = terrainPartTotal,
                TerrainPartCount      = terrainPartCounts[i],
                TerrainInstanceCount  = terrainInstanceCounts[i]
            };
            geometryTotal    += geometryCounts[i];
            terrainPartTotal += terrainPartCounts[i];
        }

        var tileBuildOrder = new int[tileCount];
        for (var i = 0; i < tileCount; ++i)
            tileBuildOrder[i] = i;
        Array.Sort
        (
            tileBuildOrder,
            (lhs, rhs) =>
            {
                var leftWeight  = terrainPartCounts[lhs] * 16 + terrainInstanceCounts[lhs] * 8 + geometryCounts[lhs] * 4;
                var rightWeight = terrainPartCounts[rhs] * 16 + terrainInstanceCounts[rhs] * 8 + geometryCounts[rhs] * 4;
                var compare     = rightWeight.CompareTo(leftWeight);
                return compare != 0 ? compare : lhs.CompareTo(rhs);
            }
        );

        var geometryInstances = new (SceneExtractor.Mesh Mesh, SceneExtractor.MeshInstance Instance)[geometryTotal];
        var terrainParts      = new NavmeshRasterizer.PartInstance[terrainPartTotal];
        var geometryOffsets   = new int[tileCount];
        var terrainPartOffsets = new int[tileCount];

        for (var i = 0; i < tileCount; ++i)
        {
            geometryOffsets[i] = result[i].GeometryStart;
            terrainPartOffsets[i] = result[i].TerrainPartStart;
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
                    }
                }

                if (!isTerrain)
                    continue;

                foreach (var part in mesh.Parts)
                {
                    GetTileRange(instance.WorldTransform, part.LocalBounds, out minX, out maxX, out minZ, out maxZ);

                    for (var z = minZ; z <= maxZ; ++z)
                    {
                        var rowBase = z * NumTilesX;

                        for (var x = minX; x <= maxX; ++x)
                        {
                            var index = rowBase + x;
                            terrainParts[terrainPartOffsets[index]++] = new(mesh.MeshType, part, instance);
                        }
                    }
                }
            }
        }

        return (result, tileBuildOrder, geometryInstances, terrainParts);
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

    private void GetTileRange(Matrix4x3 worldTransform, AABB localBounds, out int minX, out int maxX, out int minZ, out int maxZ)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = axisX * localCenter.X + axisY * localCenter.Y + axisZ * localCenter.Z + worldTransform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        var worldBounds = new AABB { Min = center - extent, Max = center + extent };
        GetTileRange(worldBounds, out minX, out maxX, out minZ, out maxZ);
    }

    private static Vector3 Abs(Vector3 value) => new(MathF.Abs(value.X), MathF.Abs(value.Y), MathF.Abs(value.Z));

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
