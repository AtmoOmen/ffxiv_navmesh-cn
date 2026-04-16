using System.Numerics;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
    private (TileBuildInput[] Inputs, int[] TileBuildOrder, RasterJob[] GeometryJobs, RasterJob[] TerrainJobs, int UniqueRasterJobCount, int TotalRasterJobReferences, long PreparedTerrainBytes)
        BucketTileInputs()
    {
        var tileCount          = NumTilesX * NumTilesZ;
        var geometryCounts     = new int[tileCount];
        var terrainCounts      = new int[tileCount];
        var primitiveCounts    = new int[tileCount];
        var spanWeights        = new int[tileCount];
        List<RasterJob> geometryJobs = [];
        List<RasterJob> terrainJobs  = [];
        var totalRasterJobReferences = 0;
        long preparedTerrainBytes    = 0;

        foreach (var mesh in Scene.Meshes.Values)
        {
            foreach (var instance in mesh.Instances)
            {
                foreach (var part in mesh.Parts)
                {
                    var worldBounds    = TransformBounds(instance.WorldTransform, part.LocalBounds);
                    var primitiveCount = part.Primitives.Count;
                    var vertexCount    = part.Vertices.Count;
                    if (primitiveCount == 0 || vertexCount == 0)
                        continue;

                    GetTileRange(worldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
                    var terrainLike = (mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0;
                    var coverage   = (maxX - minX + 1) * (maxZ - minZ + 1);
                    var spanWeight = EstimateSpanWeight(primitiveCount, vertexCount, terrainLike, coverage);
                    PreparedTerrainGeometry? preparedTerrain = null;
                    if (terrainLike)
                    {
                        preparedTerrain      = NavmeshRasterizer.PrepareTerrainGeometry(part, instance);
                        preparedTerrainBytes += preparedTerrain.MemoryBytes;
                    }

                    var job = new RasterJob
                    {
                        MeshType        = mesh.MeshType,
                        Part            = part,
                        Instance        = instance,
                        WorldBounds     = worldBounds,
                        PrimitiveCount  = primitiveCount,
                        VertexCount     = vertexCount,
                        TerrainLike     = terrainLike,
                        PreparedTerrain = preparedTerrain
                    };

                    for (var z = minZ; z <= maxZ; ++z)
                    {
                        var rowBase = z * NumTilesX;

                        for (var x = minX; x <= maxX; ++x)
                        {
                            var index = rowBase + x;
                            if (terrainLike)
                                ++terrainCounts[index];
                            else
                                ++geometryCounts[index];
                            primitiveCounts[index] += primitiveCount;
                            spanWeights[index]     += spanWeight;
                            ++totalRasterJobReferences;
                        }
                    }

                    if (terrainLike)
                        terrainJobs.Add(job);
                    else
                        geometryJobs.Add(job);
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
                GeometryJobStart    = geometryTotal,
                GeometryJobCount    = geometryCounts[i],
                TerrainJobStart     = terrainTotal,
                TerrainJobCount     = terrainCounts[i],
                PrimitiveCount      = primitiveCounts[i],
                EstimatedSpanWeight = spanWeights[i]
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
                var leftWeight  = spanWeights[lhs];
                var rightWeight = spanWeights[rhs];
                var compare     = rightWeight.CompareTo(leftWeight);
                return compare != 0 ? compare : lhs.CompareTo(rhs);
            }
        );

        var geometryJobsByTile = new RasterJob[geometryTotal];
        var terrainJobsByTile  = new RasterJob[terrainTotal];
        var geometryOffsets    = new int[tileCount];
        var terrainOffsets     = new int[tileCount];

        for (var i = 0; i < tileCount; ++i)
        {
            geometryOffsets[i] = result[i].GeometryJobStart;
            terrainOffsets[i]  = result[i].TerrainJobStart;
        }

        foreach (var job in geometryJobs)
        {
            GetTileRange(job.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
            for (var z = minZ; z <= maxZ; ++z)
            {
                var rowBase = z * NumTilesX;
                for (var x = minX; x <= maxX; ++x)
                    geometryJobsByTile[geometryOffsets[rowBase + x]++] = job;
            }
        }

        foreach (var job in terrainJobs)
        {
            GetTileRange(job.WorldBounds, out var minX, out var maxX, out var minZ, out var maxZ);
            for (var z = minZ; z <= maxZ; ++z)
            {
                var rowBase = z * NumTilesX;
                for (var x = minX; x <= maxX; ++x)
                    terrainJobsByTile[terrainOffsets[rowBase + x]++] = job;
            }
        }

        return (result, tileBuildOrder, geometryJobsByTile, terrainJobsByTile, geometryJobs.Count + terrainJobs.Count, totalRasterJobReferences, preparedTerrainBytes);
    }

    private static int EstimateSpanWeight(int primitiveCount, int vertexCount, bool terrainLike, int coverage)
        => primitiveCount * (terrainLike ? 12 : 4) + vertexCount * (terrainLike ? 3 : 1) + coverage * (terrainLike ? 16 : 4);

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
        GetTileRange(TransformBounds(worldTransform, localBounds), out minX, out maxX, out minZ, out maxZ);
    }

    private static AABB TransformBounds(Matrix4x3 worldTransform, AABB localBounds)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = axisX * localCenter.X + axisY * localCenter.Y + axisZ * localCenter.Z + worldTransform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        return new() { Min = center - extent, Max = center + extent };
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
