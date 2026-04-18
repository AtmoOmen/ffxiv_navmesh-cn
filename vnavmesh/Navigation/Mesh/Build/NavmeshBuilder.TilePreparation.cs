using System.Numerics;
using DotRecast.Recast;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Scene;
using vnavmesh.Navigation.Volume;

namespace vnavmesh.Navigation.Mesh.Build;

public partial class NavmeshBuilder
{
    private (TileBuildInput[] Inputs, int[] TileBuildOrder, RasterJob[] GeometryJobs, RasterJob[] TerrainJobs, int UniqueRasterJobCount, int TotalRasterJobReferences
        , long PreparedTerrainBytes, long TotalEstimatedTileWeight)
        BucketTileInputs()
    {
        var             tileCount                = NumTilesX * NumTilesZ;
        var             geometryCounts           = new int[tileCount];
        var             terrainCounts            = new int[tileCount];
        var             primitiveCounts          = new int[tileCount];
        var             spanWeights              = new int[tileCount];
        List<RasterJob> geometryJobs             = [];
        List<RasterJob> terrainJobs              = [];
        var             totalRasterJobReferences = 0;
        long            preparedTerrainBytes     = 0;

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
                    var                      terrainLike     = (mesh.MeshType & (SceneExtractor.MeshType.Terrain | SceneExtractor.MeshType.AnalyticPlane)) != 0;
                    var                      coverage        = (maxX - minX + 1) * (maxZ - minZ + 1);
                    var                      spanWeight      = EstimateSpanWeight(primitiveCount, vertexCount, terrainLike, coverage);
                    PreparedTerrainGeometry? preparedTerrain = null;

                    if (terrainLike)
                    {
                        preparedTerrain      =  PrepareTerrainGeometry(part, instance, minX, maxX, minZ, maxZ);
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

        var  result                   = new TileBuildInput[tileCount];
        var  geometryTotal            = 0;
        var  terrainTotal             = 0;
        long totalEstimatedTileWeight = 0;

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
            geometryTotal            += geometryCounts[i];
            terrainTotal             += terrainCounts[i];
            totalEstimatedTileWeight += Math.Max(spanWeights[i], 1);
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

        return (result, tileBuildOrder, geometryJobsByTile, terrainJobsByTile, geometryJobs.Count + terrainJobs.Count, totalRasterJobReferences,
                   preparedTerrainBytes, totalEstimatedTileWeight);
    }

    private static int EstimateSpanWeight(int primitiveCount, int vertexCount, bool terrainLike, int coverage)
        => primitiveCount * (terrainLike ? 12 : 4) + vertexCount * (terrainLike ? 3 : 1) + coverage * (terrainLike ? 16 : 4);

    private PreparedTerrainGeometry PrepareTerrainGeometry
        (SceneExtractor.MeshPart part, SceneExtractor.MeshInstance instance, int minTileX, int maxTileX, int minTileZ, int maxTileZ)
    {
        var prepared   = NavmeshRasterizer.PrepareTerrainGeometry(part, instance);
        var primitives = part.PrimitiveSpan;
        if (primitives.IsEmpty)
            return prepared;

        var primitiveInfos            = GC.AllocateUninitializedArray<PreparedTerrainGeometry.PrimitiveInfo>(primitives.Length);
        var walkableNormalThresholdSq = _walkableNormalThreshold * _walkableNormalThreshold;
        var includeVolume             = Navmesh.Volume != null;

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            ref readonly var primitive = ref primitives[primitiveIndex];
            var              offset1   = primitive.V1 * 3;
            var              offset2   = primitive.V2 * 3;
            var              offset3   = primitive.V3 * 3;
            var              v1x       = prepared.WorldVertexTriples[offset1];
            var              v1y       = prepared.WorldVertexTriples[offset1 + 1];
            var              v1z       = prepared.WorldVertexTriples[offset1 + 2];
            var              v2x       = prepared.WorldVertexTriples[offset2];
            var              v2y       = prepared.WorldVertexTriples[offset2 + 1];
            var              v2z       = prepared.WorldVertexTriples[offset2 + 2];
            var              v3x       = prepared.WorldVertexTriples[offset3];
            var              v3y       = prepared.WorldVertexTriples[offset3 + 1];
            var              v3z       = prepared.WorldVertexTriples[offset3 + 2];
            var              v12x      = v2x             - v1x;
            var              v12y      = v2y             - v1y;
            var              v12z      = v2z             - v1z;
            var              v13x      = v3x             - v1x;
            var              v13y      = v3y             - v1y;
            var              v13z      = v3z             - v1z;
            var              crossX    = v12y   * v13z   - v12z   * v13y;
            var              crossY    = v12z   * v13x   - v12x   * v13z;
            var              crossZ    = v12x   * v13y   - v12y   * v13x;
            var              lenSq     = crossX * crossX + crossY * crossY + crossZ * crossZ;

            if (lenSq == 0)
            {
                primitiveInfos[primitiveIndex] = new
                    (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, PreparedTerrainGeometry.PrimitiveInfo.BuildFlags(false, false, false, false));
                continue;
            }

            var flags           = primitive.Flags & ~instance.ForceClearPrimFlags | instance.ForceSetPrimFlags;
            var realSolid       = !flags.HasFlag(SceneExtractor.PrimitiveFlags.FlyThrough);
            var unwalkableSlope = crossY <= 0 || crossY * crossY < walkableNormalThresholdSq * lenSq;
            var unwalkable = flags.HasFlag
                                 (SceneExtractor.PrimitiveFlags.ForceUnwalkable) ||
                             unwalkableSlope                                     ||
                             includeVolume                                           &&
                             flags.HasFlag(SceneExtractor.PrimitiveFlags.Unlandable) &&
                             !flags.HasFlag(SceneExtractor.PrimitiveFlags.ForceWalkable);
            var projected  = crossY != 0;
            var planeGradX = projected ? -crossX / crossY : 0;
            var planeGradZ = projected ? -crossZ / crossY : 0;
            var planeBias  = projected ? v1y - planeGradX * v1x - planeGradZ * v1z : 0;
            primitiveInfos[primitiveIndex] = new
            (
                v1x,
                v1z,
                v12x,
                v12z,
                v13x,
                v13z,
                Math.Min(v1x, Math.Min(v2x, v3x)),
                Math.Max(v1x, Math.Max(v2x, v3x)),
                Math.Min(v1z, Math.Min(v2z, v3z)),
                Math.Max(v1z, Math.Max(v2z, v3z)),
                planeGradX,
                planeGradZ,
                planeBias,
                projected ? -1.0f / crossY : 0,
                unwalkable ? 0 : RcRecast.RC_WALKABLE_AREA,
                PreparedTerrainGeometry.PrimitiveInfo.BuildFlags(realSolid, crossY > 0, projected, true)
            );
        }

        prepared.PrimitiveInfos = primitiveInfos;
        var tileCountX   = maxTileX - minTileX + 1;
        var tileCountZ   = maxTileZ - minTileZ + 1;
        var bucketCount  = tileCountX * tileCountZ;
        var writeOffsets = new int[bucketCount];

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            if (!primitiveInfos[primitiveIndex].Valid)
                continue;

            GetPrimitiveTileRange(primitiveInfos[primitiveIndex], out var primitiveMinX, out var primitiveMaxX, out var primitiveMinZ, out var primitiveMaxZ);

            for (var z = primitiveMinZ; z <= primitiveMaxZ; ++z)
            {
                var rowBase = (z - minTileZ) * tileCountX;
                for (var x = primitiveMinX; x <= primitiveMaxX; ++x)
                    ++writeOffsets[rowBase + x - minTileX];
            }
        }

        var offsets = GC.AllocateUninitializedArray<int>(bucketCount + 1);
        for (var i = 0; i < bucketCount; ++i)
            offsets[i + 1] = offsets[i] + writeOffsets[i];

        var primitiveIndices = GC.AllocateUninitializedArray<int>(offsets[bucketCount]);
        Array.Copy(offsets, writeOffsets, bucketCount);

        for (var primitiveIndex = 0; primitiveIndex < primitives.Length; ++primitiveIndex)
        {
            if (!primitiveInfos[primitiveIndex].Valid)
                continue;

            GetPrimitiveTileRange(primitiveInfos[primitiveIndex], out var primitiveMinX, out var primitiveMaxX, out var primitiveMinZ, out var primitiveMaxZ);

            for (var z = primitiveMinZ; z <= primitiveMaxZ; ++z)
            {
                var rowBase = (z - minTileZ) * tileCountX;

                for (var x = primitiveMinX; x <= primitiveMaxX; ++x)
                {
                    var cellIndex = rowBase + x - minTileX;
                    primitiveIndices[writeOffsets[cellIndex]++] = primitiveIndex;
                }
            }
        }

        prepared.TileMinX             = minTileX;
        prepared.TileMinZ             = minTileZ;
        prepared.TileCountX           = tileCountX;
        prepared.TileCountZ           = tileCountZ;
        prepared.TilePrimitiveOffsets = offsets;
        prepared.TilePrimitiveIndices = primitiveIndices;
        return prepared;
    }

    private void GetTileRange(AABB bounds, out int minX, out int maxX, out int minZ, out int maxZ) =>
        GetTileRange(bounds.Min.X, bounds.Max.X, bounds.Min.Z, bounds.Max.Z, out minX, out maxX, out minZ, out maxZ);

    private void GetTileRange(Matrix4x3 worldTransform, AABB localBounds, out int minX, out int maxX, out int minZ, out int maxZ) =>
        GetTileRange(TransformBounds(worldTransform, localBounds), out minX, out maxX, out minZ, out maxZ);

    private void GetPrimitiveTileRange(PreparedTerrainGeometry.PrimitiveInfo primitiveInfo, out int minX, out int maxX, out int minZ, out int maxZ) =>
        GetTileRange(primitiveInfo.MinX, primitiveInfo.MaxX, primitiveInfo.MinZ, primitiveInfo.MaxZ, out minX, out maxX, out minZ, out maxZ);

    private void GetTileRange(float minWorldX, float maxWorldX, float minWorldZ, float maxWorldZ, out int minX, out int maxX, out int minZ, out int maxZ)
    {
        minX = (int)MathF.Floor((minWorldX                    - _borderSizeWorld - BoundsMin.X) * _invTileWidthWorld);
        maxX = (int)MathF.Floor((maxWorldX + _borderSizeWorld - BoundsMin.X)                    * _invTileWidthWorld);
        minZ = (int)MathF.Floor((minWorldZ                    - _borderSizeWorld - BoundsMin.Z) * _invTileHeightWorld);
        maxZ = (int)MathF.Floor((maxWorldZ + _borderSizeWorld - BoundsMin.Z)                    * _invTileHeightWorld);

        minX = Math.Clamp(minX, 0, NumTilesX - 1);
        maxX = Math.Clamp(maxX, 0, NumTilesX - 1);
        minZ = Math.Clamp(minZ, 0, NumTilesZ - 1);
        maxZ = Math.Clamp(maxZ, 0, NumTilesZ - 1);
    }

    private static AABB TransformBounds(Matrix4x3 worldTransform, AABB localBounds)
    {
        var localCenter = (localBounds.Min + localBounds.Max) * 0.5f;
        var localExtent = (localBounds.Max - localBounds.Min) * 0.5f;
        var axisX       = worldTransform.Row0;
        var axisY       = worldTransform.Row1;
        var axisZ       = worldTransform.Row2;
        var center      = axisX      * localCenter.X + axisY      * localCenter.Y + axisZ      * localCenter.Z + worldTransform.Row3;
        var extent      = Abs(axisX) * localExtent.X + Abs(axisY) * localExtent.Y + Abs(axisZ) * localExtent.Z;
        return new() { Min = center                  - extent, Max = center       + extent };
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
