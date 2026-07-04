using System.Numerics;
using DotRecast.Core.Numerics;
using DotRecast.Detour;

namespace vnavmesh.Common.Navigation.Mesh.Build;

public static class DtOffMeshConnectionTileClassifier
{
    public static bool ShouldStoreConnection(DtNavMeshCreateParams config, Vector3 start, Vector3 end)
    {
        if (config.offMeshConCount == 0 && (config.verts == null || config.detailVerts == null) && config.polyCount == 0)
            return IsWithinTileBounds(config, start);

        var (boundsMin, boundsMax) = ResolveTightBounds(config);
        var startClass = DtNavMeshBuilder.ClassifyOffMeshPoint(new RcVec3f(start.X, start.Y, start.Z), boundsMin, boundsMax);
        if (startClass == 0xff && (start.Y < boundsMin.Y || start.Y > boundsMax.Y))
            startClass = 0;

        _ = DtNavMeshBuilder.ClassifyOffMeshPoint(new RcVec3f(end.X, end.Y, end.Z), boundsMin, boundsMax);
        return startClass == 0xff;
    }

    public static (RcVec3f Min, RcVec3f Max) ResolveTightBounds(DtNavMeshCreateParams config)
    {
        var min = config.bmin;
        var max = config.bmax;

        if (config.detailVerts != null && config.detailVertsCount > 0)
        {
            var tightMin = float.MaxValue;
            var tightMax = -float.MaxValue;

            for (var i = 0; i < config.detailVertsCount; ++i)
            {
                var y = config.detailVerts[i * 3 + 1];
                tightMin = MathF.Min(tightMin, y);
                tightMax = MathF.Max(tightMax, y);
            }

            min.Y = tightMin;
            max.Y = tightMax;
            return (min, max);
        }

        if (config.verts != null && config.vertCount > 0)
        {
            var tightMin = float.MaxValue;
            var tightMax = -float.MaxValue;

            for (var i = 0; i < config.vertCount; ++i)
            {
                var y = config.bmin.Y + config.verts[i * 3 + 1] * config.ch;
                tightMin = MathF.Min(tightMin, y);
                tightMax = MathF.Max(tightMax, y);
            }

            min.Y = tightMin;
            max.Y = tightMax;
        }

        return (min, max);
    }

    private static bool IsWithinTileBounds(DtNavMeshCreateParams config, Vector3 point) =>
        point.X >= config.bmin.X &&
        point.X <= config.bmax.X &&
        point.Z >= config.bmin.Z &&
        point.Z <= config.bmax.Z;
}
