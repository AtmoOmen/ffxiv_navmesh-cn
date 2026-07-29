using System.Numerics;
using DotRecast.Core.Numerics;

namespace vnavmesh.Common.Extensions;

public static class Vector3Extension
{
    extension
    (
        Vector3 v
    )
    {
        public Vector3 Ceiling() =>
            new(MathF.Ceiling(v.X), MathF.Ceiling(v.Y), MathF.Ceiling(v.Z));

        public Vector3 Floor() =>
            new(MathF.Floor(v.X), MathF.Floor(v.Y), MathF.Floor(v.Z));

        public RcVec3f ToRecast() => new(v.X, v.Y, v.Z);
    }

    extension
    (
        RcVec3f v
    )
    {
        public Vector3 ToSystem() => new(v.X, v.Y, v.Z);
    }
}
