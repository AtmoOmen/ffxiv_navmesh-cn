using System.Numerics;
using DotRecast.Core.Numerics;

namespace vnavmesh.Common.Utilities;

public static class Extensions
{
    extension(RcVec3f v)
    {
        public Vector3 RecastToSystem() => new(v.X, v.Y, v.Z);
    }

    extension(Vector3 v)
    {
        public Vector3 Ceiling() => new(MathF.Ceiling(v.X), MathF.Ceiling(v.Y), MathF.Ceiling(v.Z));

        public Vector3 Floor() => new(MathF.Floor(v.X), MathF.Floor(v.Y), MathF.Floor(v.Z));

        public RcVec3f SystemToRecast() => new(v.X, v.Y, v.Z);
    }

    extension(Interlocked)
    {
        public static float Add(ref float location, float value)
        {
            float initial, newValue;

            do
            {
                initial  = location;
                newValue = initial + value;
            }
            while (Math.Abs(Interlocked.CompareExchange(ref location, newValue, initial) - initial) > 0.0001f);

            return newValue;
        }
    }
}
