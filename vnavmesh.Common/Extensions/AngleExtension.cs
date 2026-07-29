using vnavmesh.Common.Models;

namespace vnavmesh.Common.Extensions;

public static class AngleExtension
{
    extension
    (
        float radians
    )
    {
        public Angle Radians
            () => new(radians);

        public Angle Degrees
            () => new(radians * Angle.DEG_TO_RAD);
    }

    extension
    (
        int degrees
    )
    {
        public Angle Degrees
            () => new(degrees * Angle.DEG_TO_RAD);
    }
}
