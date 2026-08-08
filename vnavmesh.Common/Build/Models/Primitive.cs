using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Common.Build.Models;

public record struct Primitive
(
    int            V1,
    int            V2,
    int            V3,
    PrimitiveFlags Flags,
    ulong          Material = 0
);
