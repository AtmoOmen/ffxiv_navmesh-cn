using System.Numerics;

namespace Navmesh;

internal readonly record struct GroundAnchor(Vector3 Position, long PolyRef, float Clearance, bool AllowShortcut);
