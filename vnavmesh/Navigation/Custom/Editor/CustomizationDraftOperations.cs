using System.Numerics;
using DotRecast.Recast;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Editor;

public enum DraftSceneInstancePatchKind
{
    ClearInstances,
    RemoveInstance,
    Transform,
    SetFlags
}

public enum DraftScenePartPatchKind
{
    Vertex,
    PrimitiveFlags,
    PrimitiveEdit
}

public enum DraftSceneColliderInsertionKind
{
    Aabb,
    Cylinder
}

public enum DraftMeshLinkKind
{
    Points     = 0,
    ClientPath = 2,
    Shortcut   = 3
}

public sealed class DraftSceneMeshRemoval
{
    public bool   Enabled = true;
    public string MeshKey = "";
    public string Note    = "";
}

public sealed class DraftSceneInstancePatch
{
    public bool                          Enabled = true;
    public DraftSceneInstancePatchKind   Kind;
    public string                        MeshKey = "";
    public string                        Note    = "";
    public ulong                         InstanceId;
    public int                           InstanceIndex  = -1;
    public DraftMatrix4x3                WorldTransform = DraftMatrix4x3.Identity;
    public SceneExtractor.PrimitiveFlags ForceSetPrimFlags;
    public SceneExtractor.PrimitiveFlags ForceClearPrimFlags;
}

public sealed class DraftScenePartPatch
{
    public bool                          Enabled = true;
    public DraftScenePartPatchKind       Kind;
    public string                        MeshKey = "";
    public string                        Note    = "";
    public int                           PartIndex;
    public int                           VertexIndex;
    public int                           PrimitiveIndex;
    public Vector3                       Position;
    public int                           V1;
    public int                           V2;
    public int                           V3;
    public ulong                         Material;
    public SceneExtractor.PrimitiveFlags Flags;
    public SceneExtractor.PrimitiveFlags ForceSetPrimFlags;
    public SceneExtractor.PrimitiveFlags ForceClearPrimFlags;
}

public sealed class DraftSceneColliderInsertion
{
    public bool                            Enabled = true;
    public DraftSceneColliderInsertionKind Kind;
    public string                          Note = "";
    public Vector3                         Min;
    public Vector3                         Max;
    public SceneExtractor.PrimitiveFlags   ForceSetPrimFlags;
    public SceneExtractor.PrimitiveFlags   ForceClearPrimFlags;
}

public sealed class DraftMeshLinkPatch
{
    public bool                         Enabled = true;
    public DraftMeshLinkKind            Kind;
    public string                       Note = "";
    public Vector3                      Start;
    public Vector3                      End;
    public bool                         Bidirectional;
    public NavmeshLinkTraversalProfile? TraversalProfile;
}

public sealed class DraftOffMeshConnectionPatch
{
    public bool                         Enabled = true;
    public string                       Note    = "";
    public Vector3                      Start;
    public Vector3                      End;
    public float                        Radius = 0.5f;
    public bool                         Bidirectional;
    public int                          UserId;
    public NavmeshArea                  Area  = NavmeshArea.ManualOffMesh;
    public NavmeshPolyFlags             Flags = NavmeshPolyFlags.ManualOffMesh;
    public NavmeshOffMeshKind           Kind  = NavmeshOffMeshKind.ManualOffMesh;
    public NavmeshLinkTraversalProfile? TraversalProfile;
}

public sealed class DraftBuildProfileOverrides
{
    public RcPartition? PartitioningOverride;
    public float?       CellSizeOverride;
    public float?       CellHeightOverride;
    public float?       RegionMinSizeOverride;
    public float?       RegionMergeSizeOverride;
    public float?       PolyMaxEdgeLenOverride;
    public float?       PolyMaxSimplificationErrorOverride;
    public float?       AgentRadiusOverride;
    public int[]?       VolumeTilesOverride;
    public float?       DetailSampleDistOverride;
    public bool?        GenerateEdgeClimbLinksOverride;
    public bool?        GenerateEdgeJumpLinksOverride;
}

public sealed class DraftBuildSettingsOverrides
{
    public float?         CellSize;
    public float?         CellHeight;
    public float?         AgentHeight;
    public float?         AgentRadius;
    public float?         AgentMaxClimb;
    public float?         AgentMaxSlopeDeg;
    public NavmeshFilter? Filtering;
    public float?         RegionMinSize;
    public float?         RegionMergeSize;
    public RcPartition?   Partitioning;
    public float?         PolyMaxEdgeLen;
    public float?         PolyMaxSimplificationError;
    public int?           PolyMaxVerts;
    public float?         DetailSampleDist;
    public float?         DetailMaxSampleError;
    public bool?          FastBuild;
    public bool?          GenerateEdgeClimbLinks;
    public bool?          GenerateEdgeJumpLinks;
    public float?         GroundTolerance;
    public float?         ClimbDownDistance;
    public float?         ClimbDownMaxHeight;
    public float?         ClimbDownMinHeight;
    public float?         EdgeJumpEndDistance;
    public float?         EdgeJumpHeight;
    public float?         EdgeJumpMaxDrop;
    public float?         EdgeJumpMinDrop;
    public float?         GroundTileSize;
    public int?           GroundTileCountMax;
    public int[]?         VolumeTiles;
}
