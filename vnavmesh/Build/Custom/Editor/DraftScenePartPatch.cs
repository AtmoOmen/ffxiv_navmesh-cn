using System.Numerics;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Editor;

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
