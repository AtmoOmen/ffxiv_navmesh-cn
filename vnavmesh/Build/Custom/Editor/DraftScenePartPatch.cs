using System.Numerics;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Editor;

public sealed class DraftScenePartPatch
{
    public bool                    Enabled = true;
    public DraftScenePartPatchKind Kind;
    public string                  MeshKey = "";
    public string                  Note    = "";
    public int                     PartIndex;
    public int                     VertexIndex;
    public int                     PrimitiveIndex;
    public Vector3                 Position;
    public int                     V1;
    public int                     V2;
    public int                     V3;
    public ulong                   Material;
    public PrimitiveFlags          Flags;
    public PrimitiveFlags          ForceSetPrimFlags;
    public PrimitiveFlags          ForceClearPrimFlags;
}
