using System.Numerics;
using vnavmesh.Common.Build.Enums;

namespace vnavmesh.Build.Custom.Editor;

public sealed class DraftSceneInstancePatch
{
    public bool                        Enabled = true;
    public DraftSceneInstancePatchKind Kind;
    public string                      MeshKey = "";
    public string                      Note    = "";
    public ulong                       InstanceId;
    public int                         InstanceIndex  = -1;
    public DraftMatrix4x3              WorldTransform = DraftMatrix4x3.Identity;
    public ulong                       Material;
    public int                         Count = 1;
    public Vector3                     Offset;
    public PrimitiveFlags              ForceSetPrimFlags;
    public PrimitiveFlags              ForceClearPrimFlags;
}
