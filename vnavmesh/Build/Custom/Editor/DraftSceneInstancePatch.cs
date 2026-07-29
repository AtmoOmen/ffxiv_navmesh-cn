using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Editor;

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
