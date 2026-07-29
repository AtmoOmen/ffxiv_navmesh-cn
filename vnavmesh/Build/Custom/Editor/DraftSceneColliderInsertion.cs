using System.Numerics;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Editor;

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
