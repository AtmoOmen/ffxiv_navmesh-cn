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
    public Vector3                         Start;
    public Vector3                         End;
    public float                           Radius = 0.5f;
    public float                           RotationDegrees;
    public bool                            DoubleSided = true;
    public string                          MeshKeyContains = "";
    public SceneExtractor.PrimitiveFlags   ForceSetPrimFlags;
    public SceneExtractor.PrimitiveFlags   ForceClearPrimFlags;
}
