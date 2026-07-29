using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using Dalamud.Bindings.ImGui;
using Dalamud.Hooking;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Client.System.Framework;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Models;
using vnavmesh.Internal;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;
using vnavmesh.UI.Debug.Layout;
using vnavmesh.UI.Rendering;
using AABB = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB;
using Matrix4x3 = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.Matrix4x3;
using Matrix4x4 = System.Numerics.Matrix4x4;
using Vector3 = System.Numerics.Vector3;
using Vector4 = System.Numerics.Vector4;

namespace vnavmesh.UI.Debug.Collision;

// FIXME: fields moved +8 bytes in 7.4
[StructLayout(LayoutKind.Explicit, Size = 0x1F0)]
internal unsafe struct ColliderStreamedEx
{
    [FieldOffset(0x1D0)]
    public ColliderStreamed.FileHeader* Header;

    [FieldOffset(0x1D8)]
    public ColliderStreamed.FileEntry* Entries;

    [FieldOffset(0x1E0)]
    public ColliderStreamed.Element* Elements;
}

public unsafe partial class DebugGameCollision : IDisposable
{
    private const float DefaultCollisionRenderHorizontalDistance = 50f;
    private const float DefaultCollisionRenderVerticalDistance   = 10f;

    private readonly PluginConfig _config;
    private          UITree       _tree = new();
    private          DebugDrawer  _dd;
    private          BitMask      _shownLayers = new(1);
    private          BitMask      _materialMask;
    private          BitMask      _materialId;
    private          bool         _showZeroLayer = true;
    private          bool         _showOnlyFlagRaycast;
    private          bool         _showOnlyFlagVisit;
    private          float        _renderHorizontalDistance = DefaultCollisionRenderHorizontalDistance;
    private          float        _renderVerticalDistance   = DefaultCollisionRenderVerticalDistance;

    private HashSet<nint> _streamedMeshes = new();
    private BitMask       _availableLayers;
    private BitMask       _availableMaterials;

    private EffectMesh.Data          _meshDynamicData;
    private EffectMesh.Data.Builder? _meshDynamicBuilder;

    private delegate bool RaycastDelegate
    (
        SceneWrapper*  self,
        RaycastHit*    result,
        ulong          layerMask,
        RaycastParams* param
    );

    private Hook<RaycastDelegate>? _raycastHook;

    private RaycastHit? _savedHit;

    public DebugGameCollision
    (
        PluginConfig config,
        DebugDrawer  dd
    )
    {
        _config          = config;
        _dd              = dd;
        _meshDynamicData = new(dd.RenderContext, 4 * 1024 * 1024, 4 * 1024 * 1024, 512 * 1024, true);

        foreach (var s in Framework.Instance()->BGCollisionModule->SceneManager->Scenes)
        {
            _raycastHook = Service.Hook.HookFromAddress<RaycastDelegate>((nint)s->VirtualTable->Raycast, RaycastDetour);
            break;
        }
    }

    public void Dispose()
    {
        _raycastHook?.Dispose();
        _meshDynamicBuilder?.Dispose();
        _meshDynamicData.Dispose();
    }

    public void Draw()
    {
        if (_raycastHook != null)
        {
            var hook = _raycastHook.IsEnabled;

            if (ImGui.Checkbox("记录射线检测 (Raycasts)", ref hook))
            {
                if (hook)
                    _raycastHook.Enable();
                else
                    _raycastHook.Disable();
            }
        }

        if (_savedHit != null && ImGui.Button("重置记录的射线检测命中"))
            _savedHit = null;

        var module = Framework.Instance()->BGCollisionModule;
        ImGui.TextUnformatted($"模块：{(nint)module:X}->{(nint)module->SceneManager:X} ({module->SceneManager->NumScenes} 个场景，{module->LoadInProgressCounter} 次加载)");
        ImGui.TextUnformatted($"流式加载 (Streaming)：{SphereStr(module->ForcedStreamingSphere)} / {SphereStr(module->SceneManager->StreamingSphere)}");
        //module->ForcedStreamingSphere.W = 10000;

        GatherInfo();
        DrawSettings();

        var i = 0;

        foreach (var s in module->SceneManager->Scenes)
        {
            DrawSceneColliders(s->Scene, i);
            DrawSceneQuadtree(s->Scene->Quadtree, i);
            DrawSceneRaycasts(s, i);
            ++i;
        }
    }

    public void DrawVisualizers()
    {
        if (_meshDynamicBuilder != null)
        {
            _meshDynamicBuilder.Dispose();
            _meshDynamicBuilder = null;
            _dd.EffectMesh?.Draw(_dd.RenderContext, _meshDynamicData);
        }
    }

    public void DrawPath
    (
        Span<PathSegment> pathSegments
    )
    {
        for (var i = 1; i < pathSegments.Length; i++)
        {
            var a = pathSegments[i - 1];
            var b = pathSegments[i];
            _dd.DrawWorldLine(a.Position, b.Position, 0xFF00FFAA, 2);
        }
    }

    private void DrawSceneColliders
    (
        Scene* s,
        int    index
    )
    {
        using var n = _tree.Node($"场景 {index}：{s->NumColliders} 个碰撞体，{s->NumLoading} 个正在加载，流式加载={SphereStr(s->StreamingSphere)}###scene_{index}");

        if (n.SelectedOrHovered || _config.ForceShowGameCollision)
        {
            foreach (var coll in s->Colliders)
            {
                if (FilterCollider(coll))
                    VisualizeCollider(coll, _materialId, _materialMask, false);
            }
        }

        if (n.Opened)
        {
            foreach (var coll in s->Colliders)
                DrawCollider(coll);
        }
    }

    private void DrawSceneQuadtree
    (
        Quadtree* tree,
        int       index
    )
    {
        using var n = _tree.Node
        (
            $"四叉树 (Quadtree) {index}：{tree->NumLevels} 层级 ([{tree->MinX}, {tree->MaxX}]x[{tree->MinZ}, {tree->MaxZ}]，叶节点 {tree->LeafSizeX}x{tree->LeafSizeZ})，{tree->NumNodes} 个节点###tree_{index}"
        );
        if (!n.Opened)
            return;

        for (var level = 0; level < tree->NumLevels; ++level)
        {
            var cellSizeX = (tree->MaxX - tree->MinX + 1) / (1 << level);
            var cellSizeZ = (tree->MaxZ - tree->MinZ + 1) / (1 << level);
            using var ln = _tree.Node
                ($"层级 {level}，{cellSizeX}x{cellSizeZ} 单元格 (共 {Quadtree.NumNodesAtLevel(level)} 个节点，起始于 {Quadtree.StartingNodeForLevel(level)})");
            if (!ln.Opened)
                continue;

            var nodes = tree->NodesAtLevel(level);

            for (var i = 0; i < nodes.Length; ++i)
            {
                ref var node = ref nodes[i];
                if (node.Node.NodeLink.Next == null)
                    continue;

                var coord = Quadtree.CellCoords((uint)i);
                var cellX = tree->MinX + (coord.x * cellSizeX);
                var cellZ = tree->MinZ + (coord.z * cellSizeZ);
                using var cn = _tree.Node
                    ($"[{coord.x}, {coord.z}] ([{cellX}x{cellZ}]-[{cellX + cellSizeX}x{cellZ + cellSizeZ}])###node_{level}_{i}", node.Node.NodeLink.Next == null);

                if (cn.Opened)
                {
                    foreach (var coll in node.Colliders)
                        DrawCollider(coll);
                }

                if (cn.SelectedOrHovered)
                {
                    // TODO: visualize cell bounds?
                    foreach (var coll in node.Colliders)
                        VisualizeCollider(coll, _materialId, _materialMask, false);
                }
            }
        }
    }

    private void DrawSceneRaycasts
    (
        SceneWrapper* s,
        int           index
    )
    {
        using var n = _tree.Node($"场景 {index}：射线检测");
        if (!n.Opened)
            return;

        var screenPos  = ImGui.GetMousePos() - ImGuiHelpers.MainViewport.Pos;
        var windowSize = ImGuiHelpers.MainViewport.Size;

        if (screenPos.X < 0 || screenPos.X > windowSize.X || screenPos.Y < 0 || screenPos.Y > windowSize.Y)
        {
            _tree.LeafNode("鼠标在窗口外");
            return;
        }

        var res1 = GetRaycastHit(s, index, screenPos);

        if (res1 != null)
        {
            var res = res1.Value;
            _tree.LeafNode($"射线检测：{_dd.Origin} + {res.Distance} = {res.Point}");
            var ab     = res.V2 - res.V1;
            var ac     = res.V3 - res.V1;
            var normal = Vector3.Normalize(Vector3.Cross(ab, ac));
            _tree.LeafNode($"法线：{normal} (坡度={Angle.Acos(normal.Y)})");
            _tree.LeafNode($"材质：{res.Material:X}");
            DrawCollider(res.Object);
            VisualizeCollider(res.Object, _materialId, _materialMask);
            _tree.LeafNode($"顶点：{res.V1}, {res.V2}, {res.V3}");
            _dd.DrawWorldLine(res.V1, res.V2, 0xff0000ff, 2);
            _dd.DrawWorldLine(res.V2, res.V3, 0xff0000ff, 2);
            _dd.DrawWorldLine(res.V3, res.V1, 0xff0000ff, 2);
        }
        else _tree.LeafNode("射线检测：无");
    }

    private RaycastHit? GetRaycastHit
    (
        SceneWrapper* s,
        int           index,
        Vector2       screenPos
    )
    {
        if (_savedHit != null)
            return _savedHit;

        var clipPos = new Vector3((2 * screenPos.X / _dd.ViewportSize.X) - 1, 1 - (2 * screenPos.Y / _dd.ViewportSize.Y), 1);
        Matrix4x4.Invert(_dd.ViewProj, out var invViewProj);
        var cameraPosAtPlaneP = Vector4.Transform(clipPos, invViewProj);
        var cameraPosAtPlane = new Vector3
            (cameraPosAtPlaneP.X / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Y / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Z / cameraPosAtPlaneP.W);
        var dir = Vector3.Normalize(cameraPosAtPlane - _dd.Origin);
        _tree.LeafNode($"Mouse pos: screen={screenPos}, clip={clipPos}, dir={dir}");
        float maxDist = 100000;
        var   filter  = new RaycastMaterialFilter { Mask = _materialMask.Raw, Value = _materialId.Raw };
        var   res     = new RaycastHit();
        var   sphere  = new Vector4(_dd.Origin, 1);
        var   arg     = new RaycastParams { Origin = &sphere, Direction = &dir, MaxDistance = &maxDist, MaterialFilter = &filter };

        if (s->Raycast(&res, _shownLayers.Raw, &arg))
        {
            if (_savedHit == null && ImGui.GetIO().KeyShift)
                _savedHit = res;

            return res;
        }

        return null;
    }

    private void DrawCollider
    (
        Collider* coll
    )
    {
        if (coll == null || !FilterCollider(coll))
            return;

        var raycastFlag     = (coll->VisibilityFlags & 1) != 0;
        var globalVisitFlag = (coll->VisibilityFlags & 2) != 0;
        var flagsText = raycastFlag ? globalVisitFlag ?
                                          "射线检测, 全局访问" :
                                          "射线检测" :
                        globalVisitFlag ? "全局访问" : "无";

        var type           = coll->GetColliderType();
        var layoutInstance = LayoutUtil.FindInstance(LayoutWorld.Instance()->ActiveLayout, (coll->LayoutObjectId << 32) | (coll->LayoutObjectId >> 32));
        var color = layoutInstance == null || layoutInstance->Id.Type is not InstanceType.BgPart and not InstanceType.CollisionBox ?
                        0xff00ffff :
                        0xffffffff;

        if (type == ColliderType.Mesh)
        {
            var collMesh = (ColliderMesh*)coll;
            if (_streamedMeshes.Contains((nint)coll))
                color = 0xff00ff00;
            else if (collMesh->MeshIsSimple)
                color = 0xff0000ff;
        }

        using var n = _tree.Node
        (
            $"{type} {(nint)coll:X}, layers={coll->LayerMask:X8}, layout-id={coll->LayoutObjectId:X16}, refs={coll->NumRefs}, material={coll->ObjectMaterialValue:X}/{coll->ObjectMaterialMask:X}, flags={flagsText}###{(nint)coll:X}",
            false,
            color
        );

        if (ImGui.BeginPopupContextItem($"###popup{(nint)coll:X}"))
        {
            ContextCollider(coll);
            ImGui.EndPopup();
        }

        if (n.SelectedOrHovered)
            VisualizeCollider(coll, _materialId, _materialMask, false);
        if (!n.Opened)
            return;

        _tree.LeafNode($"Raw flags: {coll->VisibilityFlags:X}");

        switch (type)
        {
            case ColliderType.Streamed:
            {
                var cast = (ColliderStreamed*)coll;
                DrawResource(cast->Resource);
                var path = cast->PathBaseString;
                _tree.LeafNode($"Path: {path}/{Encoding.UTF8.GetString(cast->PathBase[(path.Length + 1)..])}");
                _tree.LeafNode($"Streamed: [{cast->StreamedMinX:f3}x{cast->StreamedMinZ:f3}] - [{cast->StreamedMaxX:f3}x{cast->StreamedMaxZ:f3}]");
                _tree.LeafNode($"Loaded: {cast->Loaded} ({cast->NumMeshesLoading} meshes load in progress)");

                if (cast->Header != null && cast->Entries != null && cast->Elements != null)
                {
                    var headerRaw = (float*)cast->Header;
                    _tree.LeafNode
                    (
                        $"Header: meshes={cast->Header->NumMeshes}, u={headerRaw[1]:f3} {headerRaw[2]:f3} {headerRaw[3]:f3} {headerRaw[4]:f3} {headerRaw[5]:f3} {headerRaw[6]:f3} {headerRaw[7]:f3}"
                    );

                    for (var i = 0; i < cast->Header->NumMeshes; ++i)
                    {
                        var entry = cast->Entries  + i;
                        var elem  = cast->Elements + i;
                        using var mn = _tree.Node
                            ($"Mesh {i}: file=tr{entry->MeshId:d4}.pcb, bounds={AABBStr(entry->Bounds)} == {(nint)elem->Mesh:X}###mesh_{i}", elem->Mesh == null);
                        if (mn.SelectedOrHovered && elem->Mesh != null)
                            VisualizeCollider(&elem->Mesh->Collider, _materialId, _materialMask, false);
                        if (mn.Opened)
                            DrawColliderMesh(elem->Mesh);
                    }
                }
            }
                break;
            case ColliderType.Mesh:
                DrawColliderMesh((ColliderMesh*)coll);
                break;
            case ColliderType.Box:
            {
                var cast = (ColliderBox*)coll;
                _tree.LeafNode($"Translation: {Vec3Str(cast->Translation)}");
                _tree.LeafNode($"Rotation: {Vec3Str(cast->Rotation)}");
                _tree.LeafNode($"Scale: {Vec3Str(cast->Scale)}");
                DrawMat4x3("World",    ref cast->World);
                DrawMat4x3("InvWorld", ref cast->InvWorld);
            }
                break;
            case ColliderType.Cylinder:
            {
                var cast = (ColliderCylinder*)coll;
                _tree.LeafNode($"Translation: {Vec3Str(cast->Translation)}");
                _tree.LeafNode($"Rotation: {Vec3Str(cast->Rotation)}");
                _tree.LeafNode($"Scale: {Vec3Str(cast->Scale)}");
                _tree.LeafNode($"Radius: {cast->Radius:f3}");
                DrawMat4x3("World",    ref cast->World);
                DrawMat4x3("InvWorld", ref cast->InvWorld);
            }
                break;
            case ColliderType.Sphere:
            {
                var cast = (ColliderSphere*)coll;
                _tree.LeafNode($"Translation: {Vec3Str(cast->Translation)}");
                _tree.LeafNode($"Rotation: {Vec3Str(cast->Rotation)}");
                _tree.LeafNode($"Scale: {Vec3Str(cast->Scale)}");
                DrawMat4x3("World",    ref cast->World);
                DrawMat4x3("InvWorld", ref cast->InvWorld);
            }
                break;
            case ColliderType.Plane:
            case ColliderType.PlaneTwoSided:
            {
                var cast = (ColliderPlane*)coll;
                _tree.LeafNode($"Normal: {cast->World.Row2 / cast->Scale.Z:f3}");
                _tree.LeafNode($"Translation: {Vec3Str(cast->Translation)}");
                _tree.LeafNode($"Rotation: {Vec3Str(cast->Rotation)}");
                _tree.LeafNode($"Scale: {Vec3Str(cast->Scale)}");
                DrawMat4x3("World",    ref cast->World);
                DrawMat4x3("InvWorld", ref cast->InvWorld);
            }
                break;
        }

        if (layoutInstance != null)
            DebugLayout.DrawInstance(_tree, "Layout instance:", LayoutWorld.Instance()->ActiveLayout, layoutInstance, this);
    }

    private void DrawColliderMesh
    (
        ColliderMesh* coll
    )
    {
        DrawResource(coll->Resource);

        if (ImGui.Button("复制 translation"))
        {
            var t = coll->Translation;
            var r = coll->Rotation;

            if (MathF.Abs(MathF.Abs(r.X) - MathF.PI) < 0.1f)
                r.Y *= -1;

            ImGui.SetClipboardText($"{LayoutUtil.Vec3ToSource(t)}, {LayoutUtil.Vec3ToSource(r)}");
        }

        _tree.LeafNode($"Translation: {Vec3Str(coll->Translation)}");
        _tree.LeafNode($"Rotation: {Vec3Str(coll->Rotation)}");
        _tree.LeafNode($"Scale: {Vec3Str(coll->Scale)}");
        DrawMat4x3("World",    ref coll->World);
        DrawMat4x3("InvWorld", ref coll->InvWorld);
        if (_tree.LeafNode($"Bounding sphere: {SphereStr(coll->BoundingSphere)}").SelectedOrHovered)
            VisualizeSphere(coll->BoundingSphere, 0xff00ff00);
        if (_tree.LeafNode($"Bounding box: {AABBStr(coll->WorldBoundingBox)}").SelectedOrHovered)
            VisualizeOBB(ref coll->WorldBoundingBox, ref Matrix4x3.Identity, 0xff00ff00);
        _tree.LeafNode($"Total size: {coll->TotalPrimitives} prims, {coll->TotalChildren} nodes");
        _tree.LeafNode
        (
            $"网格类型：{(coll->MeshIsSimple ? "简单 (simple)" : coll->MemoryData != null ? "内存 PCB (PCB in-memory)" : "文件 PCB (PCB from file)")} {(coll->Loaded ? "" : "(正在加载)")}"
        );
        if (coll->Mesh == null || coll->MeshIsSimple)
            return;

        var mesh = (MeshPCB*)coll->Mesh;
        DrawColliderMeshPCBNode
            ("Root", mesh->RootNode, ref coll->World, coll->Collider.ObjectMaterialValue & coll->Collider.ObjectMaterialMask, ~coll->Collider.ObjectMaterialMask);
    }

    private void DrawColliderMeshPCBNode
    (
        string            tag,
        MeshPCB.FileNode* node,
        ref Matrix4x3     world,
        ulong             objMatId,
        ulong             objMatInvMask
    )
    {
        if (node == null)
            return;

        using var n = _tree.Node(tag);

        if (n.SelectedOrHovered)
        {
            VisualizeColliderMeshPCBNode
                (node, ref world, new(1, 1, 0, 0.7f), objMatId, objMatId, _materialId, _materialMask, Service.ObjectTable.LocalPlayer?.Position);
        }

        if (!n.Opened)
            return;

        _tree.LeafNode($"Header: {node->Header:X16}");
        if (_tree.LeafNode($"AABB: {AABBStr(node->LocalBounds)}").SelectedOrHovered)
            VisualizeOBB(ref node->LocalBounds, ref world, 0xff00ff00);

        {
            using var nv = _tree.Node($"Vertices: {node->NumVertsRaw}+{node->NumVertsCompressed}", node->NumVertsRaw + node->NumVertsCompressed == 0);

            if (nv.Opened)
            {
                for (var i = 0; i < node->NumVertsRaw + node->NumVertsCompressed; ++i)
                {
                    var v = node->Vertex(i);
                    if (_tree.LeafNode($"[{i}] ({(i < node->NumVertsRaw ? 'r' : 'c')}): {Vec3Str(v)}").SelectedOrHovered)
                        VisualizeVertex(world.TransformCoordinate(v), 0xff00ffff);
                }
            }
        }
        {
            using var np = _tree.Node($"Primitives: {node->NumPrims}", node->NumPrims == 0);

            if (np.Opened)
            {
                var i = 0;

                foreach (ref var prim in node->Primitives)
                {
                    if (_tree.LeafNode($"[{i++}]: {prim.V1}x{prim.V2}x{prim.V3}, material={prim.Material:X8}").SelectedOrHovered)
                        VisualizeTriangle(node, ref prim, ref world, 0xff00ffff);
                }
            }
        }
        DrawColliderMeshPCBNode($"Child 1 (+{node->Child1Offset})", node->Child1, ref world, objMatId, objMatId);
        DrawColliderMeshPCBNode($"Child 2 (+{node->Child2Offset})", node->Child2, ref world, objMatId, objMatId);
    }

    private void DrawResource
    (
        Resource* res
    )
    {
        if (res != null) _tree.LeafNode($"Resource: {(nint)res:X} '{res->PathString}'");
        else _tree.LeafNode("Resource: null");
    }

    private string SphereStr
    (
        Vector4 s
    ) => $"[{s.X:f3}, {s.Y:f3}, {s.Z:f3}] R{s.W:f3}";

    private string Vec3Str
    (
        Vector3 v
    ) => $"[{v.X:f3}, {v.Y:f3}, {v.Z:f3}]";

    private string AABBStr
    (
        AABB bb
    ) => $"{Vec3Str(bb.Min)} - {Vec3Str(bb.Max)}";

    private void DrawMat4x3
    (
        string        tag,
        ref Matrix4x3 mat
    )
    {
        _tree.LeafNode($"{tag} R0: {Vec3Str(mat.Row0)}");
        _tree.LeafNode($"{tag} R1: {Vec3Str(mat.Row1)}");
        _tree.LeafNode($"{tag} R2: {Vec3Str(mat.Row2)}");
        _tree.LeafNode($"{tag} R3: {Vec3Str(mat.Row3)}");
    }

    private void ContextCollider
    (
        Collider* coll
    )
    {
        var activeLayers = new BitMask(coll->LayerMask);

        foreach (var i in _availableLayers.SetBits())
        {
            var active = activeLayers[i];

            if (ImGui.Checkbox($"Layer {i}", ref active))
            {
                activeLayers[i] = active;
                coll->LayerMask = activeLayers.Raw;
            }
        }

        var raycast = (coll->VisibilityFlags & 1) != 0;
        if (ImGui.Checkbox("标志：射线检测 (Raycast)", ref raycast))
            coll->VisibilityFlags ^= 1;

        var globalVisit = (coll->VisibilityFlags & 2) != 0;
        if (ImGui.Checkbox("标志：全局访问 (Global Visit)", ref globalVisit))
            coll->VisibilityFlags ^= 2;
    }

    private EffectMesh.Data.Builder GetDynamicMeshes() => _meshDynamicBuilder ??= _meshDynamicData.Map(_dd.RenderContext);

    private bool RaycastDetour
    (
        SceneWrapper*  self,
        RaycastHit*    result,
        ulong          layerMask,
        RaycastParams* param
    )
    {
        Service.Log.Debug
        (
            $"射线检测 (Raycast)：层级={layerMask:X}, 算法={param->Algorithm}, 原点={*param->Origin}, 方向={*param->Direction}, 最大法线={param->MaxPlaneNormalY}, 最大距离={*param->MaxDistance}, 过滤器={param->MaterialFilter->Value:X}/{param->MaterialFilter->Mask:X}"
        );
        return _raycastHook!.Original(self, result, layerMask, param);
    }
}
