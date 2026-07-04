using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Custom.Editor;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Editor.Types;
using Collider = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Collider;

namespace vnavmesh.UI.Editor;

internal static unsafe class CustomizationEditorWorldOverlay
{
    public delegate void AddColliderInsertionDelegate(Vector3 a, Vector3 b, DraftSceneColliderInsertionKind kind);

    public delegate void AddMeshLinkDelegate(Vector3 a, Vector3 b, DraftMeshLinkKind kind);

    public delegate void AddOffMeshConnectionDelegate(Vector3 a, Vector3 b);

    public struct DraftEditState
    {
        public DraftEditMode Mode;
        public int           Index;
        public Vector3       PlaneOrigin;
        public Vector3       PlaneNormal;
        public Vector3       DragStartPoint;
        public Vector3       InitialA;
        public Vector3       InitialB;
    }

    public enum DraftEditMode
    {
        None,
        ColliderInsertionMin,
        ColliderInsertionMax,
        ColliderInsertionTranslate,
        MeshLinkStart,
        MeshLinkEnd,
        MeshLinkTranslate,
        OffMeshStart,
        OffMeshEnd,
        OffMeshTranslate,
        InstanceTranslation
    }

    private const int VK_LBUTTON = 0x01;
    private const int VK_ESCAPE  = 0x1B;

    public static void Draw
    (
        ref PickKind                     pickKind,
        ref Vector3?                     pendingPickPoint,
        ref Vector3?                     currentPickPoint,
        ref bool                         lastPickMouseDown,
        ref bool                         lastWorldSelectMouseDown,
        ref bool                         lastPickEscapeDown,
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        ref Selection?                   pendingLeftPanelFocusSelection,
        ref string                       statusText,
        ref DraftEditState               draftEditState,
        Action                           onDraftEdited,
        DebugGameCollision               collision,
        DebugDrawer                      dd,
        CustomizationPreviewBuilder      previewBuilder,
        AddColliderInsertionDelegate     onAddColliderInsertion,
        AddMeshLinkDelegate              onAddMeshLink,
        AddOffMeshConnectionDelegate     onAddOffMeshConnection
    )
    {
        currentPickPoint = null;

        switch (pickKind)
        {
            case PickKind.SelectCollider:
                HandleWorldSelection
                (
                    ref selection,
                    ref pendingLeftPanelFocusSelection,
                    ref lastWorldSelectMouseDown,
                    ref currentPickPoint,
                    ref statusText,
                    ref workspace,
                    collision,
                    dd,
                    previewBuilder
                );
                break;
            case PickKind.SelectTriangle:
                HandleTriangleSelection
                (
                    ref selection,
                    ref pendingLeftPanelFocusSelection,
                    ref lastWorldSelectMouseDown,
                    ref currentPickPoint,
                    ref statusText,
                    collision,
                    dd,
                    previewBuilder
                );
                break;
            case PickKind.None:
                break;
            default:
                HandlePicking
                (
                    ref pickKind,
                    ref pendingPickPoint,
                    ref currentPickPoint,
                    ref lastPickMouseDown,
                    ref statusText,
                    collision,
                    dd,
                    onAddColliderInsertion,
                    onAddMeshLink,
                    onAddOffMeshConnection
                );
                break;
        }

        DrawPreviewInstancesOverlay(selection, previewBuilder, dd);
        DrawPreviewVertexOverlay(selection, previewBuilder, dd);
        DrawPreviewPrimitiveOverlay(selection, previewBuilder, dd);
        DrawInstancePatchOverlay(workspace, selection, previewBuilder, collision, dd);
        HandleDraftEditing(ref workspace, ref selection, ref draftEditState, ref statusText, onDraftEdited, dd);

        if (pendingPickPoint is { } pending)
        {
            dd.DrawWorldPointFilled(pending, 6, 0xFFFF00FF);
            if (currentPickPoint is { } current)
                DrawPendingPickPreview(pickKind, pending, current, dd);
        }

        for (var i = 0; i < workspace.Draft.ColliderInsertions.Count; ++i)
        {
            var insertion = workspace.Draft.ColliderInsertions[i];
            if (!insertion.Enabled)
                continue;

            var bounds = CustomizationEditorSpatial.CreateBounds(insertion.Min, insertion.Max);
            if (!collision.IsBoundsWithinEditorRenderDistance(bounds))
                continue;

            var selected = selection is { Kind: SelectionKind.ColliderInsertion, Index: var selectedIndex and >= 0 } &&
                           selectedIndex == i;
            var color = selected                                                     ? 0xFFFFD94A
                        : insertion.Kind == DraftSceneColliderInsertionKind.Cylinder ? 0xFF00FF00
                                                                                       : 0xFF00FFFF;
            if (insertion.Kind == DraftSceneColliderInsertionKind.Cylinder)
                dd.DrawWorldCylinder
                (
                    (insertion.Min + insertion.Max) * 0.5f,
                    (insertion.Max - insertion.Min) * 0.5f,
                    color,
                    selected ?
                        3 :
                        2
                );
            else
                dd.DrawWorldAABB
                (
                    (insertion.Min + insertion.Max) * 0.5f,
                    (insertion.Max - insertion.Min) * 0.5f,
                    color,
                    selected ?
                        3 :
                        2
                );
        }

        for (var i = 0; i < workspace.Draft.MeshLinks.Count; ++i)
        {
            var link = workspace.Draft.MeshLinks[i];
            if (!link.Enabled)
                continue;

            if (!collision.IsSegmentWithinEditorRenderDistance(link.Start, link.End))
                continue;

            var selected = selection is { Kind: SelectionKind.MeshLink, Index: var selectedIndex and >= 0 } &&
                           selectedIndex == i;
            var color = selected ?
                            0xFFFFD94A :
                            0xFFAAFF00;
            var thickness = selected ?
                                6 :
                                4;

            dd.DrawWorldLine(link.Start, link.End, color, thickness);

            if (!link.Bidirectional)
            {
                var dir = Vector3.Normalize(link.End - link.Start);
                var len = Vector3.Distance(link.Start, link.End);
                var arrowSize = selected ?
                                    20f :
                                    15f;

                // 每隔 4 米绘制一个单向流向箭头，距终点留出 0.5 米
                for (var d = 4.0f; d < len - 0.5f; d += 4.0f)
                {
                    var p = link.Start          + (d * dir);
                    dd.DrawWorldArrowPoint(p, p - dir, arrowSize, color, thickness);
                }

                // 终点处的箭头
                dd.DrawWorldArrowPoint(link.End, link.End - dir, arrowSize, color, thickness);
            }
        }

        for (var i = 0; i < workspace.Draft.OffMeshConnections.Count; ++i)
        {
            var link = workspace.Draft.OffMeshConnections[i];
            if (!link.Enabled)
                continue;

            if (!collision.IsSegmentWithinEditorRenderDistance(link.Start, link.End))
                continue;

            var selected = selection is { Kind: SelectionKind.OffMeshConnection, Index: var selectedIndex and >= 0 } &&
                           selectedIndex == i;
            var color = selected ?
                            0xFFFFD94A :
                            0xFFFF8800;
            var thickness = selected ?
                                6 :
                                4;

            if (link.Bidirectional)
            {
                var arrowSize = selected ?
                                    10f :
                                    6f;
                dd.DrawWorldArc(link.Start, link.End, 0.15f, arrowSize, arrowSize, color, thickness);
            }
            else
            {
                // 单向抛物线本身不画起终点箭头，而是通过多点流动绘制
                dd.DrawWorldArc(link.Start, link.End, 0.15f, 0f, 0f, color, thickness);

                // 采样切线，在抛物线上绘制多个流动的大箭头
                var delta = link.End - link.Start;
                var len   = delta.Length();
                var h     = 0.15f * len;
                var arrowSize = selected ?
                                    20f :
                                    15f;

                Vector3 EvalArc(float u)
                {
                    var res   = link.Start + (u * delta);
                    var coeff = (u              * 2f) - 1f;
                    res.Y += h * (1f                  - (coeff * coeff));
                    return res;
                }

                // 25%, 50%, 75%, 100% 处绘制流向大箭头
                var steps = new[] { 0.25f, 0.5f, 0.75f, 1.0f };

                foreach (var u in steps)
                {
                    var p = EvalArc(u);
                    var q = EvalArc(u - 0.05f); // 采样 u 前面的点作为来向 q，求得局部曲率方向
                    dd.DrawWorldArrowPoint(p, q, arrowSize, color, thickness);
                }
            }
        }

        DrawPartPatchOverlay(workspace, selection, previewBuilder, collision, dd);
    }

    private static void HandlePicking
    (
        ref PickKind                 pickKind,
        ref Vector3?                 pendingPickPoint,
        ref Vector3?                 currentPickPoint,
        ref bool                     lastPickMouseDown,
        ref string                   statusText,
        DebugGameCollision           collision,
        DebugDrawer                  dd,
        AddColliderInsertionDelegate onAddColliderInsertion,
        AddMeshLinkDelegate          onAddMeshLink,
        AddOffMeshConnectionDelegate onAddOffMeshConnection
    )
    {
        var clicked = TakeWorldPickClick(ref lastPickMouseDown);
        if (!IsWorldClickAllowed())
            return;

        if (!collision.TryGetMouseRaycastHit(out var hit))
            return;

        currentPickPoint = hit.Point;
        dd.DrawWorldPointFilled(hit.Point, 5, 0xFFFFFF00);
        collision.VisualizeCollider(hit.Object, default, default, false);

        if (!clicked)
            return;

        var point = hit.Point;

        if (pendingPickPoint == null)
        {
            pendingPickPoint = point;
            statusText       = $"{GetPickKindTitle(pickKind)}: 已记录第 1 个点 {point:f3}, 点击第 2 个世界点";
            return;
        }

        var first = pendingPickPoint.Value;
        pendingPickPoint = null;
        var completedKind = pickKind;

        switch (pickKind)
        {
            case PickKind.Aabb:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Aabb);
                break;
            case PickKind.Cylinder:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Cylinder);
                break;
            case PickKind.LinkPoints:
                onAddMeshLink(first, point, DraftMeshLinkKind.Points);
                break;
            case PickKind.LinkShortcut:
                onAddMeshLink(first, point, DraftMeshLinkKind.Shortcut);
                break;
            case PickKind.LinkClientPath:
                onAddMeshLink(first, point, DraftMeshLinkKind.ClientPath);
                break;
            case PickKind.OffMesh:
                onAddOffMeshConnection(first, point);
                break;
        }

        statusText       = $"已创建 {GetPickKindTitle(completedKind)}";
        pickKind         = PickKind.None;
        currentPickPoint = null;
    }

    private static void HandleWorldSelection
    (
        ref Selection                    selection,
        ref Selection?                   pendingLeftPanelFocusSelection,
        ref bool                         lastWorldSelectMouseDown,
        ref Vector3?                     currentPickPoint,
        ref string                       statusText,
        ref CustomizationEditorWorkspace workspace,
        DebugGameCollision               collision,
        DebugDrawer                      dd,
        CustomizationPreviewBuilder      previewBuilder
    )
    {
        var clicked = TakeWorldSelectClick(ref lastWorldSelectMouseDown);
        if (!IsWorldClickAllowed())
            return;

        if (!TryGetWorldSelectionRay(dd, out var rayOrigin, out var rayDirection))
            return;

        var hasHit = collision.TryGetMouseRaycastHit(out var hit);

        if (hasHit)
        {
            currentPickPoint = hit.Point;
            dd.DrawWorldPointFilled(hit.Point, 5, 0xFFFFFF00);
            collision.VisualizeCollider(hit.Object, default, default, false);
        }

        if (!clicked)
            return;

        if (TrySelectColliderInsertion(ref selection, ref pendingLeftPanelFocusSelection, rayOrigin, rayDirection, ref workspace, ref statusText))
            return;

        if (TrySelectMeshLink(ref selection, ref pendingLeftPanelFocusSelection, rayOrigin, rayDirection, ref workspace, ref statusText))
            return;

        if (TrySelectOffMeshConnection(ref selection, ref pendingLeftPanelFocusSelection, rayOrigin, rayDirection, ref workspace, ref statusText))
            return;

        if (hasHit && TrySelectPreviewInstance(ref selection, ref pendingLeftPanelFocusSelection, hit.Object, previewBuilder, ref statusText))
            return;

        statusText = "未选中可编辑对象";
    }

    private static void HandleTriangleSelection
    (
        ref Selection               selection,
        ref Selection?              pendingLeftPanelFocusSelection,
        ref bool                    lastWorldSelectMouseDown,
        ref Vector3?                currentPickPoint,
        ref string                  statusText,
        DebugGameCollision          collision,
        DebugDrawer                 dd,
        CustomizationPreviewBuilder previewBuilder
    )
    {
        var clicked = TakeWorldSelectClick(ref lastWorldSelectMouseDown);
        if (!IsWorldClickAllowed())
            return;

        if (!TryGetWorldSelectionRay(dd, out var rayOrigin, out var rayDirection))
            return;

        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        TriangleHit? closestHit      = null;
        var          closestDistance = float.MaxValue;

        foreach (var (key, mesh) in previewBuilder.Extractor.Meshes.OrderBy(static x => x.Key, StringComparer.Ordinal))
        {
            foreach (var instance in mesh.Instances)
            {
                if (!collision.IsBoundsWithinEditorRenderDistance(instance.WorldBounds))
                    continue;

                for (var partIndex = 0; partIndex < mesh.Parts.Count; ++partIndex)
                {
                    var part = mesh.Parts[partIndex];

                    for (var primIndex = 0; primIndex < part.Primitives.Count; ++primIndex)
                    {
                        var prim = part.Primitives[primIndex];
                        var v0   = instance.WorldTransform.TransformCoordinate(part.Vertices[prim.V1]);
                        var v1   = instance.WorldTransform.TransformCoordinate(part.Vertices[prim.V2]);
                        var v2   = instance.WorldTransform.TransformCoordinate(part.Vertices[prim.V3]);

                        if (RayIntersectsTriangle(rayOrigin, rayDirection, v0, v1, v2, out var distance, out var hitPoint))
                        {
                            if (distance < closestDistance)
                            {
                                closestDistance = distance;
                                closestHit      = new(key, partIndex, primIndex, hitPoint, v0, v1, v2);
                            }
                        }
                    }
                }
            }
        }

        if (closestHit != null)
        {
            currentPickPoint = closestHit.Value.HitPoint;
            dd.DrawWorldPointFilled(closestHit.Value.HitPoint, 5, 0xFFFFFF00);
            dd.DrawWorldTriangle(closestHit.Value.V0, closestHit.Value.V1, closestHit.Value.V2, 0xFFFFFF00, 3);

            if (clicked)
            {
                selection = new(SelectionKind.PreviewPrimitive, closestHit.Value.PartIndex, closestHit.Value.PrimitiveIndex, closestHit.Value.MeshKey);
                pendingLeftPanelFocusSelection = selection;
                statusText = $"已选三角形: {closestHit.Value.MeshKey} p{closestHit.Value.PartIndex} t{closestHit.Value.PrimitiveIndex}";
            }
        }
        else if (clicked) statusText = "未选中三角形";
    }

    private static bool RayIntersectsTriangle
    (
        Vector3     rayOrigin,
        Vector3     rayDirection,
        Vector3     v0,
        Vector3     v1,
        Vector3     v2,
        out float   distance,
        out Vector3 hitPoint
    )
    {
        distance = 0f;
        hitPoint = default;

        var edge1 = v1 - v0;
        var edge2 = v2 - v0;
        var h     = Vector3.Cross(rayDirection, edge2);
        var a     = Vector3.Dot(edge1, h);

        if (MathF.Abs(a) < 0.00001f)
            return false;

        var f = 1f / a;
        var s = rayOrigin - v0;
        var u = f * Vector3.Dot(s, h);

        if (u < 0f || u > 1f)
            return false;

        var q = Vector3.Cross(s, edge1);
        var v = f * Vector3.Dot(rayDirection, q);

        if (v < 0f || u + v > 1f)
            return false;

        var t = f * Vector3.Dot(edge2, q);

        if (t <= 0.00001f)
            return false;

        distance = t;
        hitPoint = rayOrigin + (rayDirection * t);
        return true;
    }

    private readonly record struct TriangleHit
    (
        string  MeshKey,
        int     PartIndex,
        int     PrimitiveIndex,
        Vector3 HitPoint,
        Vector3 V0,
        Vector3 V1,
        Vector3 V2
    );

    private static bool TrySelectPreviewInstance
    (
        ref Selection               selection,
        ref Selection?              pendingLeftPanelFocusSelection,
        Collider*                   collider,
        CustomizationPreviewBuilder previewBuilder,
        ref string                  statusText
    )
    {
        if (collider == null || previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return false;

        var layoutObjectId = (collider->LayoutObjectId << 32) | (collider->LayoutObjectId >> 32);

        foreach (var (key, mesh) in previewBuilder.Extractor.Meshes.OrderBy(static x => x.Key, StringComparer.Ordinal))
        {
            var instanceIndex = 0;

            foreach (var instance in mesh.Instances)
            {
                if (instance.Id != layoutObjectId)
                {
                    ++instanceIndex;
                    continue;
                }

                selection                      = new(SelectionKind.PreviewInstance, instanceIndex, Key: key);
                pendingLeftPanelFocusSelection = selection;
                statusText                     = $"已选预览实例: {key} #{instanceIndex}";
                return true;
            }
        }

        return false;
    }

    private static bool TrySelectColliderInsertion
    (
        ref Selection                    selection,
        ref Selection?                   pendingLeftPanelFocusSelection,
        Vector3                          rayOrigin,
        Vector3                          rayDirection,
        ref CustomizationEditorWorkspace workspace,
        ref string                       statusText
    )
    {
        Selection? bestSelection = null;
        var        bestDistance  = float.MaxValue;

        for (var i = 0; i < workspace.Draft.ColliderInsertions.Count; ++i)
        {
            var item = workspace.Draft.ColliderInsertions[i];
            if (!item.Enabled)
                continue;

            var bounds = new AABB { Min = item.Min, Max = item.Max };
            if (!RayIntersectsAabb(rayOrigin, rayDirection, bounds, out var distance))
                continue;

            if (distance >= bestDistance)
                continue;

            bestDistance  = distance;
            bestSelection = new(SelectionKind.ColliderInsertion, i);
        }

        if (bestSelection == null)
            return false;

        selection                      = bestSelection;
        pendingLeftPanelFocusSelection = selection;
        statusText                     = $"已选碰撞插入: {CustomizationEditorWidgets.FormatEnumDisplayName(workspace.Draft.ColliderInsertions[selection.Index].Kind)}";
        return true;
    }

    private static bool TrySelectMeshLink
    (
        ref Selection                    selection,
        ref Selection?                   pendingLeftPanelFocusSelection,
        Vector3                          rayOrigin,
        Vector3                          rayDirection,
        ref CustomizationEditorWorkspace workspace,
        ref string                       statusText
    )
    {
        Selection? bestSelection = null;
        var        bestRayT      = float.MaxValue;

        for (var i = 0; i < workspace.Draft.MeshLinks.Count; ++i)
        {
            var item = workspace.Draft.MeshLinks[i];
            if (!item.Enabled)
                continue;

            var dist = RaySegmentDistance(rayOrigin, rayDirection, item.Start, item.End, out var tOnRay, out _);
            if (dist > 0.8f)
                continue;

            if (tOnRay < bestRayT)
            {
                bestRayT      = tOnRay;
                bestSelection = new(SelectionKind.MeshLink, i);
            }
        }

        if (bestSelection == null)
            return false;

        selection = bestSelection;
        pendingLeftPanelFocusSelection = selection;
        statusText = $"已选网格连线: {CustomizationEditorWidgets.FormatEnumDisplayName(workspace.Draft.MeshLinks[selection.Index].Kind)} #{selection.Index}";
        return true;
    }

    private static bool TrySelectOffMeshConnection
    (
        ref Selection                    selection,
        ref Selection?                   pendingLeftPanelFocusSelection,
        Vector3                          rayOrigin,
        Vector3                          rayDirection,
        ref CustomizationEditorWorkspace workspace,
        ref string                       statusText
    )
    {
        Selection? bestSelection = null;
        var        bestRayT      = float.MaxValue;

        for (var i = 0; i < workspace.Draft.OffMeshConnections.Count; ++i)
        {
            var item = workspace.Draft.OffMeshConnections[i];
            if (!item.Enabled)
                continue;

            var dist = RaySegmentDistance(rayOrigin, rayDirection, item.Start, item.End, out var tOnRay, out _);
            if (dist > 0.8f)
                continue;

            if (tOnRay < bestRayT)
            {
                bestRayT      = tOnRay;
                bestSelection = new(SelectionKind.OffMeshConnection, i);
            }
        }

        if (bestSelection == null)
            return false;

        selection                      = bestSelection;
        pendingLeftPanelFocusSelection = selection;
        statusText                     = $"已选离网连接 #{selection.Index}";
        return true;
    }

    private static float RaySegmentDistance(Vector3 rayOrigin, Vector3 rayDir, Vector3 p0, Vector3 p1, out float tOnRay, out float uOnSegment)
    {
        var u = p1 - p0;
        var v = rayDir;
        var w = rayOrigin - p0;

        var a = Vector3.Dot(u, u);
        var b = Vector3.Dot(u, v);
        var c = Vector3.Dot(v, v);
        var d = Vector3.Dot(u, w);
        var e = Vector3.Dot(v, w);

        var   D = (a * c) - (b * b);
        float sc, tc;

        if (D < 0.00001f)
        {
            sc = 0.0f;
            tc = c > 0.0f ?
                     e / c :
                     0.0f;
        }
        else
        {
            sc = ((b * e) - (c * d)) / D;
            tc = ((a * e) - (b * d)) / D;
        }

        if (sc < 0.0f)
            sc = 0.0f;
        else if (sc > 1.0f)
            sc = 1.0f;

        tc = ((sc * b) + e) / c;

        if (tc < 0.0f)
        {
            tc = 0.0f;
            sc = -d / a;
            if (sc      < 0.0f) sc = 0.0f;
            else if (sc > 1.0f) sc = 1.0f;
        }

        tOnRay     = tc;
        uOnSegment = sc;

        var closestOnRay     = rayOrigin + (tc * rayDir);
        var closestOnSegment = p0        + (sc * u);
        return Vector3.Distance(closestOnRay, closestOnSegment);
    }

    private static bool TryGetWorldSelectionRay(DebugDrawer dd, out Vector3 origin, out Vector3 direction)
    {
        origin    = default;
        direction = default;

        if (!TryGetViewportCursorPosition(out var screenPos))
            return false;

        if (dd.ViewportSize.X <= 0 || dd.ViewportSize.Y <= 0)
            return false;

        var clipPos = new Vector3((2 * screenPos.X / dd.ViewportSize.X) - 1, 1 - (2 * screenPos.Y / dd.ViewportSize.Y), 1);
        Matrix4x4.Invert(dd.ViewProj, out var invViewProj);
        var cameraPosAtPlaneP = Vector4.Transform(clipPos, invViewProj);
        if (MathF.Abs(cameraPosAtPlaneP.W) < 0.0001f)
            return false;

        var cameraPosAtPlane = new Vector3
            (cameraPosAtPlaneP.X / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Y / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Z / cameraPosAtPlaneP.W);
        var dir = cameraPosAtPlane - dd.Origin;
        if (dir.LengthSquared() < 0.0001f)
            return false;

        origin    = dd.Origin;
        direction = Vector3.Normalize(dir);
        return true;
    }

    private static void HandleDraftEditing
    (
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        ref DraftEditState               draftEditState,
        ref string                       statusText,
        Action                           onDraftEdited,
        DebugDrawer                      dd
    )
    {
        if (!TryGetViewportCursorPosition(out var screenPos))
        {
            draftEditState = default;
            return;
        }

        if (!TryGetWorldSelectionRay(dd, out var rayOrigin, out var rayDirection))
        {
            draftEditState = default;
            return;
        }

        if (draftEditState.Mode != DraftEditMode.None)
        {
            UpdateDraftEditing(ref workspace, ref selection, ref draftEditState, ref statusText, onDraftEdited, rayOrigin, rayDirection);
            return;
        }

        DrawDraftEditingHandles(ref workspace, selection, rayOrigin, rayDirection, screenPos, dd, ref draftEditState);
    }

    private static void DrawDraftEditingHandles
    (
        ref CustomizationEditorWorkspace workspace,
        Selection                        selection,
        Vector3                          rayOrigin,
        Vector3                          rayDirection,
        Vector2                          screenPos,
        DebugDrawer                      dd,
        ref DraftEditState               draftEditState
    )
    {
        switch (selection.Kind)
        {
            case SelectionKind.ColliderInsertion:
                if (selection.Index >= 0 && selection.Index < workspace.Draft.ColliderInsertions.Count)
                {
                    ref var item = ref CollectionsMarshal.AsSpan(workspace.Draft.ColliderInsertions)[selection.Index];
                    if (!item.Enabled)
                        return;

                    DrawHandle
                    (
                        item.Min,
                        8f,
                        0xFF4DDBFF,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        new()
                        {
                            Mode        = DraftEditMode.ColliderInsertionMin,
                            Index       = selection.Index,
                            PlaneOrigin = item.Min,
                            InitialA    = item.Min,
                            InitialB    = item.Max
                        },
                        "拖拽最小角点"
                    );
                    DrawHandle
                    (
                        item.Max,
                        8f,
                        0xFFFFB84D,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        new()
                        {
                            Mode        = DraftEditMode.ColliderInsertionMax,
                            Index       = selection.Index,
                            PlaneOrigin = item.Max,
                            InitialA    = item.Min,
                            InitialB    = item.Max
                        },
                        "拖拽最大角点"
                    );
                    DrawTranslationHandle
                    (
                        (item.Min + item.Max) * 0.5f,
                        selection.Index,
                        DraftEditMode.ColliderInsertionTranslate,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        item.Min,
                        item.Max
                    );
                }

                break;
            case SelectionKind.MeshLink:
                if (selection.Index >= 0 && selection.Index < workspace.Draft.MeshLinks.Count)
                {
                    ref var item = ref CollectionsMarshal.AsSpan(workspace.Draft.MeshLinks)[selection.Index];
                    if (!item.Enabled)
                        return;

                    DrawSegmentHandles
                    (
                        item.Start,
                        item.End,
                        selection.Index,
                        DraftEditMode.MeshLinkStart,
                        DraftEditMode.MeshLinkEnd,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState
                    );
                    DrawTranslationHandle
                    (
                        (item.Start + item.End) * 0.5f,
                        selection.Index,
                        DraftEditMode.MeshLinkTranslate,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        item.Start,
                        item.End
                    );
                }

                break;
            case SelectionKind.OffMeshConnection:
                if (selection.Index >= 0 && selection.Index < workspace.Draft.OffMeshConnections.Count)
                {
                    ref var item = ref CollectionsMarshal.AsSpan(workspace.Draft.OffMeshConnections)[selection.Index];
                    if (!item.Enabled)
                        return;

                    DrawSegmentHandles
                    (
                        item.Start,
                        item.End,
                        selection.Index,
                        DraftEditMode.OffMeshStart,
                        DraftEditMode.OffMeshEnd,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState
                    );
                    DrawTranslationHandle
                    (
                        (item.Start + item.End) * 0.5f,
                        selection.Index,
                        DraftEditMode.OffMeshTranslate,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        item.Start,
                        item.End
                    );
                }

                break;
            case SelectionKind.InstancePatch:
                if (selection.Index >= 0 && selection.Index < workspace.Draft.InstancePatches.Count)
                {
                    ref var item = ref CollectionsMarshal.AsSpan(workspace.Draft.InstancePatches)[selection.Index];
                    if (!item.Enabled || item.Kind != DraftSceneInstancePatchKind.Transform)
                        return;

                    DrawHandle
                    (
                        item.WorldTransform.Translation,
                        9f,
                        0xFFFFD94A,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        new()
                        {
                            Mode        = DraftEditMode.InstanceTranslation,
                            Index       = selection.Index,
                            PlaneOrigin = item.WorldTransform.Translation,
                            InitialA    = item.WorldTransform.Translation
                        },
                        "拖拽实例位置"
                    );
                }

                break;
        }
    }

    private static void DrawSegmentHandles
    (
        Vector3            start,
        Vector3            end,
        int                index,
        DraftEditMode      startMode,
        DraftEditMode      endMode,
        Vector3            rayOrigin,
        Vector3            rayDirection,
        DebugDrawer        dd,
        Vector2            screenPos,
        ref DraftEditState draftEditState
    )
    {
        DrawHandle
        (
            start,
            7f,
            0xFF4DDBFF,
            rayOrigin,
            rayDirection,
            dd,
            screenPos,
            ref draftEditState,
            new()
            {
                Mode     = startMode,
                Index    = index,
                InitialA = start,
                InitialB = end
            },
            "拖拽起点"
        );
        DrawHandle
        (
            end,
            7f,
            0xFFFFB84D,
            rayOrigin,
            rayDirection,
            dd,
            screenPos,
            ref draftEditState,
            new()
            {
                Mode     = endMode,
                Index    = index,
                InitialA = start,
                InitialB = end
            },
            "拖拽终点"
        );
    }

    private static void DrawTranslationHandle
    (
        Vector3            center,
        int                index,
        DraftEditMode      mode,
        Vector3            rayOrigin,
        Vector3            rayDirection,
        DebugDrawer        dd,
        Vector2            screenPos,
        ref DraftEditState draftEditState,
        Vector3            start,
        Vector3            end
    )
    {
        DrawHandle
        (
            center,
            9f,
            0xFFFFD94A,
            rayOrigin,
            rayDirection,
            dd,
            screenPos,
            ref draftEditState,
            new()
            {
                Mode        = mode,
                Index       = index,
                InitialA    = start,
                InitialB    = end,
                PlaneOrigin = center
            },
            "拖拽整体位置"
        );
    }

    private static void DrawHandle
    (
        Vector3            position,
        float              radius,
        uint               color,
        Vector3            rayOrigin,
        Vector3            rayDirection,
        DebugDrawer        dd,
        Vector2            screenPos,
        ref DraftEditState draftEditState,
        DraftEditState     nextState,
        string             tooltip
    )
    {
        dd.DrawWorldPointFilled(position, radius, color);
        if (!TryGetScreenDistance(dd, position, screenPos, out var distance) || distance > 12f)
            return;

        dd.DrawWorldPoint(position, radius + 2f, 0xFFFFFFFF, 2);
        if (!ImGui.IsMouseClicked(ImGuiMouseButton.Left) || !IsWorldClickAllowed())
            return;

        nextState.PlaneNormal = -Vector3.Normalize(dd.Origin - position);
        nextState.PlaneOrigin = position;
        if (!TryRayPlaneIntersection(rayOrigin, rayDirection, nextState.PlaneOrigin, nextState.PlaneNormal, out nextState.DragStartPoint))
            nextState.DragStartPoint = position;
        draftEditState = nextState;
    }

    private static void UpdateDraftEditing
    (
        ref CustomizationEditorWorkspace workspace,
        ref Selection                    selection,
        ref DraftEditState               draftEditState,
        ref string                       statusText,
        Action                           onDraftEdited,
        Vector3                          rayOrigin,
        Vector3                          rayDirection
    )
    {
        if (!ImGui.IsMouseDown(ImGuiMouseButton.Left))
        {
            onDraftEdited();
            statusText     = "已更新草稿可视化编辑结果";
            draftEditState = default;
            return;
        }

        if (!TryRayPlaneIntersection(rayOrigin, rayDirection, draftEditState.PlaneOrigin, draftEditState.PlaneNormal, out var hitPoint))
            return;

        var delta = hitPoint - draftEditState.DragStartPoint;

        switch (draftEditState.Mode)
        {
            case DraftEditMode.ColliderInsertionMin:
            case DraftEditMode.ColliderInsertionMax:
            case DraftEditMode.ColliderInsertionTranslate:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.ColliderInsertions.Count)
                    return;

                ref var collider = ref CollectionsMarshal.AsSpan(workspace.Draft.ColliderInsertions)[draftEditState.Index];

                switch (draftEditState.Mode)
                {
                    case DraftEditMode.ColliderInsertionMin:
                        collider.Min = draftEditState.InitialA + delta;
                        break;
                    case DraftEditMode.ColliderInsertionMax:
                        collider.Max = draftEditState.InitialB + delta;
                        break;
                    default:
                        collider.Min = draftEditState.InitialA + delta;
                        collider.Max = draftEditState.InitialB + delta;
                        break;
                }

                NormalizeBounds(ref collider.Min, ref collider.Max);
                selection = new(SelectionKind.ColliderInsertion, draftEditState.Index);
                break;
            case DraftEditMode.MeshLinkStart:
            case DraftEditMode.MeshLinkEnd:
            case DraftEditMode.MeshLinkTranslate:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.MeshLinks.Count)
                    return;

                ref var meshLink = ref CollectionsMarshal.AsSpan(workspace.Draft.MeshLinks)[draftEditState.Index];

                switch (draftEditState.Mode)
                {
                    case DraftEditMode.MeshLinkStart:
                        meshLink.Start = draftEditState.InitialA + delta;
                        break;
                    case DraftEditMode.MeshLinkEnd:
                        meshLink.End = draftEditState.InitialB + delta;
                        break;
                    default:
                        meshLink.Start = draftEditState.InitialA + delta;
                        meshLink.End   = draftEditState.InitialB + delta;
                        break;
                }

                selection = new(SelectionKind.MeshLink, draftEditState.Index);
                break;
            case DraftEditMode.OffMeshStart:
            case DraftEditMode.OffMeshEnd:
            case DraftEditMode.OffMeshTranslate:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.OffMeshConnections.Count)
                    return;

                ref var offMesh = ref CollectionsMarshal.AsSpan(workspace.Draft.OffMeshConnections)[draftEditState.Index];

                switch (draftEditState.Mode)
                {
                    case DraftEditMode.OffMeshStart:
                        offMesh.Start = draftEditState.InitialA + delta;
                        break;
                    case DraftEditMode.OffMeshEnd:
                        offMesh.End = draftEditState.InitialB + delta;
                        break;
                    default:
                        offMesh.Start = draftEditState.InitialA + delta;
                        offMesh.End   = draftEditState.InitialB + delta;
                        break;
                }

                selection = new(SelectionKind.OffMeshConnection, draftEditState.Index);
                break;
            case DraftEditMode.InstanceTranslation:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.InstancePatches.Count)
                    return;

                ref var patch = ref CollectionsMarshal.AsSpan(workspace.Draft.InstancePatches)[draftEditState.Index];
                patch.WorldTransform.Translation = draftEditState.InitialA + delta;
                selection                        = new(SelectionKind.InstancePatch, draftEditState.Index);
                break;
        }
    }

    private static bool TryGetScreenDistance(DebugDrawer dd, Vector3 worldPosition, Vector2 screenPos, out float distance)
    {
        if (!dd.TryWorldToScreen(worldPosition, out var handlePos))
        {
            distance = float.MaxValue;
            return false;
        }

        distance = Vector2.Distance(handlePos, screenPos + ImGuiHelpers.MainViewport.Pos);
        return true;
    }

    private static bool TryRayPlaneIntersection
    (
        Vector3     rayOrigin,
        Vector3     rayDirection,
        Vector3     planeOrigin,
        Vector3     planeNormal,
        out Vector3 intersection
    )
    {
        var denom = Vector3.Dot(rayDirection, planeNormal);

        if (MathF.Abs(denom) < 0.0001f)
        {
            intersection = default;
            return false;
        }

        var distance = Vector3.Dot(planeOrigin - rayOrigin, planeNormal) / denom;

        if (distance < 0)
        {
            intersection = default;
            return false;
        }

        intersection = rayOrigin + (rayDirection * distance);
        return true;
    }

    private static bool TryGetViewportCursorPosition(out Vector2 screenPos)
    {
        if (!GetCursorPos(out var cursor))
        {
            screenPos = default;
            return false;
        }

        screenPos = new Vector2(cursor.X, cursor.Y) - ImGuiHelpers.MainViewport.Pos;
        var windowSize = ImGuiHelpers.MainViewport.Size;
        return screenPos.X >= 0 && screenPos.X <= windowSize.X && screenPos.Y >= 0 && screenPos.Y <= windowSize.Y;
    }

    private static bool RayIntersectsAabb(Vector3 origin, Vector3 direction, AABB bounds, out float distance)
    {
        var tMin = 0f;
        var tMax = float.MaxValue;

        if (!IntersectsAxis(origin.X, direction.X, bounds.Min.X, bounds.Max.X, ref tMin, ref tMax) ||
            !IntersectsAxis(origin.Y, direction.Y, bounds.Min.Y, bounds.Max.Y, ref tMin, ref tMax) ||
            !IntersectsAxis(origin.Z, direction.Z, bounds.Min.Z, bounds.Max.Z, ref tMin, ref tMax))
        {
            distance = 0f;
            return false;
        }

        distance = tMin;
        return tMax >= 0f;
    }

    private static bool IntersectsAxis(float origin, float direction, float min, float max, ref float tMin, ref float tMax)
    {
        const float EPSILON = 0.000001f;
        if (MathF.Abs(direction) < EPSILON)
            return origin >= min && origin <= max;

        var invDir = 1f             / direction;
        var t1     = (min - origin) * invDir;
        var t2     = (max - origin) * invDir;
        if (t1 > t2)
            (t1, t2) = (t2, t1);

        tMin = MathF.Max(tMin, t1);
        tMax = MathF.Min(tMax, t2);
        return tMin <= tMax;
    }

    private static void DrawPreviewInstancesOverlay(Selection selection, CustomizationPreviewBuilder previewBuilder, DebugDrawer dd)
    {
        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        if (selection is not { Kind: SelectionKind.PreviewInstance, Key: not null }   ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                       ||
            selection.Index >= mesh.Instances.Count)
            return;

        var instance = mesh.Instances[selection.Index];
        dd.DrawWorldAABB(instance.WorldBounds, 0xFFFFD94A, 3);

        if (mesh.Parts.Count > 0)
        {
            foreach (var part in mesh.Parts)
                DrawMeshPreview(part, instance.WorldTransform, dd, 0xFFFFD94A);
        }
    }

    private static void DrawPreviewVertexOverlay(Selection selection, CustomizationPreviewBuilder previewBuilder, DebugDrawer dd)
    {
        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        if (selection is not { Kind: SelectionKind.PreviewVertex, Key: not null }     ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                       ||
            selection.Index >= mesh.Parts.Count)
            return;

        var part = mesh.Parts[selection.Index];
        if (selection.SubIndex < 0 || selection.SubIndex >= part.Vertices.Count)
            return;

        var vertex = part.Vertices[selection.SubIndex];

        foreach (var instance in mesh.Instances)
        {
            var worldPos = instance.WorldTransform.TransformCoordinate(vertex);
            dd.DrawWorldPointFilled(worldPos, 6, 0xFFFFD94A);
        }
    }

    private static void DrawPreviewPrimitiveOverlay(Selection selection, CustomizationPreviewBuilder previewBuilder, DebugDrawer dd)
    {
        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        if (selection is not { Kind: SelectionKind.PreviewPrimitive, Key: not null }  ||
            !previewBuilder.Extractor.Meshes.TryGetValue(selection.Key, out var mesh) ||
            selection.Index < 0                                                       ||
            selection.Index >= mesh.Parts.Count)
            return;

        var part = mesh.Parts[selection.Index];
        if (selection.SubIndex < 0 || selection.SubIndex >= part.Primitives.Count)
            return;

        var prim = part.Primitives[selection.SubIndex];

        foreach (var instance in mesh.Instances)
        {
            var v0 = instance.WorldTransform.TransformCoordinate(part.Vertices[prim.V1]);
            var v1 = instance.WorldTransform.TransformCoordinate(part.Vertices[prim.V2]);
            var v2 = instance.WorldTransform.TransformCoordinate(part.Vertices[prim.V3]);
            dd.DrawWorldTriangle(v0, v1, v2, 0xFFFFD94A, 3);
        }
    }

    private static void DrawInstancePatchOverlay
    (
        CustomizationEditorWorkspace workspace,
        Selection                    selection,
        CustomizationPreviewBuilder  previewBuilder,
        DebugGameCollision           collision,
        DebugDrawer                  dd
    )
    {
        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        OverlaysCache.Clear();
        OverlayPoolUsed = 0;

        for (var i = 0; i < workspace.Draft.InstancePatches.Count; ++i)
        {
            var patch = workspace.Draft.InstancePatches[i];
            if (!patch.Enabled || string.IsNullOrWhiteSpace(patch.MeshKey))
                continue;

            if (patch.Kind == DraftSceneInstancePatchKind.ClearInstances) continue;

            if (!TryResolvePatchedInstance(previewBuilder.Extractor, patch, out var mesh, out var targetInstance))
                continue;

            var     key     = (patch.MeshKey, patch.InstanceId, patch.InstanceIndex);
            ref var overlay = ref CollectionsMarshal.GetValueRefOrAddDefault(OverlaysCache, key, out _);
            overlay            ??= GetOrCreateOverlay(mesh, patch.WorldTransform.ToRuntime(), targetInstance.WorldBounds);
            overlay.IsSelected |=  selection.Kind == SelectionKind.InstancePatch && selection.Index == i;

            switch (patch.Kind)
            {
                case DraftSceneInstancePatchKind.RemoveInstance:
                    overlay.HasRemove = true;
                    break;
                case DraftSceneInstancePatchKind.Transform:
                    overlay.HasTransform = true;
                    overlay.Transform    = patch.WorldTransform.ToRuntime();
                    overlay.Bounds       = CustomizationEditorSpatial.CalculateTransformedBounds(mesh.LocalBounds, overlay.Transform);
                    break;
                case DraftSceneInstancePatchKind.SetFlags:
                    overlay.HasFlags      =  true;
                    overlay.FlagSetMask   |= patch.ForceSetPrimFlags;
                    overlay.FlagClearMask |= patch.ForceClearPrimFlags;
                    overlay.Transform     =  targetInstance.WorldTransform;
                    overlay.Bounds        =  targetInstance.WorldBounds;
                    break;
            }
        }

        foreach (var overlay in OverlaysCache.Values)
        {
            if (!collision.IsBoundsWithinEditorRenderDistance(overlay.Bounds))
                continue;

            var color = overlay.IsSelected     ? 0xFFFFD94A
                        : overlay.HasRemove    ? 0xFFFF4D4D
                        : overlay.HasTransform ? 0xFF00E0FF
                                                 : 0xFF33FF66;
            var thickness = overlay.IsSelected ?
                                3 :
                                2;

            if (overlay.HasRemove) overlay.Bounds = CustomizationEditorSpatial.CalculateTransformedBounds(overlay.Mesh.LocalBounds, overlay.Transform);

            dd.DrawWorldAABB(overlay.Bounds, color, thickness);

            // 仅当被选中时，才在世界中绘制其精细的网格三角形面，防止未选中时的大量三角形坐标变换与 ImGui 顶点数据剧烈膨胀导致渲染掉帧
            if (overlay.IsSelected)
            {
                foreach (var part in overlay.Mesh.Parts)
                    DrawMeshPreview(part, overlay.Transform, dd, color, thickness);
            }

            if (overlay.HasRemove)
                DrawBoundsCross(overlay.Bounds, dd, color, thickness);

            if (overlay.HasFlags)
                DrawFlagOverlay(overlay.Bounds, overlay.FlagSetMask, overlay.FlagClearMask, dd, overlay.IsSelected);
        }

        // 清空重用池及临时缓存中的 Mesh 强引用，以防引起大对象内存泄漏
        for (var i = 0; i < OverlayPoolUsed; ++i)
        {
            OverlayPool[i].Mesh = null!;
        }

        OverlaysCache.Clear();
    }

    private static void DrawPartPatchOverlay
    (
        CustomizationEditorWorkspace workspace,
        Selection                    selection,
        CustomizationPreviewBuilder  previewBuilder,
        DebugGameCollision           collision,
        DebugDrawer                  dd
    )
    {
        if (previewBuilder.CurrentState != CustomizationPreviewBuilder.State.Ready || previewBuilder.Extractor == null)
            return;

        for (var i = 0; i < workspace.Draft.PartPatches.Count; ++i)
        {
            var patch = workspace.Draft.PartPatches[i];
            if (!patch.Enabled || string.IsNullOrWhiteSpace(patch.MeshKey))
                continue;

            if (!previewBuilder.Extractor.Meshes.TryGetValue(patch.MeshKey, out var mesh))
                continue;

            if (patch.PartIndex < 0 || patch.PartIndex >= mesh.Parts.Count)
                continue;

            var part       = mesh.Parts[patch.PartIndex];
            var isSelected = selection.Kind == SelectionKind.PartPatch && selection.Index == i;

            foreach (var instance in mesh.Instances)
            {
                if (!collision.IsBoundsWithinEditorRenderDistance(instance.WorldBounds))
                    continue;

                switch (patch.Kind)
                {
                    case DraftScenePartPatchKind.Vertex:
                        if (patch.VertexIndex >= 0 && patch.VertexIndex < part.Vertices.Count)
                        {
                            var worldPos = instance.WorldTransform.TransformCoordinate(patch.Position);
                            var color = isSelected ?
                                            0xFFFFD94A :
                                            0xFF00FF00;
                            dd.DrawWorldPointFilled
                            (
                                worldPos,
                                isSelected ?
                                    6 :
                                    4,
                                color
                            );
                        }

                        break;

                    case DraftScenePartPatchKind.PrimitiveFlags:
                    case DraftScenePartPatchKind.PrimitiveEdit:
                        if (patch.PrimitiveIndex >= 0 && patch.PrimitiveIndex < part.Primitives.Count)
                        {
                            var     prim = part.Primitives[patch.PrimitiveIndex];
                            Vector3 v0, v1, v2;

                            if (patch.Kind == DraftScenePartPatchKind.PrimitiveEdit)
                            {
                                v0 = patch.V1 >= 0 && patch.V1 < part.Vertices.Count ?
                                         part.Vertices[patch.V1] :
                                         part.Vertices[prim.V1];
                                v1 = patch.V2 >= 0 && patch.V2 < part.Vertices.Count ?
                                         part.Vertices[patch.V2] :
                                         part.Vertices[prim.V2];
                                v2 = patch.V3 >= 0 && patch.V3 < part.Vertices.Count ?
                                         part.Vertices[patch.V3] :
                                         part.Vertices[prim.V3];
                            }
                            else
                            {
                                v0 = part.Vertices[prim.V1];
                                v1 = part.Vertices[prim.V2];
                                v2 = part.Vertices[prim.V3];
                            }

                            var worldV0 = instance.WorldTransform.TransformCoordinate(v0);
                            var worldV1 = instance.WorldTransform.TransformCoordinate(v1);
                            var worldV2 = instance.WorldTransform.TransformCoordinate(v2);

                            var color = isSelected ? 0xFFFFD94A : patch.Kind == DraftScenePartPatchKind.PrimitiveEdit ? 0xFFFF00FF : 0xFF00FFFF;
                            var thickness = isSelected ?
                                                3 :
                                                2;
                            dd.DrawWorldTriangle(worldV0, worldV1, worldV2, color, thickness);
                        }

                        break;
                }
            }
        }
    }

    private static void DrawBoundsCross(AABB bounds, DebugDrawer dd, uint color, int thickness)
    {
        dd.DrawWorldLine(bounds.Min,                                    bounds.Max,                                    color, thickness);
        dd.DrawWorldLine(new(bounds.Min.X, bounds.Min.Y, bounds.Max.Z), new(bounds.Max.X, bounds.Max.Y, bounds.Min.Z), color, thickness);
        dd.DrawWorldLine(new(bounds.Min.X, bounds.Max.Y, bounds.Min.Z), new(bounds.Max.X, bounds.Min.Y, bounds.Max.Z), color, thickness);
        dd.DrawWorldLine(new(bounds.Min.X, bounds.Max.Y, bounds.Max.Z), new(bounds.Max.X, bounds.Min.Y, bounds.Min.Z), color, thickness);
    }

    private static void DrawFlagOverlay
    (
        AABB                          bounds,
        SceneExtractor.PrimitiveFlags setFlags,
        SceneExtractor.PrimitiveFlags clearFlags,
        DebugDrawer                   dd,
        bool                          isSelected
    )
    {
        var lines   = new List<string>(2);
        var setText = FormatFlagOperation("Set", setFlags);
        if (!string.IsNullOrEmpty(setText))
            lines.Add(setText);

        var clearText = FormatFlagOperation("Clear", clearFlags);
        if (!string.IsNullOrEmpty(clearText))
            lines.Add(clearText);

        if (lines.Count == 0)
            return;

        var anchor = new Vector3(bounds.Min.X, bounds.Max.Y, bounds.Min.Z);
        var color = isSelected ?
                        0xFFFFD94A :
                        0xFFFFFFFF;
        dd.DrawWorldText(anchor, string.Join("\n", lines), color);
    }

    private static string FormatFlagOperation(string prefix, SceneExtractor.PrimitiveFlags flags)
    {
        if (flags == SceneExtractor.PrimitiveFlags.None)
            return string.Empty;

        List<string> names = [];
        AppendFlagName(flags, SceneExtractor.PrimitiveFlags.ForceUnwalkable, "ForceUnwalkable", names);
        AppendFlagName(flags, SceneExtractor.PrimitiveFlags.FlyThrough,      "FlyThrough",      names);
        AppendFlagName(flags, SceneExtractor.PrimitiveFlags.Unlandable,      "Unlandable",      names);
        AppendFlagName(flags, SceneExtractor.PrimitiveFlags.ForceWalkable,   "ForceWalkable",   names);
        AppendFlagName(flags, SceneExtractor.PrimitiveFlags.Fishable,        "Fishable",        names);
        return names.Count == 0 ?
                   string.Empty :
                   $"{prefix}: {string.Join(", ", names)}";
    }

    private static void AppendFlagName
    (
        SceneExtractor.PrimitiveFlags flags,
        SceneExtractor.PrimitiveFlags target,
        string                        name,
        List<string>                  names
    )
    {
        if (flags.HasFlag(target))
            names.Add(name);
    }

    private static bool TryResolvePatchedInstance
    (
        SceneExtractor                  extractor,
        DraftSceneInstancePatch         patch,
        out SceneExtractor.Mesh         mesh,
        out SceneExtractor.MeshInstance instance
    )
    {
        mesh     = null!;
        instance = null!;

        if (!extractor.Meshes.TryGetValue(patch.MeshKey, out mesh))
            return false;

        if (patch.InstanceId != 0)
        {
            instance = mesh.Instances.FirstOrDefault(x => x.Id == patch.InstanceId)!;
            if (instance != null)
                return true;
        }

        if (patch.InstanceIndex < 0 || patch.InstanceIndex >= mesh.Instances.Count)
            return false;

        instance = mesh.Instances[patch.InstanceIndex];
        return true;
    }

    private static void DrawMeshPreview(SceneExtractor.MeshPart part, Matrix4x3 transform, DebugDrawer dd, uint color = 0xFF00FFAA, int thickness = 1)
    {
        foreach (var primitive in part.Primitives)
        {
            dd.DrawWorldTriangle
            (
                transform.TransformCoordinate(part.Vertices[primitive.V1]),
                transform.TransformCoordinate(part.Vertices[primitive.V2]),
                transform.TransformCoordinate(part.Vertices[primitive.V3]),
                color,
                thickness
            );
        }
    }

    private static readonly Dictionary<(string MeshKey, ulong InstanceId, int InstanceIndex), InstanceOverlayInfo> OverlaysCache = [];
    private static readonly List<InstanceOverlayInfo>                                                              OverlayPool   = [];
    private static          int                                                                                    OverlayPoolUsed;

    private static InstanceOverlayInfo GetOrCreateOverlay(SceneExtractor.Mesh mesh, Matrix4x3 transform, AABB bounds)
    {
        if (OverlayPoolUsed < OverlayPool.Count)
        {
            var existing = OverlayPool[OverlayPoolUsed++];
            existing.Reset(mesh, transform, bounds);
            return existing;
        }
        else
        {
            var newOverlay = new InstanceOverlayInfo();
            newOverlay.Reset(mesh, transform, bounds);
            OverlayPool.Add(newOverlay);
            OverlayPoolUsed++;
            return newOverlay;
        }
    }

    private sealed class InstanceOverlayInfo
    {
        public SceneExtractor.Mesh           Mesh = null!;
        public Matrix4x3                     Transform;
        public AABB                          Bounds;
        public bool                          HasTransform;
        public bool                          HasFlags;
        public bool                          HasRemove;
        public bool                          IsSelected;
        public SceneExtractor.PrimitiveFlags FlagSetMask;
        public SceneExtractor.PrimitiveFlags FlagClearMask;

        public void Reset(SceneExtractor.Mesh mesh, Matrix4x3 transform, AABB bounds)
        {
            Mesh          = mesh;
            Transform     = transform;
            Bounds        = bounds;
            HasTransform  = false;
            HasFlags      = false;
            HasRemove     = false;
            IsSelected    = false;
            FlagSetMask   = SceneExtractor.PrimitiveFlags.None;
            FlagClearMask = SceneExtractor.PrimitiveFlags.None;
        }
    }

    private static void DrawPendingPickPreview(PickKind pickKind, Vector3 first, Vector3 current, DebugDrawer dd)
    {
        switch (pickKind)
        {
            case PickKind.Aabb:
            case PickKind.Cylinder:
            {
                var min = Vector3.Min(first, current);
                var max = Vector3.Max(first, current);
                NormalizeBounds(ref min, ref max);
                var color = pickKind == PickKind.Cylinder ?
                                0xFF33FF66 :
                                0xFF33DDFF;
                if (pickKind == PickKind.Cylinder)
                    dd.DrawWorldCylinder((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                else
                    dd.DrawWorldAABB((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                break;
            }
            case PickKind.LinkPoints:
            case PickKind.LinkShortcut:
            case PickKind.LinkClientPath:
                dd.DrawWorldLine(first, current, 0xFFAAFF00, 2);
                break;
            case PickKind.OffMesh:
                dd.DrawWorldArc(first, current, 0.15f, 3f, 3f, 0xFFFF8800, 2);
                break;
        }
    }

    private static void NormalizeBounds(ref Vector3 min, ref Vector3 max)
    {
        if (MathF.Abs(max.Y - min.Y) < 0.1f)
        {
            var center = (min + max) * 0.5f;
            min = new(center.X - 0.5f, center.Y - 1f, center.Z - 0.5f);
            max = new(center.X + 0.5f, center.Y + 1f, center.Z + 0.5f);
        }
    }

    private static bool TakeWorldPickClick(ref bool lastPickMouseDown)
    {
        var clicked = TakeKeyPress(VK_LBUTTON, ref lastPickMouseDown);
        if (!clicked)
            return false;

        return IsWorldClickAllowed();
    }

    private static bool TakeWorldSelectClick(ref bool lastWorldSelectMouseDown)
    {
        var clicked = TakeKeyPress(VK_LBUTTON, ref lastWorldSelectMouseDown);
        if (!clicked)
            return false;

        return IsWorldClickAllowed();
    }

    private static bool IsWorldClickAllowed() =>
        !ImGui.GetIO().WantCaptureMouse && !ImGui.IsAnyItemHovered() && !ImGui.IsAnyItemActive();

    internal static bool TakeKeyPress(int virtualKey, ref bool lastDown)
    {
        var down    = IsKeyDown(virtualKey);
        var pressed = down && !lastDown;
        lastDown = down;
        return pressed;
    }

    private static bool IsKeyDown(int virtualKey) =>
        (GetAsyncKeyState(virtualKey) & 0x8000) != 0;

    internal static string GetPickKindTitle(PickKind kind) =>
        kind switch
        {
            PickKind.SelectCollider => "选中碰撞体",
            PickKind.SelectTriangle => "选中三角形",
            PickKind.Aabb           => "AABB 障碍",
            PickKind.Cylinder       => "圆柱障碍",
            PickKind.LinkPoints     => "网格连线",
            PickKind.LinkShortcut   => "普通移动捷径",
            PickKind.LinkClientPath => "客户端路径",
            PickKind.OffMesh        => "离网连接",
            _                       => "浏览"
        };

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern short GetAsyncKeyState(int virtualKey);

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool GetCursorPos(out CursorPoint point);

    private struct CursorPoint
    {
        public int X;
        public int Y;
    }
}
