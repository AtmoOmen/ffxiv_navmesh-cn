using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Build.Custom.Editor;
using vnavmesh.Build.Scene;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Editor.Types;
using Collider = FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Collider;

namespace vnavmesh.UI.Editor;

internal static unsafe class CustomizationEditorWorldOverlay
{
    public delegate void AddColliderInsertionDelegate
    (
        Vector3                         a,
        Vector3                         b,
        DraftSceneColliderInsertionKind kind
    );

    public delegate void AddMeshLinkDelegate
    (
        Vector3           a,
        Vector3           b,
        DraftMeshLinkKind kind
    );

    public delegate void AddOffMeshConnectionDelegate
    (
        Vector3 a,
        Vector3 b
    );

    public struct DraftEditState
    {
        public DraftEditMode Mode;
        public int           Index;
        public Vector3       PlaneOrigin;
        public Vector3       PlaneNormal;
        public Vector3       DragStartPoint;
        public Vector3       InitialA;
        public Vector3       InitialB;
        public float         InitialRotationDegrees;
    }

    public enum DraftEditMode
    {
        None,
        ColliderInsertionMin,
        ColliderInsertionMax,
        ColliderInsertionTranslate,
        OrientedCylinderStart,
        OrientedCylinderEnd,
        OrientedCylinderTranslate,
        MeshLinkStart,
        MeshLinkEnd,
        MeshLinkTranslate,
        OffMeshStart,
        OffMeshEnd,
        OffMeshTranslate,
        InstanceTranslation,
        InstanceOffset
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
            dd.DrawWorldText(pending, pickKind == PickKind.Sphere ? "中心" : "起点", 0xFFFF80FF);
            if (currentPickPoint is { } current)
            {
                dd.DrawWorldText(current, pickKind == PickKind.Sphere ? "半径" : "终点", 0xFFFFFF66);
                DrawPendingPickPreview(pickKind, pending, current, dd);
            }
        }

        for (var i = 0; i < workspace.Draft.ColliderInsertions.Count; ++i)
        {
            var insertion = workspace.Draft.ColliderInsertions[i];
            if (!insertion.Enabled)
                continue;

            var bounds = CustomizationEditorSpatial.CreateColliderBounds(insertion);
            if (!collision.IsBoundsWithinEditorRenderDistance(bounds))
                continue;

            var selected = selection is { Kind: SelectionKind.ColliderInsertion, Index: var selectedIndex and >= 0 } &&
                           selectedIndex == i;
            var color = selected                                                        ? 0xFFFFD94A
                        : insertion.Kind == DraftSceneColliderInsertionKind.Cylinder    ? 0xFF00FF00
                        : insertion.Kind == DraftSceneColliderInsertionKind.OrientedCylinder ? 0xFF33DDBB
                        : insertion.Kind == DraftSceneColliderInsertionKind.OrientedBox ? 0xFFFF66CC
                        : insertion.Kind == DraftSceneColliderInsertionKind.Sphere      ? 0xFF66CCFF
                        : insertion.Kind == DraftSceneColliderInsertionKind.Wall        ? 0xFFFF6688
                        : insertion.Kind == DraftSceneColliderInsertionKind.Ramp        ? 0xFF66FF99
                        : insertion.Kind == DraftSceneColliderInsertionKind.RemoveInstances ? 0xFFFF4D4D
                        : insertion.Kind == DraftSceneColliderInsertionKind.SetInstanceFlags ? 0xFFFFAA33
                                                                                          : 0xFF00FFFF;

            if (insertion.Kind == DraftSceneColliderInsertionKind.Cylinder)
            {
                dd.DrawWorldCylinder
                (
                    (insertion.Min + insertion.Max) * 0.5f,
                    (insertion.Max - insertion.Min) * 0.5f,
                    color,
                    selected ?
                        3 :
                        2
                );
            }
            else if (insertion.Kind == DraftSceneColliderInsertionKind.OrientedCylinder)
            {
                DrawOrientedCylinder(insertion.Start, insertion.End, insertion.Radius, dd, color, selected ? 3 : 2);
            }
            else if (insertion.Kind == DraftSceneColliderInsertionKind.Sphere)
            {
                DrawEllipsoid
                (
                    (insertion.Min + insertion.Max) * 0.5f,
                    Vector3.Abs(insertion.Max - insertion.Min) * 0.5f,
                    dd,
                    color,
                    selected ? 3 : 2
                );
            }
            else if (insertion.Kind == DraftSceneColliderInsertionKind.Ramp)
            {
                DrawRamp(insertion.Min, insertion.Max, insertion.RotationDegrees, dd, color, selected ? 3 : 2);
            }
            else if (CustomizationEditorSpatial.UsesYRotation(insertion.Kind))
            {
                DrawOrientedBox(insertion.Min, insertion.Max, insertion.RotationDegrees, dd, color, selected ? 3 : 2);
            }
            else
            {
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

            if (insertion.Kind == DraftSceneColliderInsertionKind.RemoveInstances)
                DrawBoundsCross(bounds, dd, color, selected ? 3 : 2);

            if (selected)
                dd.DrawWorldText(bounds.Max, $"{CustomizationEditorWidgets.FormatEnumDisplayName(insertion.Kind)} #{i}", color);
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

            if (selected)
                dd.DrawWorldText((link.Start + link.End) * 0.5f, $"{CustomizationEditorWidgets.FormatEnumDisplayName(link.Kind)} #{i}", color);

            if (!link.Bidirectional)
            {
                var delta = link.End - link.Start;
                var len   = delta.Length();
                if (len < 0.001f)
                    continue;

                var dir = delta / len;
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
            var delta = link.End - link.Start;
            var len   = delta.Length();

            if (len < 0.001f)
            {
                dd.DrawWorldPointFilled(link.Start, selected ? 7f : 5f, color);
                if (selected)
                    dd.DrawWorldText(link.Start, $"离网连接 #{i}", color);
                continue;
            }

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
                var h     = 0.15f * len;
                var arrowSize = selected ?
                                    20f :
                                    15f;

                Vector3 EvalArc
                (
                    float u
                )
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

            if (selected)
                dd.DrawWorldText((link.Start + link.End) * 0.5f, $"离网连接 #{i}", color);
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
        if (Vector3.DistanceSquared(first, point) < 0.01f)
        {
            statusText = $"{GetPickKindTitle(pickKind)}: 终点距离起点过近";
            return;
        }

        pendingPickPoint = null;
        var completedKind = pickKind;

        switch (pickKind)
        {
            case PickKind.Aabb:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Aabb);
                break;
            case PickKind.OrientedBox:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.OrientedBox);
                break;
            case PickKind.Cylinder:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Cylinder);
                break;
            case PickKind.OrientedCylinder:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.OrientedCylinder);
                break;
            case PickKind.Sphere:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Sphere);
                break;
            case PickKind.Wall:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Wall);
                break;
            case PickKind.Ramp:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.Ramp);
                break;
            case PickKind.RemoveInstancesVolume:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.RemoveInstances);
                break;
            case PickKind.SetInstanceFlagsVolume:
                onAddColliderInsertion(first, point, DraftSceneColliderInsertionKind.SetInstanceFlags);
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

        if (TrySelectInsertedInstance
            (ref selection, ref pendingLeftPanelFocusSelection, rayOrigin, rayDirection, ref workspace, previewBuilder, ref statusText))
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

            if (item.Kind == DraftSceneColliderInsertionKind.OrientedCylinder)
            {
                if (!RayIntersectsCylinder(rayOrigin, rayDirection, item.Start, item.End, item.Radius, out var cylinderDistance) ||
                    cylinderDistance >= bestDistance)
                    continue;

                bestDistance  = cylinderDistance;
                bestSelection = new(SelectionKind.ColliderInsertion, i);
                continue;
            }

            var bounds         = CustomizationEditorSpatial.CreateBounds(item.Min, item.Max);
            var localRayOrigin = rayOrigin;
            var localRayDirection = rayDirection;

            if (item.Kind == DraftSceneColliderInsertionKind.Sphere)
            {
                if (!RayIntersectsEllipsoid(rayOrigin, rayDirection, bounds, out var sphereDistance) || sphereDistance >= bestDistance)
                    continue;

                bestDistance  = sphereDistance;
                bestSelection = new(SelectionKind.ColliderInsertion, i);
                continue;
            }

            if (CustomizationEditorSpatial.UsesYRotation(item.Kind))
            {
                var center        = (bounds.Min + bounds.Max) * 0.5f;
                localRayOrigin    = center + CustomizationEditorSpatial.RotateAroundY(rayOrigin - center, -item.RotationDegrees);
                localRayDirection = CustomizationEditorSpatial.RotateAroundY(rayDirection, -item.RotationDegrees);
            }

            if (!RayIntersectsAabb(localRayOrigin, localRayDirection, bounds, out var distance))
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
        statusText                     = $"已选世界修改: {CustomizationEditorWidgets.FormatEnumDisplayName(workspace.Draft.ColliderInsertions[selection.Index].Kind)}";
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

    private static bool TrySelectInsertedInstance
    (
        ref Selection                    selection,
        ref Selection?                   pendingLeftPanelFocusSelection,
        Vector3                          rayOrigin,
        Vector3                          rayDirection,
        ref CustomizationEditorWorkspace workspace,
        CustomizationPreviewBuilder      previewBuilder,
        ref string                       statusText
    )
    {
        if (previewBuilder.Extractor == null)
            return false;

        var bestIndex    = -1;
        var bestDistance = float.MaxValue;
        for (var i = 0; i < workspace.Draft.InstancePatches.Count; ++i)
        {
            var patch = workspace.Draft.InstancePatches[i];
            if (!patch.Enabled || patch.Kind != DraftSceneInstancePatchKind.Insert ||
                !previewBuilder.Extractor.Meshes.TryGetValue(patch.MeshKey, out var mesh))
                continue;

            var transform = patch.WorldTransform.ToRuntime();
            var count     = Math.Clamp(patch.Count, 1, 1024);
            for (var copyIndex = 0; copyIndex < count; ++copyIndex)
            {
                var copyTransform = transform;
                copyTransform.Row3 += patch.Offset * copyIndex;
                var bounds = CustomizationEditorSpatial.CalculateTransformedBounds(mesh.LocalBounds, copyTransform);
                if (!RayIntersectsAabb(rayOrigin, rayDirection, bounds, out var distance) || distance >= bestDistance)
                    continue;

                bestIndex    = i;
                bestDistance = distance;
            }
        }

        if (bestIndex < 0)
            return false;

        selection                      = new(SelectionKind.InstancePatch, bestIndex);
        pendingLeftPanelFocusSelection = selection;
        statusText                     = $"已选复制实例: {workspace.Draft.InstancePatches[bestIndex].MeshKey}";
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

    private static float RaySegmentDistance
    (
        Vector3   rayOrigin,
        Vector3   rayDir,
        Vector3   p0,
        Vector3   p1,
        out float tOnRay,
        out float uOnSegment
    )
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

    private static bool TryGetWorldSelectionRay
    (
        DebugDrawer dd,
        out Vector3 origin,
        out Vector3 direction
    )
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

                    if (item.Kind == DraftSceneColliderInsertionKind.OrientedCylinder)
                    {
                        DrawSegmentHandles
                        (
                            item.Start,
                            item.End,
                            selection.Index,
                            DraftEditMode.OrientedCylinderStart,
                            DraftEditMode.OrientedCylinderEnd,
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
                            DraftEditMode.OrientedCylinderTranslate,
                            rayOrigin,
                            rayDirection,
                            dd,
                            screenPos,
                            ref draftEditState,
                            item.Start,
                            item.End
                        );
                        break;
                    }

                    var center    = (item.Min + item.Max) * 0.5f;
                    var minHandle = item.Min;
                    var maxHandle = item.Max;
                    if (CustomizationEditorSpatial.UsesYRotation(item.Kind))
                    {
                        minHandle = center + CustomizationEditorSpatial.RotateAroundY(item.Min - center, item.RotationDegrees);
                        maxHandle = center + CustomizationEditorSpatial.RotateAroundY(item.Max - center, item.RotationDegrees);
                    }

                    DrawHandle
                    (
                        minHandle,
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
                            PlaneOrigin = minHandle,
                            InitialA    = item.Min,
                            InitialB    = item.Max,
                            InitialRotationDegrees = item.RotationDegrees
                        },
                        "拖拽最小角点"
                    );
                    DrawHandle
                    (
                        maxHandle,
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
                            PlaneOrigin = maxHandle,
                            InitialA    = item.Min,
                            InitialB    = item.Max,
                            InitialRotationDegrees = item.RotationDegrees
                        },
                        "拖拽最大角点"
                    );
                    DrawTranslationHandle
                    (
                        center,
                        selection.Index,
                        DraftEditMode.ColliderInsertionTranslate,
                        rayOrigin,
                        rayDirection,
                        dd,
                        screenPos,
                        ref draftEditState,
                        item.Min,
                        item.Max,
                        item.RotationDegrees
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
                    if (!item.Enabled || item.Kind is not (DraftSceneInstancePatchKind.Transform or DraftSceneInstancePatchKind.Insert))
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

                    if (item.Kind == DraftSceneInstancePatchKind.Insert && item.Count > 1)
                    {
                        var count = Math.Clamp(item.Count, 1, 1024);
                        var offsetHandle = item.WorldTransform.Translation + (item.Offset * (count - 1));
                        DrawHandle
                        (
                            offsetHandle,
                            8f,
                            0xFF66FF99,
                            rayOrigin,
                            rayDirection,
                            dd,
                            screenPos,
                            ref draftEditState,
                            new()
                            {
                                Mode        = DraftEditMode.InstanceOffset,
                                Index       = selection.Index,
                                PlaneOrigin = offsetHandle,
                                InitialA    = item.Offset
                            },
                            "拖拽阵列末端"
                        );
                    }
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
        Vector3            end,
        float              rotationDegrees = 0f
    ) =>
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
                InitialRotationDegrees = rotationDegrees,
                PlaneOrigin = center
            },
            "拖拽整体位置"
        );

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
        dd.DrawWorldText(position, tooltip, 0xFFFFFFFF);
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
        statusText = $"{GetDraftEditModeTitle(draftEditState.Mode)} · 位移 {delta.X:f2}, {delta.Y:f2}, {delta.Z:f2}";

        switch (draftEditState.Mode)
        {
            case DraftEditMode.OrientedCylinderStart:
            case DraftEditMode.OrientedCylinderEnd:
            case DraftEditMode.OrientedCylinderTranslate:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.ColliderInsertions.Count)
                    return;

                ref var cylinder = ref CollectionsMarshal.AsSpan(workspace.Draft.ColliderInsertions)[draftEditState.Index];
                switch (draftEditState.Mode)
                {
                    case DraftEditMode.OrientedCylinderStart:
                        cylinder.Start = draftEditState.InitialA + delta;
                        break;
                    case DraftEditMode.OrientedCylinderEnd:
                        cylinder.End = draftEditState.InitialB + delta;
                        break;
                    default:
                        cylinder.Start = draftEditState.InitialA + delta;
                        cylinder.End   = draftEditState.InitialB + delta;
                        break;
                }

                var cylinderBounds = CustomizationEditorSpatial.CreateColliderBounds(cylinder);
                cylinder.Min = cylinderBounds.Min;
                cylinder.Max = cylinderBounds.Max;
                selection    = new(SelectionKind.ColliderInsertion, draftEditState.Index);
                break;
            case DraftEditMode.ColliderInsertionMin:
            case DraftEditMode.ColliderInsertionMax:
            case DraftEditMode.ColliderInsertionTranslate:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.ColliderInsertions.Count)
                    return;

                ref var collider = ref CollectionsMarshal.AsSpan(workspace.Draft.ColliderInsertions)[draftEditState.Index];
                var editDelta = CustomizationEditorSpatial.UsesYRotation(collider.Kind) &&
                                draftEditState.Mode != DraftEditMode.ColliderInsertionTranslate ?
                                    CustomizationEditorSpatial.RotateAroundY(delta, -draftEditState.InitialRotationDegrees) :
                                    delta;

                switch (draftEditState.Mode)
                {
                    case DraftEditMode.ColliderInsertionMin:
                        collider.Min = draftEditState.InitialA + editDelta;
                        break;
                    case DraftEditMode.ColliderInsertionMax:
                        collider.Max = draftEditState.InitialB + editDelta;
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
            case DraftEditMode.InstanceOffset:
                if (draftEditState.Index < 0 || draftEditState.Index >= workspace.Draft.InstancePatches.Count)
                    return;

                ref var arrayPatch = ref CollectionsMarshal.AsSpan(workspace.Draft.InstancePatches)[draftEditState.Index];
                var divisor = Math.Clamp(arrayPatch.Count, 2, 1024) - 1;
                arrayPatch.Offset = draftEditState.InitialA + (delta / divisor);
                selection         = new(SelectionKind.InstancePatch, draftEditState.Index);
                break;
        }
    }

    private static bool TryGetScreenDistance
    (
        DebugDrawer dd,
        Vector3     worldPosition,
        Vector2     screenPos,
        out float   distance
    )
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

    private static bool TryGetViewportCursorPosition
    (
        out Vector2 screenPos
    )
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

    private static bool RayIntersectsAabb
    (
        Vector3   origin,
        Vector3   direction,
        AABB      bounds,
        out float distance
    )
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

    private static bool RayIntersectsEllipsoid
    (
        Vector3   origin,
        Vector3   direction,
        AABB      bounds,
        out float distance
    )
    {
        var center  = (bounds.Min + bounds.Max) * 0.5f;
        var extents = Vector3.Max((bounds.Max - bounds.Min) * 0.5f, new Vector3(0.005f));
        var localOrigin    = (origin - center) / extents;
        var localDirection = direction / extents;
        var a = Vector3.Dot(localDirection, localDirection);
        var b = 2f * Vector3.Dot(localOrigin, localDirection);
        var c = Vector3.Dot(localOrigin, localOrigin) - 1f;
        var discriminant = (b * b) - (4f * a * c);
        if (discriminant < 0f || a <= 0.000001f)
        {
            distance = 0f;
            return false;
        }

        var root = MathF.Sqrt(discriminant);
        var near = (-b - root) / (2f * a);
        var far  = (-b + root) / (2f * a);
        distance = near >= 0f ? near : far;
        return distance >= 0f;
    }

    private static bool RayIntersectsCylinder
    (
        Vector3   origin,
        Vector3   direction,
        Vector3   start,
        Vector3   end,
        float     radius,
        out float distance
    )
    {
        radius = MathF.Max(MathF.Abs(radius), 0.005f);
        var axis       = end - start;
        var axisLength = axis.Length();
        if (axisLength <= 0.0001f)
        {
            axis       = Vector3.UnitY * 0.01f;
            axisLength = axis.Length();
            start     -= axis * 0.5f;
        }

        var axisDirection = axis / axisLength;
        var originFromStart = origin - start;
        var originAlongAxis = Vector3.Dot(originFromStart, axisDirection);
        var rayAlongAxis    = Vector3.Dot(direction, axisDirection);
        var radialOrigin    = originFromStart - (axisDirection * originAlongAxis);
        var radialDirection = direction       - (axisDirection * rayAlongAxis);
        var bestDistance    = float.MaxValue;
        var a = Vector3.Dot(radialDirection, radialDirection);
        var b = 2f * Vector3.Dot(radialOrigin, radialDirection);
        var c = Vector3.Dot(radialOrigin, radialOrigin) - (radius * radius);

        if (a > 0.000001f)
        {
            var discriminant = (b * b) - (4f * a * c);
            if (discriminant >= 0f)
            {
                var root = MathF.Sqrt(discriminant);
                var near = (-b - root) / (2f * a);
                var far  = (-b + root) / (2f * a);

                if (near >= 0f)
                {
                    var height = originAlongAxis + (near * rayAlongAxis);
                    if (height >= 0f && height <= axisLength)
                        bestDistance = near;
                }

                if (far >= 0f && far < bestDistance)
                {
                    var height = originAlongAxis + (far * rayAlongAxis);
                    if (height >= 0f && height <= axisLength)
                        bestDistance = far;
                }
            }
        }

        if (MathF.Abs(rayAlongAxis) > 0.000001f)
        {
            var nearCap = -originAlongAxis / rayAlongAxis;
            if (nearCap >= 0f)
            {
                var radialPoint = radialOrigin + (radialDirection * nearCap);
                if (radialPoint.LengthSquared() <= radius * radius)
                    bestDistance = MathF.Min(bestDistance, nearCap);
            }

            var farCap = (axisLength - originAlongAxis) / rayAlongAxis;
            if (farCap >= 0f)
            {
                var radialPoint = radialOrigin + (radialDirection * farCap);
                if (radialPoint.LengthSquared() <= radius * radius)
                    bestDistance = MathF.Min(bestDistance, farCap);
            }
        }

        distance = bestDistance;
        return bestDistance < float.MaxValue;
    }

    private static bool IntersectsAxis
    (
        float     origin,
        float     direction,
        float     min,
        float     max,
        ref float tMin,
        ref float tMax
    )
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

    private static void DrawPreviewInstancesOverlay
    (
        Selection                   selection,
        CustomizationPreviewBuilder previewBuilder,
        DebugDrawer                 dd
    )
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

    private static void DrawPreviewVertexOverlay
    (
        Selection                   selection,
        CustomizationPreviewBuilder previewBuilder,
        DebugDrawer                 dd
    )
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

    private static void DrawPreviewPrimitiveOverlay
    (
        Selection                   selection,
        CustomizationPreviewBuilder previewBuilder,
        DebugDrawer                 dd
    )
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

            if (patch.Kind == DraftSceneInstancePatchKind.Insert)
            {
                if (!previewBuilder.Extractor.Meshes.TryGetValue(patch.MeshKey, out var insertedMesh))
                    continue;

                var transform = patch.WorldTransform.ToRuntime();
                var count     = Math.Clamp(patch.Count, 1, 1024);
                for (var copyIndex = 0; copyIndex < count; ++copyIndex)
                {
                    var copyTransform = transform;
                    copyTransform.Row3 += patch.Offset * copyIndex;
                    var bounds = CustomizationEditorSpatial.CalculateTransformedBounds(insertedMesh.LocalBounds, copyTransform);
                    var insertedKey = (patch.MeshKey, (ulong)(copyIndex + 1), -i - 1);
                    ref var insertedOverlay = ref CollectionsMarshal.GetValueRefOrAddDefault(OverlaysCache, insertedKey, out _);
                    insertedOverlay                  ??= GetOrCreateOverlay(insertedMesh, copyTransform, bounds);
                    insertedOverlay.HasInsert        =   true;
                    insertedOverlay.DrawDetailedMesh =   count <= 32;
                    insertedOverlay.IsSelected       |=  selection.Kind == SelectionKind.InstancePatch && selection.Index == i;
                }
                continue;
            }

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
                        : overlay.HasInsert    ? 0xFF66FF99
                        : overlay.HasRemove    ? 0xFFFF4D4D
                        : overlay.HasTransform ? 0xFF00E0FF
                                                 : 0xFF33FF66;
            var thickness = overlay.IsSelected ?
                                3 :
                                2;

            if (overlay.HasRemove) overlay.Bounds = CustomizationEditorSpatial.CalculateTransformedBounds(overlay.Mesh.LocalBounds, overlay.Transform);

            dd.DrawWorldAABB(overlay.Bounds, color, thickness);

            // 仅当被选中时，才在世界中绘制其精细的网格三角形面，防止未选中时的大量三角形坐标变换与 ImGui 顶点数据剧烈膨胀导致渲染掉帧
            if (overlay.IsSelected && overlay.DrawDetailedMesh)
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
            OverlayPool[i].Mesh = null!;

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

    private static void DrawBoundsCross
    (
        AABB        bounds,
        DebugDrawer dd,
        uint        color,
        int         thickness
    )
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

    private static string FormatFlagOperation
    (
        string                        prefix,
        SceneExtractor.PrimitiveFlags flags
    )
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

    private static void DrawMeshPreview
    (
        SceneExtractor.MeshPart part,
        Matrix4x3               transform,
        DebugDrawer             dd,
        uint                    color     = 0xFF00FFAA,
        int                     thickness = 1
    )
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

    private static InstanceOverlayInfo GetOrCreateOverlay
    (
        SceneExtractor.Mesh mesh,
        Matrix4x3           transform,
        AABB                bounds
    )
    {
        if (OverlayPoolUsed < OverlayPool.Count)
        {
            var existing = OverlayPool[OverlayPoolUsed++];
            existing.Reset(mesh, transform, bounds);
            return existing;
        }

        var newOverlay = new InstanceOverlayInfo();
        newOverlay.Reset(mesh, transform, bounds);
        OverlayPool.Add(newOverlay);
        OverlayPoolUsed++;
        return newOverlay;
    }

    private sealed class InstanceOverlayInfo
    {
        public SceneExtractor.Mesh           Mesh = null!;
        public Matrix4x3                     Transform;
        public AABB                          Bounds;
        public bool                          HasTransform;
        public bool                          HasFlags;
        public bool                          HasInsert;
        public bool                          HasRemove;
        public bool                          IsSelected;
        public bool                          DrawDetailedMesh;
        public SceneExtractor.PrimitiveFlags FlagSetMask;
        public SceneExtractor.PrimitiveFlags FlagClearMask;

        public void Reset
        (
            SceneExtractor.Mesh mesh,
            Matrix4x3           transform,
            AABB                bounds
        )
        {
            Mesh          = mesh;
            Transform     = transform;
            Bounds        = bounds;
            HasTransform  = false;
            HasFlags      = false;
            HasInsert     = false;
            HasRemove     = false;
            IsSelected    = false;
            DrawDetailedMesh = true;
            FlagSetMask   = SceneExtractor.PrimitiveFlags.None;
            FlagClearMask = SceneExtractor.PrimitiveFlags.None;
        }
    }

    private static void DrawPendingPickPreview
    (
        PickKind    pickKind,
        Vector3     first,
        Vector3     current,
        DebugDrawer dd
    )
    {
        switch (pickKind)
        {
            case PickKind.Aabb:
            case PickKind.Cylinder:
            case PickKind.RemoveInstancesVolume:
            case PickKind.SetInstanceFlagsVolume:
            {
                var min = Vector3.Min(first, current);
                var max = Vector3.Max(first, current);
                NormalizeBounds(ref min, ref max);
                var color = pickKind switch
                {
                    PickKind.Cylinder                 => 0xFF33FF66,
                    PickKind.RemoveInstancesVolume    => 0xFFFF4D4D,
                    PickKind.SetInstanceFlagsVolume   => 0xFFFFAA33,
                    _                                 => 0xFF33DDFF
                };
                if (pickKind == PickKind.Cylinder)
                    dd.DrawWorldCylinder((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                else
                    dd.DrawWorldAABB((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                if (pickKind == PickKind.RemoveInstancesVolume)
                    DrawBoundsCross(new() { Min = min, Max = max }, dd, color, 2);
                var size = max - min;
                dd.DrawWorldText(max, $"尺寸 {size.X:f1} × {size.Y:f1} × {size.Z:f1}", color);
                break;
            }
            case PickKind.OrientedBox:
            {
                var center = (first + current) * 0.5f;
                var delta  = current - first;
                var length = MathF.Max(MathF.Sqrt((delta.X * delta.X) + (delta.Z * delta.Z)), 0.1f);
                var halfExtents     = new Vector3(length * 0.5f, 1f, 0.5f);
                var rotationDegrees = MathF.Atan2(-delta.Z, delta.X) * (180f / MathF.PI);
                DrawOrientedBox(center - halfExtents, center + halfExtents, rotationDegrees, dd, 0xFFFF66CC, 2);
                dd.DrawWorldText
                (
                    center + CustomizationEditorSpatial.RotateAroundY(halfExtents, rotationDegrees),
                    $"尺寸 {length:f1} × 2.0 × 1.0 · 角度 {rotationDegrees:f1}°",
                    0xFFFF66CC
                );
                break;
            }
            case PickKind.OrientedCylinder:
            {
                const float RADIUS = 0.5f;
                DrawOrientedCylinder(first, current, RADIUS, dd, 0xFF33DDBB, 2);
                dd.DrawWorldText(current, $"长 {Vector3.Distance(first, current):f1} · 半径 {RADIUS:f1}", 0xFF33DDBB);
                break;
            }
            case PickKind.Sphere:
            {
                var radius = MathF.Max(Vector3.Distance(first, current), 0.05f);
                DrawEllipsoid(first, new(radius), dd, 0xFF66CCFF, 2);
                dd.DrawWorldText(current, $"半径 {radius:f1}", 0xFF66CCFF);
                break;
            }
            case PickKind.Wall:
            {
                var center = (first + current) * 0.5f;
                var delta  = current - first;
                var length = MathF.Max(MathF.Sqrt((delta.X * delta.X) + (delta.Z * delta.Z)), 0.1f);
                var halfExtents     = new Vector3(length * 0.5f, 1f, 0.025f);
                var rotationDegrees = MathF.Atan2(-delta.Z, delta.X) * (180f / MathF.PI);
                DrawOrientedBox(center - halfExtents, center + halfExtents, rotationDegrees, dd, 0xFFFF6688, 2);
                dd.DrawWorldText(current, $"宽 {length:f1} · 高 2.0", 0xFFFF6688);
                break;
            }
            case PickKind.Ramp:
            {
                var low    = first.Y <= current.Y ? first : current;
                var high   = first.Y <= current.Y ? current : first;
                var delta  = high - low;
                var length = MathF.Max(MathF.Sqrt((delta.X * delta.X) + (delta.Z * delta.Z)), 0.1f);
                var height = MathF.Max(high.Y - low.Y, 0.5f);
                var center = new Vector3((low.X + high.X) * 0.5f, low.Y + (height * 0.5f), (low.Z + high.Z) * 0.5f);
                var halfExtents     = new Vector3(length * 0.5f, height * 0.5f, 0.75f);
                var rotationDegrees = MathF.Atan2(-delta.Z, delta.X) * (180f / MathF.PI);
                DrawRamp(center - halfExtents, center + halfExtents, rotationDegrees, dd, 0xFF66FF99, 2);
                dd.DrawWorldText(high, $"长 {length:f1} · 高 {height:f1} · 宽 1.5", 0xFF66FF99);
                break;
            }
            case PickKind.LinkPoints:
            case PickKind.LinkShortcut:
            case PickKind.LinkClientPath:
                dd.DrawWorldLine(first, current, 0xFFAAFF00, 2);
                if (Vector3.DistanceSquared(first, current) >= 0.0001f)
                    dd.DrawWorldArrowPoint(current, first, 12f, 0xFFAAFF00, 2);
                break;
            case PickKind.OffMesh:
                if (Vector3.DistanceSquared(first, current) >= 0.0001f)
                    dd.DrawWorldArc(first, current, 0.15f, 3f, 3f, 0xFFFF8800, 2);
                break;
        }
    }

    private static void DrawOrientedBox
    (
        Vector3     min,
        Vector3     max,
        float       rotationDegrees,
        DebugDrawer dd,
        uint        color,
        int         thickness
    )
    {
        var center      = (min + max) * 0.5f;
        var halfExtents = Vector3.Abs(max - min) * 0.5f;

        Vector3 ToWorld
        (
            float x,
            float y,
            float z
        ) =>
            center + CustomizationEditorSpatial.RotateAroundY(new(x, y, z), rotationDegrees);

        var aaa = ToWorld(-halfExtents.X, -halfExtents.Y, -halfExtents.Z);
        var aab = ToWorld(-halfExtents.X, -halfExtents.Y, +halfExtents.Z);
        var aba = ToWorld(-halfExtents.X, +halfExtents.Y, -halfExtents.Z);
        var abb = ToWorld(-halfExtents.X, +halfExtents.Y, +halfExtents.Z);
        var baa = ToWorld(+halfExtents.X, -halfExtents.Y, -halfExtents.Z);
        var bab = ToWorld(+halfExtents.X, -halfExtents.Y, +halfExtents.Z);
        var bba = ToWorld(+halfExtents.X, +halfExtents.Y, -halfExtents.Z);
        var bbb = ToWorld(+halfExtents.X, +halfExtents.Y, +halfExtents.Z);

        dd.DrawWorldLine(aaa, aab, color, thickness);
        dd.DrawWorldLine(aab, bab, color, thickness);
        dd.DrawWorldLine(bab, baa, color, thickness);
        dd.DrawWorldLine(baa, aaa, color, thickness);
        dd.DrawWorldLine(aba, abb, color, thickness);
        dd.DrawWorldLine(abb, bbb, color, thickness);
        dd.DrawWorldLine(bbb, bba, color, thickness);
        dd.DrawWorldLine(bba, aba, color, thickness);
        dd.DrawWorldLine(aaa, aba, color, thickness);
        dd.DrawWorldLine(aab, abb, color, thickness);
        dd.DrawWorldLine(baa, bba, color, thickness);
        dd.DrawWorldLine(bab, bbb, color, thickness);
    }

    private static void DrawEllipsoid
    (
        Vector3     center,
        Vector3     halfExtents,
        DebugDrawer dd,
        uint        color,
        int         thickness
    )
    {
        const int SEGMENTS = 32;
        var prevXZ = center + new Vector3(halfExtents.X, 0, 0);
        var prevXY = prevXZ;
        var prevYZ = center + new Vector3(0, halfExtents.Y, 0);

        for (var i = 1; i <= SEGMENTS; ++i)
        {
            var radians = i * (MathF.Tau / SEGMENTS);
            var cos     = MathF.Cos(radians);
            var sin     = MathF.Sin(radians);
            var nextXZ  = center + new Vector3(cos * halfExtents.X, 0, sin * halfExtents.Z);
            var nextXY  = center + new Vector3(cos * halfExtents.X, sin * halfExtents.Y, 0);
            var nextYZ  = center + new Vector3(0, cos * halfExtents.Y, sin * halfExtents.Z);
            dd.DrawWorldLine(prevXZ, nextXZ, color, thickness);
            dd.DrawWorldLine(prevXY, nextXY, color, thickness);
            dd.DrawWorldLine(prevYZ, nextYZ, color, thickness);
            prevXZ = nextXZ;
            prevXY = nextXY;
            prevYZ = nextYZ;
        }
    }

    private static void DrawOrientedCylinder
    (
        Vector3     start,
        Vector3     end,
        float       radius,
        DebugDrawer dd,
        uint        color,
        int         thickness
    )
    {
        const int SEGMENTS = 32;
        var axis       = end - start;
        var axisLength = axis.Length();
        var direction = axisLength > 0.0001f ?
                            axis / axisLength :
                            Vector3.UnitY;
        var reference = MathF.Abs(Vector3.Dot(direction, Vector3.UnitY)) < 0.999f ?
                            Vector3.UnitY :
                            Vector3.UnitX;
        var radialX = Vector3.Normalize(Vector3.Cross(reference, direction)) * MathF.Max(MathF.Abs(radius), 0.005f);
        var radialZ = Vector3.Normalize(Vector3.Cross(direction, radialX));
        radialZ *= radialX.Length();
        var previousStart = start + radialX;
        var previousEnd   = end   + radialX;

        for (var i = 1; i <= SEGMENTS; ++i)
        {
            var radians = i * (MathF.Tau / SEGMENTS);
            var radial  = (radialX * MathF.Cos(radians)) + (radialZ * MathF.Sin(radians));
            var nextStart = start + radial;
            var nextEnd   = end   + radial;
            dd.DrawWorldLine(previousStart, nextStart, color, thickness);
            dd.DrawWorldLine(previousEnd, nextEnd, color, thickness);
            if (i % (SEGMENTS / 4) == 0)
                dd.DrawWorldLine(nextStart, nextEnd, color, thickness);
            previousStart = nextStart;
            previousEnd   = nextEnd;
        }

        dd.DrawWorldLine(start, end, color, thickness);
    }

    private static void DrawRamp
    (
        Vector3     min,
        Vector3     max,
        float       rotationDegrees,
        DebugDrawer dd,
        uint        color,
        int         thickness
    )
    {
        var center      = (min + max) * 0.5f;
        var halfExtents = Vector3.Abs(max - min) * 0.5f;

        Vector3 ToWorld
        (
            float x,
            float y,
            float z
        ) =>
            center + CustomizationEditorSpatial.RotateAroundY(new(x, y, z), rotationDegrees);

        var lowFront   = ToWorld(-halfExtents.X, -halfExtents.Y, -halfExtents.Z);
        var lowBack    = ToWorld(-halfExtents.X, -halfExtents.Y, +halfExtents.Z);
        var baseFront  = ToWorld(+halfExtents.X, -halfExtents.Y, -halfExtents.Z);
        var baseBack   = ToWorld(+halfExtents.X, -halfExtents.Y, +halfExtents.Z);
        var highFront  = ToWorld(+halfExtents.X, +halfExtents.Y, -halfExtents.Z);
        var highBack   = ToWorld(+halfExtents.X, +halfExtents.Y, +halfExtents.Z);

        dd.DrawWorldLine(lowFront, lowBack, color, thickness);
        dd.DrawWorldLine(lowFront, baseFront, color, thickness);
        dd.DrawWorldLine(lowBack, baseBack, color, thickness);
        dd.DrawWorldLine(baseFront, baseBack, color, thickness);
        dd.DrawWorldLine(baseFront, highFront, color, thickness);
        dd.DrawWorldLine(baseBack, highBack, color, thickness);
        dd.DrawWorldLine(highFront, highBack, color, thickness);
        dd.DrawWorldLine(lowFront, highFront, color, thickness);
        dd.DrawWorldLine(lowBack, highBack, color, thickness);
    }

    private static void NormalizeBounds
    (
        ref Vector3 min,
        ref Vector3 max
    )
    {
        if (MathF.Abs(max.Y - min.Y) < 0.1f)
        {
            var center = (min + max) * 0.5f;
            min = new(center.X - 0.5f, center.Y - 1f, center.Z - 0.5f);
            max = new(center.X + 0.5f, center.Y + 1f, center.Z + 0.5f);
        }
    }

    private static bool TakeWorldPickClick
    (
        ref bool lastPickMouseDown
    )
    {
        var clicked = TakeKeyPress(VK_LBUTTON, ref lastPickMouseDown);
        if (!clicked)
            return false;

        return IsWorldClickAllowed();
    }

    private static bool TakeWorldSelectClick
    (
        ref bool lastWorldSelectMouseDown
    )
    {
        var clicked = TakeKeyPress(VK_LBUTTON, ref lastWorldSelectMouseDown);
        if (!clicked)
            return false;

        return IsWorldClickAllowed();
    }

    private static bool IsWorldClickAllowed() =>
        !ImGui.GetIO().WantCaptureMouse && !ImGui.IsAnyItemHovered() && !ImGui.IsAnyItemActive();

    internal static bool TakeKeyPress
    (
        int      virtualKey,
        ref bool lastDown
    )
    {
        var down    = IsKeyDown(virtualKey);
        var pressed = down && !lastDown;
        lastDown = down;
        return pressed;
    }

    private static bool IsKeyDown
    (
        int virtualKey
    ) =>
        (GetAsyncKeyState(virtualKey) & 0x8000) != 0;

    internal static string GetPickKindTitle
    (
        PickKind kind
    ) =>
        kind switch
        {
            PickKind.SelectCollider => "选中碰撞体",
            PickKind.SelectTriangle => "选中三角形",
            PickKind.Aabb           => "AABB 障碍",
            PickKind.OrientedBox    => "旋转箱体障碍",
            PickKind.Cylinder       => "圆柱障碍",
            PickKind.OrientedCylinder => "定向圆柱障碍",
            PickKind.Sphere         => "球形体积",
            PickKind.Wall           => "墙体",
            PickKind.Ramp           => "斜坡",
            PickKind.RemoveInstancesVolume  => "区域移除实例",
            PickKind.SetInstanceFlagsVolume => "区域标记实例",
            PickKind.LinkPoints     => "网格连线",
            PickKind.LinkShortcut   => "普通移动捷径",
            PickKind.LinkClientPath => "客户端路径",
            PickKind.OffMesh        => "离网连接",
            _                       => "浏览"
        };

    private static string GetDraftEditModeTitle
    (
        DraftEditMode mode
    ) =>
        mode switch
        {
            DraftEditMode.ColliderInsertionMin       => "调整障碍最小角",
            DraftEditMode.ColliderInsertionMax       => "调整障碍最大角",
            DraftEditMode.ColliderInsertionTranslate => "移动障碍",
            DraftEditMode.OrientedCylinderStart      => "调整定向圆柱起点",
            DraftEditMode.OrientedCylinderEnd        => "调整定向圆柱终点",
            DraftEditMode.OrientedCylinderTranslate  => "移动定向圆柱",
            DraftEditMode.MeshLinkStart              => "调整网格连接起点",
            DraftEditMode.MeshLinkEnd                => "调整网格连接终点",
            DraftEditMode.MeshLinkTranslate          => "移动网格连接",
            DraftEditMode.OffMeshStart               => "调整离网连接起点",
            DraftEditMode.OffMeshEnd                 => "调整离网连接终点",
            DraftEditMode.OffMeshTranslate           => "移动离网连接",
            DraftEditMode.InstanceTranslation        => "移动实例",
            DraftEditMode.InstanceOffset             => "调整阵列步进",
            _                                        => "编辑草稿"
        };

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern short GetAsyncKeyState
    (
        int virtualKey
    );

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool GetCursorPos
    (
        out CursorPoint point
    );

    private struct CursorPoint
    {
        public int X;
        public int Y;
    }
}
