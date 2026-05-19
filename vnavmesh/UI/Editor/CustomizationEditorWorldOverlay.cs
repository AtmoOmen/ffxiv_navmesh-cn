using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Navigation.Customizations.Editor;
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
        DrawInstancePatchOverlay(workspace, selection, previewBuilder, collision, dd);

        if (pendingPickPoint is { } pending)
        {
            dd.DrawWorldPointFilled(pending, 6, 0xFFFF00FF);
            if (currentPickPoint is { } current)
                DrawPendingPickPreview(pickKind, pending, current, dd);
        }

        foreach (var insertion in workspace.Draft.ColliderInsertions.Where(static x => x.Enabled))
        {
            var bounds = CustomizationEditorSpatial.CreateBounds(insertion.Min, insertion.Max);
            if (!collision.IsBoundsWithinEditorRenderDistance(bounds))
                continue;

            var selected = selection is { Kind: SelectionKind.ColliderInsertion, Index: var selectedIndex and >= 0 } &&
                           selectedIndex < workspace.Draft.ColliderInsertions.Count                                  &&
                           ReferenceEquals(workspace.Draft.ColliderInsertions[selectedIndex], insertion);
            var color = selected
                            ? 0xFFFFD94A
                            : insertion.Kind == DraftSceneColliderInsertionKind.Cylinder
                                ? 0xFF00FF00
                                : 0xFF00FFFF;
            if (insertion.Kind == DraftSceneColliderInsertionKind.Cylinder)
                dd.DrawWorldCylinder((insertion.Min + insertion.Max) * 0.5f, (insertion.Max - insertion.Min) * 0.5f, color, selected ? 3 : 2);
            else
                dd.DrawWorldAABB((insertion.Min + insertion.Max) * 0.5f, (insertion.Max - insertion.Min) * 0.5f, color, selected ? 3 : 2);
        }

        foreach (var link in workspace.Draft.MeshLinks.Where(static x => x.Enabled))
        {
            if (!collision.IsSegmentWithinEditorRenderDistance(link.Start, link.End))
                continue;

            dd.DrawWorldLine(link.Start, link.End, 0xFFAAFF00, 2);
        }

        foreach (var link in workspace.Draft.OffMeshConnections.Where(static x => x.Enabled))
        {
            if (!collision.IsSegmentWithinEditorRenderDistance(link.Start, link.End))
                continue;

            dd.DrawWorldArc(link.Start, link.End, 0.15f, 3f, 3f, 0xFFFF8800, 2);
        }
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

        if (hasHit && TrySelectPreviewInstance(ref selection, ref pendingLeftPanelFocusSelection, hit.Object, previewBuilder, ref statusText))
            return;

        statusText = "未选中可编辑对象";
    }

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

        var layoutObjectId = collider->LayoutObjectId << 32 | collider->LayoutObjectId >> 32;

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
        statusText                     = $"已选碰撞插入: {workspace.Draft.ColliderInsertions[selection.Index].Kind}";
        return true;
    }

    private static bool TryGetWorldSelectionRay(DebugDrawer dd, out Vector3 origin, out Vector3 direction)
    {
        origin    = default;
        direction = default;

        if (!TryGetViewportCursorPosition(out var screenPos))
            return false;

        if (dd.ViewportSize.X <= 0 || dd.ViewportSize.Y <= 0)
            return false;

        var clipPos = new Vector3(2 * screenPos.X / dd.ViewportSize.X - 1, 1 - 2 * screenPos.Y / dd.ViewportSize.Y, 1);
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

        Dictionary<(string MeshKey, ulong InstanceId, int InstanceIndex), InstanceOverlayInfo> overlays = [];

        for (var i = 0; i < workspace.Draft.InstancePatches.Count; ++i)
        {
            var patch = workspace.Draft.InstancePatches[i];
            if (!patch.Enabled || string.IsNullOrWhiteSpace(patch.MeshKey))
                continue;

            if (patch.Kind == DraftSceneInstancePatchKind.ClearInstances)
            {
                continue;
            }

            if (!TryResolvePatchedInstance(previewBuilder.Extractor, patch, out var mesh, out var targetInstance))
                continue;

            var key = (patch.MeshKey, patch.InstanceId, patch.InstanceIndex);
            ref var overlay = ref CollectionsMarshal.GetValueRefOrAddDefault(overlays, key, out _);
            overlay ??= new(mesh, patch.WorldTransform.ToRuntime(), targetInstance.WorldBounds);
            overlay.IsSelected |= selection.Kind == SelectionKind.InstancePatch && selection.Index == i;

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
                    overlay.HasFlags        = true;
                    overlay.FlagSetMask    |= patch.ForceSetPrimFlags;
                    overlay.FlagClearMask  |= patch.ForceClearPrimFlags;
                    overlay.Transform       = targetInstance.WorldTransform;
                    overlay.Bounds          = targetInstance.WorldBounds;
                    break;
            }
        }

        foreach (var overlay in overlays.Values)
        {
            if (!collision.IsBoundsWithinEditorRenderDistance(overlay.Bounds))
                continue;

            var color = overlay.IsSelected
                            ? 0xFFFFD94A
                            : overlay.HasRemove
                                ? 0xFFFF4D4D
                                : overlay.HasTransform
                                    ? 0xFF00E0FF
                                    : 0xFF33FF66;
            var thickness = overlay.IsSelected ? 3 : 2;

            if (overlay.HasRemove)
            {
                overlay.Bounds = CustomizationEditorSpatial.CalculateTransformedBounds(overlay.Mesh.LocalBounds, overlay.Transform);
            }

            dd.DrawWorldAABB(overlay.Bounds, color, thickness);
            foreach (var part in overlay.Mesh.Parts)
                DrawMeshPreview(part, overlay.Transform, dd, color, thickness);

            if (overlay.HasRemove)
                DrawBoundsCross(overlay.Bounds, dd, color, thickness);

            if (overlay.HasFlags)
                DrawFlagOverlay(overlay.Bounds, overlay.FlagSetMask, overlay.FlagClearMask, dd, overlay.IsSelected);
        }
    }

    private static void DrawBoundsCross(FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB bounds, DebugDrawer dd, uint color, int thickness)
    {
        dd.DrawWorldLine(bounds.Min, bounds.Max, color, thickness);
        dd.DrawWorldLine(new(bounds.Min.X, bounds.Min.Y, bounds.Max.Z), new(bounds.Max.X, bounds.Max.Y, bounds.Min.Z), color, thickness);
        dd.DrawWorldLine(new(bounds.Min.X, bounds.Max.Y, bounds.Min.Z), new(bounds.Max.X, bounds.Min.Y, bounds.Max.Z), color, thickness);
        dd.DrawWorldLine(new(bounds.Min.X, bounds.Max.Y, bounds.Max.Z), new(bounds.Max.X, bounds.Min.Y, bounds.Min.Z), color, thickness);
    }

    private static void DrawFlagOverlay
    (
        FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB bounds,
        SceneExtractor.PrimitiveFlags                                   setFlags,
        SceneExtractor.PrimitiveFlags                                   clearFlags,
        DebugDrawer                                                     dd,
        bool                                                            isSelected
    )
    {
        var lines = new List<string>(2);
        var setText = FormatFlagOperation("Set", setFlags);
        if (!string.IsNullOrEmpty(setText))
            lines.Add(setText);

        var clearText = FormatFlagOperation("Clear", clearFlags);
        if (!string.IsNullOrEmpty(clearText))
            lines.Add(clearText);

        if (lines.Count == 0)
            return;

        var anchor = new Vector3(bounds.Min.X, bounds.Max.Y, bounds.Min.Z);
        var color = isSelected ? 0xFFFFD94A : 0xFFFFFFFF;
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
        return names.Count == 0 ? string.Empty : $"{prefix}: {string.Join(", ", names)}";
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
        SceneExtractor          extractor,
        DraftSceneInstancePatch patch,
        out SceneExtractor.Mesh mesh,
        out SceneExtractor.MeshInstance instance
    )
    {
        mesh = null!;
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

    private sealed class InstanceOverlayInfo
    (
        SceneExtractor.Mesh mesh,
        Matrix4x3           transform,
        FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB bounds
    )
    {
        public SceneExtractor.Mesh Mesh = mesh;
        public Matrix4x3 Transform = transform;
        public FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math.AABB Bounds = bounds;
        public bool HasTransform;
        public bool HasFlags;
        public bool HasRemove;
        public bool IsSelected;
        public SceneExtractor.PrimitiveFlags FlagSetMask;
        public SceneExtractor.PrimitiveFlags FlagClearMask;
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
                var color = pickKind == PickKind.Cylinder ? 0xFF33FF66 : 0xFF33DDFF;
                if (pickKind == PickKind.Cylinder)
                    dd.DrawWorldCylinder((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                else
                    dd.DrawWorldAABB((min + max) * 0.5f, (max - min) * 0.5f, color, 2);
                break;
            }
            case PickKind.LinkPoints:
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
            PickKind.Aabb           => "AABB 障碍",
            PickKind.Cylinder       => "圆柱障碍",
            PickKind.LinkPoints     => "网格连线",
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
