using System.Numerics;
using System.Runtime.InteropServices;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Client.System.Framework;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;

namespace vnavmesh.UI.Debug.Collision;

public unsafe partial class DebugGameCollision
{
    public bool TryGetMouseRaycastHit
    (
        out RaycastHit hit
    )
    {
        var module = Framework.Instance()->BGCollisionModule;

        if (!TryGetViewportCursorPosition(out var screenPos))
        {
            hit = default;
            return false;
        }

        if (module == null || module->SceneManager == null)
        {
            hit = default;
            return false;
        }

        var index        = 0;
        var found        = false;
        var best         = new RaycastHit();
        var bestDistance = float.MaxValue;

        foreach (var scene in module->SceneManager->Scenes)
        {
            if (TryRaycastScene(scene, index, screenPos, out var candidate) && candidate.Distance < bestDistance)
            {
                found        = true;
                best         = candidate;
                bestDistance = candidate.Distance;
            }

            ++index;
        }

        hit = best;
        return found;
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

    private bool TryRaycastScene
    (
        SceneWrapper*  scene,
        int            index,
        Vector2        screenPos,
        out RaycastHit hit
    )
    {
        var clipPos = new Vector3((2 * screenPos.X / _dd.ViewportSize.X) - 1, 1 - (2 * screenPos.Y / _dd.ViewportSize.Y), 1);
        Matrix4x4.Invert(_dd.ViewProj, out var invViewProj);
        var cameraPosAtPlaneP = Vector4.Transform(clipPos, invViewProj);
        var cameraPosAtPlane = new Vector3
            (cameraPosAtPlaneP.X / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Y / cameraPosAtPlaneP.W, cameraPosAtPlaneP.Z / cameraPosAtPlaneP.W);
        var   dir     = Vector3.Normalize(cameraPosAtPlane - _dd.Origin);
        float maxDist = 100000;
        var   filter  = new RaycastMaterialFilter { Mask = _materialMask.Raw, Value = _materialId.Raw };
        var   result  = new RaycastHit();
        var   sphere  = new Vector4(_dd.Origin, 1);
        var   arg     = new RaycastParams { Origin = &sphere, Direction = &dir, MaxDistance = &maxDist, MaterialFilter = &filter };

        if (scene->Raycast(&result, _shownLayers.Raw, &arg))
        {
            hit = result;
            return true;
        }

        hit = default;
        return false;
    }

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
