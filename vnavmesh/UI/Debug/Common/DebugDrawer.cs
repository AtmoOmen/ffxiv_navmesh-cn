using System.Numerics;
using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility;
using FFXIVClientStructs.FFXIV.Client.Game.Control;
using FFXIVClientStructs.FFXIV.Client.Graphics.Kernel;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using SharpDX;
using vnavmesh.Common.Extensions;
using vnavmesh.UI.Rendering;
using Vector2 = System.Numerics.Vector2;
using Vector3 = System.Numerics.Vector3;
using Vector4 = System.Numerics.Vector4;

namespace vnavmesh.UI.Debug.Common;

public unsafe class DebugDrawer : IDisposable
{
    public RenderContext RenderContext { get; init; } = new();
    public RenderTarget? RenderTarget  { get; private set; }
    public EffectMesh?   EffectMesh    { get; init; }

    public Vector3   Origin;
    public Matrix4x4 View;
    public Matrix4x4 Proj;
    public Matrix4x4 ViewProj;
    public Vector4   NearPlane;
    public float     CameraAzimuth;  // facing north = 0, facing west = pi/4, facing south = +-pi/2, facing east = -pi/4
    public float     CameraAltitude; // facing horizontally = 0, facing down = pi/4, facing up = -pi/4
    public Vector2   ViewportSize;

    private List<(Vector2 from, Vector2 to, uint col, int thickness)> _viewportLines   = new();
    private List<(Vector2 center, float radius, uint color)>          _viewportCircles = new();
    private List<(Vector2 pos, uint color, string text)>              _viewportTexts   = new();

    public DebugDrawer()
    {
        try
        {
            EffectMesh = new(RenderContext);
        }
        catch (Exception ex)
        {
            Service.Log.Error($"渲染器设置失败；部分调试可视化将不可用：{ex}");
        }
    }

    public void Dispose()
    {
        EffectMesh?.Dispose();
        RenderTarget?.Dispose();
        RenderContext.Dispose();
    }

    public void StartFrame()
    {
        var controlCamera = CameraManager.Instance()->GetActiveCamera();
        var renderCamera = controlCamera != null ?
                               controlCamera->SceneCamera.RenderCamera :
                               null;

        if (renderCamera != null)
        {
            Origin   = renderCamera->Origin;
            View     = renderCamera->ViewMatrix;
            View.M44 = 1; // for whatever reason, game doesn't initialize it...
            Proj     = renderCamera->ProjectionMatrix;
            ViewProj = View * Proj;

            // note that game uses reverse-z by default, so we can't just get full plane equation by reading column 3 of vp matrix
            // so just calculate it manually: column 3 of view matrix is plane equation for a plane equation going through origin
            // proof:
            // plane equation p is such that p.dot(Q, 1) = 0 if Q lines on the plane => pw = -Q.dot(n); for view matrix, V43 is -origin.dot(forward)
            // plane equation for near plane has Q.dot(n) = O.dot(n) - near => pw = V43 + near
            NearPlane = new(View.M13, View.M23, View.M33, View.M43 + renderCamera->NearPlane);

            CameraAzimuth  = MathF.Atan2(View.M13, View.M33);
            CameraAltitude = MathF.Asin(View.M23);
            var device = Device.Instance();
            ViewportSize = new(device->Width, device->Height);
        }

        EffectMesh?.UpdateConstants(RenderContext, new() { ViewProj = ViewProj, CameraPos = Origin, LightingWorldYThreshold = 55.Degrees().Cos() });

        if (RenderTarget == null || RenderTarget.Size != ViewportSize)
        {
            RenderTarget?.Dispose();
            RenderTarget = new(RenderContext, (int)ViewportSize.X, (int)ViewportSize.Y);
        }

        RenderTarget.Bind(RenderContext);
    }

    public void EndFrame()
    {
        RenderContext.Execute();

        var dl          = ImGui.GetForegroundDrawList();
        var viewportPos = ImGuiHelpers.MainViewport.Pos;

        if (RenderTarget != null)
            dl.AddImage(RenderTarget.ImguiHandle, viewportPos, viewportPos + new Vector2(RenderTarget.Size.X, RenderTarget.Size.Y));

        foreach (var l in _viewportLines)
            dl.AddLine(l.from, l.to, l.col, l.thickness);
        foreach (var c in _viewportCircles)
            dl.AddCircleFilled(c.center, c.radius, c.color);
        foreach (var t in _viewportTexts)
            dl.AddText(t.pos, t.color, t.text);
        _viewportLines.Clear();
        _viewportCircles.Clear();
        _viewportTexts.Clear();
    }

    public void DrawWorldLine
    (
        Vector3 start,
        Vector3 end,
        uint    color,
        int     thickness = 1
    )
    {
        if (ClipLineToNearPlane(ref start, ref end))
            _viewportLines.Add((WorldToScreen(start), WorldToScreen(end), color, thickness));
    }

    public void DrawWorldPolygon
    (
        IEnumerable<Vector3> points,
        uint                 color,
        int                  thickness = 1
    )
    {
        foreach (var (a, b) in AdjacentPairs(points))
            DrawWorldLine(a, b, color, thickness);
    }

    public void DrawWorldAABB
    (
        Vector3 origin,
        Vector3 halfSize,
        uint    color,
        int     thickness = 1
    )
    {
        var min = origin - halfSize;
        var max = origin + halfSize;
        var aaa = new Vector3(min.X, min.Y, min.Z);
        var aab = new Vector3(min.X, min.Y, max.Z);
        var aba = new Vector3(min.X, max.Y, min.Z);
        var abb = new Vector3(min.X, max.Y, max.Z);
        var baa = new Vector3(max.X, min.Y, min.Z);
        var bab = new Vector3(max.X, min.Y, max.Z);
        var bba = new Vector3(max.X, max.Y, min.Z);
        var bbb = new Vector3(max.X, max.Y, max.Z);
        DrawWorldLine(aaa, aab, color, thickness);
        DrawWorldLine(aab, bab, color, thickness);
        DrawWorldLine(bab, baa, color, thickness);
        DrawWorldLine(baa, aaa, color, thickness);
        DrawWorldLine(aba, abb, color, thickness);
        DrawWorldLine(abb, bbb, color, thickness);
        DrawWorldLine(bbb, bba, color, thickness);
        DrawWorldLine(bba, aba, color, thickness);
        DrawWorldLine(aaa, aba, color, thickness);
        DrawWorldLine(aab, abb, color, thickness);
        DrawWorldLine(baa, bba, color, thickness);
        DrawWorldLine(bab, bbb, color, thickness);
    }

    public void DrawWorldAABB
    (
        AABB aabb,
        uint color,
        int  thickness = 1
    ) => DrawWorldAABB
        ((aabb.Min + aabb.Max) * 0.5f, (aabb.Max - aabb.Min) * 0.5f, color, thickness);

    public void DrawWorldCylinder
    (
        Vector3 origin,
        Vector3 halfSize,
        uint    color,
        int     thickness = 1
    )
    {
        var radius      = MathF.Max(MathF.Max(MathF.Abs(halfSize.X), MathF.Abs(halfSize.Z)), 0.01f);
        var numSegments = Math.Max(24, CurveApproxUtil.CalculateCircleSegments(radius, 360.Degrees(), 0.08f));
        var prevBottom  = origin + new Vector3(halfSize.X, -halfSize.Y, 0);
        var prevMid     = origin + new Vector3(halfSize.X, 0,           0);
        var prevTop     = origin + new Vector3(halfSize.X, halfSize.Y,  0);

        for (var i = 1; i <= numSegments; ++i)
        {
            var dir    = (i * 360.0f / numSegments).Degrees().ToDirection();
            var bottom = origin + new Vector3(dir.X * halfSize.X, -halfSize.Y, dir.Y * halfSize.Z);
            var mid    = origin + new Vector3(dir.X * halfSize.X, 0,           dir.Y * halfSize.Z);
            var top    = origin + new Vector3(dir.X * halfSize.X, halfSize.Y,  dir.Y * halfSize.Z);
            DrawWorldLine(prevBottom, bottom, color, thickness);
            DrawWorldLine(prevMid,    mid,    color, thickness);
            DrawWorldLine(prevTop,    top,    color, thickness);
            DrawWorldLine(bottom,     top,    color, thickness);
            prevBottom = bottom;
            prevMid    = mid;
            prevTop    = top;
        }
    }

    public void DrawWorldCylinder
    (
        AABB aabb,
        uint color,
        int  thickness = 1
    ) => DrawWorldCylinder
        ((aabb.Min + aabb.Max) * 0.5f, (aabb.Max - aabb.Min) * 0.5f, color, thickness);

    public void DrawWorldSphere
    (
        Vector3 center,
        float   radius,
        uint    color,
        int     thickness = 1
    )
    {
        var numSegments = CurveApproxUtil.CalculateCircleSegments(radius, 360.Degrees(), 0.1f);
        var prev1       = center + new Vector3(0,      0,      radius);
        var prev2       = center + new Vector3(0,      radius, 0);
        var prev3       = center + new Vector3(radius, 0,      0);

        for (var i = 1; i <= numSegments; ++i)
        {
            var dir   = (i * 360.0f      / numSegments).Degrees().ToDirection();
            var curr1 = center + (radius * new Vector3(dir.X, 0,     dir.Y));
            var curr2 = center + (radius * new Vector3(0,     dir.Y, dir.X));
            var curr3 = center + (radius * new Vector3(dir.Y, dir.X, 0));
            DrawWorldLine(curr1, prev1, color, thickness);
            DrawWorldLine(curr2, prev2, color, thickness);
            DrawWorldLine(curr3, prev3, color, thickness);
            prev1 = curr1;
            prev2 = curr2;
            prev3 = curr3;
        }
    }

    public void DrawWorldTriangle
    (
        Vector3 v1,
        Vector3 v2,
        Vector3 v3,
        uint    color,
        int     thickness = 1
    )
    {
        DrawWorldLine(v1, v2, color, thickness);
        DrawWorldLine(v2, v3, color, thickness);
        DrawWorldLine(v3, v1, color, thickness);
    }

    public void DrawWorldPoint
    (
        Vector3 p,
        float   radius,
        uint    color,
        int     thickness = 1
    )
    {
        if (Vector4.Dot(new(p, 1), NearPlane) >= 0)
            return;

        var ps = WorldToScreen(p);
        foreach (var (from, to) in AdjacentPairs(CurveApproxUtil.Circle(ps, radius, 1)))
            _viewportLines.Add((from, to, color, thickness));
    }

    public void DrawWorldPointFilled
    (
        Vector3 p,
        float   radius,
        uint    color
    )
    {
        if (Vector4.Dot(new(p, 1), NearPlane) >= 0)
            return;
        _viewportCircles.Add((WorldToScreen(p), radius, color));
    }

    public void DrawWorldText
    (
        Vector3 p,
        string  text,
        uint    color
    )
    {
        if (string.IsNullOrWhiteSpace(text) || Vector4.Dot(new(p, 1), NearPlane) >= 0)
            return;

        _viewportTexts.Add((WorldToScreen(p), color, text));
    }

    public bool TryWorldToScreen
    (
        Vector3     p,
        out Vector2 screen
    )
    {
        if (Vector4.Dot(new(p, 1), NearPlane) >= 0)
        {
            screen = default;
            return false;
        }

        screen = WorldToScreen(p);
        return true;
    }

    // arrow with pointer at p coming from the direction of q
    public void DrawWorldArrowPoint
    (
        Vector3 p,
        Vector3 q,
        float   l,
        uint    color,
        int     thickness = 1
    )
    {
        if (Vector4.Dot(new(p, 1), NearPlane) >= 0)
            return;

        ClipLineToNearPlane(ref p, ref q);
        var ps = WorldToScreen(p);
        var qs = WorldToScreen(q);
        var d  = Vector2.Normalize(qs - ps) * l;
        var n  = new Vector2(-d.Y, d.X)     * 0.5f;
        _viewportLines.Add((ps, ps     + d + n, color, thickness));
        _viewportLines.Add((ps, ps + d - n, color, thickness));
    }

    public void DrawWorldArc
    (
        Vector3 a,
        Vector3 b,
        float   h,
        float   arrowA,
        float   arrowB,
        uint    color,
        int     thickness = 1
    )
    {
        var delta = b - a;
        var len   = delta.Length();
        h *= len;

        Vector3 Eval
        (
            Vector3 from,
            Vector3 delta,
            float   u
        )
        {
            var res   = from + (u * delta);
            var coeff = (u        * 2) - 1;
            res.Y += h * (1            - (coeff * coeff));
            return res;
        }

        ;

        const int   NumPoints = 8;
        const float u0        = 0.05f;
        const float du        = (1.0f - (u0 * 2)) / NumPoints;
        var         from      = Eval(a, delta, u0);

        if (arrowA > 1)
            DrawWorldArrowPoint(from, Eval(a, delta, 2 * u0), arrowA, color, thickness);

        for (var i = 1; i <= NumPoints; ++i)
        {
            var to = Eval(a, delta, u0 + (i * du));
            DrawWorldLine(from, to, color, thickness);
            from = to;
        }

        if (arrowB > 1)
            DrawWorldArrowPoint(from, Eval(a, delta, 1 - (2 * u0)), arrowB, color, thickness);
    }

    private Matrix ReadMatrix
    (
        nint address
    )
    {
        var    p   = (float*)address;
        Matrix mtx = new();
        for (var i = 0; i < 16; i++)
            mtx[i] = *p++;
        return mtx;
    }

    private SharpDX.Vector2 ReadVec2
    (
        nint address
    )
    {
        var p = (float*)address;
        return new(p[0], p[1]);
    }

    private bool ClipLineToNearPlane
    (
        ref Vector3 a,
        ref Vector3 b
    )
    {
        var an = Vector4.Dot(new(a, 1), NearPlane);
        var bn = Vector4.Dot(new(b, 1), NearPlane);
        if (an >= 0 && bn >= 0)
            return false; // line fully behind near plane

        if (an > 0 || bn > 0)
        {
            var ab  = b - a;
            var abn = Vector3.Dot(ab, new(NearPlane.X, NearPlane.Y, NearPlane.Z));
            var t   = -an / abn;
            var p   = a + (t * ab);
            if (an > 0)
                a = p;
            else
                b = p;
        }

        return true;
    }

    private Vector2 WorldToScreen
    (
        Vector3 w
    )
    {
        var pp = Vector4.Transform(w, ViewProj);
        var iw = 1 / pp.W;
        return new Vector2(0.5f * ViewportSize.X * (1 + (pp.X * iw)), 0.5f * ViewportSize.Y * (1 - (pp.Y * iw))) + ImGuiHelpers.MainViewport.Pos;
    }

    private static IEnumerable<(T, T)> AdjacentPairs<T>
    (
        IEnumerable<T> v
    ) where T : struct
    {
        var en = v.GetEnumerator();
        if (!en.MoveNext())
            yield break;
        var first = en.Current;
        var from  = en.Current;

        while (en.MoveNext())
        {
            yield return (from, en.Current);
            from = en.Current;
        }

        yield return (from, first);
    }
}
