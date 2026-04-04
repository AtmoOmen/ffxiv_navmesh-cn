using System.Numerics;
using Dalamud.Plugin;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;
using vnavmesh.Utils;

namespace vnavmesh;

internal class IPCProvider : IDisposable
{
    private readonly Config       _config;
    private readonly List<Action> _disposeActions = [];

    public IPCProvider
    (
        Config                  config,
        NavmeshManager          navmeshManager,
        MovementPlanExecutor    movementExecutor,
        AsyncMoveRequest        move,
        WindowProvider          windowProvider,
        DTRProvider             dtr
    )
    {
        _config          = config;
        RegisterFunc("Nav.IsReady",               () => navmeshManager.Navmesh != null);
        RegisterFunc("Nav.BuildProgress",         () => navmeshManager.LoadTaskProgress);
        RegisterFunc("Nav.Reload",                () => navmeshManager.Reload(true));
        RegisterFunc("Nav.Rebuild",               () => navmeshManager.Reload(false));
        RegisterFunc("Nav.Pathfind",              (Vector3 from, Vector3 to, bool fly) => navmeshManager.QueryPath(from,              to, fly));
        RegisterFunc("Nav.PathfindWithTolerance", (Vector3 from, Vector3 to, bool fly, float range) => navmeshManager.QueryPath(from, to, fly, range));
        RegisterFunc
        (
            "Nav.PathfindCancelable",
            (Vector3 from, Vector3 to, bool fly, CancellationToken cancel) => navmeshManager.QueryPath(from, to, fly, externalCancel: cancel)
        );
        RegisterAction("Nav.PathfindCancelAll", () => navmeshManager.Reload(true));
        RegisterFunc("Nav.PathfindInProgress", () => navmeshManager.PathfindInProgress);
        RegisterFunc("Nav.PathfindNumQueued",  () => navmeshManager.NumQueuedPathfindRequests);
        RegisterFunc("Nav.IsAutoLoad",         () => _config.AutoLoadNavmesh);
        RegisterAction
        (
            "Nav.SetAutoLoad",
            (bool v) =>
            {
                _config.AutoLoadNavmesh = v;
                _config.Save();
            }
        );
        RegisterFunc("Nav.BuildBitmap", (Vector3 startingPos, string filename, float pixelSize) => navmeshManager.BuildBitmap(startingPos, filename, pixelSize));
        RegisterFunc
        (
            "Nav.BuildBitmapBounded",
            (
                Vector3 startingPos,
                string  filename,
                float   pixelSize,
                Vector3 minBounds,
                Vector3 maxBounds
            ) => navmeshManager.BuildBitmap
                (startingPos, filename, pixelSize, new AABB { Min = minBounds, Max = maxBounds })
        );

        RegisterFunc
        (
            "Query.Mesh.NearestPoint",
            (Vector3 p, float halfExtentXZ, float halfExtentY) => navmeshManager.Query?.FindNearestPointOnMesh(p, halfExtentXZ, halfExtentY)
        );
        RegisterFunc
        (
            "Query.Mesh.NearestPointReachable",
            (Vector3 p, float halfExtentXZ, float halfExtentY) => navmeshManager.Query?.FindNearestPointOnMesh(p, halfExtentXZ, halfExtentY, false)
        );
        RegisterFunc
        (
            "Query.Mesh.PointOnFloor",
            (Vector3 p, bool allowUnlandable, float halfExtentXZ) => navmeshManager.Query?.FindPointOnFloor(p, halfExtentXZ, allowUnlandable)
        );
        RegisterFunc("Query.Mesh.FlagToPoint", () => navmeshManager.Query is { } q ? MapUtil.FlagToPoint(q) : null);

        // 原始执行器入口：该接口直接消费外部点列，不经过算路层与后处理层。
        RegisterAction("Path.MoveTo", (List<Vector3> waypoints, bool fly) => movementExecutor.Move(waypoints, !fly));
        RegisterAction("Path.Stop",   movementExecutor.Stop);
        RegisterFunc("Path.IsRunning",          () => movementExecutor.Waypoints.Count > 0);
        RegisterFunc("Path.NumWaypoints",       () => movementExecutor.Waypoints.Count);
        RegisterFunc("Path.ListWaypoints",      () => movementExecutor.Waypoints);
        RegisterFunc("Path.GetMovementAllowed", () => movementExecutor.MovementAllowed);
        RegisterAction("Path.SetMovementAllowed", (bool v) => movementExecutor.MovementAllowed = v);
        RegisterFunc("Path.GetAlignCamera", () => _config.AlignCameraToMovement);
        RegisterAction
        (
            "Path.SetAlignCamera",
            (bool v) =>
            {
                _config.AlignCameraToMovement = v;
                _config.Save();
            }
        );
        RegisterFunc("Path.GetTolerance", () => movementExecutor.Tolerance);
        RegisterAction("Path.SetTolerance", (float v) => movementExecutor.SetNextTolerance(v));

        RegisterFunc("SimpleMove.PathfindAndMoveTo",      (Vector3 dest, bool fly) => move.MoveTo(dest,              fly));
        RegisterFunc("SimpleMove.PathfindAndMoveCloseTo", (Vector3 dest, bool fly, float range) => move.MoveTo(dest, fly, range));
        RegisterFunc("SimpleMove.PathfindInProgress",     () => move.TaskInProgress);

        RegisterFunc("Window.IsOpen", () => windowProvider.IsOpen);
        RegisterAction("Window.SetOpen", (bool v) => windowProvider.IsOpen = v);

        RegisterFunc("DTR.IsShown", () => _config.EnableDTR);
        RegisterAction
        (
            "DTR.SetShown",
            (bool v) =>
            {
                _config.EnableDTR = v;
                _config.Save();
            }
        );
    }

    public void Dispose()
    {
        foreach (var a in _disposeActions)
            a();
    }

    private void RegisterFunc<TRet>(string name, Func<TRet> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<TRet>("vnavmesh." + name);
        p.RegisterFunc(func);
        _disposeActions.Add(p.UnregisterFunc);
    }

    private void RegisterFunc<TRet, T1>(string name, Func<T1, TRet> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, TRet>("vnavmesh." + name);
        p.RegisterFunc(func);
        _disposeActions.Add(p.UnregisterFunc);
    }

    private void RegisterFunc<TRet, T1, T2>(string name, Func<T1, T2, TRet> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, T2, TRet>("vnavmesh." + name);
        p.RegisterFunc(func);
        _disposeActions.Add(p.UnregisterFunc);
    }

    private void RegisterFunc<TRet, T1, T2, T3>(string name, Func<T1, T2, T3, TRet> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, T2, T3, TRet>("vnavmesh." + name);
        p.RegisterFunc(func);
        _disposeActions.Add(p.UnregisterFunc);
    }

    private void RegisterFunc<TRet, T1, T2, T3, T4>(string name, Func<T1, T2, T3, T4, TRet> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, T2, T3, T4, TRet>("vnavmesh." + name);
        p.RegisterFunc(func);
        _disposeActions.Add(p.UnregisterFunc);
    }

    private void RegisterFunc<TRet, T1, T2, T3, T4, T5>(string name, Func<T1, T2, T3, T4, T5, TRet> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, T2, T3, T4, T5, TRet>("vnavmesh." + name);
        p.RegisterFunc(func);
        _disposeActions.Add(p.UnregisterFunc);
    }

    private void RegisterAction(string name, Action func)
    {
        var p = Service.PluginInterface.GetIpcProvider<object>("vnavmesh." + name);
        p.RegisterAction(func);
        _disposeActions.Add(p.UnregisterAction);
    }

    private void RegisterAction<T1>(string name, Action<T1> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, object>("vnavmesh." + name);
        p.RegisterAction(func);
        _disposeActions.Add(p.UnregisterAction);
    }

    private void RegisterAction<T1, T2>(string name, Action<T1, T2> func)
    {
        var p = Service.PluginInterface.GetIpcProvider<T1, T2, object>("vnavmesh." + name);
        p.RegisterAction(func);
        _disposeActions.Add(p.UnregisterAction);
    }
}
