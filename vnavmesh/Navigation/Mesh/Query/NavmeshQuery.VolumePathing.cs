using System.Numerics;
using vnavmesh.Bootstrap;
using vnavmesh.Movement.Planning;
using vnavmesh.Navigation.Planning;
using vnavmesh.Navigation.Volume;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Query;

public partial class NavmeshQuery
{
    internal PlannerResult PlanVolumePathDetailed(Vector3 from, Vector3 to, bool useRaycast, CancellationToken cancel)
    {
        if (VolumeQuery == null)
        {
            Service.Log.Error("体素导航体未构建，无法执行飞行算路");
            return CreateFlightFailure(to);
        }

        var locateTimer    = StopWatchTimer.Create();
        var startVoxel     = FindNearestVolumeVoxel(from);
        var endVoxel       = FindNearestVolumeVoxel(to);
        var locateDuration = locateTimer.Value();
        Service.Log.Debug($"[算路] 飞行体素 {startVoxel:X} -> {endVoxel:X}");

        if (startVoxel == VoxelMap.InvalidVoxel || endVoxel == VoxelMap.InvalidVoxel)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 无法定位空体素");
            return CreateFlightFailure(to);
        }

        var searchTimer = StopWatchTimer.Create();
        var voxelPath = VolumeQuery.FindPath
            (startVoxel, endVoxel, from, to, useRaycast, false, cancel);
        var telemetry = VolumeQuery.LastTelemetry;

        if (voxelPath.Count == 0)
        {
            Service.Log.Error($"飞行算路失败：起点 = {from:f3}，终点 = {to:f3}，体素 = {startVoxel:X} -> {endVoxel:X}，原因 = 体素路径为空");
            return CreateFlightFailure(to);
        }

        Service.Log.Debug
        (
            $"[算路] 飞行路径查询完成：空体素定位耗时 = {locateDuration.TotalSeconds:f3} 秒，主体搜索耗时 = {searchTimer.Value().TotalSeconds:f3} 秒，访问节点 = {telemetry.VisitedNodes}，生成节点 = {telemetry.GeneratedNodes}，LoS 检查 = {telemetry.LineOfSightChecks}，LoS 命中 = {telemetry.LineOfSightHits}，开放表峰值 = {telemetry.PeakOpenListSize}，路径点 = {voxelPath.Count}"
        );

        List<Vector3> rawWaypoints = new(voxelPath.Count + 1);
        foreach (var step in voxelPath)
            rawWaypoints.Add(step.p);
        rawWaypoints.Add(to);
        return new()
        {
            Status               = PathfindStatus.Complete,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = to,
            FinalDestination     = rawWaypoints[^1],
            DestinationTolerance = 0,
            Segments =
            [
                new()
                {
                    MovementMode         = MovementMode.Flight,
                    SegmentKind          = MovementSegmentKind.FlightTraverse,
                    AllowVerticalControl = true,
                    ReachabilitySource   = PathReachabilitySource.Volume,
                    GeometryKind         = PlannerSegmentGeometryKind.DiscretePoints,
                    StartPosition        = from,
                    EndPosition          = to,
                    Points               = [.. rawWaypoints]
                }
            ]
        };
    }

    private static PlannerResult CreateFlightFailure(Vector3 destination) =>
        new()
        {
            Status               = PathfindStatus.Failed,
            RequestedMode        = MovementMode.Flight,
            RequestedDestination = destination,
            FinalDestination     = destination,
            DestinationTolerance = 0
        };
}
