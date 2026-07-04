using System.Numerics;
using FFXIVClientStructs.FFXIV.Client.UI.Agent;

namespace vnavmesh.Navigation.Scene;

public static class MapUtil
{
    public static Vector3? FlagToPoint(NavmeshQuery q)
    {
        var flag = GetFlagPosition();
        return flag == null ?
                   null :
                   q.FindPointOnFloor(new(flag.Value.X, 1024, flag.Value.Y));
    }

    private static unsafe Vector2? GetFlagPosition()
    {
        var map = AgentMap.Instance();
        if (map == null || map->FlagMarkerCount == 0)
            return null;
        var marker = map->FlagMapMarkers[0];
        return new(marker.XFloat, marker.YFloat);
    }
}
