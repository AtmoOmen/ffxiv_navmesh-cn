namespace vnavmesh.Common.Utils;

public struct StopWatchTimer
{
    public DateTime Start;

    public static StopWatchTimer Create() => new() { Start = DateTime.Now };

    public TimeSpan Value()
    {
        var now   = DateTime.Now;
        var delta = now - Start;
        Start = now;
        return delta;
    }
}
