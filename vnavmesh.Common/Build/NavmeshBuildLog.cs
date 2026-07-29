namespace vnavmesh.Common.Build;

public static class NavmeshBuildLog
{
    public static Action<string> DebugSink { get; set; } = message => Console.Error.WriteLine(message);

    public static Action<string> InformationSink { get; set; } = message => Console.Error.WriteLine(message);

    public static void Debug
    (
        string message
    ) =>
        DebugSink(message);

    public static void Information
    (
        string message
    ) =>
        InformationSink(message);
}
