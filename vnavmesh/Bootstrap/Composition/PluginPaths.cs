namespace vnavmesh.Bootstrap.Composition;

public sealed class PluginPaths
(
    DirectoryInfo pluginDirectory,
    DirectoryInfo configDirectory
)
{
    public DirectoryInfo PluginDirectory    { get; } = pluginDirectory;
    public DirectoryInfo ConfigDirectory    { get; } = configDirectory;
    public DirectoryInfo MeshCacheDirectory { get; } = new(Path.Combine(configDirectory.FullName, "meshcache"));
    public DirectoryInfo WorkerStateDirectory { get; } = new(Path.Combine(configDirectory.FullName, "worker"));
}
