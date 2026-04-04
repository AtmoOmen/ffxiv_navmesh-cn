namespace vnavmesh.Bootstrap.Composition;

public sealed class PluginPaths
(
    DirectoryInfo configDirectory
)
{
    public DirectoryInfo ConfigDirectory    { get; } = configDirectory;
    public DirectoryInfo MeshCacheDirectory { get; } = new(Path.Combine(configDirectory.FullName, "meshcache"));
}
