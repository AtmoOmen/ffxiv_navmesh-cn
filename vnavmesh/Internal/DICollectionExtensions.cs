using Microsoft.Extensions.DependencyInjection;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Windows;

namespace vnavmesh.Internal;

public static class DICollectionExtensions
{
    public static IServiceCollection AddPluginServices(this IServiceCollection services)
    {
        var pluginFile = new FileInfo(Service.PluginInterface.AssemblyLocation.FullName);
        var pluginDirectory = pluginFile.Directory ?? throw new InvalidOperationException("无法定位插件目录");
        services.AddSingleton(new PluginPaths(pluginDirectory, Service.PluginInterface.ConfigDirectory));
        services.AddSingleton(Service.PluginInterface.GetPluginConfig() as PluginConfig ?? new());

        services.AddSingleton(sp => new NavmeshManager(sp.GetRequiredService<PluginPaths>(), sp.GetRequiredService<PluginConfig>()));
        services.AddSingleton<MovementPlanExecutor>();
        services.AddSingleton<AsyncMoveRequest>();
        services.AddSingleton<SceneTransitionPathCleaner>();
        services.AddSingleton<PluginDTR>();
        services.AddSingleton<MainWindow>();
        services.AddSingleton<PluginWindows>();
        services.AddSingleton<PluginCommands>();
        services.AddSingleton<PluginIPC>();
        services.AddSingleton<PluginRuntime>();

        return services;
    }
}
