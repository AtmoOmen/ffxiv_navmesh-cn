using Microsoft.Extensions.DependencyInjection;
using vnavmesh.Configuration;
using vnavmesh.Integration.Commands;
using vnavmesh.Integration.Ipc;
using vnavmesh.Integration.Status;
using vnavmesh.Integration.Windowing;
using vnavmesh.Movement.Execution;
using vnavmesh.Movement.Requests;
using vnavmesh.Navigation.Mesh.Runtime;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Windows;

namespace vnavmesh.Bootstrap.Composition;

public static class DICollectionExtensions
{
    public static IServiceCollection AddPluginServices(this IServiceCollection services)
    {
        services.AddSingleton(new PluginPaths(Service.PluginInterface.ConfigDirectory));
        services.AddSingleton(Service.PluginInterface.GetPluginConfig() as Config ?? new());

        services.AddSingleton(sp => new NavmeshManager(sp.GetRequiredService<PluginPaths>().MeshCacheDirectory, sp.GetRequiredService<Config>()));
        services.AddSingleton<MovementPlanExecutor>();
        services.AddSingleton<AsyncMoveRequest>();
        services.AddSingleton<SceneTransitionPathCleaner>();
        services.AddSingleton<DTRProvider>();
        services.AddSingleton<MainWindow>();
        services.AddSingleton<WindowProvider>();
        services.AddSingleton<CommandProvider>();
        services.AddSingleton<IPCProvider>();
        services.AddSingleton<PluginRuntime>();

        return services;
    }
}
