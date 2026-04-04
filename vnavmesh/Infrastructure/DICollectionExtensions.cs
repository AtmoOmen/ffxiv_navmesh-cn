using Dalamud.Plugin;
using Microsoft.Extensions.DependencyInjection;
using vnavmesh.Interface;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navmesh;

namespace vnavmesh.Infrastructure;

public static class DICollectionExtensions
{
    public static IServiceCollection AddPluginServices(this IServiceCollection services)
    {
        services.AddSingleton(new PluginPaths(Service.PluginInterface.ConfigDirectory));
        services.AddSingleton(Service.PluginInterface.GetPluginConfig() as Config ?? new());

        services.AddSingleton(sp => new NavmeshManager(sp.GetRequiredService<PluginPaths>().MeshCacheDirectory, sp.GetRequiredService<Config>()));
        services.AddSingleton<MovementPlanExecutor>();
        services.AddSingleton<AsyncMoveRequest>();
        services.AddSingleton<DTRProvider>();
        services.AddSingleton<MainWindow>();
        services.AddSingleton<WindowProvider>();
        services.AddSingleton<CommandProvider>();
        services.AddSingleton<IPCProvider>();
        services.AddSingleton<PluginRuntime>();

        return services;
    }
}
