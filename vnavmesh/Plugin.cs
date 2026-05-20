using System.Reflection;
using Dalamud.Plugin;
using Microsoft.Extensions.DependencyInjection;
using vnavmesh.Internal;
using vnavmesh.Movement;
using vnavmesh.Movement.Execution;
using vnavmesh.Navigation;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Windows;

namespace vnavmesh;

public sealed class Plugin : IDalamudPlugin
{
    private readonly ServiceProvider serviceProvider;
    
    public void Dispose() =>
        serviceProvider.Dispose();

    public Plugin(IDalamudPluginInterface dalamud)
    {
        MarkCurrentThreadAsMainThread();
        
        if (!dalamud.ConfigDirectory.Exists)
            dalamud.ConfigDirectory.Create();

        dalamud.Create<Service>();
        serviceProvider = new ServiceCollection()
                          .AddPluginServices()
                          .BuildServiceProvider
                          (
                              new ServiceProviderOptions
                              {
                                  ValidateOnBuild = true,
                                  ValidateScopes  = true
                              }
                          );
        serviceProvider.ActivatePluginServices();
        
        return;

        void MarkCurrentThreadAsMainThread()
        {
            var threadSafetyType = dalamud.GetType().Assembly.GetType("Dalamud.Utility.ThreadSafety", true);
            var markMainThread   = threadSafetyType?.GetMethod("MarkMainThread", BindingFlags.Static | BindingFlags.NonPublic);
            markMainThread?.Invoke(null, null);
        }
    }

    public static void DuoLog(Exception ex)
    {
        DuoLog(ex, ex.Message);
        throw ex;
    }

    public static void DuoLog(Exception ex, string message)
    {
        Service.ChatGui.Print($"[{Service.PluginInterface.Manifest.Name}] {message}");
        Service.Log.Error(ex, message);
    }
}

public static class DICollectionExtensions
{
    public static IServiceCollection AddPluginServices(this IServiceCollection services)
    {
        var pluginFile      = new FileInfo(Service.PluginInterface.AssemblyLocation.FullName);
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

    public static void ActivatePluginServices(this IServiceProvider serviceProvider)
    {
        serviceProvider.GetRequiredService<PluginRuntime>();
        serviceProvider.GetRequiredService<SceneTransitionPathCleaner>();
        serviceProvider.GetRequiredService<PluginWindows>();
        serviceProvider.GetRequiredService<PluginCommands>();
        serviceProvider.GetRequiredService<PluginIPC>();
    }
}
