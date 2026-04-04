using Dalamud.Plugin;
using Microsoft.Extensions.DependencyInjection;
using vnavmesh.Bootstrap.Composition;

namespace vnavmesh.Bootstrap;

public sealed class Plugin : IDalamudPlugin
{
    private readonly ServiceProvider serviceProvider;

    public Plugin(IDalamudPluginInterface dalamud)
    {
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
        serviceProvider.GetRequiredService<PluginRuntime>();
    }

    public void Dispose() =>
        serviceProvider.Dispose();

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
