using System.Reflection;
using Dalamud.Plugin;
using Microsoft.Extensions.DependencyInjection;
using vnavmesh.Bootstrap.Composition;

namespace vnavmesh.Bootstrap;

public sealed class Plugin : IAsyncDalamudPlugin
{
    private readonly ServiceProvider serviceProvider;

    public async Task LoadAsync(CancellationToken cancellationToken)
    {
        // 也是入口，先构造函数再调用这里
    }

    public async ValueTask DisposeAsync() =>
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
        serviceProvider.GetRequiredService<PluginRuntime>();
        
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
