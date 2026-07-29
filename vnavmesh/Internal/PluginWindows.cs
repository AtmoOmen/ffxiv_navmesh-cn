using Dalamud.Interface.Windowing;
using vnavmesh.UI.Windows;

namespace vnavmesh.Internal;

internal sealed class PluginWindows : IDisposable
{
    private readonly WindowSystem windowSystem = new("vnavmesh");
    private readonly MainWindow   mainWindow;
    private readonly PluginConfig config;

    public PluginWindows
    (
        MainWindow   mainWindow,
        PluginConfig config
    )
    {
        this.mainWindow = mainWindow;
        this.config     = config;

        windowSystem.AddWindow(this.mainWindow);
        Service.PluginInterface.UiBuilder.Draw         += Draw;
        Service.PluginInterface.UiBuilder.OpenConfigUi += OpenConfigUi;
        ApplyUiBuilderVisibilityOptions();
    }

    public bool IsOpen
    {
        get => mainWindow.IsOpen;
        set => mainWindow.IsOpen = value;
    }

    public void Dispose()
    {
        Service.PluginInterface.UiBuilder.OpenConfigUi -= OpenConfigUi;
        Service.PluginInterface.UiBuilder.Draw         -= Draw;
        windowSystem.RemoveAllWindows();
    }

    private void OpenConfigUi() =>
        IsOpen = true;

    private void Draw()
    {
        ApplyUiBuilderVisibilityOptions();
        mainWindow.StartFrame();
        windowSystem.Draw();
        mainWindow.EndFrame();
    }

    private void ApplyUiBuilderVisibilityOptions() =>
        Service.PluginInterface.UiBuilder.DisableUserUiHide = config.RenderWhenGameUiHidden;
}
