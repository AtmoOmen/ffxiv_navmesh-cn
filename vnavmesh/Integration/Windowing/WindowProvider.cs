using Dalamud.Interface.Windowing;
using vnavmesh.Bootstrap;
using vnavmesh.Configuration;
using vnavmesh.UI.Windows;

namespace vnavmesh.Integration.Windowing;

internal sealed class WindowProvider : IDisposable
{
    private readonly WindowSystem _windowSystem = new("vnavmesh");
    private readonly MainWindow   _mainWindow;
    private readonly Config       _config;

    public WindowProvider(MainWindow mainWindow, Config config)
    {
        _mainWindow = mainWindow;
        _config     = config;

        _windowSystem.AddWindow(_mainWindow);
        Service.PluginInterface.UiBuilder.Draw         += Draw;
        Service.PluginInterface.UiBuilder.OpenConfigUi += OpenConfigUi;
        ApplyUiBuilderVisibilityOptions();
    }

    public bool IsOpen
    {
        get => _mainWindow.IsOpen;
        set => _mainWindow.IsOpen = value;
    }

    public void Dispose()
    {
        Service.PluginInterface.UiBuilder.OpenConfigUi -= OpenConfigUi;
        Service.PluginInterface.UiBuilder.Draw         -= Draw;
        _windowSystem.RemoveAllWindows();
    }

    private void OpenConfigUi() =>
        IsOpen = true;

    private void Draw()
    {
        ApplyUiBuilderVisibilityOptions();
        _mainWindow.StartFrame();
        _windowSystem.Draw();
        _mainWindow.EndFrame();
    }

    private void ApplyUiBuilderVisibilityOptions() =>
        Service.PluginInterface.UiBuilder.DisableUserUiHide = _config.RenderWhenGameUiHidden;
}
