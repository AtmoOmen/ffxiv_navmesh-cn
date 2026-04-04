using Dalamud.Interface.Windowing;
using vnavmesh.Interface;

namespace vnavmesh;

internal sealed class WindowProvider : IDisposable
{
    private readonly WindowSystem _windowSystem = new("vnavmesh");
    private readonly MainWindow   _mainWindow;

    public WindowProvider(MainWindow mainWindow)
    {
        _mainWindow = mainWindow;

        _windowSystem.AddWindow(_mainWindow);
        Service.PluginInterface.UiBuilder.Draw         += Draw;
        Service.PluginInterface.UiBuilder.OpenConfigUi += OpenConfigUi;
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
        _mainWindow.StartFrame();
        _windowSystem.Draw();
        _mainWindow.EndFrame();
    }
}
