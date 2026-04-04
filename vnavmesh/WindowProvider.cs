using System;
using Dalamud.Interface.Windowing;
using Dalamud.Plugin;
using vnavmesh.Interface;

namespace vnavmesh;

internal sealed class WindowProvider : IDisposable
{
    private readonly WindowSystem _windowSystem = new("vnavmesh");
    private readonly IDalamudPluginInterface _pluginInterface;
    private readonly MainWindow _mainWindow;

    public WindowProvider(IDalamudPluginInterface pluginInterface, MainWindow mainWindow)
    {
        _pluginInterface = pluginInterface;
        _mainWindow = mainWindow;

        _windowSystem.AddWindow(_mainWindow);
        _pluginInterface.UiBuilder.Draw += Draw;
        _pluginInterface.UiBuilder.OpenConfigUi += OpenConfigUi;
    }

    public bool IsOpen
    {
        get => _mainWindow.IsOpen;
        set => _mainWindow.IsOpen = value;
    }

    public void Dispose()
    {
        _pluginInterface.UiBuilder.OpenConfigUi -= OpenConfigUi;
        _pluginInterface.UiBuilder.Draw -= Draw;
        _windowSystem.RemoveAllWindows();
    }

    private void OpenConfigUi()
    {
        IsOpen = true;
    }

    private void Draw()
    {
        _mainWindow.StartFrame();
        _windowSystem.Draw();
        _mainWindow.EndFrame();
    }
}
