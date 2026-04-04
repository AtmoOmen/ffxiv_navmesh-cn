using SharpDX.Direct3D11;

namespace vnavmesh.UI.Rendering;

// device + deferred context
public class RenderContext : IDisposable
{
    public Device        Device  { get; private set; }
    public DeviceContext Context { get; private set; }

    public unsafe RenderContext()
    {
        Device  = new((nint)FFXIVClientStructs.FFXIV.Client.Graphics.Kernel.Device.Instance()->D3D11Forwarder);
        Context = new(Device);
    }

    public void Dispose() =>
        Context.Dispose();

    public void Execute()
    {
        using var cmds = Context.FinishCommandList(true);
        Device.ImmediateContext.ExecuteCommandList(cmds, true);
        Context.ClearState();
    }
}
