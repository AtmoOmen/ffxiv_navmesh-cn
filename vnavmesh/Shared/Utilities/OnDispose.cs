namespace vnavmesh.Shared.Utilities;

// usage: using var x = new OnDispose(action);
public readonly record struct OnDispose
(
    Action A
) : IDisposable
{
    public void Dispose() => A();
}
