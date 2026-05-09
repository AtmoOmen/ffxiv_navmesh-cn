namespace vnavmesh.Shared.Utilities;

public readonly record struct OnDispose(Action A) : IDisposable
{
    public void Dispose() => A();
}
