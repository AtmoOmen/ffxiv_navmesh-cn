namespace vnavmesh.Common.Utils;

public readonly record struct OnDispose
(
    Action A
) : IDisposable
{
    public void Dispose() => A();
}
