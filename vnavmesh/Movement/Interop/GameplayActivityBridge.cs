using FFXIVClientStructs.FFXIV.Client.UI;

namespace vnavmesh.Movement.Interop;

internal static class GameplayActivityBridge
{
    public static unsafe void ResetAFKTime()
    {
        var module = UIModule.Instance()->GetInputTimerModule();
        module->AfkTimer          = 0;
        module->ContentInputTimer = 0;
        module->InputTimer        = 0;
    }
}