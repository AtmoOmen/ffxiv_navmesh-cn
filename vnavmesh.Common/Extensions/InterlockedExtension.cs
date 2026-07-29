namespace vnavmesh.Common.Extensions;

public static class InterlockedExtension
{
    extension
    (
        Interlocked
    )
    {
        public static float Add
        (
            ref float location,
            float     value
        )
        {
            float initial, newValue;

            do
            {
                initial  = location;
                newValue = initial + value;
            }
            while (Math.Abs(Interlocked.CompareExchange(ref location, newValue, initial) - initial) > 0.0001f);

            return newValue;
        }
    }
}
