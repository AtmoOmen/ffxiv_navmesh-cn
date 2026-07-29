namespace vnavmesh.Build.Custom.Attributes;

[AttributeUsage(AttributeTargets.Class, AllowMultiple = true, Inherited = false)]
public sealed class CustomizationScenePriorityAboveAttribute
(
    Type lowerPriorityType
) : Attribute
{
    public Type LowerPriorityType { get; } = lowerPriorityType;
}
