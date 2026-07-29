namespace vnavmesh.Build.Custom.Attributes;

// attribute that defines which territories particular customization applies to
[AttributeUsage(AttributeTargets.Class, AllowMultiple = true, Inherited = false)]
public class CustomizationTerritoryAttribute
(
    uint territoryID
) : Attribute
{
    public uint TerritoryID = territoryID;
}
