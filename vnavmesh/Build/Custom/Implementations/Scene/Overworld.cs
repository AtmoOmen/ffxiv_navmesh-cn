using Lumina.Excel.Sheets;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Scene;

public class Overworld : SceneNavmeshCustomization
{
    public override bool Matches
    (
        SceneDefinition definition
    )
    {
        if (Service.LuminaRow<TerritoryType>(definition.TerritoryID) is not { TerritoryIntendedUse.RowId: 1 })
            return false;

        ApplyAgentRadiusOneSettings();
        return true;
    }
}
