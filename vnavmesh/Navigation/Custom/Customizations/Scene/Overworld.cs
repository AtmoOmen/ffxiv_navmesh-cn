using Lumina.Excel.Sheets;
using vnavmesh.Navigation.Custom.Abstractions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Custom.Customizations.Scene;

public class Overworld : SceneNavmeshCustomization
{
    public override bool Matches(SceneDefinition definition)
    {
        if (Service.LuminaRow<TerritoryType>(definition.TerritoryID) is not { TerritoryIntendedUse.RowId: 1 })
            return false;

        ApplyAgentRadiusOneSettings();
        return true;
    }
}
