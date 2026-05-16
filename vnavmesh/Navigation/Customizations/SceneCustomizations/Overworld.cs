using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.SceneCustomizations;

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
