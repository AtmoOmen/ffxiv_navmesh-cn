using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.SceneDefinitions;

// 给所有副本的
public class Duty : SceneNavmeshCustomization
{
    public override bool Matches(SceneDefinition definition)
    {
        if (Service.LuminaRow<TerritoryType>(definition.TerritoryID) is not { ContentFinderCondition.RowId: > 0 })
            return false;

        Settings.CellSize    = 0.25f;
        Settings.CellHeight  = 0.125f;
        Settings.AgentRadius = 0.5f;
        return true;
    }
}
