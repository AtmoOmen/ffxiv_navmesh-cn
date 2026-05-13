using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Customizations.Abstractions;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.SceneCustomizations;

// PVP 区域
public class PVPDuty : SceneNavmeshCustomization
{
    public override bool Matches(SceneDefinition definition)
    {
        if (Service.LuminaRow<TerritoryType>(definition.TerritoryID) is not { IsPvpZone: true })
            return false;

        Settings.AgentRadius = 1f;
        return true;
    }
}
