using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Customizations.Attributes;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.SceneDefinitions;

[CustomizationScenePriorityAbove(typeof(Duty))]
public class PVPDuty : SceneNavmeshCustomization
{
    // 主要是避免被 Duty 打到
    public override bool Matches(SceneDefinition definition) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID) is { IsPvpZone: true };
}
