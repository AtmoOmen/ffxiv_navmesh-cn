using Lumina.Excel.Sheets;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Scene;
using vnavmesh.Common.Build;

namespace vnavmesh.Build.Custom.Implementations.Scene;

public class Overworld : SceneNavmeshCustomization
{
    public override bool Matches
    (
        SceneDefinition definition
    ) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID) is { TerritoryIntendedUse.RowId: 1 };

    public override void CustomizeBuildSettings
    (
        SceneDefinition definition,
        NavmeshSettings settings
    ) =>
        settings.AgentRadius = 1f;
}
