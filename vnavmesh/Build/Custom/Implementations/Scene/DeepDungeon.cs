using Lumina.Excel.Sheets;
using vnavmesh.Build.Custom.Abstractions;
using vnavmesh.Build.Scene;

namespace vnavmesh.Build.Custom.Implementations.Scene;

public class DeepDungeon : SceneNavmeshCustomization
{
    public override int Version => 1;

    public override bool Matches
    (
        SceneDefinition definition
    ) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID) is { TerritoryIntendedUse.RowId: 31 };
}
