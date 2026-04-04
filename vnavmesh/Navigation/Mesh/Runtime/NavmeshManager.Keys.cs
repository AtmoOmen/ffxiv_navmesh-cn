using System.Globalization;
using System.Numerics;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Mesh.Runtime;

public sealed partial class NavmeshManager
{
    private unsafe string GetCurrentKey()
    {
        var layout = LayoutWorld.Instance()->ActiveLayout;
        if (layout == null || layout->InitState != 7 || layout->FestivalStatus is > 0 and < 5)
            return "";

        var filter    = LayoutUtil.FindFilter(layout);
        var filterKey = filter != null ? filter->Key : 0;
        var terrRow   = Service.LuminaRow<TerritoryType>(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);

        if (terrRow?.TerritoryIntendedUse.RowId == 60)
        {
            var fest = layout->ActiveFestivals[0];
            if (fest.Id == 0 && fest.Phase == 0)
                return "";
        }

        var sgs = LayoutUtil.GetZoneSharedGroupsEnabled(filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId);
        return $"{terrRow?.Bg}//{filterKey:X}//{LayoutUtil.FestivalsString(layout->ActiveFestivals)}//{string.Join('.', sgs)}";
    }

    internal static unsafe string GetCacheKey(SceneDefinition scene)
    {
        var layout    = LayoutWorld.Instance()->ActiveLayout;
        var filter    = LayoutUtil.FindFilter(layout);
        var filterKey = filter != null ? filter->Key : 0;
        var terrId    = filter != null ? filter->TerritoryTypeId : layout->TerritoryTypeId;
        var terrRow   = Service.LuminaRow<TerritoryType>(terrId);
        return $"{terrRow?.Bg.ToString().Replace('/', '_')}__{filterKey:X}__{FormatHexNumbers(scene.FestivalLayers)}__{FormatHexNumbers(scene.ZoneSGs)}";
    }

    private static string FormatHexNumbers<T>(IEnumerable<T> nums) where T : INumber<T> =>
        string.Join('.', nums.Select(n => n.ToString("X", CultureInfo.InvariantCulture)));
}
