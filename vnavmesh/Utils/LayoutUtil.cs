using Dalamud.Memory;
using FFXIVClientStructs.FFXIV.Client.Game;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.Interop;
using FFXIVClientStructs.STD;
using Lumina.Excel.Sheets;
using vnavmesh.Models;

namespace vnavmesh.Utils;

public static unsafe class LayoutUtil
{
    private static delegate* unmanaged<ExdZoneSharedGroup*, uint> _getEnabledRequirementIndex;

    static LayoutUtil() =>
        _getEnabledRequirementIndex = (delegate* unmanaged<ExdZoneSharedGroup*, uint>)Service.SigScanner.ScanText("E8 ?? ?? ?? ?? 0F B6 53 6C");

    public static uint[] GetZoneSharedGroupsEnabled(uint territoryType)
    {
        var tt = Service.LuminaRow<TerritoryType>(territoryType);
        if (tt == null)
            return [];

        var rows    = tt.Value.ZoneSharedGroup.Value.ToList();
        var indices = new uint[rows.Count];

        for (var i = 0; i < rows.Count; i++)
        {
            ExdZoneSharedGroup exd = rows[i];
            indices[i] = _getEnabledRequirementIndex(&exd);
        }

        return indices;
    }

    public static string ReadString(byte* data) => data != null ? MemoryHelper.ReadStringNullTerminated((nint)data) : "";

    public static string ReadString(RefCountedString* data) => data != null ? data->DataString : "";

    public static V* FindPtr<K, V>(ref this StdMap<K, Pointer<V>> map, K key) where K : unmanaged, IComparable where V : unmanaged =>
        map.TryGetValuePointer(key, out var ptr) && ptr != null ? ptr->Value : null;

    public static ILayoutInstance* FindInstance(LayoutManager* layout, ulong key)
    {
        foreach (var (ikt, ikv) in layout->InstancesByType)
        {
            var iter = ikv.Value->FindPtr(key);
            if (iter != null)
                return iter;
        }

        return null;
    }

    public static LayoutManager.Filter* FindFilter(LayoutManager* layout)
    {
        if (layout->CfcId != 0) // note: some code paths check cfcid match only if TerritoryTypeId != 0; don't think it actually matters
        {
            foreach (var (k, v) in layout->Filters)
            {
                if (v.Value->CfcId == layout->CfcId)
                    return v.Value;
            }
        }

        if (layout->TerritoryTypeId != 0)
        {
            foreach (var (k, v) in layout->Filters)
            {
                if (v.Value->TerritoryTypeId == layout->TerritoryTypeId)
                    return v.Value;
            }
        }

        return layout->TerritoryTypeId == 0 ? layout->Filters.FindPtr(layout->LayerFilterKey) : null;
    }

    public static bool LayerActiveFestival(FileLayerGroupLayer* layer, Span<GameMain.Festival> festivals)
    {
        if (layer->Festival.Id == 0)
            return true; // non-festival, always active

        if (layer->Festival.Phase != 0)
        {
            foreach (var f in festivals)
            {
                if (f.Id == layer->Festival.Id && f.Phase == layer->Festival.Phase)
                    return true;
            }

            return false;
        }

        foreach (var f in festivals)
        {
            if (f.Id == layer->Festival.Id)
                return true;
        }

        return false;
    }

    public static bool LayerActiveFilter(FileLayerGroupLayer* layer, uint filterId)
    {
        var filter = layer->Filter;
        return filter == null ||
               filter->Operation switch
               {
                   FileLayerGroupLayerFilter.Op.Match   => filter->Entries.Contains(filterId),
                   FileLayerGroupLayerFilter.Op.NoMatch => !filter->Entries.Contains(filterId),
                   _                                    => true
               };
    }

    public static string FestivalString(GameMain.Festival f) => $"{(uint)(f.Phase << 16) | f.Id:X}";

    public static string FestivalsString
        (ReadOnlySpan<GameMain.Festival> f) => $"{FestivalString(f[0])}.{FestivalString(f[1])}.{FestivalString(f[2])}.{FestivalString(f[3])}";
}
