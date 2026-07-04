using System.Numerics;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine.Layer;

namespace vnavmesh.Navigation.Scene;

// compact scene definition as extracted from the game's layout
// the goal is to fill it quickly on main thread, and then extract real data in background thread
public class SceneDefinition
{
    public uint TerritoryID;
    public uint ContentsFinderConditionID;
    public SortedSet<uint> FestivalLayers = [];
    public List<uint> ZoneSGs = [];
    public List<string> Terrains = [];
    public Dictionary<uint, (Transform transform, Vector3 bbMin, Vector3 bbMax)> AnalyticShapes = new(); // key = crc; used by bgparts
    public Dictionary<uint, string> MeshPaths = new(); // key = crc, value = pcb path; used by all colliders
    public List<(ulong key, Transform transform, uint crc, ulong matId, ulong matMask, bool analytic)> BgParts = [];
    public List<(ulong key, Transform transform, uint crc, ulong matId, ulong matMask, ColliderType type)> Colliders = [];
    public List<(ulong key, Transform transform)> ExitRanges = [];

    public unsafe void FillFromActiveLayout()
    {
        FillFromLayout(LayoutWorld.Instance()->GlobalLayout);
        FillFromLayout(LayoutWorld.Instance()->ActiveLayout);
    }

    public unsafe void FillFromLayout(LayoutManager* layout)
    {
        if (layout == null || layout->InitState != 7 || layout->FestivalStatus is > 0 and < 5)
            return;

        var filter = LayoutUtil.FindFilter(layout);
        TerritoryID = filter != null ?
                          filter->TerritoryTypeId :
                          layout->TerritoryTypeId;
        ContentsFinderConditionID = filter != null ?
                                        filter->CfcId :
                                        layout->CfcId;
        ZoneSGs.AddRange(LayoutUtil.GetZoneSharedGroupsEnabled(TerritoryID));

        foreach (var (_, v) in layout->Layers)
        {
            if (v.Value->FestivalId != 0)
                FestivalLayers.Add((uint)v.Value->FestivalSubId << 16 | v.Value->FestivalId);
        }

        foreach (var (k, v) in layout->CrcToAnalyticShapeData)
            AnalyticShapes[k.Key] = (v.Transform, v.BoundsMin, v.BoundsMax);

        foreach (var (_, v) in layout->Terrains)
            Terrains.Add($"{v.Value->PathString}/collision");

        var bgParts = layout->InstancesByType.FindPtr(InstanceType.BgPart);

        if (bgParts != null)
        {
            foreach (var (k, v) in *bgParts)
            {
                var cast = (BgPartsLayoutInstance*)v.Value;
                if ((cast->Flags3 & 0x10) == 0)
                    continue;

                if (cast->AnalyticShapeDataCrc != 0)
                {
                    BgParts.Add
                    (
                        (k, *v.Value->GetTransformImpl(), cast->AnalyticShapeDataCrc, (ulong)cast->CollisionMaterialIdHigh << 32 | cast->CollisionMaterialIdLow,
                            (ulong)cast->CollisionMaterialMaskHigh << 32 | cast->CollisionMaterialMaskLow, true)
                    );
                }
                else if (cast->CollisionMeshPathCrc != 0)
                {
                    if (!MeshPaths.ContainsKey(cast->CollisionMeshPathCrc))
                        MeshPaths[cast->CollisionMeshPathCrc] = LayoutUtil.ReadString(layout->CrcToPath.FindPtr(cast->CollisionMeshPathCrc));
                    BgParts.Add
                    (
                        (k, *v.Value->GetTransformImpl(), cast->CollisionMeshPathCrc, (ulong)cast->CollisionMaterialIdHigh << 32 | cast->CollisionMaterialIdLow,
                            (ulong)cast->CollisionMaterialMaskHigh << 32 | cast->CollisionMaterialMaskLow, false)
                    );
                }
            }
        }

        var colliders = layout->InstancesByType.FindPtr(InstanceType.CollisionBox);

        if (colliders != null)
        {
            foreach (var (k, v) in *colliders)
            {
                var cast = (CollisionBoxLayoutInstance*)v.Value;
                if ((cast->Flags3 & 0x10) == 0)
                    continue;

                if (cast->PcbPathCrc != 0 && !MeshPaths.ContainsKey(cast->PcbPathCrc))
                    MeshPaths[cast->PcbPathCrc] = LayoutUtil.ReadString(layout->CrcToPath.FindPtr(cast->PcbPathCrc));
                Colliders.Add
                (
                    (k, cast->Transform, cast->PcbPathCrc, (ulong)cast->MaterialIdHigh << 32 | cast->MaterialIdLow,
                        (ulong)cast->MaterialMaskHigh                                  << 32 | cast->MaterialMaskLow, cast->TriggerBoxLayoutInstance.Type)
                );
            }
        }

        var exitRanges = layout->InstancesByType.FindPtr(InstanceType.ExitRange);

        if (exitRanges != null)
        {
            foreach (var (k, v) in *exitRanges)
                ExitRanges.Add((k, *v.Value->GetTransformImpl()));
        }
    }
}
