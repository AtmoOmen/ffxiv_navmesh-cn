using Dalamud.Bindings.ImGui;
using FFXIVClientStructs.FFXIV.Client.Game;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.Interop;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.UI.Debug.Layout;

public unsafe partial class DebugLayout
{
    private T* FindSection<T>(FileHeader* header, uint magic) where T : unmanaged
    {
        foreach (var s in header->Sections)
        {
            if (s->Magic == magic)
                return s->Data<T>();
        }

        return null;
    }

    private void FillInstancesFromFileScene(FileSceneHeader* scene, uint filterId, Span<GameMain.Festival> festivals)
    {
        if (scene == null)
            return;

        for (var i = 0; i < scene->NumEmbeddedLayerGroups; ++i)
            FillInstancesFromFileLayerGroup(scene->EmbeddedLayerGroups.GetPointer(i), filterId, festivals);

        foreach (var off in scene->LayerGroupResourceOffsets)
        {
            var lcb = Service.DataManager.GetFile(LayoutUtil.ReadString(scene->LayerGroupResource(off)));

            if (lcb != null)
            {
                fixed (byte* lcbData = &lcb.Data[0])
                    FillInstancesFromFileLayerGroup(FindSection<FileLayerGroupHeader>((FileHeader*)lcbData, 0x3150474C), filterId, festivals);
            }
        }
    }

    private void FillInstancesFromFileLayerGroup(FileLayerGroupHeader* lg, uint filterId, Span<GameMain.Festival> festivals)
    {
        if (lg == null)
            return;

        foreach (var layerOff in lg->LayerOffsets)
        {
            var layer          = lg->Layer(layerOff);
            var expectedInGame = LayoutUtil.LayerActiveFestival(layer, festivals) && LayoutUtil.LayerActiveFilter(layer, filterId);
            FillInstancesFromFileLayer(layer, lg->Id, layer->Key, 0, 32, expectedInGame);
        }
    }

    private void FillInstancesFromFilePrefab(FileSceneHeader* scene, int layerGroupId, ushort layerId, ulong prefabKey, int subShift, bool expectedInGame)
    {
        if (scene == null || subShift < 0)
            return;
        if (scene->NumLayerGroupResources != 0)
            Service.Log.Error($"Prefab {prefabKey:X} has {scene->NumLayerGroupResources} layer group resources");

        if (scene->NumEmbeddedLayerGroups != 1)
        {
            Service.Log.Error($"Prefab {prefabKey:X} has {scene->NumEmbeddedLayerGroups} embedded layer groups");
            return;
        }

        ref var lg = ref scene->EmbeddedLayerGroups[0];

        if (lg.NumLayers != 1)
        {
            Service.Log.Error($"Prefab {prefabKey:X} has {lg.NumLayers} layers");
            return;
        }

        FillInstancesFromFileLayer(lg.Layer(lg.LayerOffsets[0]), layerGroupId, layerId, prefabKey, subShift, expectedInGame);
    }

    private void FillInstancesFromFileLayer(FileLayerGroupLayer* layer, int layerGroupId, ushort layerId, ulong prefabKey, int subShift, bool expectedInGame)
    {
        foreach (var instOffset in layer->InstanceOffsets)
        {
            var inst = layer->Instance(instOffset);
            var key  = prefabKey == 0 ? (ulong)inst->Key << 32 : prefabKey | inst->Key << subShift;

            if (_insts.ContainsKey(key))
            {
                Service.Log.Error($"Duplicate instances with key {key:X16}");
                continue;
            }

            _insts.Add
            (
                key,
                new()
                {
                    LayerGroupId       = layerGroupId,
                    LayerId            = layerId,
                    InstanceId         = (uint)(key >> 32),
                    SubId              = (uint)key,
                    Type               = inst->Type,
                    InFile             = true,
                    ExpectedToBeInGame = expectedInGame
                }
            );

            if (inst->Type is InstanceType.SharedGroup or InstanceType.HelperObject)
            {
                var instPrefab = (FileLayerGroupInstanceSharedGroup*)inst;
                var sgb        = Service.DataManager.GetFile(LayoutUtil.ReadString(instPrefab->Path));

                if (sgb != null)
                {
                    fixed (byte* sgbData = &sgb.Data[0])
                        FillInstancesFromFilePrefab
                            (FindSection<FileSceneHeader>((FileHeader*)sgbData, 0x314E4353), layerGroupId, layerId, key, subShift - 8, expectedInGame);
                }
            }
        }
    }

    private void FillInstancesFromGame(LayoutManager* layout)
    {
        foreach (var (_, ikv) in layout->InstancesByType)
        {
            foreach (var (ik, iv) in *ikv.Value)
            {
                var inst = _insts.GetValueOrDefault(ik);

                if (inst == null)
                {
                    _insts[ik] = inst = new()
                    {
                        LayerGroupId       = iv.Value->Layer->LayerGroupId,
                        LayerId            = iv.Value->Id.LayerKey,
                        InstanceId         = iv.Value->Id.InstanceKey,
                        SubId              = iv.Value->SubId,
                        Type               = iv.Value->Id.Type,
                        ExpectedToBeInGame = true
                    };
                }

                inst.Instance = iv.Value;
                inst.Collider = iv.Value->GetCollider();
            }
        }
    }

    private void DrawInstancesByLayerGroup(IEnumerable<InstanceData> insts)
    {
        if (_groupByLayerGroup)
        {
            foreach (var g in insts.GroupBy(i => i.LayerGroupId))
            {
                using var n = _tree.Node($"层级组 {g.Key}");
                if (n.Opened)
                    DrawInstancesByLayer(g);
            }
        }
        else DrawInstancesByLayer(insts);
    }

    private void DrawInstancesByLayer(IEnumerable<InstanceData> insts)
    {
        if (_groupByLayer)
        {
            foreach (var g in insts.GroupBy(i => i.LayerId))
            {
                using var n = _tree.Node($"层级 {g.Key:X}");
                if (n.Opened)
                    DrawInstancesByType(g);
            }
        }
        else DrawInstancesByType(insts);
    }

    private void DrawInstancesByType(IEnumerable<InstanceData> insts)
    {
        if (_groupByInstanceType)
        {
            foreach (var g in insts.GroupBy(i => i.Type))
            {
                using var n = _tree.Node($"类型 {g.Key}");
                if (n.Opened)
                    DrawInstancesByMaterial(g);
            }
        }
        else DrawInstancesByMaterial(insts);
    }

    private void DrawInstancesByMaterial(IEnumerable<InstanceData> insts)
    {
        if (_groupByMaterial)
        {
            foreach (var m in insts.GroupBy
                     (i =>
                         {
                             var (m1, m2) = GetMaterial(i.Instance);
                             return $"{m1:X}/{m2:X}";
                         }
                     ))
            {
                using var n = _tree.Node($"材质 {m.Key}");
                if (n.Opened)
                    DrawInstances(m);
            }
        }
        else DrawInstances(insts);
    }

    private void DrawInstances(IEnumerable<InstanceData> insts)
    {
        foreach (var inst in insts)
        {
            var nodeLabel =
                $"{inst.Type} {inst.InstanceId:X8}.{inst.SubId:X8} L{inst.LayerId:X4} LG{inst.LayerGroupId:X}: in-file={inst.InFile}, in-game={(nint)inst.Instance:X}, coll={(nint)inst.Collider:X}###{inst.InstanceId:X}.{inst.SubId:X}";
            var matches = _filterById.Length == 0 || nodeLabel.Contains(_filterById, StringComparison.InvariantCultureIgnoreCase);

            if (!matches)
                continue;

            var flags = GetFlags(inst);
            var color = ColorInstance(flags);

            using var n = _tree.Node(nodeLabel, inst.Instance == null, color);

            if (n.SelectedOrHovered)
            {
                if (inst.Collider == null)
                {
                    if (inst.Instance != null)
                    {
                        var trans = inst.Instance->GetTranslationImpl();
                        _dd.DrawWorldLine(Service.ObjectTable.LocalPlayer?.Position ?? default, *trans, 0xFFFF00FF);
                    }
                }
                else _coll.VisualizeCollider(inst.Collider, default, default);
            }

            if (n.Hovered)
                ImGui.SetTooltip(flags.ToString());
            if (!n.Opened)
                continue;

            DrawInstance("Game", inst.Instance->Layout, inst.Instance);
        }
    }
}
