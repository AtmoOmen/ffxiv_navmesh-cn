using Dalamud.Bindings.ImGui;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine.Group;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine.Layer;
using FFXIVClientStructs.FFXIV.Client.System.Resource.Handle;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;
using FFXIVClientStructs.Interop;
using Lumina.Excel.Sheets;
using vnavmesh.Navigation.Scene;
using vnavmesh.UI.Debug.Collision;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;

namespace vnavmesh.UI.Debug.Layout;

public unsafe partial class DebugLayout : IDisposable
{
    private class InstanceData
    {
        public int              LayerGroupId;
        public ushort           LayerId;
        public uint             InstanceId;
        public uint             SubId;
        public InstanceType     Type;
        public bool             InFile;
        public bool             ExpectedToBeInGame;
        public ILayoutInstance* Instance;
        public Collider*        Collider;
    }

    private UITree                          _tree = new();
    private DebugGameCollision              _coll;
    private DebugDrawer                     _dd;
    private Dictionary<ulong, InstanceData> _insts               = new();
    private bool                            _groupByLayerGroup   = true;
    private bool                            _groupByLayer        = true;
    private bool                            _groupByInstanceType = true;
    private bool                            _groupByMaterial;
    private string                          _filterById = "";

    public DebugLayout(DebugDrawer dd, DebugGameCollision coll)
    {
        _dd   = dd;
        _coll = coll;
    }

    public void Dispose()
    {
    }

    public void Draw()
    {
        DrawWorld(LayoutWorld.Instance());
        var terr = Service.LuminaRow<TerritoryType>(Service.ClientState.TerritoryType);
        if (terr != null)
            DrawFile($"Territory {Service.ClientState.TerritoryType}", $"bg/{terr.Value.Bg}.lvb");
        DrawComparison(LayoutWorld.Instance()->ActiveLayout);
        _insts.Clear();
    }

    [Flags]
    private enum InstanceFlags
    {
        None           = 0,
        InFile         = 1 << 0,
        InGame         = 1 << 1,
        InGameMismatch = 1 << 2,
        HasCollider    = 1 << 3
    }

    private static InstanceFlags GetFlags(InstanceData inst)
    {
        var flags = InstanceFlags.None;
        if (inst.InFile)
            flags |= InstanceFlags.InFile;
        var inGame = inst.Instance != null;
        if (inGame)
            flags |= InstanceFlags.InGame;
        if (inGame != inst.ExpectedToBeInGame)
            flags |= InstanceFlags.InGameMismatch;
        if (inst.Collider != null)
            flags |= InstanceFlags.HasCollider;
        return flags;
    }

    private static uint ColorInstance(InstanceFlags flags)
    {
        if (!flags.HasFlag(InstanceFlags.InFile))
            return 0xFF0000FF;

        if (flags.HasFlag(InstanceFlags.InGameMismatch))
            return 0xFFFF00FF;

        if (!flags.HasFlag(InstanceFlags.InGame))
            return 0xFF00FFFF;

        if (!flags.HasFlag(InstanceFlags.HasCollider))
            return 0xFF00FF00;

        return 0xFFFFFFFF;
    }

    public static bool DrawInstance(UITree tree, string tag, LayoutManager* layout, ILayoutInstance* inst, DebugGameCollision coll)
    {
        using var ni = tree.Node
        (
            $"{tag} {inst->Id.Type} L{inst->Id.LayerKey:X4} I{inst->Id.InstanceKey:X8}.{inst->SubId:X8} ({inst->Id.u0:X2}) = {(nint)inst:X}, pool-idx={inst->IndexInPool}, prefab-index={inst->IndexInPrefab}, nesting={inst->NestingLevel}, u29low={inst->Flags1 & 0xF}, u29hi={inst->Flags1 >> 7 != 0}, flags={inst->Flags2:X2} {inst->Flags3:X2}###{tag}"
        );
        var collider = inst->GetCollider();

        if (ni.Opened)
        {
            tree.LeafNode($"Primary: {inst->HavePrimary()} '{LayoutUtil.ReadString(inst->GetPrimaryPath())}'");
            tree.LeafNode($"Secondary: {inst->HaveSecondary()} '{LayoutUtil.ReadString(inst->GetSecondaryPath())}'");
            tree.LeafNode($"Translation: {*inst->GetTranslationImpl()}");
            tree.LeafNode($"Rotation: {*inst->GetRotationImpl()}");
            tree.LeafNode($"Scale: {*inst->GetScaleImpl()}");
            tree.LeafNode($"Graphics: {(nint)inst->GetGraphics():X}");
            tree.LeafNode($"Collider: {(nint)collider:X} (loaded={inst->IsColliderLoaded()}, active={inst->IsColliderActive()})");
            tree.LeafNode($"Want to be active: {inst->WantToBeActive()}");

            switch (inst->Id.Type)
            {
                case InstanceType.BgPart:
                    var instBgPart = (BgPartsLayoutInstance*)inst;
                    tree.LeafNode($"Gfx obj: {(nint)instBgPart->GraphicsObject:X}");
                    tree.LeafNode($"Collider: {(nint)instBgPart->Collider:X} ({instBgPart->CollisionMeshPathCrc:X8} / {instBgPart->AnalyticShapeDataCrc:X8})");

                    if (instBgPart->CollisionMeshPathCrc != 0)
                    {
                        foreach (var (k, v) in layout->CrcToPath)
                        {
                            if (k == instBgPart->CollisionMeshPathCrc)
                            {
                                tree.LeafNode($"Collider path: {v.Value->DataString}");
                                break;
                            }
                        }
                    }

                    if (instBgPart->AnalyticShapeDataCrc != 0)
                    {
                        foreach (var (k, v) in layout->CrcToAnalyticShapeData)
                        {
                            if (k.Key == instBgPart->AnalyticShapeDataCrc)
                            {
                                DrawAnalyticShape(tree, "Shape data:", v);
                                break;
                            }
                        }
                    }

                    tree.LeafNode
                    (
                        $"Collision material: {instBgPart->CollisionMaterialIdHigh:X8}{instBgPart->CollisionMaterialIdLow:X8} / {instBgPart->CollisionMaterialMaskHigh:X8}{instBgPart->CollisionMaterialMaskLow:X8}"
                    );
                    //tree.LeafNode($"unks: {instBgPart->u58} {instBgPart->u5C:X}");
                    break;
                case InstanceType.SharedGroup:
                case InstanceType.HelperObject:
                    var instPrefab = (SharedGroupLayoutInstance*)inst;
                    tree.LeafNode($"Resource: {(instPrefab->ResourceHandle != null ? instPrefab->ResourceHandle->FileName : "<null>")}");
                    tree.LeafNode($"Flags: {instPrefab->PrefabFlags1:X8} {instPrefab->PrefabFlags2:X8}");

                    using (var nc = tree.Node($"Instances ({instPrefab->Instances.Instances.Count})###instances", instPrefab->Instances.Instances.Count == 0))
                    {
                        if (nc.Opened)
                        {
                            var index = 0;

                            foreach (var part in instPrefab->Instances.Instances)
                            {
                                if (DrawInstance(tree, $"[{index++}]", layout, part.Value->Instance, coll))
                                    coll.VisualizeCollider(part.Value->Instance->GetCollider(), default, default);
                            }
                        }
                    }

                    //using (var nc = tree.Node($"uA8 ({instPrefab->uA8.Instances.Size()})###a8", instPrefab->uA8.Instances.Size() == 0))
                    //{
                    //    if (nc.Opened)
                    //    {
                    //        int index = 0;
                    //        foreach (var part in instPrefab->uA8.Instances.Span)
                    //        {
                    //            DrawInstance(tree, $"[{index++}]", layout, part.Value->Instance);
                    //        }
                    //    }
                    //}
                    break;
                case InstanceType.CollisionBox:
                    var instCollGeneric = (CollisionBoxLayoutInstance*)inst;
                    var pcbPath = instCollGeneric->PcbPathCrc != 0 ?
                                      layout->CrcToPath.FindPtr(instCollGeneric->PcbPathCrc) :
                                      null;
                    tree.LeafNode
                    (
                        $"Type: {instCollGeneric->TriggerBoxLayoutInstance.Type} (pcb={instCollGeneric->PcbPathCrc:X} '{(pcbPath != null ? pcbPath->DataString : "")}')"
                    );
                    tree.LeafNode($"Layer: {instCollGeneric->GetLayerMask():X} (is-43h={instCollGeneric->LayerMaskIs43h})");
                    tree.LeafNode
                    (
                        $"Material: {instCollGeneric->MaterialIdHigh:X8}{instCollGeneric->MaterialIdLow:X8}/{instCollGeneric->MaterialMaskHigh:X8}{instCollGeneric->MaterialMaskLow:X8}"
                    );
                    tree.LeafNode($"Misc: active-by-default={instCollGeneric->TriggerBoxLayoutInstance.ActiveByDefault}");
                    //tree.LeafNode($"Unk: {instCollGeneric->ColliderLayoutInstance.u70}");
                    break;
                case InstanceType.ClientPath:
                    var instPath = (PathLayoutInstance*)inst;
                    var def      = instPath->Definition;

                    using (var npn = tree.Node($"路径节点数：{def->Segments.Length}"))
                    {
                        if (npn.Opened)
                        {
                            for (var i = 0; i < def->Segments.Length; i++)
                            {
                                var segment = def->Segments[i];
                                ImGui.Text($"[{i}] {segment.Position} {segment.UnkWord:X2} {segment.UnkByte:X1}");
                            }
                        }

                        if (npn.SelectedOrHovered)
                            coll.DrawPath(def->Segments);
                    }

                    break;
            }
        }

        return ni.SelectedOrHovered;
    }

    private UITree.NodeRaii DrawManagerBase(string tag, IManagerBase* manager, string extra) => _tree.Node
    (
        $"{tag} {(nint)manager:X}{(manager != null ? $" (owner={(nint)manager->Owner:X}, id={manager->Id:X})" : "")} {extra}###{tag}_{(nint)manager:X}",
        manager == null
    );

    private static (ulong mat, ulong mask) GetMaterial(ILayoutInstance* inst)
    {
        if (inst == null)
            return (0, 0);

        switch (inst->Id.Type)
        {
            case InstanceType.BgPart:
                var instBgPart = (BgPartsLayoutInstance*)inst;
                return (instBgPart->CollisionMaterialIdHigh      << 32 | instBgPart->CollisionMaterialIdLow,
                           instBgPart->CollisionMaterialMaskHigh << 32 | instBgPart->CollisionMaterialMaskLow);
            case InstanceType.CollisionBox:
                var instCollGeneric = (CollisionBoxLayoutInstance*)inst;
                return (instCollGeneric->MaterialIdHigh      << 32 | instCollGeneric->MaterialIdLow,
                           instCollGeneric->MaterialMaskHigh << 32 | instCollGeneric->MaterialMaskLow);
            case InstanceType.SharedGroup:
                ulong mat  = 0;
                ulong mask = 0;
                SumMaterials((SharedGroupLayoutInstance*)inst, ref mat, ref mask);
                return (mat, mask);
            default:
                return (0, 0);
        }
    }

    private static void SumMaterials(SharedGroupLayoutInstance* inst, ref ulong mat, ref ulong mask)
    {
        foreach (var part in inst->Instances.Instances)
        {
            var (mat1, mask1) =  GetMaterial(part.Value->Instance);
            mat               |= mat1;
            mask              |= mask1;
        }
    }

    private void DrawWorld(LayoutWorld* w)
    {
        using var nw = DrawManagerBase("世界 (World)", &w->IManagerBase, $"t={w->MillisecondsSinceLastUpdate}ms");
        if (!nw.Opened)
            return;

        DrawLayout("全局", w->GlobalLayout);
        DrawLayout("激活", w->ActiveLayout);
        //DrawLayout("u28", w->UnkLayout28);
        //DrawLayout("u30", w->UnkLayout30);

        using (var n = _tree.Node($"已加载布局：{w->LoadedLayouts.Count}###loaded", w->LoadedLayouts.Count == 0))
        {
            if (n.Opened)
            {
                foreach (var (k, v) in w->LoadedLayouts)
                    DrawLayout($"领地 {(int)k} (lvb crc={k >> 32:X})", v);
            }
        }

        //using (var n = _tree.Node($"u90 layouts: {w->UnkLayouts90.Count}###u90", w->UnkLayouts90.Count == 0))
        //{
        //    if (n.Opened)
        //    {
        //        foreach (var (k, v) in w->UnkLayouts90)
        //        {
        //            DrawLayout($"Terr {(int)k} (lvb crc={k >> 32:X})", v);
        //        }
        //    }
        //}
    }

    private void DrawLayout(string tag, LayoutManager* manager)
    {
        using var nr = DrawManagerBase($"{tag} 布局", &manager->IManagerBase, "");
        if (!nr.Opened)
            return;

        _tree.LeafNode($"初始化状态：{manager->InitState}");
        _tree.LeafNode($"初始化参数：类型={manager->Type}，领地 ID={manager->TerritoryTypeId}，CFC={manager->CfcId}，筛选键={manager->LayerFilterKey:X}");
        _tree.LeafNode($"季节/节日 (Festivals)：状态={manager->FestivalStatus} [{LayoutUtil.FestivalsString(manager->ActiveFestivals)}]");
        _tree.LeafNode($"流式加载管理器：{(nint)manager->StreamingManager:X}");
        _tree.LeafNode($"环境管理器：{(nint)manager->Environment:X}");
        _tree.LeafNode($"OBSet 管理器：{(nint)manager->OBSetManager:X}");
        _tree.LeafNode($"流式加载参数：强制更新={manager->ForceUpdateAllStreaming}，跳过地形碰撞={manager->SkipAddingTerrainCollider}");
        _tree.LeafNode($"流式加载原点：强制={manager->ForcedStreamingOrigin}，最后更新={manager->LastUpdatedStreamingOrigin}，类型={manager->StreamingOriginType}");
        _tree.LeafNode($"最后更新：耗时={manager->LastUpdateDT:f3}，翻转={manager->LastUpdateOdd}");
        DrawStringTable(ref manager->ResourcePaths);

        using (var n = _tree.Node("资源"))
        {
            if (n.Opened)
            {
                DrawResourceHandle("LVB", manager->LvbResourceHandle);
                DrawResourceHandle("SVB", manager->SvbResourceHandle);
                DrawResourceHandle("LCB", manager->LcbResourceHandle);
                DrawResourceHandle("UWB", manager->UwbResourceHandle);
                var i = 0;
                foreach (var rsrc in manager->LayerGroupResourceHandles)
                    DrawResourceHandle($"LGB {i++}", rsrc.Value);
            }
        }

        using (var n = _tree.Node($"地形 ({manager->Terrains.Count})###terrains", manager->Terrains.Count == 0))
        {
            if (n.Opened)
            {
                foreach (var (k, v) in manager->Terrains)
                {
                    var nterr = _tree.LeafNode($"{k:X8} = {(nint)v.Value:X}，路径={v.Value->PathString}，碰撞={(nint)v.Value->Collider:X}");
                    if (nterr.SelectedOrHovered && v.Value->Collider != null)
                        _coll.VisualizeCollider(&v.Value->Collider->Collider, default, default);
                }
            }
        }

        using (var n = _tree.Node($"层级 ({manager->Layers.Count})###layers", manager->Layers.Count == 0))
        {
            if (n.Opened)
            {
                foreach (var (lk, lv) in manager->Layers)
                    DrawLayer($"{lk:X4}", manager, lv.Value);
            }
        }

        using (var n = _tree.Node($"实例 ({manager->InstancesByType.Count} 种类型)###insts", manager->InstancesByType.Count == 0))
        {
            if (n.Opened)
            {
                foreach (var (itk, itv) in manager->InstancesByType)
                {
                    using var nt = _tree.Node($"{itk} ({itv.Value->Count} 个实例)##{itk}");

                    if (nt.Opened)
                    {
                        foreach (var (ik, iv) in *itv.Value)
                            DrawInstance($"{ik >> 32:X8}.{(uint)ik:X8}", manager, iv.Value);
                    }
                }
            }
        }

        using (var n = _tree.Node($"路径 ({manager->CrcToPath.Count})###paths", manager->CrcToPath.Count == 0))
        {
            if (n.Opened)
            {
                foreach (var (k, v) in manager->CrcToPath)
                    _tree.LeafNode($"{k:X8} = [{v.Value->NumRefs}] {v.Value->DataString}");
            }
        }

        using (var n = _tree.Node($"解析形状 (Analytic Shapes) ({manager->CrcToAnalyticShapeData.Count})###shapes", manager->CrcToAnalyticShapeData.Count == 0))
        {
            if (n.Opened)
            {
                foreach (var (k, v) in manager->CrcToAnalyticShapeData)
                    DrawAnalyticShape(_tree, $"{k.Key:X8} =", v);
            }
        }

        using (var n = _tree.Node($"过滤器 ({manager->Filters.Count})", manager->Filters.Count == 0))
        {
            if (n.Opened)
            {
                var activeFilter = LayoutUtil.FindFilter(manager);
                foreach (var (k, v) in manager->Filters)
                    _tree.LeafNode($"{k:X} = terr={v.Value->TerritoryTypeId} cfc={v.Value->CfcId}{(v.Value == activeFilter ? " (激活)" : "")}");
            }
        }
    }

    private void DrawLayer(string tag, LayoutManager* layout, LayerManager* layer)
    {
        var unks = ""; // $", u1F={layer->u1F}, u20={layer->u20:X4}";
        using var nl = _tree.Node
        (
            $"[{tag}] LG{layer->LayerGroupId}, festival={layer->FestivalId}/{layer->FestivalSubId}, flags={layer->Flags:X}{unks}, {layer->Instances.Count} instances == {(nint)layer:X}",
            layer->Instances.Count == 0
        );
        if (!nl.Opened)
            return;
        foreach (var (ik, iv) in layer->Instances) DrawInstance($"{ik:X8}", layout, iv.Value);
    }

    private void DrawInstance(string tag, LayoutManager* layout, ILayoutInstance* inst)
    {
        if (DrawInstance(_tree, tag, layout, inst, _coll))
        {
            if (inst->Id.Type == InstanceType.SharedGroup)
            {
                var sg = (SharedGroupLayoutInstance*)inst;

                foreach (var obj in sg->Instances.Instances)
                {
                    var subcol = obj.Value->Instance->GetCollider();
                    if (subcol != null)
                        _coll.VisualizeCollider(subcol, default, default);
                }
            }

            var collider = inst->GetCollider();
            if (collider != null) _coll.VisualizeCollider(collider, default, default);
        }
    }

    private void DrawStringTable(ref StringTable strings)
    {
        using var n = _tree.Node($"字符串表 ({strings.Strings.Count} 个，其中 {strings.NumNulls} 个为空)###strings", strings.Strings.Count == 0);
        if (!n.Opened)
            return;
        foreach (var str in strings.Strings)
            _tree.LeafNode($"[{str.Value->NumRefs}] {str.Value->DataString}");
    }

    private void DrawResourceHandle(string tag, ResourceHandle* rsrc)
    {
        if (rsrc == null)
            _tree.LeafNode($"{tag}: null");
        else
            _tree.LeafNode($"{tag}: {rsrc->FileName}");
    }

    private void DrawFile(string tag, string path)
    {
        using var n = _tree.Node($"{tag}: '{path}'");
        if (!n.Opened)
            return;

        var lgb = Service.DataManager.GetFile(path);
        if (lgb == null)
            return;

        fixed (byte* data = &lgb.Data[0])
            DrawFileData((FileHeader*)data);
    }

    private void DrawFileData(FileHeader* header)
    {
        using var n = _tree.Node
        (
            $"{(char)(header->Magic & 0xFF)}{(char)(header->Magic >> 8 & 0xFF)}{(char)(header->Magic >> 16 & 0xFF)}{(char)(header->Magic >> 24 & 0xFF)} (size={header->TotalSize}): {header->NumSections} sections"
        );
        if (!n.Opened)
            return;

        foreach (var section in header->Sections)
        {
            var tag =
                $"{(char)(section->Magic & 0xFF)}{(char)(section->Magic >> 8 & 0xFF)}{(char)(section->Magic >> 16 & 0xFF)}{(char)(section->Magic >> 24 & 0xFF)} (size={section->TotalSize})";
            var _ = section->Magic switch
            {
                0x314E4353 => DrawFileSectionScene(tag, section->Data<FileSceneHeader>()),
                0x3150474C => DrawFileSectionLayerGroup(tag, section->Data<FileLayerGroupHeader>()),
                _          => _tree.LeafNode($"{tag}: unknown").Opened
            };
        }
    }

    private bool DrawFileSectionScene(string tag, FileSceneHeader* header)
    {
        using var n = _tree.Node($"{tag}: general at +{header->OffsetGeneral}");
        if (!n.Opened)
            return false;
        var general = header->General;
        _tree.LeafNode($"Have layer groups: {general->HaveLayerGroups}");
        _tree.LeafNode($"Terrain: {LayoutUtil.ReadString(general->PathTerrain)}");
        _tree.LeafNode($"Env spaces: {general->NumEnvSpaces} at +{general->OffsetEnvSpaces}");
        _tree.LeafNode($"Sky visibility: {LayoutUtil.ReadString(general->PathSkyVisibility)}");
        _tree.LeafNode($"LCB: {LayoutUtil.ReadString(general->PathLCB)} (uw={general->HaveLCBUW})");

        using (var ng = _tree.Node
                   ($"Embedded layer groups ({header->NumEmbeddedLayerGroups} at +{header->OffsetEmbeddedLayerGroups})", header->NumEmbeddedLayerGroups == 0))
        {
            if (ng.Opened)
            {
                for (var i = 0; i < header->NumEmbeddedLayerGroups; ++i)
                    DrawFileSectionLayerGroup(i.ToString(), header->EmbeddedLayerGroups.GetPointer(i));
            }
        }

        using (var ng = _tree.Node
                   ($"Layer group resources ({header->NumLayerGroupResources} at +{header->OffsetLayerGroupResources})", header->NumLayerGroupResources == 0))
        {
            if (ng.Opened)
            {
                for (var i = 0; i < header->NumLayerGroupResources; ++i)
                    DrawFile($"Layer group {i}", LayoutUtil.ReadString(header->LayerGroupResource(header->LayerGroupResourceOffsets[i])));
            }
        }

        var filterList = header->Filters;

        if (filterList != null)
        {
            using (var nl = _tree.Node($"Filters ({filterList->NumEntries} at +{header->OffsetFilters}+{filterList->OffsetEntries})", filterList->NumEntries == 0))
            {
                if (nl.Opened)
                {
                    foreach (ref var f in filterList->Entries)
                    {
                        var unks = ""; // $"; unks: {f.u0} {f.u8} {f.uC} {f.u14} {f.u18}";
                        _tree.LeafNode($"[{f.Key:X}] = terr={f.TerritoryTypeId} cfc={f.CfcId}{unks}");
                    }
                }
            }
        }

        return true;
    }

    private bool DrawFileSectionLayerGroup(string tag, FileLayerGroupHeader* header)
    {
        using var n = _tree.Node
        (
            $"{tag}: {header->Id} '{LayoutUtil.ReadString(header->LayerGroupName)}', {header->NumLayers} layers at +{header->OffsetLayers}",
            header->NumLayers == 0
        );
        if (!n.Opened)
            return false;

        foreach (var layerOffset in header->LayerOffsets)
        {
            var layer = header->Layer(layerOffset);

            var filter = layer->Filter;
            var filterString = filter != null ?
                                   $"{filter->Operation} [{string.Join(',', Enumerable.Range(0, filter->NumListEntries).Select(j => $"{filter->Entries[j]:X}"))}]" :
                                   "<none>";

            var layerUnks = ""; // $", u20={layer->u20}, u1C={layer->u1C}, u10={layer->u10}, u11={layer->u11}";
            using var nl = _tree.Node
            (
                $"[{layer->Key:X4}] '{LayoutUtil.ReadString(layer->LayerName)}': festival={layer->Festival.Id}/{layer->Festival.Phase}, filter={filterString}, {layer->NumInstances} instances at +{layer->OffsetInstances}{layerUnks}, offset=+{header->OffsetLayers}+{layerOffset}",
                layer->NumInstances == 0 /*, layer->u20 != 0 || layer->u1C != 0 ? 0xff0000ff : 0xffffffff*/
            );

            if (nl.Opened)
            {
                foreach (var instOffset in layer->InstanceOffsets)
                {
                    var instance = layer->Instance(instOffset);
                    var instUnk  = ""; // $", u8={instance->u8}";
                    using var ni = _tree.Node
                    (
                        $"[{instance->Key:X}] '{LayoutUtil.ReadString(instance->Name)}': type={instance->Type}{instUnk}, trans={instance->Transform.Translation}, rot={instance->Transform.Rotation}, scale={instance->Transform.Scale}, offset=+{layer->OffsetInstances}+{instOffset}"
                    );

                    if (ni.Opened)
                    {
                        switch (instance->Type)
                        {
                            case InstanceType.BgPart:
                                var instanceBgPart = (FileLayerGroupInstanceBgPart*)instance;
                                _tree.LeafNode($"Mdl: {LayoutUtil.ReadString(instanceBgPart->PathMdl)}");
                                _tree.LeafNode($"Pcb: {LayoutUtil.ReadString(instanceBgPart->PathPcb)}");
                                _tree.LeafNode($"Collider type: {instanceBgPart->ColliderType}");
                                _tree.LeafNode
                                (
                                    $"Material: {instanceBgPart->MaterialIdHigh:X8}{instanceBgPart->MaterialIdLow:X8}/{instanceBgPart->MaterialMaskHigh:X8}{instanceBgPart->MaterialMaskLow:X8}"
                                );

                                //_tree.LeafNode($"unks: {instanceBgPart->u50} {instanceBgPart->u51} {instanceBgPart->u52} {instanceBgPart->u53} {instanceBgPart->u54} {instanceBgPart->u58}");
                                using (var ns = _tree.Node
                                           ($"Shape data: +{instanceBgPart->OffsetColliderAnalyticData}", instanceBgPart->OffsetColliderAnalyticData == 0))
                                {
                                    if (ns.Opened)
                                    {
                                        var data = instanceBgPart->ColliderAnalyticData;
                                        _tree.LeafNode($"Type: {data->ColliderType}");
                                        _tree.LeafNode($"Material: {data->MaterialId:X} / {data->MaterialMask:X}");
                                        _tree.LeafNode($"Transform: {data->Transform.Translation} {data->Transform.Rotation} {data->Transform.Scale}");
                                        _tree.LeafNode($"Bounds: {data->Bounds}");
                                        //_tree.LeafNode($"unks: {data->u8} {data->uC}");
                                    }
                                }

                                break;
                            case InstanceType.SharedGroup:
                            case InstanceType.HelperObject:
                                var instancePrefab = (FileLayerGroupInstanceSharedGroup*)instance;
                                DrawFile("Path", LayoutUtil.ReadString(instancePrefab->Path));
                                //_tree.LeafNode($"Unks: types={instancePrefab->u34} {instancePrefab->u40}, other={instancePrefab->u38} {instancePrefab->u3C}");
                                break;
                            case InstanceType.CollisionBox:
                                var instanceCollGen = (FileLayerGroupInstanceCollisionBox*)instance;
                                _tree.LeafNode($"Type: {instanceCollGen->ColliderType}");
                                _tree.LeafNode
                                (
                                    $"Material: {instanceCollGen->MaterialIdHigh:X8}{instanceCollGen->MaterialIdLow:X8}/{instanceCollGen->MaterialMaskHigh:X8}{instanceCollGen->MaterialMaskLow:X8}"
                                );
                                _tree.LeafNode($"Misc: active={instanceCollGen->ActiveByDefault}, layer43h={instanceCollGen->Layer43h}");
                                //_tree.LeafNode($"Unks: u34={instanceCollGen->u34}");
                                _tree.LeafNode($"Pcb: {LayoutUtil.ReadString(instanceCollGen->Path)}");
                                break;
                        }
                    }
                }
            }
        }

        return true;
    }

    private void DrawComparison(LayoutManager* layout)
    {
        var activeFilter = LayoutUtil.FindFilter(layout);
        var terrId = activeFilter != null ?
                         activeFilter->TerritoryTypeId :
                         layout->TerritoryTypeId;
        var cfcId = activeFilter != null ?
                        activeFilter->CfcId :
                        layout->CfcId;

        var terr = Service.LuminaRow<TerritoryType>(terrId);
        if (terr == null || layout == null)
            return;

        using var n = _tree.Node($"对比：领地 {terrId}/{cfcId} '{terr.Value.Bg}'###comparison");
        if (!n.Opened)
            return;

        var lvb = Service.DataManager.GetFile($"bg/{terr.Value.Bg}.lvb");

        if (lvb != null)
        {
            fixed (byte* lvbData = &lvb.Data[0])
            {
                FillInstancesFromFileScene
                (
                    FindSection<FileSceneHeader>((FileHeader*)lvbData, 0x314E4353),
                    activeFilter != null ?
                        activeFilter->Key :
                        0,
                    layout->ActiveFestivals
                );
            }
        }

        FillInstancesFromGame(layout);

        ImGui.Checkbox("按层级组分组",  ref _groupByLayerGroup);
        ImGui.Checkbox("按层级分组",   ref _groupByLayer);
        ImGui.Checkbox("按实例类型分组", ref _groupByInstanceType);
        ImGui.Checkbox("按材质分组",   ref _groupByMaterial);
        ImGui.InputText("按 ID 筛选", ref _filterById, 255);
        DrawInstancesByLayerGroup(_insts.Values);
    }


    private static void DrawAnalyticShape(UITree tree, string tag, AnalyticShapeData v)
    {
        var unks = ""; //$" {v.u8:X} {v.uC} {v.u3C} {v.u60} {v.u64}";
        tree.LeafNode
        (
            $"{tag} [{v.NumRefs}] {v.Type}{unks} trans=[{v.Transform.Translation} {v.Transform.Rotation} {v.Transform.Scale}] bb=[{v.BoundsMin}-{v.BoundsMax}], mat={v.MaterialId:X}/{v.MaterialMask:X}"
        );
    }
}
