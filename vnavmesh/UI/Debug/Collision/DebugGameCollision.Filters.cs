using Dalamud.Bindings.ImGui;
using Dalamud.Interface.Utility.Raii;
using FFXIVClientStructs.FFXIV.Client.System.Framework;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;
using vnavmesh.Shared.Models;

namespace vnavmesh.UI.Debug.Collision;

public unsafe partial class DebugGameCollision
{
    private void GatherInfo()
    {
        _streamedMeshes.Clear();
        _availableLayers.Reset();
        _availableMaterials.Reset();

        foreach (var s in Framework.Instance()->BGCollisionModule->SceneManager->Scenes)
        {
            foreach (var coll in s->Scene->Colliders)
            {
                _availableLayers    |= new BitMask(coll->LayerMask);
                _availableMaterials |= new BitMask(coll->ObjectMaterialValue);

                var collType = coll->GetColliderType();

                if (collType == ColliderType.Streamed)
                {
                    var cast = (ColliderStreamedEx*)coll;

                    if (cast->Header != null && cast->Elements != null)
                    {
                        for (var i = 0; i < cast->Header->NumMeshes; ++i)
                        {
                            var m = cast->Elements[i].Mesh;
                            if (m != null)
                                _streamedMeshes.Add((nint)m);
                        }
                    }
                }
                else if (collType == ColliderType.Mesh)
                {
                    var cast = (ColliderMesh*)coll;

                    if (!cast->MeshIsSimple && cast->Mesh != null)
                    {
                        var mesh = (MeshPCB*)cast->Mesh;
                        var mask = new BitMask(coll->ObjectMaterialMask);
                        GatherMeshNodeMaterials(mesh->RootNode, ~mask);
                    }
                }
            }
        }
    }

    private bool FilterCollider(Collider* coll)
    {
        if (coll->LayerMask == 0 ? !_showZeroLayer : (_shownLayers.Raw & coll->LayerMask) == 0)
            return false;
        if (_showOnlyFlagRaycast && (coll->VisibilityFlags & 1) == 0)
            return false;
        if (_showOnlyFlagVisit && (coll->VisibilityFlags & 2) == 0)
            return false;
        var matFilter = _availableMaterials & _materialMask;
        if (matFilter.Any() && coll->GetColliderType()                             != ColliderType.Mesh)
            return (matFilter.Raw & (coll->ObjectMaterialValue ^ _materialId.Raw)) == 0;
        return true;
    }

    private void DrawSettings()
    {
        using var n = _tree.Node("设置");
        if (!n.Opened)
            return;

        ImGui.Checkbox("显示零层级 (Zero Layer) 对象", ref _showZeroLayer);
        {
            var       shownLayers = _availableLayers & _shownLayers;
            using var layers = ImRaii.Combo("显示的层级", shownLayers == _availableLayers ? "全选" : shownLayers.None() ? "无" : string.Join(", ", shownLayers.SetBits()));

            if (layers)
            {
                foreach (var i in _availableLayers.SetBits())
                {
                    var shown = _shownLayers[i];
                    if (ImGui.Checkbox($"层级 {i}", ref shown))
                        _shownLayers[i] = shown;
                }
            }
        }

        {
            var       matMask   = _materialMask & _availableMaterials;
            using var materials = ImRaii.Combo("材质掩码 (Material Mask)", matMask.None() ? "无" : matMask.Raw.ToString("X"));

            if (materials)
            {
                foreach (var i in _availableMaterials.SetBits())
                {
                    var filter = _materialMask[i];
                    if (ImGui.Checkbox($"材质 {1u << i:X16}", ref filter))
                        _materialMask[i] = filter;
                }
            }
        }

        {
            var       matId     = _materialId & _availableMaterials;
            using var materials = ImRaii.Combo("材质 ID", matId.None() ? "无" : matId.Raw.ToString("X"));

            if (materials)
            {
                foreach (var i in _availableMaterials.SetBits())
                {
                    var filter = _materialId[i];
                    if (ImGui.Checkbox($"材质 {1u << i:X16}", ref filter))
                        _materialId[i] = filter;
                }
            }
        }

        {
            using var flags = ImRaii.Combo
                ("标志筛选", _showOnlyFlagRaycast ? _showOnlyFlagVisit ? "仅当两个标志均设置时" : "仅当射线检测标志设置时" : _showOnlyFlagVisit ? "仅当全局访问标志设置时" : "显示全部");

            if (flags)
            {
                ImGui.Checkbox("隐藏无射线检测标志的对象 (0x1)", ref _showOnlyFlagRaycast);
                ImGui.Checkbox("隐藏无全局访问标志的对象 (0x2)", ref _showOnlyFlagVisit);
            }
        }
    }
}
