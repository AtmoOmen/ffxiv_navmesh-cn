using FFXIVClientStructs.FFXIV.Common.Component.BGCollision;
using FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Math;
using vnavmesh.Bootstrap;
using vnavmesh.Common.Models;
using vnavmesh.Shared.Models;
using vnavmesh.Shared.Utilities;
using vnavmesh.UI.Rendering;
using Vector3 = System.Numerics.Vector3;
using Vector4 = System.Numerics.Vector4;

namespace vnavmesh.UI.Debug.Collision;

public unsafe partial class DebugGameCollision
{
    public void VisualizeCollider(Collider* coll, BitMask filterId, BitMask filterMask)
    {
        if (coll == null)
            return;

        switch (coll->GetColliderType())
        {
            case ColliderType.Streamed:
            {
                var cast = (ColliderStreamedEx*)coll;

                if (cast->Header != null && cast->Elements != null)
                {
                    for (var i = 0; i < cast->Header->NumMeshes; ++i)
                    {
                        var elem = cast->Elements + i;
                        VisualizeColliderMesh(elem->Mesh, new(0, 1, 0, 0.7f), _materialId, _materialMask);
                    }
                }
            }
                break;
            case ColliderType.Mesh:
                VisualizeColliderMesh((ColliderMesh*)coll, new(_streamedMeshes.Contains((nint)coll) ? 0 : 1, 1, 0, 0.7f), _materialId, _materialMask);
                break;
            case ColliderType.Box:
            {
                var cast   = (ColliderBox*)coll;
                var render = GetDynamicMeshes();
                var box    = new AnalyticMeshBox(render);
                var icnt   = render.NumInstances;
                render.AddInstance(new(cast->World, new(1, 0, 0, 0.7f)));
                render.AddMesh(box.FirstVertex, box.FirstPrimitive, box.NumPrimitives, icnt, 1);
            }
                break;
            case ColliderType.Cylinder:
                VisualizeCylinder(ref ((ColliderCylinder*)coll)->World, 0xff0000ff);
                break;
            case ColliderType.Sphere:
                _dd.DrawWorldSphere(((ColliderSphere*)coll)->Translation, ((ColliderSphere*)coll)->Scale.X, 0xff0000ff);
                break;
            case ColliderType.Plane:
            case ColliderType.PlaneTwoSided:
            {
                var cast = (ColliderPlane*)coll;
                var a    = cast->World.TransformCoordinate(new(-1, +1, 0));
                var b    = cast->World.TransformCoordinate(new(-1, -1, 0));
                var c    = cast->World.TransformCoordinate(new(+1, -1, 0));
                var d    = cast->World.TransformCoordinate(new(+1, +1, 0));
                _dd.DrawWorldLine(a, b, 0xff0000ff);
                _dd.DrawWorldLine(b, c, 0xff0000ff);
                _dd.DrawWorldLine(c, d, 0xff0000ff);
                _dd.DrawWorldLine(d, a, 0xff0000ff);
            }
                break;
        }

        Vector3 trans;
        coll->GetTranslation(&trans);
        _dd.DrawWorldLine(Service.ObjectTable.LocalPlayer?.Position ?? default, trans, 0xFFFF00FF);
    }

    private void VisualizeColliderMesh(ColliderMesh* coll, Vector4 color, BitMask filterId, BitMask filterMask)
    {
        if (coll != null && !coll->MeshIsSimple && coll->Mesh != null)
        {
            var mesh = (MeshPCB*)coll->Mesh;
            VisualizeColliderMeshPCBNode
            (
                mesh->RootNode,
                ref coll->World,
                color,
                coll->Collider.ObjectMaterialValue & coll->Collider.ObjectMaterialMask,
                ~coll->Collider.ObjectMaterialMask,
                filterId,
                filterMask
            );
        }
    }

    private void VisualizeColliderMeshPCBNode
    (
        MeshPCB.FileNode* node,
        ref Matrix4x3     world,
        Vector4           color,
        ulong             objMatId,
        ulong             objMatInvMask,
        BitMask           filterId,
        BitMask           filterMask
    )
    {
        if (node == null)
            return;

        if (node->NumPrims > 0)
        {
            var renderer = GetDynamicMeshes();
            renderer.AddMesh(renderer.NumVertices, renderer.NumPrimitives, node->NumPrims, renderer.NumInstances, 1);
            for (var i = 0; i < node->NumVertsRaw + node->NumVertsCompressed; ++i)
                renderer.AddVertex(node->Vertex(i));

            foreach (ref var prim in node->Primitives)
            {
                var pass = true;

                if (filterMask.Any())
                {
                    var effMat = objMatId | objMatInvMask & prim.Material;
                    pass = (filterMask.Raw & (effMat ^ filterId.Raw)) == 0;
                }

                if (pass)
                    renderer.AddTriangle(prim.V1, prim.V3, prim.V2);
                else
                    renderer.AddTriangle(prim.V1, prim.V1, prim.V1);
                renderer.AddInstance(new(world, color));
            }
        }

        VisualizeColliderMeshPCBNode(node->Child1, ref world, color, objMatId, objMatInvMask, filterId, filterMask);
        VisualizeColliderMeshPCBNode(node->Child2, ref world, color, objMatId, objMatInvMask, filterId, filterMask);
    }

    private void VisualizeOBB(ref AABB localBB, ref Matrix4x3 world, uint color)
    {
        var aaa = world.TransformCoordinate(new(localBB.Min.X, localBB.Min.Y, localBB.Min.Z));
        var aab = world.TransformCoordinate(new(localBB.Min.X, localBB.Min.Y, localBB.Max.Z));
        var aba = world.TransformCoordinate(new(localBB.Min.X, localBB.Max.Y, localBB.Min.Z));
        var abb = world.TransformCoordinate(new(localBB.Min.X, localBB.Max.Y, localBB.Max.Z));
        var baa = world.TransformCoordinate(new(localBB.Max.X, localBB.Min.Y, localBB.Min.Z));
        var bab = world.TransformCoordinate(new(localBB.Max.X, localBB.Min.Y, localBB.Max.Z));
        var bba = world.TransformCoordinate(new(localBB.Max.X, localBB.Max.Y, localBB.Min.Z));
        var bbb = world.TransformCoordinate(new(localBB.Max.X, localBB.Max.Y, localBB.Max.Z));
        _dd.DrawWorldLine(aaa, aab, color);
        _dd.DrawWorldLine(aab, bab, color);
        _dd.DrawWorldLine(bab, baa, color);
        _dd.DrawWorldLine(baa, aaa, color);
        _dd.DrawWorldLine(aba, abb, color);
        _dd.DrawWorldLine(abb, bbb, color);
        _dd.DrawWorldLine(bbb, bba, color);
        _dd.DrawWorldLine(bba, aba, color);
        _dd.DrawWorldLine(aaa, aba, color);
        _dd.DrawWorldLine(aab, abb, color);
        _dd.DrawWorldLine(baa, bba, color);
        _dd.DrawWorldLine(bab, bbb, color);
    }

    private void VisualizeCylinder(ref Matrix4x3 world, uint color)
    {
        var numSegments = CurveApproxUtil.CalculateCircleSegments(world.Row0.Length(), 360.Degrees(), 0.1f);
        var prev1       = world.TransformCoordinate(new(0, +1, 1));
        var prev2       = world.TransformCoordinate(new(0, -1, 1));

        for (var i = 1; i <= numSegments; ++i)
        {
            var dir   = (i * 360.0f / numSegments).Degrees().ToDirection();
            var curr1 = world.TransformCoordinate(new(dir.X, +1, dir.Y));
            var curr2 = world.TransformCoordinate(new(dir.X, -1, dir.Y));
            _dd.DrawWorldLine(curr1, prev1, color);
            _dd.DrawWorldLine(curr2, prev2, color);
            _dd.DrawWorldLine(curr1, curr2, color);
            prev1 = curr1;
            prev2 = curr2;
        }
    }

    private void VisualizeSphere(Vector4 sphere, uint color) => _dd.DrawWorldSphere(new(sphere.X, sphere.Y, sphere.Z), sphere.W, color);

    private void VisualizeVertex(Vector3 worldPos, uint color)
    {
        _dd.DrawWorldSphere(worldPos, 0.1f, color);
        if (Service.ObjectTable.LocalPlayer is { } p)
            _dd.DrawWorldLine(p.Position, worldPos, color);
    }

    private void VisualizeTriangle
        (MeshPCB.FileNode* node, ref FFXIVClientStructs.FFXIV.Common.Component.BGCollision.Mesh.Primitive prim, ref Matrix4x3 world, uint color)
    {
        var v1 = world.TransformCoordinate(node->Vertex(prim.V1));
        var v2 = world.TransformCoordinate(node->Vertex(prim.V2));
        var v3 = world.TransformCoordinate(node->Vertex(prim.V3));
        _dd.DrawWorldLine(v1, v2, color);
        _dd.DrawWorldLine(v2, v3, color);
        _dd.DrawWorldLine(v3, v1, color);
    }

    private void GatherMeshNodeMaterials(MeshPCB.FileNode* node, BitMask invMask)
    {
        if (node == null)
            return;
        foreach (ref var prim in node->Primitives)
            _availableMaterials |= invMask & new BitMask(prim.Material);
        GatherMeshNodeMaterials(node->Child1, invMask);
        GatherMeshNodeMaterials(node->Child2, invMask);
    }
}
