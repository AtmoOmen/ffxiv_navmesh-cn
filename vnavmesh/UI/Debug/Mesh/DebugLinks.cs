using System.Numerics;
using vnavmesh.Common.Navigation.Mesh.Runtime;
using vnavmesh.UI.Debug.Common;
using vnavmesh.UI.Debug.Common.Components;

namespace vnavmesh.UI.Debug.Mesh;

public class DebugLinks
(
    Navmesh     mesh,
    DebugDrawer dd
)
{
    private readonly UITree _tree = new();

    public void Draw()
    {
        using var n = _tree.Node($"连接 ({mesh.Links.Count})###links", mesh.Links.Count == 0);

        if (n.SelectedOrHovered)
        {
            foreach (var link in mesh.Links)
                DrawLink(link.Start, link.End, LinkColor(link.Kind), link.Bidirectional);
        }

        if (n.Opened)
        {
            for (var i = 0; i < mesh.Links.Count; i++)
            {
                var link = mesh.Links[i];
                using var nt = _tree.Node($"{DescribeLink(link)}###link{i}", true);
                if (nt.SelectedOrHovered)
                    DrawLink(link.Start, link.End, LinkColor(link.Kind), link.Bidirectional);
            }
        }
    }

    private void DrawLink(Vector3 from, Vector3 to, uint color, bool bidirectional)
    {
        dd.DrawWorldPointFilled(from, 5, color);
        dd.DrawWorldPointFilled(to, 5, color);
        dd.DrawWorldLine(from, to, color, 2);
        dd.DrawWorldArrowPoint(to, from, 40, color, 2);
        if (bidirectional)
            dd.DrawWorldArrowPoint(from, to, 40, color, 2);
    }

    private static string DescribeLink(NavmeshLink link) =>
        $"{link.Kind}: {link.Start} -> {link.End}，双向 = {(link.Bidirectional ? "是" : "否")}，UserId = {link.UserId}，代价 = {DescribeTraversal(link)}";

    private static string DescribeTraversal(NavmeshLink link)
    {
        var profile = NavmeshLinkTraversalProfiles.Resolve(link.Kind, link.TraversalProfile);
        return $"距离系数 {profile.DistanceScale:f2}, 固定代价 {profile.FixedPenalty:f2}";
    }

    private static uint LinkColor(NavmeshOffMeshKind kind) => kind switch
    {
        NavmeshOffMeshKind.GeneratedClimbDown => 0xFF2ECC71,
        NavmeshOffMeshKind.GeneratedEdgeJump  => 0xFF3498DB,
        NavmeshOffMeshKind.ManualOffMesh      => 0xFFF39C12,
        NavmeshOffMeshKind.Teleport           => 0xFFE74C3C,
        NavmeshOffMeshKind.ClientPath         => 0xFF9B59B6,
        _                                     => 0xFFFFFFFF
    };
}
