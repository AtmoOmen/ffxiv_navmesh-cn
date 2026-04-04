using System.Numerics;
using vnavmesh.Interface.Debug.Components;

namespace vnavmesh.Interface.Debug;

public class DebugLinks(Navmesh.Navmesh mesh, DebugDrawer dd)
{
	private readonly UITree _tree = new();

	public void Draw()
	{
		using var n = _tree.Node($"连接 ({mesh.Links.Count})###links", mesh.Links.Count == 0);
		if (n.SelectedOrHovered)
			foreach (var (from, to) in mesh.Links)
				DrawLink(from, to);
		if (n.Opened)
		{
			for (var i = 0; i < mesh.Links.Count; i++)
			{
				var (f, t) = mesh.Links[i];
				using var nt = _tree.Node($"{f} -> {t}###link{i}", true);
				if (nt.SelectedOrHovered)
					DrawLink(f, t);
			}
		}
	}

	void DrawLink(Vector3 from, Vector3 to, uint color = 0xFF00FF00)
	{
		dd.DrawWorldPointFilled(from, 5, color);
		dd.DrawWorldLine(from, to, color, 2);
		dd.DrawWorldArrowPoint(to, from, 40, color, 2);
	}
}
