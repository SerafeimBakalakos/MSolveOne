using System.Collections.Generic;
using System.Linq;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Mesh
{
	public class DiscontinuousOutputMesh : IOutputMesh
	{
		private readonly List<VtkCell> outCells;
		private readonly SortedDictionary<int, VtkPoint> outVertices;

		public DiscontinuousOutputMesh(IEnumerable<IXFiniteElement> elements)
		{
			this.OriginalCells = elements;

			var nodes = new HashSet<XNode>();
			foreach (IXFiniteElement element in elements)
			{
				nodes.UnionWith(element.Nodes);
			}
			this.OriginalVertices = nodes;

			this.outVertices = new SortedDictionary<int, VtkPoint>();
			this.outCells = new List<VtkCell>();
			foreach (IXFiniteElement element in elements)
			{
				var verticesOfCell = new List<VtkPoint>();
				for (int i = 0; i < element.Nodes.Count; ++i)
				{
					var vertex = new VtkPoint(outVertices.Count, element.Nodes[i].Coordinates);
					verticesOfCell.Add(vertex);
					outVertices[vertex.ID] = vertex;
				}
				outCells.Add(new VtkCell(element.CellType, verticesOfCell));
			}
		}

		public int NumOutCells => outCells.Count;

		public int NumOutVertices => outVertices.Count;

		public IEnumerable<IXFiniteElement> OriginalCells { get; }

		/// <summary>
		/// Same order as the corresponding one in <see cref="OutVertices"/>.
		/// </summary>
		public IEnumerable<XNode> OriginalVertices { get; }

		public IEnumerable<VtkCell> OutCells => outCells;

		/// <summary>
		/// Same order as the corresponding one in <see cref="OriginalVertices"/>.
		/// </summary>
		public IEnumerable<VtkPoint> OutVertices => outVertices.Values;
	}
}
