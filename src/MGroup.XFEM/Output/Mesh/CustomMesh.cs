using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Mesh
{
	public class CustomMesh : IOutputMesh
	{
		public List<VtkCell> Cells { get; } = new List<VtkCell>();

		public SortedDictionary<int, VtkPoint> Vertices { get; } = new SortedDictionary<int, VtkPoint>();

		public int NumOutCells => Cells.Count;

		public int NumOutVertices => Vertices.Count;

		public IEnumerable<VtkCell> OutCells => Cells;

		public IEnumerable<VtkPoint> OutVertices => Vertices.Values;
	}
}
