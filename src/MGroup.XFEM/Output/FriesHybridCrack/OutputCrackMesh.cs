using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Geometry.HybridFries;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.FriesHybridCrack
{
	public class OutputCrackMesh : IOutputMesh
	{
		private readonly List<VtkCell> outCells;
		private readonly SortedDictionary<int, VtkPoint> outVertices;

		public OutputCrackMesh(IReadOnlyList<Vertex3D> vertices, IReadOnlyList<TriangleCell3D> cells)
		{
			var map = new Dictionary<int, VtkPoint>();

			// Vertices
			outVertices = new SortedDictionary<int, VtkPoint>();
			for (int v = 0; v < vertices.Count; ++v)
			{
				Vertex3D vertex = vertices[v];
				var outVertex = new VtkPoint(v, vertex.CoordsGlobal);
				outVertices[vertex.ID] = outVertex;
				map[vertex.ID] = outVertex;
			}

			// Cells
			outCells = new List<VtkCell>();
			foreach (TriangleCell3D cell in cells)
			{
				var verticesOfCell = new VtkPoint[cell.Vertices.Length];
				for (int v = 0; v < verticesOfCell.Length; ++v)
				{
					verticesOfCell[v] = map[cell.Vertices[v].ID];
				}
				var outCell = new VtkCell(CellType.Tri3, verticesOfCell);
				outCells.Add(outCell);
			}
		}

		public OutputCrackMesh(IReadOnlyList<Vertex2D> vertices, IReadOnlyList<LineCell2D> cells)
		{
			var map = new Dictionary<int, VtkPoint>();

			// Vertices
			outVertices = new SortedDictionary<int, VtkPoint>();
			for (int v = 0; v < vertices.Count; ++v)
			{
				Vertex2D vertex = vertices[v];
				var outVertex = new VtkPoint(v, new double[] { vertex.CoordsGlobal[0], vertex.CoordsGlobal[1], 0 });
				outVertices[v] = outVertex;
				map[vertex.ID] = outVertex;
			}

			// Cells
			outCells = new List<VtkCell>();
			foreach (LineCell2D cell in cells)
			{
				var verticesOfCell = new VtkPoint[cell.Vertices.Length];
				for (int v = 0; v < verticesOfCell.Length; ++v)
				{
					verticesOfCell[v] = map[cell.Vertices[v].ID];
				}
				var outCell = new VtkCell(CellType.Line2, verticesOfCell);
				outCells.Add(outCell);
			}
		}

		public OutputCrackMesh(ICrackFront3D crackFront)
		{
			var map = new Dictionary<Vertex3D, VtkPoint>();

			// Vertices
			outVertices = new SortedDictionary<int, VtkPoint>();
			for (int v = 0; v < crackFront.Vertices.Count; ++v)
			{
				Vertex3D vertex = crackFront.Vertices[v];
				var outVertex = new VtkPoint(v, vertex.CoordsGlobal);
				outVertices[v] = outVertex;
				map[vertex] = outVertex;
			}

			// Cells
			outCells = new List<VtkCell>();
			foreach (Edge3D edge in crackFront.Edges)
			{
				var verticesOfCell = new VtkPoint[2];
				verticesOfCell[0] = map[edge.Start];
				verticesOfCell[1] = map[edge.End];
				var outCell = new VtkCell(CellType.Line2, verticesOfCell);
				outCells.Add(outCell);
			}
		}

		public int NumOutCells => outCells.Count;

		public int NumOutVertices => outVertices.Count;

		public IEnumerable<VtkCell> OutCells => outCells;

		public IEnumerable<VtkPoint> OutVertices => outVertices.Values; 
	}
}
