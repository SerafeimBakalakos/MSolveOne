using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Mesh
{
	public class LsmIntersectionSegmentsMesh : IOutputMesh
	{
		private SortedDictionary<int, VtkPoint> outVertices;

		public LsmIntersectionSegmentsMesh(IEnumerable<IElementDiscontinuityInteraction> intersections)
		{
			var vertices = new SortedDictionary<int, VtkPoint>();
			this.ParentElementIDsOfVertices = new List<double>();
			this.ParentGeometryIDsOfVertices = new List<double>();
			var cells = new List<VtkCell>();
			int vertexID = 0;
			foreach (IElementDiscontinuityInteraction intersection in intersections)
			{
				// Vertices of the intersection mesh
				IIntersectionMesh intersectionMesh = intersection.ApproximateGlobalCartesian();
				IList<double[]> intersectionPoints = intersectionMesh.Vertices;
				var verticesOfIntersection = new List<VtkPoint>();
				for (int v = 0; v < intersectionPoints.Count; ++v)
				{
					double[] point = intersectionPoints[v];
					var vertex = new VtkPoint(vertexID++, point);
					vertices[vertex.ID] = vertex;
					verticesOfIntersection.Add(vertex);
					ParentElementIDsOfVertices.Add(intersection.Element.ID);
					ParentGeometryIDsOfVertices.Add(intersection.ParentGeometryID);
				}
				
				// Cells of the intersection mesh
				for (int c = 0; c < intersectionMesh.Cells.Count; ++c)
				{
					(CellType cellType, int[] connectivity) = intersectionMesh.Cells[c];
					var verticesOfCell = new VtkPoint[connectivity.Length];
					for (int v = 0; v < verticesOfCell.Length; ++v)
					{
						verticesOfCell[v] = verticesOfIntersection[connectivity[v]];
					}
					cells.Add(new VtkCell(cellType, verticesOfCell));
				}
			}
			this.outVertices = vertices;
			NumOutVertices = vertices.Count;
			OutCells = cells;
			NumOutCells = cells.Count;
		}

		public int NumOutCells { get; }


		public int NumOutVertices { get; }

		public IEnumerable<VtkCell> OutCells { get; }

		public IEnumerable<VtkPoint> OutVertices => outVertices.Values;

		public List<double> ParentElementIDsOfVertices { get; }

		public List<double> ParentGeometryIDsOfVertices { get; }
	}
}
