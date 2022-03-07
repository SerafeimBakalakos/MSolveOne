using System.Collections.Generic;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Meshes.Structured;

namespace MGroup.MSolve.Meshes.Output.VTK
{
	public class VtkMeshDiscontinuous_v2 : IVtkMesh
	{
		public VtkMeshDiscontinuous_v2(IStructuredMesh originalMesh)
		{
			var vtkPoints = new List<VtkPoint>();
			var vtkCells = new List<VtkCell>(originalMesh.NumElementsTotal);
			int pointID = 0;

			foreach ((int elementID, int[] nodeIDs) in originalMesh.EnumerateElements())
			{
				var cellVertices = new VtkPoint[nodeIDs.Length];
				for (int i = 0; i < nodeIDs.Length; ++i)
				{
					double[] nodeCoords = originalMesh.GetNodeCoordinates(nodeIDs[i]);
					var point = new VtkPoint(pointID++, nodeCoords);
					cellVertices[i] = point;
					vtkPoints.Add(point);
				}
				var cell = new VtkCell(originalMesh.CellType, cellVertices);
				vtkCells.Add(cell);
			}
			this.VtkPoints = vtkPoints;
			this.VtkCells = vtkCells;
		}

		public IReadOnlyList<VtkCell> VtkCells { get; }

		public IReadOnlyList<VtkPoint> VtkPoints { get; }

		public void OffsetVerticesTowardsCentroids(double movementPercent)
		{
			foreach (VtkCell cell in VtkCells)
			{
				double[] centroid = FindCentroidOfCell(cell);
				foreach (VtkPoint point in cell.Vertices)
				{
					double[] direction = { centroid[0] - point.X, centroid[1] - point.Y, centroid[2] - point.Z };
					point.X += movementPercent * direction[0];
					point.Y += movementPercent * direction[1];
					point.Z += movementPercent * direction[2];
				}
			}
		}

		private double[] FindCentroidOfCell(VtkCell cell)
		{
			double[] centroid = new double[3];
			foreach (VtkPoint point in cell.Vertices)
			{
				centroid[0] += point.X / cell.Vertices.Count;
				centroid[1] += point.Y / cell.Vertices.Count;
				centroid[2] += point.Z / cell.Vertices.Count;
			}
			return centroid;
		}
	}
}
