using System.Collections.Generic;
using MGroup.MSolve.Discretization;

//TODO: Also implement this logic but without dependencies on INode, IElement. It should work with the vertices and cells in
//      IStructuredMesh. It should also offset the nodes of each element towards its centroid by a user defiend percent of the 
//      original distance. This is crucial when inspecting 3D meshes, where it is impossible to discern where one Tet4 ends and 
//      where the next begins.
namespace MGroup.MSolve.Meshes.Output.VTK
{
	public class VtkMeshDiscontinuous : IVtkMesh 
	{
		public VtkMeshDiscontinuous(IReadOnlyList<INode> nodes, IReadOnlyList<IElement> elements)
		{
			this.OriginalNodes = nodes;
			this.OriginalElements = elements;
			var vtkPoints = new List<VtkPoint>();
			var vtkCells = new VtkCell[elements.Count];
			int pointID = 0;

			for (int e = 0; e < elements.Count; ++e)
			{
				IElement element = elements[e];
				var cellVertices = new VtkPoint[element.Nodes.Count];
				for (int i = 0; i < element.Nodes.Count; ++i)
				{
					INode node = element.Nodes[i];

					var point = new VtkPoint(pointID++, node.X, node.Y, node.Z);
					cellVertices[i] = point;
					vtkPoints.Add(point);
				}
				var cell = new VtkCell(element.ElementType.CellType, cellVertices);
				vtkCells[e] = cell;
			}
			this.VtkPoints = vtkPoints;
			this.VtkCells = vtkCells;
		}

		public IReadOnlyList<IElement> OriginalElements { get; }

		public IReadOnlyList<INode> OriginalNodes { get; }

		public IReadOnlyList<VtkCell> VtkCells { get; }

		public IReadOnlyList<VtkPoint> VtkPoints { get; }
	}
}
