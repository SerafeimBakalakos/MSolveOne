using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization;

//TODO: The whole conversion INode, IElement -> VtkPoint, VtkCell should not be necessary. It just doubles the memory 
//      requirement for the mesh. Instead INode, IElement should be used. The conversion is necessary for discontinuous meshes
//      though.
namespace MGroup.Solvers.DDM.Output.VTK
{
    public class VtkMesh: IVtkMesh
    {
        public VtkMesh(IReadOnlyList<INode> nodes, IReadOnlyList<IElement> elements)
        {
            this.OriginalNodes = nodes;
            this.OriginalElements = elements;

            var vtkPoints = new VtkPoint[nodes.Count];
            var nodes2Points = new Dictionary<int, VtkPoint>();
            for (int i = 0; i < vtkPoints.Length; ++i)
            {
                INode node = nodes[i];
                vtkPoints[i] = new VtkPoint(i, node.X, node.Y, node.Z);
                nodes2Points[node.ID] = vtkPoints[i]; //TODO: Even more memory waste.
            }
            this.VtkPoints = vtkPoints;

            var vtkCells = new VtkCell[elements.Count];
            for (int i = 0; i < vtkCells.Length; ++i)
            {
                IElement element = elements[i];
                var vertices = new VtkPoint[element.Nodes.Count];
                for (int j = 0; j < vertices.Length; ++j) vertices[j] = nodes2Points[element.Nodes[j].ID];
                vtkCells[i] = new VtkCell(element.ElementType.CellType, vertices);
            }
            this.VtkCells = vtkCells;
        }

        public IReadOnlyList<IElement> OriginalElements { get; }

        public IReadOnlyList<INode> OriginalNodes { get; }

        public IReadOnlyList<VtkCell> VtkCells { get; }

        public IReadOnlyList<VtkPoint> VtkPoints { get; }
    }
}
