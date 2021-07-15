using System.Collections.Generic;
using MGroup.MSolve.Discretization;

namespace MGroup.Solvers.DDM.Output.VTK
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
                    var point = new VtkPoint(pointID++, element.Nodes[i]);
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
