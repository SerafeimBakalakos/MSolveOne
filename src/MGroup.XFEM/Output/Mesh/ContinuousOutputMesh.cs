using System.Collections.Generic;
using System.Linq;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Mesh
{
    public class ContinuousOutputMesh : IOutputMesh
    {
        private readonly List<VtkCell> outCells;
        private readonly SortedDictionary<int, VtkPoint> outVertices;

        public ContinuousOutputMesh(IEnumerable<XNode> originalVertices, IEnumerable<IXFiniteElement> originalCells)
        {
            this.OriginalVertices = originalVertices;
            this.OriginalCells = originalCells;


            this.outVertices = new SortedDictionary<int, VtkPoint>();
            foreach (XNode vertex in originalVertices)
            {
                var outVertex = new VtkPoint(vertex.ID, vertex.Coordinates);
                outVertices[vertex.ID] = outVertex;
            }

            this.outCells = new List<VtkCell>();
            foreach (IXFiniteElement cell in originalCells)
            {
                List<VtkPoint> vertices = cell.Nodes.Select(v => outVertices[v.ID]).ToList();
                outCells.Add(new VtkCell(cell.CellType, vertices));
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
