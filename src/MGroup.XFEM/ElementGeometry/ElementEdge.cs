using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;

namespace MGroup.XFEM.ElementGeometry
{
    public class ElementEdge
    {
        public ElementEdge(int id)
        {
            this.ID = id;
        }

        public ElementEdge(int id, IReadOnlyList<int> nodes, IReadOnlyList<double[]> nodesNatural, int start, int end)
        {
            this.ID = id;
            CellType = CellType.Line2;
            NodeIDs = new int[] { nodes[start], nodes[end] };
            NodesNatural = new double[][] { nodesNatural[start], nodesNatural[end] };
        }

        public CellType CellType { get; set; }

        public int ID { get; }

        /// <summary>
        /// Their order is the same as defined in <see cref="CellType"/>.
        /// </summary>
        public int[] NodeIDs { get; set; }

        /// <summary>
        /// Their order is the same as defined in <see cref="CellType"/>.
        /// </summary>
        public IReadOnlyList<double[]> NodesNatural { get; set; }

        public HashSet<ElementFace> FindFacesOfEdge(IEnumerable<ElementFace> faces)
        {
            var facesOfEdge = new HashSet<ElementFace>();
            foreach (ElementFace face in faces)
            {
                if (face.Edges.Contains(this)) facesOfEdge.Add(face);
            }
            return facesOfEdge;
        }
    }
}
