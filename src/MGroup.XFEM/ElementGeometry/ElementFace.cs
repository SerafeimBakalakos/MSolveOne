using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Discretization.Mesh;

namespace MGroup.XFEM.ElementGeometry
{
    public class ElementFace
    {
        public int ID { get; set; }

        public CellType CellType { get; set; }

        /// <summary>
        /// Their order is the same as defined in <see cref="CellType"/>.
        /// </summary>
        public int[] NodeIDs { get; set; }

        /// <summary>
        /// Their order is the same as defined in <see cref="CellType"/>.
        /// </summary>
        public IReadOnlyList<double[]> NodesNatural { get; set; }

        public ElementEdge[] Edges { get; set; }

        public static HashSet<ElementFace> FindFacesOfNode(int nodeID, IEnumerable<ElementFace> faces)
        {
            var facesOfNode = new HashSet<ElementFace>();
            foreach (ElementFace face in faces)
            {
                if (face.NodeIDs.Contains(nodeID)) facesOfNode.Add(face);
            }
            return facesOfNode;
        }
    }
}
