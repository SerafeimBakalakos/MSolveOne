using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.ElementGeometry
{
    public class ElementTet4Geometry : IElementGeometry
    {
        public double CalcBulkSizeCartesian(IReadOnlyList<XNode> nodes)
        {
            double volume = Utilities.CalcTetrahedronVolume(nodes.Select(n => n.Coordinates).ToArray());
            Debug.Assert(volume > 0);
            return volume;
        }

        public double CalcBulkSizeNatural() => 1.0 / 6;


        public (ElementEdge[], ElementFace[]) FindEdgesFaces(IReadOnlyList<int> nodeIDs)
        {
            IReadOnlyList<double[]> nodesNatural = InterpolationTet4.UniqueInstance.NodalNaturalCoordinates;
            var edges = new ElementEdge[6];
            edges[0] = new ElementEdge(0, nodeIDs, nodesNatural, 0, 1);
            edges[1] = new ElementEdge(1, nodeIDs, nodesNatural, 0, 2);
            edges[2] = new ElementEdge(2, nodeIDs, nodesNatural, 0, 3);
            edges[3] = new ElementEdge(3, nodeIDs, nodesNatural, 1, 2);
            edges[4] = new ElementEdge(4, nodeIDs, nodesNatural, 2, 3);
            edges[5] = new ElementEdge(5, nodeIDs, nodesNatural, 3, 1);

            var faces = new ElementFace[4];
            faces[0] = new ElementFace();
            faces[0].ID = 0;
            faces[0].NodeIDs = new int[]
            {
                nodeIDs[0], nodeIDs[2], nodeIDs[1]
            };
            faces[0].NodesNatural = new double[][]
            {
                nodesNatural[0], nodesNatural[2], nodesNatural[1]
            };
            faces[0].Edges = new ElementEdge[] { edges[1], edges[3], edges[0] };

            faces[1] = new ElementFace();
            faces[1].ID = 1;
            faces[1].NodeIDs = new int[]
            {
                nodeIDs[0], nodeIDs[1], nodeIDs[3]
            };
            faces[1].NodesNatural = new double[][]
            {
                nodesNatural[0], nodesNatural[1], nodesNatural[3]
            };
            faces[1].Edges = new ElementEdge[] { edges[0], edges[5], edges[2] };

            faces[2] = new ElementFace();
            faces[2].ID = 2;
            faces[2].NodeIDs = new int[]
            {
                nodeIDs[0], nodeIDs[3], nodeIDs[2]
            };
            faces[2].NodesNatural = new double[][]
            {
                nodesNatural[0], nodesNatural[3], nodesNatural[2]
            };
            faces[2].Edges = new ElementEdge[] { edges[2], edges[4], edges[1] };

            faces[3] = new ElementFace();
            faces[3].ID = 3;
            faces[3].NodeIDs = new int[]
            {
                nodeIDs[1], nodeIDs[2], nodeIDs[3]
            };
            faces[3].NodesNatural = new double[][]
            {
                nodesNatural[1], nodesNatural[2], nodesNatural[3]
            };
            faces[3].Edges = new ElementEdge[] { edges[3], edges[4], edges[5] };

            return (edges, faces);
        }
    }
}
