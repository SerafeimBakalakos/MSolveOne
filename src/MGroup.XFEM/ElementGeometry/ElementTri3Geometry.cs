using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Interpolation;

namespace MGroup.XFEM.ElementGeometry
{
    public class ElementTri3Geometry : IElementGeometry
    {
        public double CalcBulkSizeCartesian(IReadOnlyList<XNode> nodes)
        {
            double area = Utilities.CalcPolygonArea(nodes.Select(n => n.Coordinates).ToArray());
            Debug.Assert(area > 0);
            return area;
        }

        public double CalcBulkSizeNatural() => 0.5;

        public (ElementEdge[], ElementFace[]) FindEdgesFaces(IReadOnlyList<int> nodeIDs)
        {
            IReadOnlyList<double[]> nodesNatural = InterpolationTri3.UniqueInstance.NodalNaturalCoordinates;
            var edges = new ElementEdge[3];
            edges[0] = new ElementEdge(0, nodeIDs, nodesNatural, 0, 1);
            edges[1] = new ElementEdge(1, nodeIDs, nodesNatural, 1, 2);
            edges[2] = new ElementEdge(2, nodeIDs, nodesNatural, 2, 0);
            return (edges, new ElementFace[0]);
        }
    }
}
